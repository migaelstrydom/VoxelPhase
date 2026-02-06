//! Physics world containing all simulation state.

use generational_arena::Arena;
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use std::cmp::Ordering;
use std::collections::HashMap;

use super::body::{RigidBody, RigidBodyDesc};
use super::collider::{Collider, ColliderDesc, ColliderShape};
use super::collision::swept_sphere_sphere;
use super::handle::{ColliderHandle, RigidBodyHandle};
use super::math::integrate_orientation;
use super::pipeline::integration::integrate_forces;
use super::pipeline::solver::{combine_materials, solve_contacts, ContactConstraint};
use super::static_geometry::StaticGeometry;
use crate::debug::DebugLines;

/// Configuration for the physics simulation.
#[derive(Debug, Clone)]
pub struct PhysicsConfig {
    /// Gravity acceleration vector.
    pub gravity: Vector3<f32>,
    /// Number of solver iterations per step.
    pub solver_iterations: u32,
    /// Maximum number of CCD loops a body can go through per step.
    pub max_ccd_iterations: u32,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            gravity: Vector3::new(0.0, -20.0, 0.0),
            solver_iterations: 1,
            max_ccd_iterations: 3,
        }
    }
}

/// The physics simulation world.
///
/// Owns all rigid bodies, colliders, and joints. Provides a handle-based API
/// for creating and manipulating physics objects.
pub struct PhysicsWorld {
    config: PhysicsConfig,
    bodies: Arena<RigidBody>,
    colliders: Arena<Collider>,
    manifold_cache: HashMap<ManifoldKey, CachedManifold>,
    static_contact_cache: HashMap<StaticContactKey, StaticContactEntry>,
    warm_start_cache: HashMap<WarmStartKey, WarmStartEntry>,
    frame_index: u64,
}

const MANIFOLD_TTL_FRAMES: u64 = 10;
const STATIC_CONTACT_TTL_FRAMES: u64 = 10;
const STATIC_CONTACT_REST_THRESHOLD: f32 = 0.5;
const STATIC_CONTACT_SLOP: f32 = 0.02;
const WARM_START_TTL_FRAMES: u64 = 20;
const WARM_START_DECAY: f32 = 0.9;
const WARM_START_REST_THRESHOLD: f32 = 0.5;

#[derive(Debug, Clone)]
struct BodyPrediction {
    segment_start_pos: Point3<f32>,
    segment_start_rot: UnitQuaternion<f32>,
    segment_start_time: f32,
    predicted_pos: Point3<f32>,
    predicted_rot: UnitQuaternion<f32>,
    iterations: u32,
}

#[derive(Debug, Clone)]
struct CollisionEvent {
    time: f32,
    body_a: Option<RigidBodyHandle>,
    body_b: RigidBodyHandle,
    contacts: Vec<ContactConstraint>,
}

#[derive(Debug, Clone, Copy)]
struct BodySnapshot {
    position: Point3<f32>,
    linear_velocity: Vector3<f32>,
    angular_velocity: Vector3<f32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct ManifoldKey {
    body_a: RigidBodyHandle,
    body_b: RigidBodyHandle,
}

#[derive(Debug, Clone)]
struct CachedManifold {
    normal: Vector3<f32>,
    last_seen_frame: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct StaticContactKey {
    body: RigidBodyHandle,
    collider: ColliderHandle,
}

#[derive(Debug, Clone)]
struct StaticContactEntry {
    normal: Vector3<f32>,
    point: Point3<f32>,
    last_seen_frame: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct WarmStartKey {
    body_a: Option<RigidBodyHandle>,
    body_b: RigidBodyHandle,
    normal_key: [i16; 3],
}

#[derive(Debug, Clone, Copy)]
struct WarmStartEntry {
    normal_impulse: f32,
    last_seen_frame: u64,
}

impl PhysicsWorld {
    pub fn new(config: PhysicsConfig) -> Self {
        Self {
            config,
            bodies: Arena::new(),
            colliders: Arena::new(),
            manifold_cache: HashMap::new(),
            static_contact_cache: HashMap::new(),
            warm_start_cache: HashMap::new(),
            frame_index: 0,
        }
    }

    /// Get the physics configuration.
    pub fn config(&self) -> &PhysicsConfig {
        &self.config
    }

    /// Set gravity.
    pub fn set_gravity(&mut self, gravity: Vector3<f32>) {
        self.config.gravity = gravity;
    }

    // === Body Management ===

    /// Create a new rigid body and return its handle.
    pub fn create_body(&mut self, desc: RigidBodyDesc) -> RigidBodyHandle {
        let body = RigidBody::new(desc);
        RigidBodyHandle(self.bodies.insert(body))
    }

    /// Remove a rigid body and all its attached colliders.
    pub fn remove_body(&mut self, handle: RigidBodyHandle) -> bool {
        let Some(body) = self.bodies.remove(handle.0) else {
            return false;
        };

        // Remove all attached colliders
        for collider_handle in body.colliders() {
            self.colliders.remove(collider_handle.0);
        }

        true
    }

    /// Get a reference to a rigid body.
    pub fn body(&self, handle: RigidBodyHandle) -> Option<&RigidBody> {
        self.bodies.get(handle.0)
    }

    /// Get a mutable reference to a rigid body.
    pub fn body_mut(&mut self, handle: RigidBodyHandle) -> Option<&mut RigidBody> {
        self.bodies.get_mut(handle.0)
    }

    /// Iterate over all rigid body handles.
    pub fn body_handles(&self) -> impl Iterator<Item = RigidBodyHandle> + '_ {
        self.bodies.iter().map(|(idx, _)| RigidBodyHandle(idx))
    }

    // === Collider Management ===

    /// Attach a collider to a rigid body.
    pub fn attach_collider(
        &mut self,
        body_handle: RigidBodyHandle,
        desc: ColliderDesc,
    ) -> Option<ColliderHandle> {
        // Verify body exists
        if self.bodies.get(body_handle.0).is_none() {
            return None;
        }

        let collider = Collider::new(body_handle, desc);
        let collider_handle = ColliderHandle(self.colliders.insert(collider));

        // Update body's mass properties
        if let Some(body) = self.bodies.get_mut(body_handle.0) {
            body.add_collider(collider_handle);
            self.recompute_mass_properties(body_handle);
        }

        Some(collider_handle)
    }

    /// Remove a collider from its body.
    pub fn remove_collider(&mut self, handle: ColliderHandle) -> bool {
        let Some(collider) = self.colliders.remove(handle.0) else {
            return false;
        };

        let body_handle = collider.body();

        // Update the body
        if let Some(body) = self.bodies.get_mut(body_handle.0) {
            body.remove_collider(handle);
            self.recompute_mass_properties(body_handle);
        }

        true
    }

    /// Get a reference to a collider.
    pub fn collider(&self, handle: ColliderHandle) -> Option<&Collider> {
        self.colliders.get(handle.0)
    }

    // === Simulation ===

    /// Step the physics simulation forward by dt seconds.
    pub fn step(
        &mut self,
        dt: f32,
        static_geometry: &dyn StaticGeometry,
        _debug_lines: &mut DebugLines,
    ) {
        self.frame_index = self.frame_index.wrapping_add(1);
        self.prune_manifold_cache();
        self.prune_static_contact_cache();
        self.prune_warm_start_cache();

        // 1. Integrate forces into velocities
        integrate_forces(&mut self.bodies, dt, self.config.gravity);

        // 2. Predict body motion without committing positions
        let mut predictions = self.build_predictions(dt);

        // 3. Run CCD for all pairs and static geometry
        let mut events = Vec::new();
        self.collect_cached_manifold_events(dt, &predictions, &mut events);
        self.collect_ccd_events(static_geometry, dt, &predictions, &mut events);

        _debug_lines.add("Events", format!("{}", events.len()));
        _debug_lines.add("Num predictions", format!("{}", predictions.len()));
        _debug_lines.add("Num bodies", format!("{}", self.bodies.len()));
        let first_body = self.bodies.iter().next().unwrap().1;
        _debug_lines.add(
            "First body position",
            format!(
                "{:.2}, {:.2}, {:.2}",
                first_body.position().x,
                first_body.position().y,
                first_body.position().z
            ),
        );

        // 4. Resolve collisions in time order with limited re-CCD
        while let Some(event) = self.pop_next_event(&mut events) {
            if !self.event_is_valid(&event, &predictions) {
                continue;
            }

            self.advance_bodies_to_time(&event, dt, &predictions);

            self.apply_warm_start_impulses(&event.contacts);

            let mut accumulated_impulses = vec![0.0; event.contacts.len()];
            for _ in 0..self.config.solver_iterations {
                let impulses = solve_contacts(&mut self.bodies, &event.contacts);
                for (accumulated, impulse) in accumulated_impulses.iter_mut().zip(impulses) {
                    *accumulated += impulse;
                }
            }

            self.store_warm_start_impulses(&event.contacts, &accumulated_impulses);
            self.store_manifold_from_event(&event);

            let mut reccd_bodies = Vec::new();
            if let Some(handle_a) = event.body_a {
                reccd_bodies.push(handle_a);
            }
            reccd_bodies.push(event.body_b);

            for body_handle in reccd_bodies {
                let Some(body) = self.bodies.get(body_handle.0) else {
                    continue;
                };
                if body.is_static() {
                    continue;
                }
                let Some(prediction) = predictions.get_mut(&body_handle.0) else {
                    continue;
                };
                if prediction.iterations >= self.config.max_ccd_iterations {
                    continue;
                }

                self.update_prediction_after_collision(body_handle, prediction, dt, event.time);
                self.collect_ccd_events_for_body(
                    body_handle,
                    static_geometry,
                    &predictions,
                    &mut events,
                );
            }
        }

        // 5. Commit predicted positions at the end of the step
        for (idx, prediction) in predictions {
            let Some(body) = self.bodies.get_mut(idx) else {
                continue;
            };
            if body.is_static() {
                continue;
            }
            body.set_position(prediction.predicted_pos);
            body.set_rotation(prediction.predicted_rot);
        }
    }

    // === Internal Methods ===

    fn recompute_mass_properties(&mut self, body_handle: RigidBodyHandle) {
        let Some(body) = self.bodies.get(body_handle.0) else {
            return;
        };

        let collider_handles: Vec<_> = body.colliders().to_vec();

        let mut total_mass = 0.0f32;
        let mut total_inertia = Matrix3::zeros();

        for ch in &collider_handles {
            if let Some(collider) = self.colliders.get(ch.0) {
                total_mass += collider.mass();
                // For now, just add local inertias (ignoring offset transforms)
                // A proper implementation would use parallel axis theorem
                total_inertia += collider.local_inertia();
            }
        }

        if let Some(body) = self.bodies.get_mut(body_handle.0) {
            body.set_mass_properties(total_mass, total_inertia);
        }
    }

    fn build_predictions(&self, dt: f32) -> HashMap<generational_arena::Index, BodyPrediction> {
        let mut predictions = HashMap::new();
        for (idx, body) in self.bodies.iter() {
            let start_pos = body.position();
            let start_rot = body.rotation();
            let (predicted_pos, predicted_rot) = if body.is_static() {
                (start_pos, start_rot)
            } else {
                (
                    start_pos + body.linear_velocity() * dt,
                    integrate_orientation(start_rot, body.angular_velocity(), dt),
                )
            };
            predictions.insert(
                idx,
                BodyPrediction {
                    segment_start_pos: start_pos,
                    segment_start_rot: start_rot,
                    segment_start_time: 0.0,
                    predicted_pos,
                    predicted_rot,
                    iterations: 0,
                },
            );
        }
        predictions
    }

    fn collect_ccd_events(
        &mut self,
        static_geometry: &dyn StaticGeometry,
        dt: f32,
        predictions: &HashMap<generational_arena::Index, BodyPrediction>,
        events: &mut Vec<CollisionEvent>,
    ) {
        let body_handles: Vec<_> = self
            .bodies
            .iter()
            .filter_map(|(idx, body)| {
                if body.is_static() {
                    None
                } else {
                    Some(RigidBodyHandle(idx))
                }
            })
            .collect();
        for body_handle in body_handles {
            self.collect_ccd_events_for_body(body_handle, static_geometry, predictions, events);
        }

        self.collect_body_body_ccd_events(dt, predictions, events);
    }

    fn collect_ccd_events_for_body(
        &mut self,
        body_handle: RigidBodyHandle,
        static_geometry: &dyn StaticGeometry,
        predictions: &HashMap<generational_arena::Index, BodyPrediction>,
        events: &mut Vec<CollisionEvent>,
    ) {
        let (collider_handles, body_snapshot) = {
            let Some(body) = self.bodies.get(body_handle.0) else {
                return;
            };
            if body.is_static() {
                return;
            }
            (
                body.colliders().to_vec(),
                BodySnapshot {
                    position: body.position(),
                    linear_velocity: body.linear_velocity(),
                    angular_velocity: body.angular_velocity(),
                },
            )
        };
        let Some(prediction) = predictions.get(&body_handle.0) else {
            return;
        };
        if prediction.segment_start_time >= 1.0 {
            return;
        }

        for collider_handle in &collider_handles {
            let Some(collider) = self.colliders.get(collider_handle.0) else {
                continue;
            };
            let radius = match collider.shape() {
                ColliderShape::Sphere { radius } => radius,
            };

            let start_center =
                collider.world_center(prediction.segment_start_pos, prediction.segment_start_rot);
            let end_center =
                collider.world_center(prediction.predicted_pos, prediction.predicted_rot);

            let movement = end_center - start_center;
            let cache_key = StaticContactKey {
                body: body_handle,
                collider: *collider_handle,
            };
            if self.try_cached_static_contact(
                cache_key,
                *radius,
                start_center,
                &body_snapshot,
                prediction.segment_start_time,
                collider.material(),
                events,
            ) {
                self.touch_static_contact_cache(cache_key);
                continue;
            }
            if movement.magnitude_squared() > 1e-6 {
                if let Some(sweep_hit) =
                    static_geometry.sweep_sphere(start_center, end_center, *radius)
                {
                    let event_time = prediction.segment_start_time
                        + (1.0 - prediction.segment_start_time) * sweep_hit.t;
                    let center_at_t = start_center + movement * sweep_hit.t;
                    let to_center = center_at_t - sweep_hit.point;
                    let contact_normal = if to_center.magnitude_squared() > 1e-6 {
                        to_center.normalize()
                    } else {
                        sweep_hit.normal
                    };

                    let contact = ContactConstraint {
                        body_a: None,
                        body_b: body_handle,
                        point: sweep_hit.point,
                        normal: contact_normal,
                        depth: 0.0,
                        restitution: collider.material().restitution,
                        friction: collider.material().friction,
                    };

                    events.push(CollisionEvent {
                        time: event_time,
                        body_a: None,
                        body_b: body_handle,
                        contacts: vec![contact],
                    });
                }
            } else {
                let static_contacts = static_geometry.query_sphere(start_center, *radius);
                if !static_contacts.is_empty() {
                    let contacts: Vec<ContactConstraint> = static_contacts
                        .into_iter()
                        .map(|sc| ContactConstraint {
                            body_a: None,
                            body_b: body_handle,
                            point: sc.point,
                            normal: sc.normal,
                            depth: sc.depth,
                            restitution: collider.material().restitution,
                            friction: collider.material().friction,
                        })
                        .collect();
                    self.store_static_contact_cache(body_handle, *collider_handle, &contacts);
                    events.push(CollisionEvent {
                        time: prediction.segment_start_time,
                        body_a: None,
                        body_b: body_handle,
                        contacts,
                    });
                }
            }
        }
    }

    fn collect_body_body_ccd_events(
        &self,
        dt: f32,
        predictions: &HashMap<generational_arena::Index, BodyPrediction>,
        events: &mut Vec<CollisionEvent>,
    ) {
        let mut spheres = Vec::new();
        for (idx, body) in self.bodies.iter() {
            if body.is_static() {
                continue;
            }
            let Some((collider_handle, radius, material)) = self.primary_sphere_collider(body)
            else {
                continue;
            };
            let Some(prediction) = predictions.get(&idx) else {
                continue;
            };
            spheres.push((
                RigidBodyHandle(idx),
                collider_handle,
                radius,
                material,
                prediction.segment_start_time,
            ));
        }

        for i in 0..spheres.len() {
            for j in (i + 1)..spheres.len() {
                let (handle_a, collider_a, radius_a, mat_a, start_time_a) = spheres[i];
                let (handle_b, collider_b, radius_b, mat_b, start_time_b) = spheres[j];

                let start_time = start_time_a.max(start_time_b);
                if start_time >= 1.0 {
                    continue;
                }

                let Some(pred_a) = predictions.get(&handle_a.0) else {
                    continue;
                };
                let Some(pred_b) = predictions.get(&handle_b.0) else {
                    continue;
                };
                let Some(body_a) = self.bodies.get(handle_a.0) else {
                    continue;
                };
                let Some(body_b) = self.bodies.get(handle_b.0) else {
                    continue;
                };
                let Some(collider_a_ref) = self.colliders.get(collider_a.0) else {
                    continue;
                };
                let Some(collider_b_ref) = self.colliders.get(collider_b.0) else {
                    continue;
                };

                let (start_pos_a, start_rot_a) = Self::pose_at_time(body_a, pred_a, dt, start_time);
                let (start_pos_b, start_rot_b) = Self::pose_at_time(body_b, pred_b, dt, start_time);
                let end_pos_a = pred_a.predicted_pos;
                let end_rot_a = pred_a.predicted_rot;
                let end_pos_b = pred_b.predicted_pos;
                let end_rot_b = pred_b.predicted_rot;

                let start_center_a = collider_a_ref.world_center(start_pos_a, start_rot_a);
                let start_center_b = collider_b_ref.world_center(start_pos_b, start_rot_b);
                let end_center_a = collider_a_ref.world_center(end_pos_a, end_rot_a);
                let end_center_b = collider_b_ref.world_center(end_pos_b, end_rot_b);

                let relative_motion =
                    (end_center_a - start_center_a) - (end_center_b - start_center_b);
                if relative_motion.magnitude_squared() <= 1e-10 {
                    let combined_radius = radius_a + radius_b;
                    let dist_sq = (start_center_b - start_center_a).magnitude_squared();
                    if dist_sq <= combined_radius * combined_radius {
                        let (point, normal, depth) = compute_sphere_contact(
                            start_center_a,
                            radius_a,
                            start_center_b,
                            radius_b,
                        );
                        let (restitution, friction) = combine_materials(&mat_a, &mat_b);
                        events.push(CollisionEvent {
                            time: start_time,
                            body_a: Some(handle_a),
                            body_b: handle_b,
                            contacts: vec![ContactConstraint {
                                body_a: Some(handle_a),
                                body_b: handle_b,
                                point,
                                normal,
                                depth,
                                restitution,
                                friction,
                            }],
                        });
                    }
                    continue;
                }

                if let Some(t) = swept_sphere_sphere(
                    start_center_a,
                    end_center_a,
                    radius_a,
                    start_center_b,
                    end_center_b,
                    radius_b,
                ) {
                    let event_time = start_time + (1.0 - start_time) * t;
                    let (center_a, center_b) = Self::centers_at_time(
                        collider_a_ref,
                        collider_b_ref,
                        body_a,
                        body_b,
                        pred_a,
                        pred_b,
                        dt,
                        event_time,
                    );
                    let (point, normal, depth) =
                        compute_sphere_contact(center_a, radius_a, center_b, radius_b);
                    let (restitution, friction) = combine_materials(&mat_a, &mat_b);
                    events.push(CollisionEvent {
                        time: event_time,
                        body_a: Some(handle_a),
                        body_b: handle_b,
                        contacts: vec![ContactConstraint {
                            body_a: Some(handle_a),
                            body_b: handle_b,
                            point,
                            normal,
                            depth,
                            restitution,
                            friction,
                        }],
                    });
                }
            }
        }
    }

    fn pop_next_event(&self, events: &mut Vec<CollisionEvent>) -> Option<CollisionEvent> {
        if events.is_empty() {
            return None;
        }
        events.sort_by(|a, b| a.time.partial_cmp(&b.time).unwrap_or(Ordering::Equal));
        Some(events.remove(0))
    }

    fn event_is_valid(
        &self,
        event: &CollisionEvent,
        predictions: &HashMap<generational_arena::Index, BodyPrediction>,
    ) -> bool {
        if event.time < 0.0 || event.time > 1.0 {
            return false;
        }
        let Some(pred_b) = predictions.get(&event.body_b.0) else {
            return false;
        };
        if pred_b.iterations >= self.config.max_ccd_iterations {
            return false;
        }
        if event.time + 1e-6 < pred_b.segment_start_time {
            return false;
        }
        if let Some(handle_a) = event.body_a {
            let Some(pred_a) = predictions.get(&handle_a.0) else {
                return false;
            };
            if pred_a.iterations >= self.config.max_ccd_iterations {
                return false;
            }
            if event.time + 1e-6 < pred_a.segment_start_time {
                return false;
            }
        }
        true
    }

    fn advance_bodies_to_time(
        &mut self,
        event: &CollisionEvent,
        dt: f32,
        predictions: &HashMap<generational_arena::Index, BodyPrediction>,
    ) {
        if let Some(handle_a) = event.body_a {
            if let (Some(body), Some(prediction)) = (
                self.bodies.get_mut(handle_a.0),
                predictions.get(&handle_a.0),
            ) {
                let (pos, rot) = Self::pose_at_time(body, prediction, dt, event.time);
                body.set_position(pos);
                body.set_rotation(rot);
            }
        }
        if let (Some(body), Some(prediction)) = (
            self.bodies.get_mut(event.body_b.0),
            predictions.get(&event.body_b.0),
        ) {
            let (pos, rot) = Self::pose_at_time(body, prediction, dt, event.time);
            body.set_position(pos);
            body.set_rotation(rot);
        }
    }

    fn update_prediction_after_collision(
        &mut self,
        body_handle: RigidBodyHandle,
        prediction: &mut BodyPrediction,
        dt: f32,
        time: f32,
    ) {
        let Some(body) = self.bodies.get(body_handle.0) else {
            return;
        };
        prediction.segment_start_time = time;
        prediction.segment_start_pos = body.position();
        prediction.segment_start_rot = body.rotation();
        prediction.iterations += 1;

        let remaining_dt = (1.0 - time).max(0.0) * dt;
        prediction.predicted_pos = body.position() + body.linear_velocity() * remaining_dt;
        prediction.predicted_rot =
            integrate_orientation(body.rotation(), body.angular_velocity(), remaining_dt);
    }

    fn pose_at_time(
        body: &RigidBody,
        prediction: &BodyPrediction,
        dt: f32,
        time: f32,
    ) -> (Point3<f32>, UnitQuaternion<f32>) {
        if time <= prediction.segment_start_time {
            return (prediction.segment_start_pos, prediction.segment_start_rot);
        }
        if time >= 1.0 {
            return (prediction.predicted_pos, prediction.predicted_rot);
        }
        let span = 1.0 - prediction.segment_start_time;
        if span <= 1e-6 {
            return (prediction.segment_start_pos, prediction.segment_start_rot);
        }
        let alpha = ((time - prediction.segment_start_time) / span).clamp(0.0, 1.0);
        let pos = prediction.segment_start_pos
            + (prediction.predicted_pos - prediction.segment_start_pos) * alpha;
        let rot = integrate_orientation(
            prediction.segment_start_rot,
            body.angular_velocity(),
            dt * (time - prediction.segment_start_time),
        );
        (pos, rot)
    }

    fn centers_at_time(
        collider_a: &Collider,
        collider_b: &Collider,
        body_a: &RigidBody,
        body_b: &RigidBody,
        pred_a: &BodyPrediction,
        pred_b: &BodyPrediction,
        dt: f32,
        time: f32,
    ) -> (Point3<f32>, Point3<f32>) {
        let (pos_a, rot_a) = Self::pose_at_time(body_a, pred_a, dt, time);
        let (pos_b, rot_b) = Self::pose_at_time(body_b, pred_b, dt, time);
        (
            collider_a.world_center(pos_a, rot_a),
            collider_b.world_center(pos_b, rot_b),
        )
    }

    fn primary_sphere_collider(
        &self,
        body: &RigidBody,
    ) -> Option<(ColliderHandle, f32, super::collider::ColliderMaterial)> {
        let collider_handle = body.colliders().first()?;
        let collider = self.colliders.get(collider_handle.0)?;
        let radius = match collider.shape() {
            ColliderShape::Sphere { radius } => radius,
        };
        Some((*collider_handle, *radius, *collider.material()))
    }

    fn collect_cached_manifold_events(
        &mut self,
        dt: f32,
        predictions: &HashMap<generational_arena::Index, BodyPrediction>,
        events: &mut Vec<CollisionEvent>,
    ) {
        if self.manifold_cache.is_empty() {
            return;
        }

        let cached_entries: Vec<(ManifoldKey, CachedManifold)> = self
            .manifold_cache
            .iter()
            .map(|(key, manifold)| (*key, manifold.clone()))
            .collect();

        let slop = 0.02;
        for (key, cached) in cached_entries {
            let (Some(body_a), Some(body_b)) =
                (self.bodies.get(key.body_a.0), self.bodies.get(key.body_b.0))
            else {
                continue;
            };
            if body_a.is_static() && body_b.is_static() {
                continue;
            }
            let Some(pred_a) = predictions.get(&key.body_a.0) else {
                continue;
            };
            let Some(pred_b) = predictions.get(&key.body_b.0) else {
                continue;
            };

            let Some((collider_a_handle, radius_a, mat_a)) = self.primary_sphere_collider(body_a)
            else {
                continue;
            };
            let Some((collider_b_handle, radius_b, mat_b)) = self.primary_sphere_collider(body_b)
            else {
                continue;
            };
            let Some(collider_a) = self.colliders.get(collider_a_handle.0) else {
                continue;
            };
            let Some(collider_b) = self.colliders.get(collider_b_handle.0) else {
                continue;
            };

            let start_time = pred_a.segment_start_time.max(pred_b.segment_start_time);
            if start_time >= 1.0 {
                continue;
            }

            let (start_pos_a, start_rot_a) = Self::pose_at_time(body_a, pred_a, dt, start_time);
            let (start_pos_b, start_rot_b) = Self::pose_at_time(body_b, pred_b, dt, start_time);
            let end_pos_a = pred_a.predicted_pos;
            let end_rot_a = pred_a.predicted_rot;
            let end_pos_b = pred_b.predicted_pos;
            let end_rot_b = pred_b.predicted_rot;

            let start_center_a = collider_a.world_center(start_pos_a, start_rot_a);
            let start_center_b = collider_b.world_center(start_pos_b, start_rot_b);
            let end_center_a = collider_a.world_center(end_pos_a, end_rot_a);
            let end_center_b = collider_b.world_center(end_pos_b, end_rot_b);

            let relative_motion = (end_center_a - start_center_a) - (end_center_b - start_center_b);
            if relative_motion.magnitude_squared() > 1e-8 {
                continue;
            }

            let combined_radius = radius_a + radius_b;
            let delta = start_center_b - start_center_a;
            let dist_sq = delta.magnitude_squared();
            if dist_sq > (combined_radius + slop) * (combined_radius + slop) {
                continue;
            }

            let dist = dist_sq.sqrt();
            let mut normal = if dist > 1e-6 {
                delta / dist
            } else {
                cached.normal
            };
            if normal.dot(&cached.normal) < 0.0 {
                normal = -normal;
            }
            let depth = (combined_radius - dist).max(0.0);
            let point = start_center_a + normal * (radius_a - depth * 0.5);
            let (restitution, friction) = combine_materials(&mat_a, &mat_b);

            events.push(CollisionEvent {
                time: start_time,
                body_a: Some(key.body_a),
                body_b: key.body_b,
                contacts: vec![ContactConstraint {
                    body_a: Some(key.body_a),
                    body_b: key.body_b,
                    point,
                    normal,
                    depth,
                    restitution,
                    friction,
                }],
            });

            if let Some(entry) = self.manifold_cache.get_mut(&key) {
                entry.last_seen_frame = self.frame_index;
            }
        }
    }

    fn store_manifold_from_event(&mut self, event: &CollisionEvent) {
        let Some(handle_a) = event.body_a else {
            return;
        };
        let Some(first_contact) = event.contacts.first() else {
            return;
        };
        let key = ManifoldKey {
            body_a: handle_a,
            body_b: event.body_b,
        };
        self.manifold_cache.insert(
            key,
            CachedManifold {
                normal: first_contact.normal,
                last_seen_frame: self.frame_index,
            },
        );
    }

    fn prune_manifold_cache(&mut self) {
        let frame_index = self.frame_index;
        self.manifold_cache.retain(|key, cached| {
            if frame_index.saturating_sub(cached.last_seen_frame) > MANIFOLD_TTL_FRAMES {
                return false;
            }
            self.bodies.get(key.body_a.0).is_some() && self.bodies.get(key.body_b.0).is_some()
        });
    }

    fn try_cached_static_contact(
        &self,
        key: StaticContactKey,
        radius: f32,
        center: Point3<f32>,
        body: &BodySnapshot,
        time: f32,
        material: &super::collider::ColliderMaterial,
        events: &mut Vec<CollisionEvent>,
    ) -> bool {
        let Some(entry) = self.static_contact_cache.get(&key) else {
            return false;
        };
        let entry_point = entry.point;
        let entry_normal = entry.normal;
        let vel_along_normal = self
            .static_contact_velocity_along_normal(body, entry_point, entry_normal)
            .abs();
        if vel_along_normal > STATIC_CONTACT_REST_THRESHOLD {
            return false;
        }
        let expected_center = entry_point + entry_normal * radius;
        if (center - expected_center).magnitude() > STATIC_CONTACT_SLOP {
            return false;
        }
        let depth = (radius - (center - entry_point).dot(&entry_normal)).max(0.0);
        if depth <= 0.0 {
            return false;
        }

        let contact = ContactConstraint {
            body_a: None,
            body_b: key.body,
            point: entry_point,
            normal: entry_normal,
            depth,
            restitution: material.restitution,
            friction: material.friction,
        };
        events.push(CollisionEvent {
            time,
            body_a: None,
            body_b: key.body,
            contacts: vec![contact],
        });
        true
    }

    fn store_static_contact_cache(
        &mut self,
        body_handle: RigidBodyHandle,
        collider_handle: ColliderHandle,
        contacts: &[ContactConstraint],
    ) {
        let Some(contact) = contacts
            .iter()
            .max_by(|a, b| a.depth.partial_cmp(&b.depth).unwrap_or(Ordering::Equal))
        else {
            return;
        };
        if contact.depth <= 0.0 {
            return;
        }
        self.static_contact_cache.insert(
            StaticContactKey {
                body: body_handle,
                collider: collider_handle,
            },
            StaticContactEntry {
                normal: contact.normal,
                point: contact.point,
                last_seen_frame: self.frame_index,
            },
        );
    }

    fn touch_static_contact_cache(&mut self, key: StaticContactKey) {
        if let Some(entry) = self.static_contact_cache.get_mut(&key) {
            entry.last_seen_frame = self.frame_index;
        }
    }

    fn prune_static_contact_cache(&mut self) {
        let frame_index = self.frame_index;
        self.static_contact_cache.retain(|key, cached| {
            if frame_index.saturating_sub(cached.last_seen_frame) > STATIC_CONTACT_TTL_FRAMES {
                return false;
            }
            let body_ok = self.bodies.get(key.body.0).is_some();
            let collider_ok = self.colliders.get(key.collider.0).is_some();
            body_ok && collider_ok
        });
    }

    fn apply_warm_start_impulses(&mut self, contacts: &[ContactConstraint]) {
        for contact in contacts {
            if contact.depth <= 0.0 {
                continue;
            }
            if self.relative_velocity_along_normal(contact) > WARM_START_REST_THRESHOLD {
                continue;
            }
            let key = self.warm_start_key(contact);
            let Some(entry) = self.warm_start_cache.get_mut(&key) else {
                continue;
            };
            entry.last_seen_frame = self.frame_index;
            let impulse = contact.normal * (entry.normal_impulse * WARM_START_DECAY);
            self.apply_contact_impulse(contact, impulse);
        }
    }

    fn store_warm_start_impulses(&mut self, contacts: &[ContactConstraint], impulses: &[f32]) {
        for (contact, impulse) in contacts.iter().zip(impulses.iter()) {
            if contact.depth <= 0.0 {
                continue;
            }
            if self.relative_velocity_along_normal(contact) > WARM_START_REST_THRESHOLD {
                continue;
            }
            let key = self.warm_start_key(contact);
            let clamped = impulse.max(0.0);
            self.warm_start_cache.insert(
                key,
                WarmStartEntry {
                    normal_impulse: clamped,
                    last_seen_frame: self.frame_index,
                },
            );
        }
    }

    fn warm_start_key(&self, contact: &ContactConstraint) -> WarmStartKey {
        WarmStartKey {
            body_a: contact.body_a,
            body_b: contact.body_b,
            normal_key: quantize_normal(contact.normal),
        }
    }

    fn apply_contact_impulse(&mut self, contact: &ContactConstraint, impulse: Vector3<f32>) {
        if let Some(handle_a) = contact.body_a {
            if let Some(body_a) = self.bodies.get_mut(handle_a.0) {
                if body_a.is_dynamic() {
                    body_a.apply_impulse_at_point(-impulse, contact.point);
                }
            }
        }

        if let Some(body_b) = self.bodies.get_mut(contact.body_b.0) {
            if body_b.is_dynamic() {
                body_b.apply_impulse_at_point(impulse, contact.point);
            }
        }
    }

    fn relative_velocity_along_normal(&self, contact: &ContactConstraint) -> f32 {
        let (pos_b, vel_b, angular_vel_b) = {
            let Some(body_b) = self.bodies.get(contact.body_b.0) else {
                return 0.0;
            };
            (
                body_b.position(),
                body_b.linear_velocity(),
                body_b.angular_velocity(),
            )
        };

        let (pos_a, vel_a, angular_vel_a) = match contact.body_a {
            Some(handle) => {
                let Some(body_a) = self.bodies.get(handle.0) else {
                    return 0.0;
                };
                (
                    body_a.position(),
                    body_a.linear_velocity(),
                    body_a.angular_velocity(),
                )
            }
            None => (contact.point, Vector3::zeros(), Vector3::zeros()),
        };

        let r_a = contact.point - pos_a;
        let r_b = contact.point - pos_b;
        let vel_at_contact_a = vel_a + angular_vel_a.cross(&r_a);
        let vel_at_contact_b = vel_b + angular_vel_b.cross(&r_b);
        let rel_vel = vel_at_contact_b - vel_at_contact_a;
        rel_vel.dot(&contact.normal)
    }

    fn static_contact_velocity_along_normal(
        &self,
        body: &BodySnapshot,
        point: Point3<f32>,
        normal: Vector3<f32>,
    ) -> f32 {
        let r = point - body.position;
        let vel_at_contact = body.linear_velocity + body.angular_velocity.cross(&r);
        vel_at_contact.dot(&normal)
    }

    fn prune_warm_start_cache(&mut self) {
        let frame_index = self.frame_index;
        self.warm_start_cache.retain(|key, cached| {
            if frame_index.saturating_sub(cached.last_seen_frame) > WARM_START_TTL_FRAMES {
                return false;
            }
            let body_a_ok = match key.body_a {
                Some(handle_a) => self.bodies.get(handle_a.0).is_some(),
                None => true,
            };
            body_a_ok && self.bodies.get(key.body_b.0).is_some()
        });
    }
}

fn compute_sphere_contact(
    center_a: Point3<f32>,
    radius_a: f32,
    center_b: Point3<f32>,
    radius_b: f32,
) -> (Point3<f32>, Vector3<f32>, f32) {
    let delta = center_b - center_a;
    let dist_sq = delta.magnitude_squared();
    let dist = dist_sq.sqrt();
    let normal = if dist < 1e-6 {
        Vector3::y()
    } else {
        delta / dist
    };
    let combined_radius = radius_a + radius_b;
    let depth = (combined_radius - dist).max(0.0);
    let point = center_a + normal * (radius_a - depth * 0.5);
    (point, normal, depth)
}

fn quantize_normal(normal: Vector3<f32>) -> [i16; 3] {
    let len_sq = normal.magnitude_squared();
    let unit = if len_sq > 1e-10 {
        normal / len_sq.sqrt()
    } else {
        Vector3::y()
    };
    let scale = 1000.0;
    [
        (unit.x * scale).round() as i16,
        (unit.y * scale).round() as i16,
        (unit.z * scale).round() as i16,
    ]
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new(PhysicsConfig::default())
    }
}
