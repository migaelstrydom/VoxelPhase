//! Physics world containing all simulation state.

use generational_arena::Arena;
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use std::collections::{HashMap, HashSet};

use super::body::{RigidBody, RigidBodyDesc};
use super::collider::{Collider, ColliderDesc, ColliderMaterial, ColliderShape};
use super::collision::obb::Obb;
use super::collision::obb_triangle::obb_triangle_contacts;
use super::debug::{PhysicsDebugConfig, PhysicsDebugger};
use super::grounding::{GroundingConfig, GroundingDetector};
use super::handle::{ColliderHandle, RigidBodyHandle};
use super::narrowphase::{
    filter_internal_edge_contacts, filter_internal_vertex_contacts, generate_dynamic_contacts,
    generate_static_contacts, ContactSource as MeshContactSource, NormalClusterConfig,
    SourcedContact,
};
use super::pipeline::integration::{integrate_bodies, integrate_forces};
use super::pipeline::manifold::ManifoldCache;
use super::pipeline::normal_smoothing::NormalSmoothingConfig;
use super::pipeline::post_stabilizer::PostStabiliseConfig;
use super::pipeline::solver::{solve, solve_contacts, ContactConstraint};
use super::sleep::{SleepManager, SleepManagerConfig};
use super::static_geometry::StaticGeometry;
use crate::debug::DebugLines;

/// Configuration for the physics simulation.
#[derive(Debug, Clone)]
pub struct PhysicsConfig {
    /// Gravity acceleration vector.
    pub gravity: Vector3<f32>,
    /// Number of solver iterations per step.
    pub solver_iterations: u32,
    /// Minimum approach speed for restitution to apply. Below this threshold,
    /// restitution is zeroed to prevent micro-bouncing at resting contacts.
    pub restitution_velocity_threshold: f32,
    /// Margin added to collision queries so the narrowphase detects contacts
    /// slightly before geometric overlap. Contacts within the margin skin
    /// receive velocity-only correction (depth=0); actual penetrations get
    /// position correction.
    pub contact_margin: f32,
    /// CCD activation threshold. A body requires CCD when:
    /// `|linear_velocity| * dt > radius * ccd_threshold`.
    /// Below this, the narrowphase handles contacts; above, CCD sweeps
    /// prevent tunneling.
    pub ccd_threshold: f32,
    /// Local-space distance threshold for matching contact points across frames.
    pub contact_match_threshold: f32,
    /// Multiplier applied to contact_match_threshold for static contacts.
    pub contact_match_threshold_static_multiplier: f32,
    /// Frames without a narrowphase refresh before a manifold point is pruned.
    pub manifold_max_age: u8,
    /// Scale factor applied to warm-start impulses (0..=1).
    pub warm_start_scale: f32,
    /// Split-impulse configuration for post-stabilization.
    pub post_stabilise: PostStabiliseConfig,
    /// Configuration for smoothing matched contact normals.
    pub normal_smoothing: NormalSmoothingConfig,
    /// Configuration for clustering static contact normals.
    pub normal_clustering: NormalClusterConfig,
    /// Configuration for grounded detection.
    pub grounding: GroundingConfig,
    /// Allow warm-start when raw depth exceeds this (can be negative).
    pub warm_start_depth_slop: f32,
    /// Allow restitution when raw depth exceeds this (can be negative).
    pub restitution_depth_slop: f32,
    /// Enable speculative contacts to close the CCD activation gap.
    pub enable_speculative_contacts: bool,
    /// Minimum linear speed required for speculative contact generation.
    pub speculative_min_speed: f32,
    /// Multiplier for contact_margin when gating speculative contacts.
    pub speculative_margin_multiplier: f32,
    /// Configuration for the sleep system.
    pub sleep: SleepManagerConfig,
    /// Debug rendering configuration.
    pub debug: PhysicsDebugConfig,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            gravity: Vector3::new(0.0, -20.0, 0.0),
            solver_iterations: 4,
            restitution_velocity_threshold: 1.0,
            contact_margin: 0.02,
            ccd_threshold: 0.5,
            contact_match_threshold: 0.1,
            contact_match_threshold_static_multiplier: 2.0,
            manifold_max_age: 3,
            warm_start_scale: 0.6,
            post_stabilise: PostStabiliseConfig::default(),
            normal_smoothing: NormalSmoothingConfig::default(),
            normal_clustering: NormalClusterConfig::default(),
            grounding: GroundingConfig::default(),
            warm_start_depth_slop: 0.02,
            restitution_depth_slop: 0.005,
            enable_speculative_contacts: true,
            speculative_min_speed: 1.0,
            speculative_margin_multiplier: 2.0,
            sleep: SleepManagerConfig::default(),
            debug: PhysicsDebugConfig::default(),
        }
    }
}

/// Data collected for a body that needs CCD sweeping.
struct CcdCandidate {
    body_handle: RigidBodyHandle,
    /// Bounding sphere radius (sphere radius for spheres, half_extents.norm() for boxes).
    radius: f32,
    shape: ColliderShape,
    material: ColliderMaterial,
    pre_body_pos: Point3<f32>,
    post_body_pos: Point3<f32>,
    pre_rot: UnitQuaternion<f32>,
    pre_center: Point3<f32>,
    post_center: Point3<f32>,
}

/// The physics simulation world.
///
/// Owns all rigid bodies, colliders, and joints. Provides a handle-based API
/// for creating and manipulating physics objects.
pub struct PhysicsWorld {
    config: PhysicsConfig,
    bodies: Arena<RigidBody>,
    colliders: Arena<Collider>,
    manifold_cache: ManifoldCache,
    last_contacts: Vec<ContactEvent>,
    debugger: PhysicsDebugger,
    frame_index: u64,
    sleep_manager: SleepManager,
    grounding_detector: GroundingDetector,
}

impl PhysicsWorld {
    pub fn new(config: PhysicsConfig) -> Self {
        let manifold_cache = ManifoldCache::new(
            config.contact_match_threshold,
            config.contact_match_threshold_static_multiplier,
            config.manifold_max_age,
            config.warm_start_depth_slop,
            config.normal_smoothing,
        );
        let sleep_manager = SleepManager::new(config.sleep);
        let grounding_detector = GroundingDetector::new(config.grounding);
        let debugger = PhysicsDebugger::new(config.debug.clone());
        Self {
            config,
            bodies: Arena::new(),
            colliders: Arena::new(),
            manifold_cache,
            last_contacts: Vec::new(),
            debugger,
            frame_index: 0,
            sleep_manager,
            grounding_detector,
        }
    }

    /// Get the physics configuration.
    pub fn config(&self) -> &PhysicsConfig {
        &self.config
    }

    /// Get the physics debugger.
    pub fn debugger(&self) -> &PhysicsDebugger {
        &self.debugger
    }

    pub fn sleeping_bodies(&self) -> Vec<RigidBodyHandle> {
        if !self.config.sleep.enabled {
            return Vec::new();
        }
        self.sleep_manager.sleeping_snapshot().into_iter().collect()
    }

    pub fn apply_radial_impulse(
        &mut self,
        center: Point3<f32>,
        radius: f32,
        strength: f32,
        upward_boost: f32,
    ) {
        if radius <= 0.0 || strength.abs() < 1e-6 {
            return;
        }
        for (idx, body) in self.bodies.iter_mut() {
            if !body.is_dynamic() {
                continue;
            }
            let delta = body.position() - center;
            let distance = delta.magnitude();
            if distance >= radius {
                continue;
            }
            let (direction, falloff) = if distance < 1e-4 {
                (Vector3::y(), 1.0)
            } else {
                (delta / distance, 1.0 - (distance / radius))
            };
            let impulse = direction * strength * falloff
                + Vector3::new(0.0, strength * falloff * upward_boost, 0.0);
            body.apply_impulse(impulse);
            self.sleep_manager.wake_body(RigidBodyHandle(idx));
        }
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

        for collider_handle in body.colliders() {
            self.manifold_cache.remove_collider(*collider_handle);
            self.colliders.remove(collider_handle.0);
        }

        self.sleep_manager.sync_bodies(&self.bodies);
        true
    }

    /// Get a reference to a rigid body.
    pub fn body(&self, handle: RigidBodyHandle) -> Option<&RigidBody> {
        self.bodies.get(handle.0)
    }

    /// Update a kinematic body's transform (position + rotation).
    pub fn set_kinematic_transform(
        &mut self,
        handle: RigidBodyHandle,
        position: Point3<f32>,
        rotation: UnitQuaternion<f32>,
    ) -> bool {
        let Some(body) = self.bodies.get_mut(handle.0) else {
            return false;
        };
        if !body.is_kinematic() {
            return false;
        }
        body.set_position(position);
        body.set_rotation(rotation);
        self.sleep_manager.note_kinematic_move(handle);
        true
    }

    /// Update a kinematic body's velocities.
    pub fn set_kinematic_velocity(
        &mut self,
        handle: RigidBodyHandle,
        linear: Vector3<f32>,
        angular: Vector3<f32>,
    ) -> bool {
        let Some(body) = self.bodies.get_mut(handle.0) else {
            return false;
        };
        if !body.is_kinematic() {
            return false;
        }
        body.set_linear_velocity(linear);
        body.set_angular_velocity(angular);
        self.sleep_manager.note_kinematic_move(handle);
        true
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
    #[allow(dead_code)]
    pub fn remove_collider(&mut self, handle: ColliderHandle) -> bool {
        let Some(collider) = self.colliders.remove(handle.0) else {
            return false;
        };

        let body_handle = collider.body();

        if let Some(body) = self.bodies.get_mut(body_handle.0) {
            body.remove_collider(handle);
            self.recompute_mass_properties(body_handle);
        }

        self.manifold_cache.remove_collider(handle);

        true
    }

    /// Get a reference to a collider.
    #[allow(dead_code)]
    pub fn collider(&self, handle: ColliderHandle) -> Option<&Collider> {
        self.colliders.get(handle.0)
    }

    // === Simulation ===

    /// Step the physics simulation forward by dt seconds.
    ///
    /// Pipeline order (semi-implicit Euler):
    /// 1. Integrate forces into velocities
    /// 2. Narrowphase: generate contacts at current positions
    /// 3. Manifold cache: merge with persistent contacts, populate warm-start data
    /// 4. Solve velocity constraints (warm-start + sequential impulses)
    /// 5. Write solved impulses back to manifold cache
    /// 6. Integrate positions (velocities → positions)
    /// 7. CCD pass (fast bodies only: sweep, correct position, re-solve)
    pub fn step(
        &mut self,
        dt: f32,
        static_geometry: &dyn StaticGeometry,
        debug_lines: &mut DebugLines,
    ) {
        self.frame_index = self.frame_index.wrapping_add(1);
        self.sleep_manager.sync_bodies(&self.bodies);
        self.sleep_manager.apply_wake_events(&[], &self.bodies);
        let sleeping_snapshot = if self.config.sleep.enabled {
            Some(self.sleep_manager.sleeping_snapshot())
        } else {
            None
        };

        // Phase 1-2: Integrate forces into velocities
        integrate_forces(
            &mut self.bodies,
            dt,
            self.config.gravity,
            sleeping_snapshot.as_ref(),
        );

        // Phase 3: Narrowphase contact generation
        let mut raw_contacts = generate_static_contacts(
            &self.bodies,
            &self.colliders,
            static_geometry,
            self.config.contact_margin,
            dt,
            self.config.ccd_threshold,
            self.config.enable_speculative_contacts,
            self.config.speculative_min_speed,
            self.config.speculative_margin_multiplier,
            self.config.normal_clustering,
            sleeping_snapshot.as_ref(),
        );
        raw_contacts.extend(generate_dynamic_contacts(
            &self.bodies,
            &self.colliders,
            self.config.contact_margin,
            dt,
            self.config.ccd_threshold,
            self.config.enable_speculative_contacts,
            self.config.speculative_min_speed,
            self.config.speculative_margin_multiplier,
            sleeping_snapshot.as_ref(),
        ));

        // Phase 4: Merge with manifold cache (populates warm-start impulses)
        let contacts = self.manifold_cache.update(&raw_contacts, &self.bodies);

        self.last_contacts.clear();
        self.last_contacts
            .extend(contacts.iter().map(|c| ContactEvent::from_narrowphase(c)));

        self.sleep_manager
            .note_contact_wakes(&contacts, &self.bodies);
        self.sleep_manager
            .apply_wake_events(&contacts, &self.bodies);
        let active_contacts = self.sleep_manager.filter_active_contacts(&contacts);
        self.debugger.update(
            &self.bodies,
            &raw_contacts,
            &contacts,
            &active_contacts,
            &self.last_contacts,
        );

        // Phase 5: Solve velocity constraints (warm-start + N iterations)
        let solved = solve(&mut self.bodies, &active_contacts, &self.config, dt);
        self.debugger
            .update_post_solve(&self.bodies, &active_contacts);

        // Phase 6: Write solved impulses back to manifold cache
        self.manifold_cache
            .write_back(&active_contacts, &solved, &self.bodies);
        self.manifold_cache.prune();

        // Bodies with static narrowphase contacts are managed by the solver.
        // CCD should only catch bodies in free flight that might tunnel.
        let narrowphase_handled: HashSet<RigidBodyHandle> = active_contacts
            .iter()
            .filter(|c| c.body_a.is_none())
            .map(|c| c.body_b)
            .collect();

        // Save pre-integration state for CCD
        let sleeping_snapshot = if self.config.sleep.enabled {
            Some(self.sleep_manager.sleeping_snapshot())
        } else {
            None
        };
        let pre_states: HashMap<generational_arena::Index, (Point3<f32>, UnitQuaternion<f32>)> =
            self.bodies
                .iter()
                .filter(|(idx, body)| {
                    if body.is_static() {
                        return false;
                    }
                    if let Some(sleeping) = sleeping_snapshot.as_ref() {
                        return !sleeping.contains(&RigidBodyHandle(*idx));
                    }
                    true
                })
                .map(|(idx, body)| (idx, (body.position(), body.rotation())))
                .collect();

        // Phase 7: Integrate positions
        let sleeping_snapshot = if self.config.sleep.enabled {
            Some(self.sleep_manager.sleeping_snapshot())
        } else {
            None
        };
        integrate_bodies(&mut self.bodies, dt, sleeping_snapshot.as_ref());

        // Phase 8: CCD pass (fast bodies only, excluding narrowphase-managed bodies)
        let _ccd_count = self.ccd_pass(
            dt,
            static_geometry,
            &pre_states,
            &narrowphase_handled,
            sleeping_snapshot.as_ref(),
        );

        let _ = debug_lines;

        self.sleep_manager
            .update_sleep_states(&mut self.bodies, &contacts);
    }

    /// Contacts generated in the most recent step.
    pub fn contact_events(&self) -> &[ContactEvent] {
        &self.last_contacts
    }

    /// Get access to the rigid bodies arena.
    pub fn bodies(&self) -> &Arena<RigidBody> {
        &self.bodies
    }

    /// Bodies grounded by static contacts in the most recent step.
    pub fn grounded_handles(&self) -> HashSet<RigidBodyHandle> {
        self.grounding_detector
            .grounded_bodies(self.contact_events())
            .into_iter()
            .filter_map(|(handle, grounded)| grounded.then_some(handle))
            .collect()
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

    /// CCD pass: sweep fast-moving bodies against static geometry to prevent tunneling.
    ///
    /// A body requires CCD when `|linear_velocity| * dt > radius * ccd_threshold`
    /// AND the narrowphase did not already generate static contacts for it.
    /// Bodies with narrowphase contacts are managed by the solver — CCD only
    /// catches bodies in free flight that might skip past geometry entirely.
    fn ccd_pass(
        &mut self,
        dt: f32,
        static_geometry: &dyn StaticGeometry,
        pre_states: &HashMap<generational_arena::Index, (Point3<f32>, UnitQuaternion<f32>)>,
        narrowphase_handled: &HashSet<RigidBodyHandle>,
        sleeping: Option<&HashSet<RigidBodyHandle>>,
    ) -> u32 {
        let mut corrections = 0u32;

        // Collect CCD candidate data (all owned/copied) to avoid borrow conflicts
        let candidates: Vec<CcdCandidate> = self
            .bodies
            .iter()
            .filter(|(idx, body)| {
                if body.is_static() {
                    return false;
                }
                if let Some(sleeping) = sleeping {
                    return !sleeping.contains(&RigidBodyHandle(*idx));
                }
                true
            })
            .filter_map(|(idx, body)| {
                let handle = RigidBodyHandle(idx);
                if narrowphase_handled.contains(&handle) {
                    return None;
                }
                let collider_handle = *body.colliders().first()?;
                let collider = self.colliders.get(collider_handle.0)?;
                let radius = collider.shape().bounding_radius();
                let speed = body.linear_velocity().magnitude();
                if speed * dt <= radius * self.config.ccd_threshold {
                    return None;
                }
                let &(pre_pos, pre_rot) = pre_states.get(&idx)?;
                let post_pos = body.position();
                Some(CcdCandidate {
                    body_handle: handle,
                    radius,
                    shape: collider.shape().clone(),
                    material: *collider.material(),
                    pre_body_pos: pre_pos,
                    post_body_pos: post_pos,
                    pre_rot,
                    pre_center: collider.world_center(pre_pos, pre_rot),
                    post_center: collider.world_center(post_pos, body.rotation()),
                })
            })
            .collect();

        // Sweep each candidate against static geometry
        for candidate in &candidates {
            let Some(hit) = sweep_sphere_against_static(
                candidate.pre_center,
                candidate.post_center,
                candidate.radius,
                static_geometry,
            ) else {
                continue;
            };

            // Move body to impact position
            let mut hit_pos =
                candidate.pre_body_pos + (candidate.post_body_pos - candidate.pre_body_pos) * hit.t;
            let mut hit_rot = candidate.pre_rot;
            if let Some(body) = self.bodies.get_mut(candidate.body_handle.0) {
                let post_rot = body.rotation();
                hit_rot = candidate.pre_rot.slerp(&post_rot, hit.t);
                if let ColliderShape::Box { half_extents } = &candidate.shape {
                    let obb = Obb::new(hit_pos, hit_rot, *half_extents);
                    let support = obb.project_half_extent(&hit.normal);
                    let extra = (candidate.radius - support).max(0.0);
                    hit_pos -= hit.normal * extra;
                }
                body.set_position(hit_pos);
                body.set_rotation(hit_rot);
            }

            let ccd_contacts = match &candidate.shape {
                ColliderShape::Sphere { .. } => {
                    vec![ContactConstraint {
                        body_a: None,
                        body_b: candidate.body_handle,
                        collider_a: None,
                        collider_b: None,
                        point: hit.point,
                        normal: hit.normal,
                        raw_normal: hit.normal,
                        depth: 0.0,
                        raw_depth: 0.0,
                        restitution: candidate.material.restitution,
                        friction: candidate.material.friction,
                        warm_normal_impulse: 0.0,
                        warm_tangent_impulse: [0.0, 0.0],
                    }]
                }
                ColliderShape::Box { half_extents } => self.box_ccd_contacts(
                    candidate,
                    *half_extents,
                    hit_pos,
                    hit_rot,
                    static_geometry,
                ),
            };

            for contact in &ccd_contacts {
                self.last_contacts.push(ContactEvent::from_ccd(contact));
            }
            solve_contacts(
                &mut self.bodies,
                &ccd_contacts,
                self.config.restitution_velocity_threshold,
                self.config.restitution_depth_slop,
            );

            corrections += 1;
        }

        corrections
    }

    /// Generate precise box-terrain contacts at a CCD hit position.
    ///
    /// Bounding-sphere sweep found the approximate hit. Now build an OBB at
    /// the hit position (using the pre-integration rotation) and run SAT
    /// against nearby triangles for accurate contact normals.
    fn box_ccd_contacts(
        &self,
        candidate: &CcdCandidate,
        half_extents: Vector3<f32>,
        hit_pos: Point3<f32>,
        rotation: UnitQuaternion<f32>,
        static_geometry: &dyn StaticGeometry,
    ) -> Vec<ContactConstraint> {
        let obb = Obb::new(hit_pos, rotation, half_extents);
        let (aabb_min, aabb_max) = obb.enclosing_aabb();
        let margin = Vector3::new(
            self.config.contact_margin,
            self.config.contact_margin,
            self.config.contact_margin,
        );
        let query = crate::collision::AABB::new(aabb_min - margin, aabb_max + margin);
        let patch = static_geometry.query_region(&query);

        let mut sourced = Vec::new();
        for (tri_idx, pt) in patch.triangles.iter().enumerate() {
            for c in obb_triangle_contacts(&obb, &pt.triangle) {
                sourced.push(SourcedContact {
                    constraint: ContactConstraint {
                        body_a: None,
                        body_b: candidate.body_handle,
                        collider_a: None,
                        collider_b: None,
                        point: c.point,
                        normal: c.normal,
                        raw_normal: c.normal,
                        depth: 0.0,
                        raw_depth: 0.0,
                        restitution: candidate.material.restitution,
                        friction: candidate.material.friction,
                        warm_normal_impulse: 0.0,
                        warm_tangent_impulse: [0.0, 0.0],
                    },
                    source: MeshContactSource {
                        triangle_idx: tri_idx as u32,
                        feature: c.feature,
                    },
                });
            }
        }
        let threshold = self.config.normal_clustering.normal_cluster_dot.max(0.95);
        let sourced = filter_internal_edge_contacts(sourced, &patch, threshold);
        let sourced = filter_internal_vertex_contacts(sourced, &patch, threshold);
        let mut contacts: Vec<_> = sourced.into_iter().map(|c| c.constraint).collect();

        if contacts.is_empty() {
            // Fallback: use the sweep hit directly
            if let Some(hit) = sweep_sphere_against_static(
                candidate.pre_center,
                candidate.post_center,
                candidate.radius,
                static_geometry,
            ) {
                contacts.push(ContactConstraint {
                    body_a: None,
                    body_b: candidate.body_handle,
                    collider_a: None,
                    collider_b: None,
                    point: hit.point,
                    normal: hit.normal,
                    raw_normal: hit.normal,
                    depth: 0.0,
                    raw_depth: 0.0,
                    restitution: candidate.material.restitution,
                    friction: candidate.material.friction,
                    warm_normal_impulse: 0.0,
                    warm_tangent_impulse: [0.0, 0.0],
                });
            }
        }

        contacts
    }
}

/// Sweep a sphere from `start` to `end` against static geometry.
///
/// Builds the enclosing AABB, queries the region, and returns the earliest
/// swept contact along the path.
fn sweep_sphere_against_static(
    start: Point3<f32>,
    end: Point3<f32>,
    radius: f32,
    static_geometry: &dyn StaticGeometry,
) -> Option<crate::collision::SweptContact> {
    let query = crate::collision::AABB::new(
        Point3::new(
            start.x.min(end.x) - radius,
            start.y.min(end.y) - radius,
            start.z.min(end.z) - radius,
        ),
        Point3::new(
            start.x.max(end.x) + radius,
            start.y.max(end.y) + radius,
            start.z.max(end.z) + radius,
        ),
    );
    let patch = static_geometry.query_region(&query);

    let mut earliest: Option<crate::collision::SweptContact> = None;
    for pt in &patch.triangles {
        if let Some(contact) =
            crate::collision::swept_sphere_triangle(start, end, radius, &pt.triangle)
        {
            if earliest.is_none() || contact.t < earliest.as_ref().unwrap().t {
                earliest = Some(contact);
            }
        }
    }
    earliest
}

/// Contact event produced by collision detection.
#[derive(Debug, Clone)]
pub struct ContactEvent {
    pub body_a: Option<RigidBodyHandle>,
    pub body_b: RigidBodyHandle,
    pub point: Point3<f32>,
    pub normal: Vector3<f32>,
    pub raw_normal: Vector3<f32>,
    pub depth: f32,
    pub source: ContactSource,
}

impl ContactEvent {
    fn from_narrowphase(contact: &ContactConstraint) -> Self {
        Self {
            body_a: contact.body_a,
            body_b: contact.body_b,
            point: contact.point,
            normal: contact.normal,
            raw_normal: contact.raw_normal,
            depth: contact.depth,
            source: ContactSource::Narrowphase,
        }
    }

    fn from_ccd(contact: &ContactConstraint) -> Self {
        Self {
            body_a: contact.body_a,
            body_b: contact.body_b,
            point: contact.point,
            normal: contact.normal,
            raw_normal: contact.normal,
            depth: contact.depth,
            source: ContactSource::Ccd,
        }
    }
}

/// Source of the contact event in the pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContactSource {
    Narrowphase,
    Ccd,
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new(PhysicsConfig::default())
    }
}
