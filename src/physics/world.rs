//! Physics world containing all simulation state.

use generational_arena::Arena;
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use std::collections::{HashMap, HashSet};

use super::body::{RigidBody, RigidBodyDesc};
use super::collider::{Collider, ColliderDesc, ColliderMaterial, ColliderShape};
use super::handle::{ColliderHandle, RigidBodyHandle};
use super::narrowphase::{generate_sphere_sphere_contacts, generate_sphere_static_contacts};
use super::pipeline::integration::{integrate_bodies, integrate_forces};
use super::pipeline::manifold::ManifoldCache;
use super::pipeline::solver::{solve, solve_contacts, ContactConstraint};
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
    /// Frames without a narrowphase refresh before a manifold point is pruned.
    pub manifold_max_age: u8,
    /// Scale factor applied to warm-start impulses (0..=1).
    pub warm_start_scale: f32,
    /// Baumgarte position correction factor.
    pub baumgarte_factor: f32,
    /// Baumgarte slop for penetration correction.
    pub baumgarte_slop: f32,
    /// Normal alignment threshold for warm-start reuse.
    pub normal_alignment_threshold: f32,
    /// Allow warm-start when raw depth exceeds this (can be negative).
    pub warm_start_depth_slop: f32,
    /// Allow restitution when raw depth exceeds this (can be negative).
    pub restitution_depth_slop: f32,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            gravity: Vector3::new(0.0, -20.0, 0.0),
            solver_iterations: 4,
            restitution_velocity_threshold: 1.0,
            contact_margin: 0.02,
            ccd_threshold: 0.5,
            contact_match_threshold: 0.05,
            manifold_max_age: 3,
            warm_start_scale: 0.6,
            baumgarte_factor: 0.05,
            baumgarte_slop: 0.005,
            normal_alignment_threshold: 0.95,
            warm_start_depth_slop: 0.01,
            restitution_depth_slop: 0.005,
        }
    }
}

/// Data collected for a body that needs CCD sweeping.
struct CcdCandidate {
    body_handle: RigidBodyHandle,
    radius: f32,
    material: ColliderMaterial,
    pre_body_pos: Point3<f32>,
    post_body_pos: Point3<f32>,
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
    frame_index: u64,
}

impl PhysicsWorld {
    pub fn new(config: PhysicsConfig) -> Self {
        let manifold_cache = ManifoldCache::new(
            config.contact_match_threshold,
            config.manifold_max_age,
            config.normal_alignment_threshold,
            config.warm_start_depth_slop,
        );
        Self {
            config,
            bodies: Arena::new(),
            colliders: Arena::new(),
            manifold_cache,
            last_contacts: Vec::new(),
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

        for collider_handle in body.colliders() {
            self.manifold_cache.remove_collider(*collider_handle);
            self.colliders.remove(collider_handle.0);
        }

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
        _debug_lines: &mut DebugLines,
    ) {
        self.frame_index = self.frame_index.wrapping_add(1);

        // Phase 1-2: Integrate forces into velocities
        integrate_forces(&mut self.bodies, dt, self.config.gravity);

        // Phase 3: Narrowphase contact generation
        let mut raw_contacts = generate_sphere_static_contacts(
            &self.bodies,
            &self.colliders,
            static_geometry,
            self.config.contact_margin,
        );
        raw_contacts.extend(generate_sphere_sphere_contacts(
            &self.bodies,
            &self.colliders,
            self.config.contact_margin,
        ));

        // Phase 4: Merge with manifold cache (populates warm-start impulses)
        let contacts = self.manifold_cache.update(&raw_contacts, &self.bodies);

        self.last_contacts.clear();
        self.last_contacts
            .extend(contacts.iter().map(|c| ContactEvent::from_narrowphase(c)));

        // Phase 5: Solve velocity constraints (warm-start + N iterations)
        let solved = solve(&mut self.bodies, &contacts, &self.config);

        // Phase 6: Write solved impulses back to manifold cache
        self.manifold_cache.write_back(&contacts, &solved);
        self.manifold_cache.prune();

        // Bodies with static narrowphase contacts are managed by the solver.
        // CCD should only catch bodies in free flight that might tunnel.
        let narrowphase_handled: HashSet<RigidBodyHandle> = contacts
            .iter()
            .filter(|c| c.body_a.is_none())
            .map(|c| c.body_b)
            .collect();

        // Save pre-integration state for CCD
        let pre_states: HashMap<generational_arena::Index, (Point3<f32>, UnitQuaternion<f32>)> =
            self.bodies
                .iter()
                .filter(|(_, body)| !body.is_static())
                .map(|(idx, body)| (idx, (body.position(), body.rotation())))
                .collect();

        // Phase 7: Integrate positions
        integrate_bodies(&mut self.bodies, dt);

        // Phase 8: CCD pass (fast bodies only, excluding narrowphase-managed bodies)
        let _ccd_count = self.ccd_pass(dt, static_geometry, &pre_states, &narrowphase_handled);
    }

    /// Contacts generated in the most recent step.
    pub fn contact_events(&self) -> &[ContactEvent] {
        &self.last_contacts
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
    ) -> u32 {
        let mut corrections = 0u32;

        // Collect CCD candidate data (all owned/copied) to avoid borrow conflicts
        let candidates: Vec<CcdCandidate> = self
            .bodies
            .iter()
            .filter(|(_, body)| !body.is_static())
            .filter_map(|(idx, body)| {
                let handle = RigidBodyHandle(idx);
                // The narrowphase already manages bodies with static contacts
                if narrowphase_handled.contains(&handle) {
                    return None;
                }
                let collider_handle = *body.colliders().first()?;
                let collider = self.colliders.get(collider_handle.0)?;
                let radius = match collider.shape() {
                    ColliderShape::Sphere { radius } => *radius,
                };
                let speed = body.linear_velocity().magnitude();
                if speed * dt <= radius * self.config.ccd_threshold {
                    return None;
                }
                let &(pre_pos, pre_rot) = pre_states.get(&idx)?;
                let post_pos = body.position();
                let post_rot = body.rotation();
                Some(CcdCandidate {
                    body_handle: handle,
                    radius,
                    material: *collider.material(),
                    pre_body_pos: pre_pos,
                    post_body_pos: post_pos,
                    pre_center: collider.world_center(pre_pos, pre_rot),
                    post_center: collider.world_center(post_pos, post_rot),
                })
            })
            .collect();

        // Sweep each candidate against static geometry
        for candidate in &candidates {
            let Some(hit) = static_geometry.sweep_sphere(
                candidate.pre_center,
                candidate.post_center,
                candidate.radius,
            ) else {
                continue;
            };

            // Move body to impact position
            let hit_pos =
                candidate.pre_body_pos + (candidate.post_body_pos - candidate.pre_body_pos) * hit.t;
            if let Some(body) = self.bodies.get_mut(candidate.body_handle.0) {
                body.set_position(hit_pos);
            }

            // Solve CCD contact to correct velocity (transient, not cached)
            let contact = ContactConstraint {
                body_a: None,
                body_b: candidate.body_handle,
                collider_a: None,
                collider_b: None,
                point: hit.point,
                normal: hit.normal,
                depth: 0.0,
                raw_depth: 0.0,
                restitution: candidate.material.restitution,
                friction: candidate.material.friction,
                warm_normal_impulse: 0.0,
                warm_tangent_impulse: [0.0, 0.0],
            };
            self.last_contacts.push(ContactEvent::from_ccd(&contact));
            solve_contacts(
                &mut self.bodies,
                &[contact],
                self.config.restitution_velocity_threshold,
                self.config.restitution_depth_slop,
            );

            corrections += 1;
        }

        corrections
    }
}

/// Contact event produced by collision detection.
#[derive(Debug, Clone)]
pub struct ContactEvent {
    pub body_a: Option<RigidBodyHandle>,
    pub body_b: RigidBodyHandle,
    pub point: Point3<f32>,
    pub normal: Vector3<f32>,
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
