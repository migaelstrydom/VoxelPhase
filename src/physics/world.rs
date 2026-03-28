//! Physics world containing all simulation state.

use generational_arena::Arena;
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use std::collections::{HashMap, HashSet};

use super::body::{RigidBody, RigidBodyDesc};
use super::ccd::{CcdContext, CcdStrategy, SweepClampCcd};
use super::collider::{Collider, ColliderDesc, ColliderShape};
use super::constraint::expand::{expand_constraints, write_back_constraints};
use super::constraint::projection::project_angular_velocities;
use super::constraint::types::{Constraint, ConstraintRow};
use super::constraint::ConstraintHandle;
use super::contact_event::{ContactEvent, ContactSource};
use super::debug::{PhysicsDebugConfig, PhysicsDebugger};
use super::force_provider::{ForceContext, ForceOutput, SubstepForceProvider};
use super::grounding::{GroundingConfig, GroundingDetector};
use super::handle::{ColliderHandle, RigidBodyHandle};
use super::impulses::PhysicsImpulse;
use super::narrowphase::{
    generate_dynamic_contacts, generate_static_contacts, NarrowphaseWorkBuffer, SatCacheMap,
};
use super::pipeline::integration::{integrate_bodies, integrate_forces};
use super::pipeline::manifold::ManifoldCache;
use super::pipeline::normal_smoothing::NormalSmoothingConfig;
use super::pipeline::pair::SolverManifold;
use super::sleep::{SleepManager, SleepManagerConfig};
use super::solver::{ConstraintSolver, ManifoldConditioner, ManifoldConditions, PgsNgsSolver};
use super::static_geometry::StaticGeometry;
use crate::collision::capsule::Capsule;
use crate::collision::discrete::sphere_capsule::sphere_capsule_manifold;
use crate::collision::obb::Obb;
use crate::debug::DebugLines;
use crate::sensing::{ProbeHit, ProbeTarget};
use crate::{collision::continuous::swept_sphere_sphere, physics::ShockPropagationConditioner};

/// Configuration for the physics simulation.
#[derive(Debug, Clone)]
pub struct PhysicsConfig {
    /// Gravity acceleration vector.
    pub gravity: Vector3<f32>,
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
    /// Frames without a narrowphase refresh before a manifold point is pruned.
    pub manifold_max_age: u8,
    /// When true, sort manifold output contacts for deterministic solver ordering.
    ///
    /// This is primarily intended for reproducible tests and diagnostics.
    pub deterministic_contact_ordering: bool,
    /// Configuration for smoothing matched contact normals.
    pub normal_smoothing: NormalSmoothingConfig,
    /// Configuration for grounded detection.
    pub grounding: GroundingConfig,
    /// Allow warm-start when raw depth exceeds this (can be negative).
    pub warm_start_depth_slop: f32,
    /// Enable speculative contacts to close the CCD activation gap.
    pub enable_speculative_contacts: bool,
    /// Minimum linear speed required for speculative contact generation.
    pub speculative_min_speed: f32,
    /// Multiplier for contact_margin when gating speculative contacts.
    pub speculative_margin_multiplier: f32,
    /// Configuration for the sleep system.
    pub sleep: SleepManagerConfig,
    /// Position correction factor for joint constraints (beta).
    /// Controls how aggressively constraint drift is corrected.
    /// Higher values correct faster but may oscillate.
    pub constraint_position_beta: f32,
    /// Debug rendering configuration.
    pub debug: PhysicsDebugConfig,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            gravity: Vector3::new(0.0, -9.81, 0.0),
            restitution_velocity_threshold: 0.3,
            contact_margin: 0.02,
            ccd_threshold: 0.5,
            manifold_max_age: 3,
            deterministic_contact_ordering: false,
            normal_smoothing: NormalSmoothingConfig::default(),
            grounding: GroundingConfig::default(),
            warm_start_depth_slop: 0.02,
            enable_speculative_contacts: true,
            speculative_min_speed: 1.0,
            speculative_margin_multiplier: 2.0,
            sleep: SleepManagerConfig::default(),
            constraint_position_beta: 0.2,
            debug: PhysicsDebugConfig::default(),
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
    manifold_cache: ManifoldCache,
    last_contacts: Vec<ContactEvent>,
    debugger: PhysicsDebugger,
    frame_index: u64,
    sleep_manager: SleepManager,
    grounding_detector: GroundingDetector,
    /// User-defined constraints (persistent across frames).
    constraints: Arena<Constraint>,
    /// Solver-ready constraint rows, expanded each frame from `constraints`.
    /// Pre-allocated work buffer — cleared and refilled in `update_contacts()`.
    cached_constraint_rows: Vec<ConstraintRow>,
    /// Constraint solver (velocity + position correction for contacts and joints).
    solver: Box<dyn ConstraintSolver + Send + Sync>,
    /// Manifold conditioner (reordering + per-manifold metadata like shock scales).
    conditioner: Box<dyn ManifoldConditioner + Send + Sync>,
    /// Per-manifold conditions produced by the conditioner, reused across substeps.
    manifold_conditions: ManifoldConditions,
    /// Continuous collision detection strategy.
    /// Wrapped in `Option` so it can be temporarily taken during `substep()`
    /// to avoid self-referential borrows (the strategy needs mutable access
    /// to bodies/contacts while being a field of the same struct).
    ccd: Option<Box<dyn CcdStrategy + Send + Sync>>,
    /// Active solver manifolds from the most recent `update_contacts()` call,
    /// reused across multiple `substep()` calls.
    cached_active_manifolds: Vec<SolverManifold>,
    /// All solver manifolds (including sleeping) for sleep state bookkeeping.
    cached_all_manifolds: Vec<SolverManifold>,
    /// Bodies with static narrowphase contacts, excluded from CCD.
    cached_narrowphase_handled: HashSet<RigidBodyHandle>,
    /// SAT axis cache for OBB-OBB dynamic pair early-out.
    sat_cache_map: SatCacheMap,
    /// Reusable work buffer for dynamic narrowphase (avoids per-frame allocation).
    narrowphase_work_buffer: NarrowphaseWorkBuffer,
}

impl PhysicsWorld {
    pub fn new(config: PhysicsConfig) -> Self {
        Self::with_components(
            config,
            Box::new(PgsNgsSolver::default()),
            Box::new(ShockPropagationConditioner::default()),
            Box::new(SweepClampCcd::default()),
        )
    }

    /// Create a physics world with a specific constraint solver.
    pub fn with_solver(
        config: PhysicsConfig,
        solver: Box<dyn ConstraintSolver + Send + Sync>,
    ) -> Self {
        Self::with_components(
            config,
            solver,
            Box::new(ShockPropagationConditioner::default()),
            Box::new(SweepClampCcd::default()),
        )
    }

    /// Create a physics world with specific solver, conditioner, and CCD strategy.
    pub fn with_components(
        config: PhysicsConfig,
        solver: Box<dyn ConstraintSolver + Send + Sync>,
        conditioner: Box<dyn ManifoldConditioner + Send + Sync>,
        ccd: Box<dyn CcdStrategy + Send + Sync>,
    ) -> Self {
        let manifold_cache = ManifoldCache::new(
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
            constraints: Arena::new(),
            cached_constraint_rows: Vec::new(),
            solver,
            conditioner,
            manifold_conditions: ManifoldConditions::new(),
            ccd: Some(ccd),
            cached_active_manifolds: Vec::new(),
            cached_all_manifolds: Vec::new(),
            cached_narrowphase_handled: HashSet::new(),
            sat_cache_map: SatCacheMap::new(),
            narrowphase_work_buffer: NarrowphaseWorkBuffer::new(),
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

    /// Check whether a body is currently sleeping.
    pub fn is_sleeping(&self, handle: RigidBodyHandle) -> bool {
        self.sleep_manager.is_sleeping(handle)
    }

    /// Wake a sleeping body due to an external environmental change
    /// (e.g. water surface moved under it). No-op if the body is already awake
    /// or doesn't exist.
    pub fn wake_body(&mut self, handle: RigidBodyHandle) {
        self.sleep_manager.wake_body(handle);
    }

    /// Apply one-shot impulses to all dynamic bodies.
    fn apply_impulses(&mut self, impulses: &[PhysicsImpulse]) {
        for (idx, body) in self.bodies.iter_mut() {
            if !body.is_dynamic() {
                continue;
            }
            let pos = body.position();
            let mut wake = false;

            for impulse in impulses {
                if let Some(v) = impulse.impulse_at(pos) {
                    body.apply_impulse(v);
                    wake = true;
                }
            }

            if wake {
                self.sleep_manager.wake_body(RigidBodyHandle(idx));
            }
        }
    }

    // === Body Management ===

    /// Create a new rigid body and return its handle.
    pub fn create_body(&mut self, desc: RigidBodyDesc) -> RigidBodyHandle {
        let body = RigidBody::new(desc);
        RigidBodyHandle(self.bodies.insert(body))
    }

    /// Remove a rigid body, all its attached colliders, and any constraints
    /// referencing it.
    pub fn remove_body(&mut self, handle: RigidBodyHandle) -> bool {
        let Some(body) = self.bodies.remove(handle.0) else {
            return false;
        };

        for collider_handle in body.colliders() {
            self.manifold_cache.remove_collider(*collider_handle);
            self.colliders.remove(collider_handle.0);
        }

        // Remove constraints that reference this body.
        let to_remove: Vec<_> = self
            .constraints
            .iter()
            .filter(|(_, c)| c.kind.references_body(handle))
            .map(|(idx, _)| idx)
            .collect();
        for idx in to_remove {
            self.constraints.remove(idx);
        }

        self.sleep_manager.sync_bodies(&self.bodies);
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

    /// Get a reference to a collider.
    pub fn collider(&self, handle: ColliderHandle) -> Option<&Collider> {
        self.colliders.get(handle.0)
    }

    // === Constraint Management ===

    /// Create a new constraint and return its handle.
    pub fn create_constraint(
        &mut self,
        kind: super::constraint::ConstraintKind,
    ) -> ConstraintHandle {
        // Wake all bodies referenced by this constraint so the solver
        // processes them immediately (e.g. grabbing a sleeping body).
        for handle in kind.referenced_bodies() {
            self.sleep_manager.wake_body(handle);
        }
        ConstraintHandle(self.constraints.insert(Constraint::new(kind)))
    }

    /// Remove a constraint.
    pub fn remove_constraint(&mut self, handle: ConstraintHandle) -> bool {
        self.constraints.remove(handle.0).is_some()
    }

    /// Get a reference to a constraint.
    pub fn constraint(&self, handle: ConstraintHandle) -> Option<&Constraint> {
        self.constraints.get(handle.0)
    }

    /// Get a mutable reference to a constraint.
    pub fn constraint_mut(&mut self, handle: ConstraintHandle) -> Option<&mut Constraint> {
        self.constraints.get_mut(handle.0)
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

    /// Set velocity on any non-static body.
    ///
    /// Directly overwrites the body's velocity. Suitable for one-shot pushes
    /// or initial conditions. For per-frame velocity control (player character,
    /// moving platforms), use `set_body_velocity_drive` instead.
    #[allow(unused)]
    pub fn set_body_velocity(
        &mut self,
        handle: RigidBodyHandle,
        linear: Vector3<f32>,
        angular: Vector3<f32>,
    ) -> bool {
        let Some(body) = self.bodies.get_mut(handle.0) else {
            return false;
        };
        if body.is_static() {
            return false;
        }
        body.set_linear_velocity(linear);
        body.set_angular_velocity(angular);
        self.sleep_manager.wake_body(handle);
        true
    }

    /// Set a per-substep velocity drive on a non-static body.
    ///
    /// Instead of directly overwriting the body's velocity, this sets a drive
    /// that accelerates toward `linear` each substep during force integration.
    /// The solver can then oppose the drive via contact impulses, allowing
    /// smooth pushing of heavy objects at a speed determined by mass ratio.
    pub fn set_body_velocity_drive(
        &mut self,
        handle: RigidBodyHandle,
        linear: Vector3<f32>,
        angular: Vector3<f32>,
        max_accel: f32,
        angular_max_accel: f32,
    ) -> bool {
        let Some(body) = self.bodies.get_mut(handle.0) else {
            return false;
        };
        if body.is_static() {
            return false;
        }
        body.set_velocity_drive(linear, max_accel);
        body.set_angular_velocity_drive(angular, angular_max_accel);
        self.sleep_manager.wake_body(handle);
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

        let collider = Collider::new(desc);
        let collider_handle = ColliderHandle(self.colliders.insert(collider));

        // Update body's mass properties
        if let Some(body) = self.bodies.get_mut(body_handle.0) {
            body.add_collider(collider_handle);
            self.recompute_mass_properties(body_handle);
        }

        Some(collider_handle)
    }

    // === Simulation ===

    /// Run narrowphase contact generation and manifold cache update.
    ///
    /// Call once before a series of `substep()` calls. This performs:
    /// 1. Sleep bookkeeping
    /// 2. One-shot impulse application
    /// 3. Narrowphase contact generation (static + dynamic)
    /// 4. Manifold cache merge (warm-start population)
    /// 5. Sleep/debug contact processing
    ///
    /// The resulting manifolds are cached internally for `substep()` to consume.
    pub fn update_contacts(
        &mut self,
        dt: f32,
        static_geometry: &dyn StaticGeometry,
        impulses: &[PhysicsImpulse],
        debug_lines: &mut DebugLines,
    ) {
        let _ = debug_lines;

        self.frame_index = self.frame_index.wrapping_add(1);
        self.sleep_manager.sync_bodies(&self.bodies);
        self.sleep_manager.apply_wake_events(&[], &self.bodies);
        let sleeping_snapshot = if self.config.sleep.enabled {
            Some(self.sleep_manager.sleeping_snapshot())
        } else {
            None
        };

        // Apply one-shot impulses and persistent force fields
        self.apply_impulses(impulses);

        // Narrowphase contact generation
        let mut raw_manifolds = generate_static_contacts(
            &self.bodies,
            &self.colliders,
            static_geometry,
            self.config.contact_margin,
            dt,
            self.config.ccd_threshold,
            self.config.enable_speculative_contacts,
            self.config.speculative_min_speed,
            self.config.speculative_margin_multiplier,
            sleeping_snapshot.as_ref(),
        );
        generate_dynamic_contacts(
            &self.bodies,
            &self.colliders,
            self.config.contact_margin,
            dt,
            self.config.ccd_threshold,
            self.config.enable_speculative_contacts,
            self.config.speculative_min_speed,
            self.config.speculative_margin_multiplier,
            sleeping_snapshot.as_ref(),
            &mut self.sat_cache_map,
            &mut self.narrowphase_work_buffer,
        );
        raw_manifolds.extend_from_slice(self.narrowphase_work_buffer.manifolds());

        // Merge with manifold cache (populates warm-start impulses)
        let solver_manifolds = self
            .manifold_cache
            .merge(&raw_manifolds, self.config.deterministic_contact_ordering);

        self.last_contacts.clear();
        for manifold in &solver_manifolds {
            for contact in &manifold.contacts {
                self.last_contacts.push(ContactEvent::from_solver(
                    &manifold.header,
                    contact,
                    ContactSource::Narrowphase,
                ));
            }
        }

        self.sleep_manager
            .note_contact_wakes(&solver_manifolds, &self.bodies);
        self.sleep_manager
            .apply_wake_events(&solver_manifolds, &self.bodies);

        let active_manifolds = self
            .sleep_manager
            .filter_active_manifolds(&solver_manifolds);

        self.debugger.update(
            &self.bodies,
            &raw_manifolds,
            &solver_manifolds,
            &active_manifolds,
            &self.last_contacts,
        );

        // Cache narrowphase-handled set for CCD exclusion
        self.cached_narrowphase_handled.clear();
        self.cached_narrowphase_handled.extend(
            active_manifolds
                .iter()
                .filter(|m| m.header.body_a.is_none())
                .map(|m| m.header.body_b),
        );

        self.cached_active_manifolds = active_manifolds;
        self.cached_all_manifolds = solver_manifolds;

        // Condition manifolds (reorder + compute shock scales) before solving
        let gravity_dir = {
            let len = self.config.gravity.norm();
            if len > 1e-6 {
                self.config.gravity / len
            } else {
                Vector3::new(0.0, -1.0, 0.0)
            }
        };
        self.conditioner.condition(
            &self.bodies,
            &mut self.cached_active_manifolds,
            gravity_dir,
            &mut self.manifold_conditions,
        );

        // Expand user-defined constraints into solver-ready rows
        expand_constraints(
            &self.constraints,
            &self.bodies,
            dt,
            self.config.constraint_position_beta,
            &mut self.cached_constraint_rows,
        );

        self.solver.prepare(&self.bodies);
    }

    /// Solve velocity constraints and integrate positions using cached manifolds.
    ///
    /// Call one or more times after `update_contacts()`. Each call performs:
    /// 1. Integrate forces (gravity) into velocities
    /// 2. Solve velocity constraints (warm-start + sequential impulses)
    /// 3. Write solved impulses back to manifold cache
    /// 4. Integrate positions
    /// 5. CCD pass (fast bodies only)
    /// 6. Update sleep states
    pub fn substep(
        &mut self,
        dt: f32,
        static_geometry: &dyn StaticGeometry,
        force_providers: &[&dyn SubstepForceProvider],
    ) {
        let sleeping_snapshot = if self.config.sleep.enabled {
            Some(self.sleep_manager.sleeping_snapshot())
        } else {
            None
        };

        // Apply per-substep external forces from providers.
        // Forces are cleared first so bodies that leave an affected region
        // stop receiving stale forces.
        self.apply_substep_forces(force_providers);

        // Integrate forces (gravity) into velocities
        integrate_forces(
            &mut self.bodies,
            dt,
            self.config.gravity,
            sleeping_snapshot.as_ref(),
        );

        // Solve velocity constraints + position correction
        self.solver.solve(
            &mut self.bodies,
            &mut self.cached_active_manifolds,
            &self.manifold_conditions,
            &mut self.cached_constraint_rows,
            dt,
        );
        self.debugger
            .update_post_solve(&self.bodies, &self.cached_active_manifolds);

        // Write solved impulses back to manifold cache
        self.manifold_cache
            .write_back(&self.cached_active_manifolds);
        self.manifold_cache.prune();

        // Write solved constraint impulses back to persistent constraints
        write_back_constraints(&mut self.constraints, &self.cached_constraint_rows);

        // Hard projection: remove angular velocity components forbidden by
        // constraints. This guarantees correctness regardless of solver
        // iteration count and handles large-angle tilt where linearized
        // Jacobians become degenerate.
        project_angular_velocities(
            &self.constraints,
            &mut self.bodies,
            dt,
            self.config.constraint_position_beta,
        );

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

        // Integrate positions
        let sleeping_snapshot = if self.config.sleep.enabled {
            Some(self.sleep_manager.sleeping_snapshot())
        } else {
            None
        };
        integrate_bodies(&mut self.bodies, dt, sleeping_snapshot.as_ref());

        // CCD pass (fast bodies only, excluding narrowphase-managed bodies)
        let narrowphase_handled = std::mem::take(&mut self.cached_narrowphase_handled);
        if let Some(mut ccd) = self.ccd.take() {
            let mut ctx = CcdContext {
                bodies: &mut self.bodies,
                colliders: &self.colliders,
                contact_events: &mut self.last_contacts,
                narrowphase_handled: &narrowphase_handled,
                sleeping: sleeping_snapshot.as_ref(),
                pre_states: &pre_states,
                contact_margin: self.config.contact_margin,
                restitution_velocity_threshold: self.config.restitution_velocity_threshold,
                ccd_threshold: self.config.ccd_threshold,
            };
            let _ccd_count = ccd.run(&mut ctx, dt, static_geometry);
            self.ccd = Some(ccd);
        }
        self.cached_narrowphase_handled = narrowphase_handled;

        let all_manifolds = std::mem::take(&mut self.cached_all_manifolds);
        self.sleep_manager
            .update_sleep_states(&mut self.bodies, &all_manifolds, &self.constraints);
        self.cached_all_manifolds = all_manifolds;
    }

    /// Contacts generated in the most recent step.
    pub fn contact_events(&self) -> &[ContactEvent] {
        &self.last_contacts
    }

    /// Get access to the rigid bodies arena.
    pub fn bodies(&self) -> &Arena<RigidBody> {
        &self.bodies
    }

    /// Apply per-substep forces from external providers.
    ///
    /// Clears all force/torque/drag accumulators first, then asks each provider
    /// to recompute forces for its affected bodies based on current positions.
    fn apply_substep_forces(&mut self, providers: &[&dyn SubstepForceProvider]) {
        // Clear accumulators so bodies that leave a force region get zero.
        for (_, body) in self.bodies.iter_mut() {
            body.set_force(Vector3::zeros());
            body.set_torque(Vector3::zeros());
            body.set_drag(0.0, 0.0);
        }

        if providers.is_empty() {
            return;
        }

        // Collect outputs first (immutable borrow of bodies/colliders),
        // then apply them (mutable borrow of bodies).
        let ctx = ForceContext {
            bodies: &self.bodies,
            colliders: &self.colliders,
            gravity: self.config.gravity,
            gravity_magnitude: self.config.gravity.magnitude(),
        };

        let mut outputs: Vec<(RigidBodyHandle, ForceOutput)> = Vec::new();
        for provider in providers {
            for &handle in provider.affected_bodies() {
                let output = provider.compute_force(handle, &ctx);
                outputs.push((handle, output));
            }
        }

        for (handle, output) in outputs {
            if let Some(body) = self.bodies.get_mut(handle.0) {
                body.set_force(output.force);
                body.set_torque(output.torque);
                body.set_drag(output.linear_drag_coeff, output.angular_drag_coeff);
            }
        }
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
                let m = collider.mass();
                total_mass += m;

                // Rotate the local inertia tensor into the body frame.
                let r = collider.offset().rotation.to_rotation_matrix();
                let rotated_inertia = r * collider.local_inertia() * r.transpose();

                // Parallel axis theorem: shift inertia to body center of mass.
                let d = collider.offset().translation.vector;
                let d_sq = d.dot(&d);
                let steiner = m * (d_sq * Matrix3::identity() - d * d.transpose());

                total_inertia += rotated_inertia + steiner;
            }
        }

        if let Some(body) = self.bodies.get_mut(body_handle.0) {
            body.set_mass_properties(total_mass, total_inertia);
        }
    }

    /// Swept-sphere probe that identifies which body was hit.
    /// Skips static bodies and any body in `exclude`.
    pub fn probe_bodies(
        &self,
        origin: Point3<f32>,
        direction: Vector3<f32>,
        length: f32,
        radius: f32,
        exclude: &[RigidBodyHandle],
    ) -> Option<BodyProbeHit> {
        let end = origin + direction * length;
        let mut earliest: Option<BodyProbeHit> = None;

        for (idx, body) in self.bodies.iter() {
            if body.is_static() {
                continue;
            }
            let handle = RigidBodyHandle(idx);
            if exclude.contains(&handle) {
                continue;
            }
            let body_pos = body.position();
            let body_rot = body.rotation();

            for ch in body.colliders() {
                let Some(collider) = self.colliders.get(ch.0) else {
                    continue;
                };
                let center = collider.world_center(body_pos, body_rot);
                let hit = match collider.shape() {
                    ColliderShape::Sphere { radius: r } => {
                        probe_vs_sphere(origin, end, radius, center, *r)
                    }
                    ColliderShape::Box { half_extents } => {
                        let obb = Obb::new(center, body_rot, *half_extents);
                        probe_vs_obb(origin, end, radius, &obb, *half_extents)
                    }
                    ColliderShape::Capsule {
                        half_height,
                        radius: cap_radius,
                    } => probe_vs_capsule(
                        origin,
                        end,
                        radius,
                        center,
                        body_rot,
                        *half_height,
                        *cap_radius,
                    ),
                };
                if let Some(hit) = hit {
                    if earliest.as_ref().map_or(true, |e| hit.t < e.hit.t) {
                        earliest = Some(BodyProbeHit { body: handle, hit });
                    }
                }
            }
        }

        earliest
    }
}

/// Result of a body-identifying probe.
pub struct BodyProbeHit {
    pub body: RigidBodyHandle,
    pub hit: ProbeHit,
}

impl ProbeTarget for PhysicsWorld {
    fn swept_probe(
        &self,
        origin: Point3<f32>,
        direction: Vector3<f32>,
        length: f32,
        radius: f32,
    ) -> Option<ProbeHit> {
        let end = origin + direction * length;
        let mut earliest: Option<ProbeHit> = None;

        for (_idx, body) in self.bodies.iter() {
            if body.is_static() {
                continue;
            }
            let body_pos = body.position();
            let body_rot = body.rotation();

            for ch in body.colliders() {
                let Some(collider) = self.colliders.get(ch.0) else {
                    continue;
                };
                let center = collider.world_center(body_pos, body_rot);
                let hit = match collider.shape() {
                    ColliderShape::Sphere { radius: r } => {
                        probe_vs_sphere(origin, end, radius, center, *r)
                    }
                    ColliderShape::Box { half_extents } => {
                        let obb = Obb::new(center, body_rot, *half_extents);
                        probe_vs_obb(origin, end, radius, &obb, *half_extents)
                    }
                    ColliderShape::Capsule {
                        half_height,
                        radius: cap_radius,
                    } => probe_vs_capsule(
                        origin,
                        end,
                        radius,
                        center,
                        body_rot,
                        *half_height,
                        *cap_radius,
                    ),
                };
                if let Some(hit) = hit {
                    if earliest.as_ref().map_or(true, |e: &ProbeHit| hit.t < e.t) {
                        earliest = Some(hit);
                    }
                }
            }
        }

        earliest
    }
}

/// Sweep a probe sphere against a static sphere body.
fn probe_vs_sphere(
    origin: Point3<f32>,
    end: Point3<f32>,
    probe_radius: f32,
    center: Point3<f32>,
    sphere_radius: f32,
) -> Option<ProbeHit> {
    // Skip bodies the probe origin already overlaps (e.g. the probe source's own body).
    let initial_dist = (origin - center).norm();
    if initial_dist < probe_radius + sphere_radius {
        return None;
    }
    let t = swept_sphere_sphere(origin, end, probe_radius, center, center, sphere_radius)?;
    let probe_at_t = origin + (end - origin) * t;
    let to_surface = probe_at_t - center;
    let len = to_surface.norm();
    if len < 1e-6 {
        return None;
    }
    let normal = to_surface / len;
    Some(ProbeHit {
        t,
        point: center + normal * sphere_radius,
        normal,
    })
}

/// Sweep a probe sphere against a box body using a slab test in OBB local space.
///
/// Transforms the probe into the OBB's local frame and runs a standard
/// ray-vs-AABB slab test against the box expanded by `probe_radius`
/// (the Minkowski sum of box and sphere, approximated at faces).
/// This gives the correct entry time regardless of the box aspect ratio,
/// unlike a bounding-sphere proxy which over-reports for flat/wide boxes.
fn probe_vs_obb(
    origin: Point3<f32>,
    end: Point3<f32>,
    probe_radius: f32,
    obb: &Obb,
    half_extents: Vector3<f32>,
) -> Option<ProbeHit> {
    let inv_rot = obb.rotation.inverse();
    let local_start: Vector3<f32> = inv_rot * (origin - obb.center);
    let local_dir: Vector3<f32> = inv_rot * (end - origin);

    let expanded = half_extents + Vector3::repeat(probe_radius);
    let t = obb_slab_entry(local_start, local_dir, expanded)?;

    let probe_at_t = origin + (end - origin) * t;
    let closest = obb.closest_point(probe_at_t);
    let to_probe = probe_at_t - closest;
    let len = to_probe.norm();
    if len < 1e-6 {
        return None;
    }
    Some(ProbeHit {
        t,
        point: closest,
        normal: to_probe / len,
    })
}

/// Sweep a probe sphere against a capsule body using iterative sampling.
///
/// Uses the analytic sphere-capsule manifold as a predicate along the probe path,
/// first finding a bracketing interval with a coarse scan and then refining the
/// entry time with binary search. This avoids adding a dedicated swept test while
/// still handling the full capsule geometry (cylinder + caps).
fn probe_vs_capsule(
    origin: Point3<f32>,
    end: Point3<f32>,
    probe_radius: f32,
    center: Point3<f32>,
    rotation: UnitQuaternion<f32>,
    half_height: f32,
    cap_radius: f32,
) -> Option<ProbeHit> {
    let capsule = Capsule::new(center, rotation, half_height, cap_radius);

    // Skip bodies the probe origin already overlaps (e.g. the probe source's own body).
    if !sphere_capsule_manifold(&capsule, origin, probe_radius, 0.0).is_empty() {
        return None;
    }

    let dir = end - origin;
    let steps = 16;
    let mut t_prev = 0.0f32;
    let mut hit_interval: Option<(f32, f32)> = None;

    // Coarse scan to find the first interval [t_prev, t] where we enter the capsule.
    for i in 1..=steps {
        let t = i as f32 / steps as f32;
        let pos = origin + dir * t;
        if !sphere_capsule_manifold(&capsule, pos, probe_radius, 0.0).is_empty() {
            hit_interval = Some((t_prev, t));
            break;
        }
        t_prev = t;
    }

    let Some((mut t_lo, mut t_hi)) = hit_interval else {
        return None;
    };

    // Refine with binary search to approximate the entry time.
    for _ in 0..8 {
        let mid = 0.5 * (t_lo + t_hi);
        let pos = origin + dir * mid;
        if !sphere_capsule_manifold(&capsule, pos, probe_radius, 0.0).is_empty() {
            t_hi = mid;
        } else {
            t_lo = mid;
        }
    }

    let t = t_hi;
    let probe_at_t = origin + dir * t;
    let manifold = sphere_capsule_manifold(&capsule, probe_at_t, probe_radius, 0.0);
    if manifold.is_empty() {
        return None;
    }
    let contact = &manifold.points[0];

    Some(ProbeHit {
        t,
        point: contact.point,
        normal: contact.normal,
    })
}

/// Ray-vs-AABB slab test in local space.
///
/// Returns the first entry time t ∈ (0, 1] where the ray enters the box.
/// Returns None if the ray misses the box, is parallel to a slab it doesn't
/// overlap, or if the origin is already inside the box (treated as overlap).
fn obb_slab_entry(start: Vector3<f32>, dir: Vector3<f32>, half: Vector3<f32>) -> Option<f32> {
    let mut t_enter = f32::NEG_INFINITY;
    let mut t_exit = f32::INFINITY;

    for i in 0..3 {
        if dir[i].abs() < 1e-8 {
            // Ray is parallel to this slab — miss if outside it
            if start[i] < -half[i] || start[i] > half[i] {
                return None;
            }
        } else {
            let inv = 1.0 / dir[i];
            let t1 = (-half[i] - start[i]) * inv;
            let t2 = (half[i] - start[i]) * inv;
            let (t_near, t_far) = if t1 <= t2 { (t1, t2) } else { (t2, t1) };
            t_enter = t_enter.max(t_near);
            t_exit = t_exit.min(t_far);
        }
    }

    if t_enter > t_exit {
        return None; // Miss
    }
    // t_enter < 0: origin is inside the expanded box — skip (treat as overlap)
    // t_enter > 1: box is beyond probe end
    if t_enter >= 0.0 && t_enter <= 1.0 {
        Some(t_enter)
    } else {
        None
    }
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new(PhysicsConfig::default())
    }
}

/// Per-step manifold cache diagnostics for harness/metrics export.
use super::pipeline::manifold::ManifoldFrameStats;
impl PhysicsWorld {
    pub fn manifold_frame_stats(&self) -> ManifoldFrameStats {
        self.manifold_cache.frame_stats()
    }
}
