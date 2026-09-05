//! Physics world containing all simulation state.

use generational_arena::Arena;
use nalgebra::{Isometry3, Matrix3, Point3, UnitQuaternion, UnitVector3, Vector3};
use rustc_hash::FxHashMap;

use super::body::{BodyDrive, BodyType, RigidBody, RigidBodyDesc, SupportDrive};
use super::ccd::{
    pair_separation, CcdContext, CcdStrategy, ContactPairKey, NarrowphaseOwnership, SweepClampCcd,
};
use super::collider::{Collider, ColliderDesc, ColliderShape};
use super::constraint::types::Constraint;
use super::constraint::{ConstraintHandle, ConstraintKind};
use super::contact_event::{ContactEvent, ContactSource};
use super::debug::{PhysicsDebugConfig, PhysicsDebugger};
use super::drive::{
    apply_allowances, stamp_non_support_grip, AllowanceCommand, AllowanceLedger, AllowanceUsage,
    DriveCommand, ReactionAnchor, SupportConfig, SupportResolver, SupportSets, TractionLedger,
    TractionPlanner, TractionUsage,
};
use super::force_provider::{ForceContext, ForceOutput, SubstepForceProvider};
use super::grounding::{GroundedBodies, GroundingDetector};
use super::handle::{ColliderHandle, RigidBodyHandle};
use super::impact::ImpactLedger;
use super::impulses::PhysicsImpulse;
use super::narrowphase::{
    generate_dynamic_contacts, generate_static_contacts, GjkCacheMap, NarrowphaseConfig,
    NarrowphaseWorkBuffer, SatCacheMap, SpeculativeConfig,
};
use super::pipeline::integration::{integrate_bodies, integrate_forces};
use super::pipeline::manifold::ManifoldCache;
use super::pipeline::normal_smoothing::NormalSmoothingConfig;
use super::pipeline::pair::SolverManifold;
use super::sleep::{SleepManager, SleepManagerConfig};
use super::solver::{ConstraintSolver, ManifoldConditioner, ManifoldConditions, PgsNgsSolver};
use super::static_geometry::StaticGeometry;
use crate::collision::convex_hull::ConvexHull;
use crate::collision::obb::Obb;
use crate::debug::DebugLines;
use crate::physics::ShockPropagationConditioner;
use crate::sensing::{ProbeHit, ProbeTarget};

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
    /// Per-substep CCD activation threshold. A body requires CCD when:
    /// `|linear_velocity| * substep_dt > radius * ccd_threshold`.
    /// Below this, the narrowphase handles contacts; above, CCD sweeps
    /// prevent tunneling. Also partitions speculative contact generation,
    /// which covers the band below this threshold.
    ///
    /// See `ccd_frame_coverage` for the frame-level gate that catches bodies
    /// slow enough to pass this test yet fast enough to outrun the
    /// once-per-frame narrowphase.
    pub ccd_threshold: f32,
    /// Frame-level CCD activation, as a multiple of the collider radius.
    ///
    /// The narrowphase samples once per frame, so what it can miss is set by
    /// travel over the *whole* frame, not one substep. Two consecutive frame
    /// samples of a sphere of radius `r` still bracket any plane between them
    /// while the frame travel stays under `2r`; above that a surface can pass
    /// between samples undetected. The default sits below that limit for
    /// slack, since the bracketing argument is exact only for planes and
    /// degrades on small or angled triangles.
    ///
    /// Independent of `ccd_threshold`, which is per-substep: a body activates
    /// CCD if *either* gate trips.
    pub ccd_frame_coverage: f32,
    /// Frames without a narrowphase refresh before a manifold point is pruned.
    pub manifold_max_age: u8,
    /// When true, sort manifold output contacts for deterministic solver ordering.
    ///
    /// This is primarily intended for reproducible tests and diagnostics.
    pub deterministic_contact_ordering: bool,
    /// Configuration for smoothing matched contact normals.
    pub normal_smoothing: NormalSmoothingConfig,
    /// Configuration for what counts as a contact holding a body up.
    pub support: SupportConfig,
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
    /// Debug rendering configuration.
    pub debug: PhysicsDebugConfig,
}

impl PhysicsConfig {
    /// The world's down direction: gravity, normalised.
    ///
    /// `None` in a world with no gravity, where "down" is not a question the
    /// configuration can answer. Callers that need an up axis — the drive's
    /// stand-in for a support normal until the Support Set exists — read it
    /// here rather than writing a literal `Vector3::y()`.
    pub fn gravity_direction(&self) -> Option<UnitVector3<f32>> {
        UnitVector3::try_new(self.gravity, 1e-6)
    }

    /// Project the narrowphase's slice of this config.
    ///
    /// Built fresh each frame rather than stored, so the narrowphase can never
    /// drift out of step with a `PhysicsConfig` mutated between frames.
    pub fn narrowphase(&self) -> NarrowphaseConfig {
        NarrowphaseConfig {
            contact_margin: self.contact_margin,
            speculative: SpeculativeConfig {
                enabled: self.enable_speculative_contacts,
                min_speed: self.speculative_min_speed,
                margin_multiplier: self.speculative_margin_multiplier,
                ccd_threshold: self.ccd_threshold,
            },
        }
    }
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            gravity: Vector3::new(0.0, -9.81, 0.0),
            restitution_velocity_threshold: 0.3,
            contact_margin: 0.02,
            ccd_threshold: 0.5,
            ccd_frame_coverage: 1.5,
            manifold_max_age: 3,
            deterministic_contact_ordering: false,
            normal_smoothing: NormalSmoothingConfig::default(),
            support: SupportConfig::default(),
            warm_start_depth_slop: 0.02,
            enable_speculative_contacts: true,
            speculative_min_speed: 1.0,
            speculative_margin_multiplier: 2.0,
            sleep: SleepManagerConfig::default(),
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
    /// Normal impulses delivered to each body this frame.
    impacts: ImpactLedger,
    debugger: PhysicsDebugger,
    frame_index: u64,
    sleep_manager: SleepManager,
    /// Classifies contacts into per-body Support Sets.
    support_resolver: SupportResolver,
    /// Turns support-anchored commands into per-contact tangential targets.
    traction_planner: TractionPlanner,
    /// The Support Sets the frame's active manifolds were classified into.
    ///
    /// Held for the frame because three things read the same classification —
    /// the grip stamp, the traction plan and the gain ledger — and a
    /// `ContactSite` is only meaningful against the slice it came from.
    frame_supports: SupportSets,
    /// What the drive gain spent, per body. Instrumentation only.
    traction_ledger: TractionLedger,
    /// What each body's allowance has conjured, per `physics::drive::allowance`.
    allowance_ledger: AllowanceLedger,
    /// Projects Support Sets to the grounded set, carrying sleeping bodies.
    grounding_detector: GroundingDetector,
    /// User-defined constraints (persistent across frames).
    constraints: Arena<Constraint>,
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
    /// Bodies with static narrowphase contacts, excluded from CCD until they
    /// integrate away from the position the contacts were generated at.
    narrowphase_ownership: NarrowphaseOwnership,
    /// Substeps the caller will run for the current frame, as declared to
    /// `update_contacts()`. Sizes the frame-level CCD gate and the CCD query
    /// cache's lookahead.
    substeps_this_frame: u32,
    /// Substeps run since the frame's contacts were updated, so the
    /// edge-triggered half of a drive command fires exactly once per frame.
    substeps_taken: u32,
    /// SAT axis cache for OBB-OBB dynamic pair early-out.
    sat_cache_map: SatCacheMap,
    /// GJK warm-start cache for wildcard dynamic pairs.
    gjk_cache_map: GjkCacheMap,
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
        let support_resolver = SupportResolver::new(config.support);
        let grounding_detector = GroundingDetector::new();
        let debugger = PhysicsDebugger::new(config.debug.clone());
        Self {
            config,
            bodies: Arena::new(),
            colliders: Arena::new(),
            manifold_cache,
            last_contacts: Vec::new(),
            impacts: ImpactLedger::default(),
            debugger,
            frame_index: 0,
            sleep_manager,
            support_resolver,
            traction_planner: TractionPlanner,
            frame_supports: SupportSets::default(),
            traction_ledger: TractionLedger::default(),
            allowance_ledger: AllowanceLedger::default(),
            grounding_detector,
            constraints: Arena::new(),
            solver,
            conditioner,
            manifold_conditions: ManifoldConditions::new(),
            ccd: Some(ccd),
            cached_active_manifolds: Vec::new(),
            cached_all_manifolds: Vec::new(),
            narrowphase_ownership: NarrowphaseOwnership::new(),
            substeps_this_frame: 1,
            substeps_taken: 0,
            sat_cache_map: SatCacheMap::new(),
            gjk_cache_map: GjkCacheMap::new(),
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

    /// Apply an instantaneous linear impulse to one body, waking it.
    ///
    /// Prefer this over `body_mut(h).apply_impulse(..)`. The body-level methods
    /// know nothing about the sleep manager, so an impulse applied through them
    /// lands on a sleeping body's velocity and is then ignored — the body sits
    /// there accumulating speed it never uses until something else wakes it.
    /// Waking belongs with the impulse, not with the caller's memory.
    ///
    /// Returns false if the handle refers to no body.
    pub fn apply_impulse(&mut self, handle: RigidBodyHandle, impulse: Vector3<f32>) -> bool {
        let Some(body) = self.body_mut(handle) else {
            return false;
        };
        body.apply_impulse(impulse);
        self.wake_body(handle);
        true
    }

    /// Apply an instantaneous angular impulse to one body, waking it.
    ///
    /// See [`apply_impulse`](Self::apply_impulse) for why this exists.
    ///
    /// Returns false if the handle refers to no body.
    pub fn apply_angular_impulse(
        &mut self,
        handle: RigidBodyHandle,
        angular_impulse: Vector3<f32>,
    ) -> bool {
        let Some(body) = self.body_mut(handle) else {
            return false;
        };
        body.apply_angular_impulse(angular_impulse);
        self.wake_body(handle);
        true
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
    ///
    /// Dynamic bodies with zero velocity are created asleep so that pre-placed
    /// level geometry doesn't pay broadphase/solver cost until something
    /// interacts with it.
    pub fn create_body(&mut self, desc: RigidBodyDesc) -> RigidBodyHandle {
        let start_asleep = self.config.sleep.enabled
            && desc.body_type == BodyType::Dynamic
            && desc.linear_velocity.magnitude_squared() < 1e-12
            && desc.angular_velocity.magnitude_squared() < 1e-12;

        let body = RigidBody::new(desc);
        let handle = RigidBodyHandle(self.bodies.insert(body));

        if start_asleep {
            self.sleep_manager.sleep_body(handle);
        }

        handle
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
    /// moving platforms), use `set_body_drive` instead.
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

    /// Push one frame's drive command into a non-static body.
    ///
    /// The single entry point for driving a body, and the reason a body can
    /// never hold two drives at once: the command's `anchor` decides which
    /// delivery path this call establishes, and establishing one retires the
    /// other. A body carrying both a support drive and medium rows toward the
    /// same target would get roughly twice the authority its actuator
    /// declares, and the rows' bounds would stop meaning anything.
    ///
    /// - `ReactionAnchor::Support` records a support-relative target, which
    ///   the traction planner turns into a target for the tangential row at
    ///   each contact holding the body up. The reaction lands on the support,
    ///   and the authority is the contact's own `μ·N`.
    /// - `ReactionAnchor::Medium` maintains world-anchored motor rows in the
    ///   constraint arena, solved alongside gravity and every contact.
    ///
    /// Wakes the body either way. The wake is load-bearing rather than
    /// incidental: `EnergyTracker` is purely velocity-based, so a driven body
    /// with a saturated bound and near-zero velocity — a character walking
    /// into a wall — would otherwise sleep and stop being solved, and stay
    /// asleep after the command changed. A sleeping body discards any impulse
    /// aimed at it, so the player would freeze against the wall and stay
    /// frozen after releasing the stick.
    ///
    /// That makes this one of R11's three surviving drive-aware sites, and the
    /// only one the design expected to keep. Sleep is the exception because it
    /// is the one subsystem that reasons about a body's *future* from its
    /// present velocity, and a drive is exactly the thing that invalidates
    /// that inference.
    pub fn set_body_drive(&mut self, handle: RigidBodyHandle, command: &DriveCommand) -> bool {
        let Some(body) = self.bodies.get(handle.0) else {
            return false;
        };
        if body.is_static() {
            return false;
        }

        match command.anchor {
            ReactionAnchor::Support => {
                self.retire_medium_drive(handle);
                let Some(body) = self.bodies.get_mut(handle.0) else {
                    return false;
                };
                body.set_support_drive(SupportDrive {
                    linear_target: command.linear_target,
                    angular_target: command.angular_target,
                    gain: command.drive_gain,
                    patch_radius: command.patch_radius,
                });
            }
            ReactionAnchor::Medium => self.set_medium_drive(handle, command),
        }

        if let Some(body) = self.bodies.get_mut(handle.0) {
            body.set_allowance_command(command.allowance);
        }

        self.sleep_manager.wake_body(handle);
        true
    }

    /// Take a body out of service: no drive, no allowance, honest grip.
    ///
    /// The counterpart to `set_body_drive`, for when the thing that was
    /// commanding a body stops existing. Without it a corpse keeps whatever it
    /// was last asked for — a stale target its contacts still answer, and,
    /// worse, an allowance that has no contact to answer to at all and would
    /// go on steering and turning the body for as long as it lay there.
    pub fn clear_body_drive(&mut self, handle: RigidBodyHandle) -> bool {
        self.retire_medium_drive(handle);
        let Some(body) = self.bodies.get_mut(handle.0) else {
            return false;
        };
        body.take_drive();
        body.set_allowance_command(AllowanceCommand::default());
        body.set_non_support_grip(1.0);
        true
    }

    /// Create or update the motor rows behind a medium-anchored drive.
    ///
    /// Updating in place rather than recreating keeps the constraint's
    /// warm-start impulses, which is what stops a platform re-converging on
    /// its target from scratch every frame.
    ///
    /// This and its two counterparts — `retire_medium_drive` and
    /// `clear_body_drive` — are R11's third surviving drive-aware site: the
    /// lifetime of the constraint a `BodyDrive::Medium` names. It survives
    /// because the rows outlive the frame that asked for them and something
    /// has to own them; nothing downstream is aware of them, since the solver
    /// cannot tell a `MediumDrive` row from a hinge's.
    fn set_medium_drive(&mut self, handle: RigidBodyHandle, command: &DriveCommand) {
        let kind = ConstraintKind::MediumDrive {
            body: handle,
            linear_target: command.linear_target,
            angular_target: command.angular_target,
            max_accel: command.max_accel,
            angular_max_accel: command.angular_max_accel,
        };

        let existing = self
            .bodies
            .get(handle.0)
            .and_then(|body| body.medium_drive())
            .filter(|c| self.constraints.contains(c.0));

        match existing {
            Some(constraint) => {
                if let Some(stored) = self.constraints.get_mut(constraint.0) {
                    stored.kind = kind;
                    stored.active = true;
                }
            }
            None => {
                let constraint = self.create_constraint(kind);
                if let Some(body) = self.bodies.get_mut(handle.0) {
                    body.set_medium_drive(constraint);
                }
            }
        }
    }

    /// Drop the motor rows behind a body's medium drive, if it has any.
    ///
    /// Leaves the body undriven: whatever replaces the drive is the caller's
    /// business, and nothing else in the engine may own this constraint.
    fn retire_medium_drive(&mut self, handle: RigidBodyHandle) {
        let Some(body) = self.bodies.get_mut(handle.0) else {
            return;
        };
        if let Some(BodyDrive::Medium(constraint)) = body.take_drive() {
            self.constraints.remove(constraint.0);
        }
    }

    /// Set what fraction of a contact's tangential budget a body may draw
    /// where that contact is not holding it up.
    ///
    /// The actuator's declaration, pushed down each frame. `1.0` — the default
    /// — grips everything the body touches equally; a character sets it near
    /// zero so jumps along vertical surfaces are not grabbed. It scales only
    /// this body's own share, so nothing it leans on inherits the tuning.
    pub fn set_body_non_support_grip(&mut self, handle: RigidBodyHandle, grip: f32) -> bool {
        let Some(body) = self.bodies.get_mut(handle.0) else {
            return false;
        };
        body.set_non_support_grip(grip);
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

    /// Detach a single collider from a body, removing it from the physics world.
    ///
    /// Returns the detached `Collider` so the caller can use its shape, offset,
    /// density, and material to create a new body. Recomputes the body's mass
    /// properties after removal. Also purges the manifold cache for the collider.
    pub fn detach_collider(
        &mut self,
        body_handle: RigidBodyHandle,
        collider_handle: ColliderHandle,
    ) -> Option<Collider> {
        let body = self.bodies.get_mut(body_handle.0)?;
        if !body.remove_collider(collider_handle) {
            return None;
        }

        self.manifold_cache.remove_collider(collider_handle);
        let collider = self.colliders.remove(collider_handle.0)?;
        self.recompute_mass_properties(body_handle);
        Some(collider)
    }

    // === Simulation ===

    /// Run narrowphase contact generation and manifold cache update.
    ///
    /// Call once before a series of `substep()` calls. `substeps` is how many
    /// `substep()` calls will follow; contacts generated here must cover that
    /// whole span, so CCD sizes its activation gate and query cache from it.
    /// This performs:
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
        substeps: u32,
        static_geometry: &dyn StaticGeometry,
        impulses: &[PhysicsImpulse],
        debug_lines: &mut DebugLines,
    ) {
        let _ = debug_lines;

        self.substeps_this_frame = substeps.max(1);
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

        // Narrowphase contact generation. Both passes append to one buffer,
        // static first, so the caller owns the reset rather than either pass.
        let narrowphase_config = self.config.narrowphase();
        self.narrowphase_work_buffer.begin_frame();
        generate_static_contacts(
            &self.bodies,
            &self.colliders,
            static_geometry,
            &narrowphase_config,
            dt,
            sleeping_snapshot.as_ref(),
            &mut self.narrowphase_work_buffer,
        );
        generate_dynamic_contacts(
            &self.bodies,
            &self.colliders,
            &narrowphase_config,
            dt,
            sleeping_snapshot.as_ref(),
            &mut self.sat_cache_map,
            &mut self.gjk_cache_map,
            &mut self.narrowphase_work_buffer,
        );
        let raw_manifolds = self.narrowphase_work_buffer.manifolds();

        // Merge with manifold cache (populates warm-start impulses)
        let solver_manifolds = self
            .manifold_cache
            .merge(raw_manifolds, self.config.deterministic_contact_ordering);

        self.last_contacts.clear();
        self.impacts.clear();
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

        // Record narrowphase ownership for CCD exclusion, anchored at the
        // separation each manifold was generated at so ownership can expire as
        // the pair integrates away from it across substeps.
        self.narrowphase_ownership.clear();
        for manifold in active_manifolds.iter() {
            let header = &manifold.header;
            let Some(key) = ContactPairKey::from_header(header) else {
                continue;
            };
            if let Some(separation) = pair_separation(&self.bodies, header.body_a, header.body_b) {
                self.narrowphase_ownership.insert(key, separation);
            }
        }

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
            &self.constraints,
            &mut self.cached_active_manifolds,
            gravity_dir,
            &mut self.manifold_conditions,
        );

        // What a body may draw at each contact, and what it is driving that
        // contact toward, are both decided once, here, from the same slice the
        // solver is about to read: a Support Set names contacts by position in
        // it, and the conditioner has already finished reordering it.
        self.frame_supports = self.support_resolver.resolve(
            &self.cached_active_manifolds,
            self.config.gravity_direction(),
        );
        stamp_non_support_grip(
            &self.bodies,
            &self.frame_supports,
            &mut self.cached_active_manifolds,
        );

        // A drive is friction with a non-zero target, and this is where the
        // target is written: at the contacts holding a driven body up, in the
        // same slice the grip was just stamped on.
        self.traction_planner.plan(
            &self.bodies,
            &self.frame_supports,
            &mut self.cached_active_manifolds,
        );
        self.traction_ledger.open_frame();
        self.substeps_taken = 0;

        self.solver.prepare(&self.bodies, &self.constraints, dt);

        // Static geometry is fixed for the frame; let CCD reset the query
        // cache it reuses across this frame's substeps.
        if let Some(ccd) = self.ccd.as_mut() {
            ccd.begin_frame(self.substeps_this_frame);
        }
    }

    /// Solve velocity constraints and integrate positions using cached manifolds.
    ///
    /// Call one or more times after `update_contacts()`. Each call performs:
    /// 1. Integrate forces (gravity) into velocities
    /// 2. Spend the frame's allowances, where a body has authority no contact
    ///    could bound
    /// 3. Solve velocity constraints (warm-start + sequential impulses)
    /// 4. Write solved impulses back to manifold cache
    /// 5. Integrate positions
    /// 6. CCD pass (fast bodies only)
    /// 7. Update sleep states
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

        // Non-conservative authority, spent before the solve so that whatever
        // it conjures is still answerable to every contact and constraint. The
        // edge-triggered verbs fire on the frame's first substep only: a jump
        // repeated per substep would be as strong as the frame rate is slow.
        apply_allowances(
            &mut self.bodies,
            &self.frame_supports,
            self.config.gravity_direction().map(|down| -down),
            dt,
            self.substeps_taken == 0,
            &mut self.allowance_ledger,
        );
        self.substeps_taken += 1;

        // Solve velocity constraints + position correction
        self.solver.solve(
            &mut self.bodies,
            &mut self.cached_active_manifolds,
            &self.manifold_conditions,
            &self.constraints,
            dt,
        );
        self.impacts.record_solved(&self.cached_active_manifolds);
        self.traction_ledger.record_substep(
            &self.cached_active_manifolds,
            &self.frame_supports,
            dt,
        );
        self.debugger
            .update_post_solve(&self.bodies, &self.cached_active_manifolds);

        // Write solved impulses back to manifold cache
        self.manifold_cache
            .write_back(&self.cached_active_manifolds);
        self.manifold_cache.prune();

        // Write solved constraint impulses back to persistent constraints
        self.solver.write_back(&mut self.constraints);

        // Hard projection: remove angular velocity components forbidden by
        // constraints. This guarantees correctness regardless of solver
        // iteration count and handles large-angle tilt where linearized
        // Jacobians become degenerate.
        self.solver.project_velocities(&mut self.bodies, dt);

        // Save pre-integration state for CCD
        let sleeping_snapshot = if self.config.sleep.enabled {
            Some(self.sleep_manager.sleeping_snapshot())
        } else {
            None
        };
        let pre_states: FxHashMap<generational_arena::Index, (Point3<f32>, UnitQuaternion<f32>)> =
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
        let narrowphase_ownership = std::mem::take(&mut self.narrowphase_ownership);
        if let Some(mut ccd) = self.ccd.take() {
            let mut ctx = CcdContext {
                bodies: &mut self.bodies,
                colliders: &self.colliders,
                contact_events: &mut self.last_contacts,
                impacts: &mut self.impacts,
                narrowphase_ownership: &narrowphase_ownership,
                sleeping: sleeping_snapshot.as_ref(),
                pre_states: &pre_states,
                contact_margin: self.config.contact_margin,
                restitution_velocity_threshold: self.config.restitution_velocity_threshold,
                ccd_threshold: self.config.ccd_threshold,
                ccd_frame_coverage: self.config.ccd_frame_coverage,
                substeps_per_frame: self.substeps_this_frame,
            };
            let _ccd_count = ccd.run(&mut ctx, dt, static_geometry);
            self.ccd = Some(ccd);
        }
        self.narrowphase_ownership = narrowphase_ownership;

        let all_manifolds = std::mem::take(&mut self.cached_all_manifolds);
        self.sleep_manager
            .update_sleep_states(&mut self.bodies, &all_manifolds, &self.constraints);
        self.cached_all_manifolds = all_manifolds;
    }

    /// Contacts generated in the most recent step.
    pub fn contact_events(&self) -> &[ContactEvent] {
        &self.last_contacts
    }

    /// Normal impulses delivered to each body over the current frame.
    ///
    /// Unlike `contact_events`, these are post-solve, so they describe how much
    /// momentum each contact actually transferred rather than merely that a
    /// contact occurred.
    pub fn impacts(&self) -> &ImpactLedger {
        &self.impacts
    }

    /// Get access to the rigid bodies arena.
    pub fn bodies(&self) -> &Arena<RigidBody> {
        &self.bodies
    }

    /// Get access to the collider arena.
    pub fn colliders_arena(&self) -> &Arena<Collider> {
        &self.colliders
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

    /// Bodies something held up in the most recent step, each with the normal
    /// holding it up.
    ///
    /// The projection of `support_sets` a character reads, plus the carry-over
    /// that keeps a sleeping body standing on the floor it fell asleep on.
    pub fn grounded_bodies(&mut self) -> GroundedBodies {
        let supports = self.support_sets();
        let sleeping = self.sleep_manager.sleeping_snapshot();
        let bodies = &self.bodies;
        self.grounding_detector
            .grounded_bodies(&supports, &sleeping, |handle, point| {
                bodies
                    .get(handle.0)
                    .map(|body| body.velocity_at(point))
                    .unwrap_or_else(Vector3::zeros)
            })
    }

    /// What a body's allowance has conjured, since the world was created.
    ///
    /// `None` for every body that has never spent one — which is every body
    /// without an actuator that declared an allowance.
    pub fn allowance_usage(&self, handle: RigidBodyHandle) -> Option<&AllowanceUsage> {
        self.allowance_ledger.usage(handle)
    }

    /// What a body's drive gain spent, over the most recent frame.
    ///
    /// `None` for every body driving at the honest `μ·N`, which is every body
    /// that has not declared a gain above `1.0`.
    pub fn traction_usage(&self, handle: RigidBodyHandle) -> Option<&TractionUsage> {
        self.traction_ledger.usage(handle)
    }

    /// Which contacts hold each body up, from the most recent step.
    pub fn support_sets(&self) -> SupportSets {
        self.support_resolver
            .resolve(&self.cached_all_manifolds, self.config.gravity_direction())
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

    /// Ray probe that identifies which body was hit.
    /// Skips static bodies and any body in `exclude`.
    pub fn probe_bodies(
        &self,
        origin: Point3<f32>,
        direction: Vector3<f32>,
        length: f32,
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
                let world_xform = collider.world_transform(body_pos, body_rot);
                let hit = match collider.shape() {
                    ColliderShape::Sphere { radius: r } => {
                        ray_vs_sphere(origin, direction, length, center, *r)
                    }
                    ColliderShape::Box { half_extents } => {
                        let obb = Obb::new(center, body_rot, *half_extents);
                        ray_vs_obb(origin, end, &obb, *half_extents)
                    }
                    ColliderShape::Capsule {
                        half_height,
                        radius: cap_radius,
                    } => ray_vs_capsule(
                        origin,
                        direction,
                        length,
                        center,
                        body_rot,
                        *half_height,
                        *cap_radius,
                    ),
                    ColliderShape::ConvexHull { hull } => {
                        ray_vs_convex_hull(origin, direction, length, &world_xform, hull)
                    }
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
    fn raycast(
        &self,
        origin: Point3<f32>,
        direction: Vector3<f32>,
        length: f32,
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
                let world_xform = collider.world_transform(body_pos, body_rot);
                let hit = match collider.shape() {
                    ColliderShape::Sphere { radius: r } => {
                        ray_vs_sphere(origin, direction, length, center, *r)
                    }
                    ColliderShape::Box { half_extents } => {
                        let obb = Obb::new(center, body_rot, *half_extents);
                        ray_vs_obb(origin, end, &obb, *half_extents)
                    }
                    ColliderShape::Capsule {
                        half_height,
                        radius: cap_radius,
                    } => ray_vs_capsule(
                        origin,
                        direction,
                        length,
                        center,
                        body_rot,
                        *half_height,
                        *cap_radius,
                    ),
                    ColliderShape::ConvexHull { hull } => {
                        ray_vs_convex_hull(origin, direction, length, &world_xform, hull)
                    }
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

/// Cast a ray against a sphere. Returns the hit in [0,1] parametric space.
fn ray_vs_sphere(
    origin: Point3<f32>,
    direction: Vector3<f32>,
    length: f32,
    center: Point3<f32>,
    sphere_radius: f32,
) -> Option<ProbeHit> {
    let oc = origin - center;
    let a = direction.dot(&direction);
    let b = 2.0 * oc.dot(&direction);
    let c = oc.dot(&oc) - sphere_radius * sphere_radius;
    let discriminant = b * b - 4.0 * a * c;
    if discriminant < 0.0 {
        return None;
    }
    let t_ray = (-b - discriminant.sqrt()) / (2.0 * a);
    if t_ray < 0.0 || t_ray > length {
        return None;
    }
    let point = origin + direction * t_ray;
    let normal = (point - center).normalize();
    Some(ProbeHit {
        t: t_ray / length,
        point,
        normal,
    })
}

/// Cast a ray against an OBB using a slab test in the OBB's local frame.
fn ray_vs_obb(
    origin: Point3<f32>,
    end: Point3<f32>,
    obb: &Obb,
    half_extents: Vector3<f32>,
) -> Option<ProbeHit> {
    let inv_rot = obb.rotation.inverse();
    let local_start: Vector3<f32> = inv_rot * (origin - obb.center);
    let local_dir: Vector3<f32> = inv_rot * (end - origin);

    let t = obb_slab_entry(local_start, local_dir, half_extents)?;

    let hit_point = origin + (end - origin) * t;
    let closest = obb.closest_point(hit_point);
    let to_ray = hit_point - closest;
    let len = to_ray.norm();
    let normal = if len < 1e-6 {
        // Ray hit exactly on the surface — derive normal from the slab axis.
        let local_hit = inv_rot * (hit_point - obb.center);
        let mut best_axis = 0;
        let mut best_dist = f32::MAX;
        for i in 0..3 {
            let dist = (local_hit[i].abs() - half_extents[i]).abs();
            if dist < best_dist {
                best_dist = dist;
                best_axis = i;
            }
        }
        let mut n = Vector3::zeros();
        n[best_axis] = local_hit[best_axis].signum();
        obb.rotation * n
    } else {
        to_ray / len
    };
    Some(ProbeHit {
        t,
        point: closest,
        normal,
    })
}

/// Cast a ray against a capsule (cylinder + hemisphere caps).
fn ray_vs_capsule(
    origin: Point3<f32>,
    direction: Vector3<f32>,
    length: f32,
    center: Point3<f32>,
    rotation: UnitQuaternion<f32>,
    half_height: f32,
    cap_radius: f32,
) -> Option<ProbeHit> {
    // Transform ray into capsule local space (capsule axis = local Y).
    let inv_rot = rotation.inverse();
    let local_origin = inv_rot * (origin - center);
    let local_dir = inv_rot * direction;

    let cyl_half = half_height - cap_radius;

    // Test against the infinite cylinder (XZ radius).
    let dx = local_dir.x;
    let dz = local_dir.z;
    let ox = local_origin.x;
    let oz = local_origin.z;

    let a = dx * dx + dz * dz;
    let b = 2.0 * (ox * dx + oz * dz);
    let c = ox * ox + oz * oz - cap_radius * cap_radius;

    let mut best_t = f32::MAX;
    let mut best_normal_local = Vector3::zeros();

    // Cylinder body hit.
    if a > 1e-12 {
        let disc = b * b - 4.0 * a * c;
        if disc >= 0.0 {
            let t_cyl = (-b - disc.sqrt()) / (2.0 * a);
            if t_cyl >= 0.0 && t_cyl <= length {
                let y_at_t = local_origin.y + local_dir.y * t_cyl;
                if y_at_t.abs() <= cyl_half {
                    best_t = t_cyl;
                    let p = local_origin + local_dir * t_cyl;
                    best_normal_local = Vector3::new(p.x, 0.0, p.z).normalize();
                }
            }
        }
    }

    // Test both hemisphere caps.
    for &cap_y in &[cyl_half, -cyl_half] {
        let cap_center = Vector3::new(0.0, cap_y, 0.0);
        let oc = local_origin - cap_center;
        let a_s = local_dir.dot(&local_dir);
        let b_s = 2.0 * oc.dot(&local_dir);
        let c_s = oc.dot(&oc) - cap_radius * cap_radius;
        let disc = b_s * b_s - 4.0 * a_s * c_s;
        if disc < 0.0 {
            continue;
        }
        let t_cap = (-b_s - disc.sqrt()) / (2.0 * a_s);
        if t_cap >= 0.0 && t_cap <= length && t_cap < best_t {
            let hit_y = local_origin.y + local_dir.y * t_cap;
            // Ensure hit is on the hemisphere side (not inside the cylinder).
            if (cap_y > 0.0 && hit_y >= cap_y) || (cap_y < 0.0 && hit_y <= cap_y) {
                best_t = t_cap;
                let p = local_origin + local_dir * t_cap;
                best_normal_local = (p - cap_center).normalize();
            }
        }
    }

    if best_t > length {
        return None;
    }

    let point_local = local_origin + local_dir * best_t;
    let point = center + rotation * point_local;
    let normal = rotation * best_normal_local;

    Some(ProbeHit {
        t: best_t / length,
        point,
        normal,
    })
}

/// Cast a ray against a convex hull by testing each face plane.
///
/// Uses a slab-style approach: the ray must be inside all face half-spaces
/// simultaneously. We track the latest entry and earliest exit across all
/// face planes to find the intersection interval.
fn ray_vs_convex_hull(
    origin: Point3<f32>,
    direction: Vector3<f32>,
    length: f32,
    world_xform: &Isometry3<f32>,
    hull: &ConvexHull,
) -> Option<ProbeHit> {
    let inv_rot = world_xform.rotation.inverse();
    let local_origin = inv_rot * (origin - Point3::from(world_xform.translation.vector));
    let local_dir = inv_rot * direction;

    let mut t_enter = 0.0f32;
    let mut t_exit = length;
    let mut enter_normal = Vector3::zeros();

    for face in &hull.faces {
        // Use the first vertex on the face to define the plane.
        let face_point = hull.vertices[face.vertex_indices[0] as usize];
        let denom = face.normal.dot(&local_dir);
        let dist = face.normal.dot(&(face_point - local_origin));

        if denom.abs() < 1e-8 {
            // Ray is parallel to this face plane.
            if dist < 0.0 {
                return None; // Origin is outside this half-space.
            }
            continue;
        }

        let t = dist / denom;
        if denom < 0.0 {
            // Ray entering this half-space.
            if t > t_enter {
                t_enter = t;
                enter_normal = face.normal;
            }
        } else {
            // Ray exiting this half-space.
            if t < t_exit {
                t_exit = t;
            }
        }

        if t_enter > t_exit {
            return None;
        }
    }

    if t_enter < 0.0 || t_enter > length || t_enter > t_exit {
        return None;
    }

    let local_point = local_origin + local_dir * t_enter;
    let point = Point3::from(world_xform.translation.vector) + world_xform.rotation * local_point;
    let normal = world_xform.rotation * enter_normal;

    Some(ProbeHit {
        t: t_enter / length,
        point,
        normal,
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

#[cfg(test)]
mod tests {
    use super::*;

    /// A body placed at rest, which `create_body` starts asleep.
    ///
    /// The collider is not incidental: mass and inertia come from it, and the
    /// body-level impulse methods no-op on a body with neither.
    fn sleeping_body(world: &mut PhysicsWorld) -> RigidBodyHandle {
        let handle = world.create_body(RigidBodyDesc::dynamic().position(Point3::origin()));
        world.attach_collider(handle, ColliderDesc::sphere(0.5).density(1000.0));
        assert!(
            world.is_sleeping(handle),
            "a body placed at rest is expected to start asleep — the rest of \
             this test is meaningless otherwise"
        );
        handle
    }

    #[test]
    fn applying_an_angular_impulse_wakes_the_body() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let handle = sleeping_body(&mut world);

        world.apply_angular_impulse(handle, Vector3::new(0.0, 0.0, 50.0));

        assert!(
            !world.is_sleeping(handle),
            "a driven body must wake, or it accumulates spin the integrator ignores"
        );
        assert!(world.body(handle).unwrap().angular_velocity().magnitude() > 0.0);
    }

    #[test]
    fn applying_a_linear_impulse_wakes_the_body() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let handle = sleeping_body(&mut world);

        world.apply_impulse(handle, Vector3::new(0.0, 0.0, 50.0));

        assert!(!world.is_sleeping(handle));
    }

    #[test]
    fn impulses_to_a_removed_body_report_failure_rather_than_panicking() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let handle = sleeping_body(&mut world);
        world.remove_body(handle);

        assert!(!world.apply_impulse(handle, Vector3::z()));
        assert!(!world.apply_angular_impulse(handle, Vector3::z()));
    }

    /// The number of motor rows a medium drive on `handle` currently owns.
    fn medium_rows(world: &PhysicsWorld, handle: RigidBodyHandle) -> usize {
        world
            .body(handle)
            .unwrap()
            .medium_drive()
            .and_then(|c| world.constraint(c))
            .map_or(0, |c| c.kind.row_count())
    }

    #[test]
    fn a_medium_anchor_puts_its_command_in_the_constraint_arena() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let handle = sleeping_body(&mut world);

        assert!(world.set_body_drive(
            handle,
            &DriveCommand::medium(Vector3::y() * 2.0, Vector3::zeros(), 40.0, 0.0),
        ));

        assert_eq!(medium_rows(&world, handle), 6);
        assert!(
            !world.is_sleeping(handle),
            "a driven body must wake, or its rows are never solved"
        );
    }

    #[test]
    fn a_second_command_updates_the_rows_rather_than_growing_them() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let handle = sleeping_body(&mut world);

        world.set_body_drive(
            handle,
            &DriveCommand::medium(Vector3::y() * 2.0, Vector3::zeros(), 40.0, 0.0),
        );
        let first = world.body(handle).unwrap().medium_drive().unwrap();
        world.set_body_drive(
            handle,
            &DriveCommand::medium(Vector3::y() * -2.0, Vector3::zeros(), 40.0, 0.0),
        );

        assert_eq!(
            world.body(handle).unwrap().medium_drive(),
            Some(first),
            "the rows are updated in place so their warm-start impulses survive"
        );
        assert_eq!(world.constraints.len(), 1);
        assert!(matches!(
            world.constraint(first).unwrap().kind,
            ConstraintKind::MediumDrive { linear_target, .. } if linear_target.y == -2.0
        ));
    }

    /// The exclusivity the whole stage turns on: a body carrying both a support
    /// drive and medium rows toward the same target would get roughly twice the
    /// authority its actuator declares, and the rows' bounds would stop meaning
    /// anything. `BodyDrive` makes that unrepresentable; this is the check that
    /// the world's entry point maintains it in both directions.
    #[test]
    fn a_body_may_hold_only_one_drive_at_a_time() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let handle = sleeping_body(&mut world);

        world.set_body_drive(
            handle,
            &DriveCommand::medium(Vector3::y() * 2.0, Vector3::zeros(), 40.0, 0.0),
        );
        world.set_body_drive(
            handle,
            &DriveCommand::support(Vector3::x() * 5.0, Vector3::zeros(), 40.0, 0.0),
        );

        assert_eq!(
            medium_rows(&world, handle),
            0,
            "taking the support anchor must retire the motor rows"
        );
        assert!(
            world.constraints.is_empty(),
            "and retire them from the arena, not merely forget them"
        );

        world.set_body_drive(
            handle,
            &DriveCommand::medium(Vector3::y() * 2.0, Vector3::zeros(), 40.0, 0.0),
        );
        assert_eq!(medium_rows(&world, handle), 6);
    }

    #[test]
    fn removing_a_body_takes_its_motor_rows_with_it() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let handle = sleeping_body(&mut world);

        world.set_body_drive(
            handle,
            &DriveCommand::medium(Vector3::y() * 2.0, Vector3::zeros(), 40.0, 0.0),
        );
        world.remove_body(handle);

        assert!(world.constraints.is_empty());
    }

    #[test]
    fn a_static_body_takes_no_drive_at_all() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let handle = world.create_body(RigidBodyDesc::static_body().position(Point3::origin()));

        assert!(!world.set_body_drive(
            handle,
            &DriveCommand::medium(Vector3::y(), Vector3::zeros(), 40.0, 0.0),
        ));
        assert!(world.constraints.is_empty());
    }
}
