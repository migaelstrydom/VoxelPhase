//! Physics world containing all simulation state.

use generational_arena::Arena;
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use smallvec::{smallvec, SmallVec};
use std::collections::{HashMap, HashSet};

use super::body::{RigidBody, RigidBodyDesc};
use super::collider::{Collider, ColliderDesc, ColliderMaterial, ColliderShape};
use super::debug::{PhysicsDebugConfig, PhysicsDebugger};
use super::grounding::{GroundingConfig, GroundingDetector};
use super::handle::{ColliderHandle, RigidBodyHandle};
use super::impulses::{ForceField, ForceFieldRegistry, PhysicsImpulse};
use super::narrowphase::{
    generate_dynamic_contacts, generate_static_contacts, NarrowphaseWorkBuffer, SatCacheMap,
};
use super::pipeline::integration::{integrate_bodies, integrate_forces};
use super::pipeline::manifold::ManifoldCache;
use super::pipeline::normal_smoothing::NormalSmoothingConfig;
use super::pipeline::pair::{PairHeader, SolverContact, SolverManifold};
use super::pipeline::post_stabilizer::PostStabiliseConfig;
use super::pipeline::solver::{solve, solve_contacts};
use super::sleep::{SleepManager, SleepManagerConfig};
use super::static_geometry::StaticGeometry;
use crate::collision::contact::FeatureId;
use crate::collision::continuous::swept_sphere_sphere;
use crate::collision::mesh::obb_patch::obb_patch_manifold;
use crate::collision::mesh::seam_filter::filter_patch;
use crate::collision::obb::Obb;
use crate::debug::DebugLines;
use crate::sensing::{ProbeHit, ProbeTarget};

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
    /// Frames without a narrowphase refresh before a manifold point is pruned.
    pub manifold_max_age: u8,
    /// Scale factor applied to warm-start impulses (0..=1).
    pub warm_start_scale: f32,
    /// When true, sort manifold output contacts for deterministic solver ordering.
    ///
    /// This is primarily intended for reproducible tests and diagnostics.
    pub deterministic_contact_ordering: bool,
    /// Split-impulse configuration for post-stabilization.
    pub post_stabilise: PostStabiliseConfig,
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
    /// Debug rendering configuration.
    pub debug: PhysicsDebugConfig,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            gravity: Vector3::new(0.0, -9.81, 0.0),
            solver_iterations: 3,
            restitution_velocity_threshold: 0.3,
            contact_margin: 0.02,
            ccd_threshold: 0.5,
            manifold_max_age: 3,
            warm_start_scale: 0.6,
            deterministic_contact_ordering: false,
            post_stabilise: PostStabiliseConfig::default(),
            normal_smoothing: NormalSmoothingConfig::default(),
            grounding: GroundingConfig::default(),
            warm_start_depth_slop: 0.02,
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
    force_fields: ForceFieldRegistry,
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
            force_fields: ForceFieldRegistry::default(),
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

    /// Register a persistent force field. Returns its index for later removal.
    pub fn add_force_field(&mut self, field: ForceField) -> usize {
        self.force_fields.add(field)
    }

    /// Apply one-shot impulses and persistent force fields to all dynamic bodies.
    fn apply_impulses(&mut self, impulses: &[PhysicsImpulse], dt: f32) {
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

            for field in self.force_fields.iter() {
                if let Some(v) = field.impulse_at(pos, dt) {
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
    ///
    /// Only drives horizontal (X/Z) axes. Vertical velocity is set directly
    /// for jumps, with gravity handling the rest.
    pub fn set_body_velocity_drive(
        &mut self,
        handle: RigidBodyHandle,
        linear: Vector3<f32>,
        angular: Vector3<f32>,
        max_accel: f32,
    ) -> bool {
        let Some(body) = self.bodies.get_mut(handle.0) else {
            return false;
        };
        if body.is_static() {
            return false;
        }
        body.set_velocity_drive(linear, max_accel);
        body.set_linear_velocity_y(linear.y);
        body.set_angular_velocity(angular);
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
        self.apply_impulses(impulses, dt);

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
    pub fn substep(&mut self, dt: f32, static_geometry: &dyn StaticGeometry) {
        let sleeping_snapshot = if self.config.sleep.enabled {
            Some(self.sleep_manager.sleeping_snapshot())
        } else {
            None
        };

        // Integrate forces (gravity) into velocities
        integrate_forces(
            &mut self.bodies,
            dt,
            self.config.gravity,
            sleeping_snapshot.as_ref(),
        );

        // Solve velocity constraints
        solve(
            &mut self.bodies,
            &mut self.cached_active_manifolds,
            &self.config,
            dt,
        );
        self.debugger
            .update_post_solve(&self.bodies, &self.cached_active_manifolds);

        // Write solved impulses back to manifold cache
        self.manifold_cache
            .write_back(&self.cached_active_manifolds);
        self.manifold_cache.prune();

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
        let _ccd_count = self.ccd_pass(
            dt,
            static_geometry,
            &pre_states,
            &narrowphase_handled,
            sleeping_snapshot.as_ref(),
        );
        self.cached_narrowphase_handled = narrowphase_handled;

        let all_manifolds = std::mem::take(&mut self.cached_all_manifolds);
        self.sleep_manager
            .update_sleep_states(&mut self.bodies, &all_manifolds);
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

            let ccd_header = PairHeader {
                body_a: None,
                body_b: candidate.body_handle,
                collider_a: None,
                collider_b: None,
                restitution: candidate.material.restitution,
                friction: candidate.material.friction,
            };

            let ccd_contacts: SmallVec<[SolverContact; 4]> = match &candidate.shape {
                ColliderShape::Sphere { .. } => {
                    smallvec![cold_solver_contact(
                        hit.point,
                        hit.normal,
                        hit.normal,
                        0.0,
                        0.0,
                        FeatureId::SINGLE,
                    )]
                }
                ColliderShape::Box { half_extents } => self.box_ccd_solver_contacts(
                    candidate,
                    *half_extents,
                    hit_pos,
                    hit_rot,
                    static_geometry,
                ),
                ColliderShape::Capsule { .. } => {
                    // Capsule CCD: single contact at sweep hit point (same as sphere).
                    smallvec![cold_solver_contact(
                        hit.point,
                        hit.normal,
                        hit.normal,
                        0.0,
                        0.0,
                        FeatureId::SINGLE,
                    )]
                }
            };

            let mut ccd_manifold = SolverManifold {
                header: ccd_header,
                contacts: ccd_contacts,
            };

            for contact in &ccd_manifold.contacts {
                self.last_contacts.push(ContactEvent::from_solver(
                    &ccd_manifold.header,
                    contact,
                    ContactSource::Ccd,
                ));
            }
            solve_contacts(
                &mut self.bodies,
                std::slice::from_mut(&mut ccd_manifold),
                self.config.restitution_velocity_threshold,
            );

            corrections += 1;
        }

        corrections
    }

    /// Generate precise box-terrain contacts at a CCD hit position.
    ///
    /// Bounding-sphere sweep found the approximate hit. Now build an OBB at
    /// the hit position (using the pre-integration rotation) and run the
    /// mesh-aware manifold pipeline for accurate contact normals.
    fn box_ccd_solver_contacts(
        &self,
        candidate: &CcdCandidate,
        half_extents: Vector3<f32>,
        hit_pos: Point3<f32>,
        rotation: UnitQuaternion<f32>,
        static_geometry: &dyn StaticGeometry,
    ) -> SmallVec<[SolverContact; 4]> {
        let obb = Obb::new(hit_pos, rotation, half_extents);
        let (aabb_min, aabb_max) = obb.enclosing_aabb();
        let margin = Vector3::new(
            self.config.contact_margin,
            self.config.contact_margin,
            self.config.contact_margin,
        );
        let query = crate::collision::AABB::new(aabb_min - margin, aabb_max + margin);
        let patch = static_geometry.query_region(&query);
        let filtered = filter_patch(&patch, 0.98);
        let manifold = obb_patch_manifold(&obb, &filtered, self.config.contact_margin);

        let mut contacts: SmallVec<[SolverContact; 4]> = manifold
            .points
            .into_iter()
            .map(|cp| {
                cold_solver_contact(cp.point, cp.normal, cp.raw_normal, 0.0, 0.0, cp.feature_id)
            })
            .collect();

        if contacts.is_empty() {
            // Fallback: use the sweep hit directly
            if let Some(hit) = sweep_sphere_against_static(
                candidate.pre_center,
                candidate.post_center,
                candidate.radius,
                static_geometry,
            ) {
                contacts.push(cold_solver_contact(
                    hit.point,
                    hit.normal,
                    hit.normal,
                    0.0,
                    0.0,
                    FeatureId::SINGLE,
                ));
            }
        }

        contacts
    }
}

/// Build a cold (no warm-start) `SolverContact` for transient CCD contacts.
fn cold_solver_contact(
    point: Point3<f32>,
    normal: Vector3<f32>,
    raw_normal: Vector3<f32>,
    depth: f32,
    raw_depth: f32,
    feature_id: FeatureId,
) -> SolverContact {
    SolverContact {
        point,
        normal,
        raw_normal,
        depth,
        raw_depth,
        feature_id,
        warm_normal_impulse: 0.0,
        warm_friction_impulse_ws: Vector3::zeros(),
        accumulated_normal_impulse: 0.0,
        accumulated_friction_impulse_ws: Vector3::zeros(),
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
) -> Option<crate::collision::continuous::SweptContact> {
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

    let mut earliest: Option<crate::collision::continuous::SweptContact> = None;
    for pt in &patch.triangles {
        if let Some(contact) =
            crate::collision::continuous::swept_sphere_triangle(start, end, radius, &pt.triangle)
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
    fn from_solver(header: &PairHeader, contact: &SolverContact, source: ContactSource) -> Self {
        Self {
            body_a: header.body_a,
            body_b: header.body_b,
            point: contact.point,
            normal: contact.normal,
            raw_normal: contact.raw_normal,
            depth: contact.depth,
            source,
        }
    }
}

/// Source of the contact event in the pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContactSource {
    Narrowphase,
    Ccd,
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
                    ColliderShape::Capsule { half_height, .. } => {
                        // Treat capsule as a sphere with bounding radius for probe.
                        probe_vs_sphere(origin, end, radius, center, *half_height)
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
#[cfg(test)]
use super::pipeline::manifold::ManifoldFrameStats;
#[cfg(test)]
impl PhysicsWorld {
    pub fn manifold_frame_stats(&self) -> ManifoldFrameStats {
        self.manifold_cache.frame_stats()
    }
}
