//! Physics world containing all simulation state.

use generational_arena::Arena;
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use std::collections::{HashMap, HashSet};

use super::body::{RigidBody, RigidBodyDesc};
use super::collider::{Collider, ColliderDesc, ColliderMaterial, ColliderShape};
use super::collision::obb::Obb;
use super::collision::obb_triangle::obb_triangle_contacts;
use super::grounding::{GroundingConfig, GroundingDetector};
use super::handle::{ColliderHandle, RigidBodyHandle};
use super::narrowphase::{
    generate_dynamic_contacts, generate_static_contacts, NormalClusterConfig,
};
use super::pipeline::integration::{integrate_bodies, integrate_forces};
use super::pipeline::manifold::ManifoldCache;
use super::pipeline::normal_smoothing::NormalSmoothingConfig;
use super::pipeline::post_stabilizer::PostStabiliseConfig;
use super::pipeline::solver::{solve, solve_contacts, ContactConstraint};
use super::sleep::SleepManager;
use super::static_geometry::StaticGeometry;
use crate::debug::{DebugLines, DebugLog};

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
    /// Draw contact points and normals as debug overlays.
    pub debug_draw_contacts: bool,
    /// Draw raw (pre-smoothed) contact normals for comparison.
    pub debug_draw_contact_raw_normals: bool,
    /// Draw sleep state markers over sleeping bodies.
    pub debug_draw_sleeping: bool,
    /// Enable speculative contacts to close the CCD activation gap.
    pub enable_speculative_contacts: bool,
    /// Minimum linear speed required for speculative contact generation.
    pub speculative_min_speed: f32,
    /// Multiplier for contact_margin when gating speculative contacts.
    pub speculative_margin_multiplier: f32,
    /// Enable sleeping for dynamic bodies.
    pub enable_sleeping: bool,
    /// Kinetic energy threshold below which a body is a sleep candidate.
    pub sleep_threshold: f32,
    /// Frames a body must remain below the threshold before sleeping.
    pub sleep_delay_frames: u32,
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
            debug_draw_contacts: true,
            debug_draw_contact_raw_normals: false,
            debug_draw_sleeping: true,
            enable_speculative_contacts: true,
            speculative_min_speed: 1.0,
            speculative_margin_multiplier: 2.0,
            enable_sleeping: true,
            sleep_threshold: 0.1,
            sleep_delay_frames: 30,
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
    last_contact_debug: Option<ContactDebugSnapshot>,
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
        let mut sleep_manager =
            SleepManager::new(config.sleep_threshold, config.sleep_delay_frames);
        sleep_manager.set_enabled(config.enable_sleeping);
        let grounding_detector = GroundingDetector::new(config.grounding);
        Self {
            config,
            bodies: Arena::new(),
            colliders: Arena::new(),
            manifold_cache,
            last_contacts: Vec::new(),
            last_contact_debug: None,
            frame_index: 0,
            sleep_manager,
            grounding_detector,
        }
    }

    /// Get the physics configuration.
    pub fn config(&self) -> &PhysicsConfig {
        &self.config
    }

    pub fn sleeping_bodies(&self) -> Vec<RigidBodyHandle> {
        if !self.config.enable_sleeping {
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
        self.sleep_manager
            .set_thresholds(self.config.sleep_threshold, self.config.sleep_delay_frames);
        self.sleep_manager.set_enabled(self.config.enable_sleeping);
        self.sleep_manager.sync_bodies(&self.bodies);
        self.sleep_manager.apply_wake_events(&[], &self.bodies);
        let sleeping_snapshot = if self.config.enable_sleeping {
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
        self.last_contact_debug = Some(ContactDebugSnapshot::new(
            &self.bodies,
            &raw_contacts,
            &contacts,
            &active_contacts,
            &self.last_contacts,
        ));

        // Phase 5: Solve velocity constraints (warm-start + N iterations)
        let solved = solve(&mut self.bodies, &active_contacts, &self.config, dt);
        if let Some(snapshot) = self.last_contact_debug.as_mut() {
            if let Some(primary) = snapshot.primary_body.as_ref() {
                snapshot.post_solve_body =
                    compute_post_solve_body_stats(&self.bodies, &active_contacts, primary.handle);
            }
        }

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
        let sleeping_snapshot = if self.config.enable_sleeping {
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
        let sleeping_snapshot = if self.config.enable_sleeping {
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

    /// Write debug statistics to the debug log resource.
    ///
    /// Should be called every frame by the physics system. The output will
    /// only be printed to stdout when F3 is pressed.
    pub fn write_debug_log(&self, debug_log: &mut DebugLog) {
        let Some(snapshot) = &self.last_contact_debug else {
            debug_log.add("Physics/Status", "No data yet");
            return;
        };

        debug_log.add("Physics/Events/Total", snapshot.events.total.to_string());
        debug_log.add("Physics/Events/CCD", snapshot.events.ccd.to_string());
        debug_log.add(
            "Physics/Events/Narrowphase",
            snapshot.events.narrow.to_string(),
        );

        debug_log.add("Physics/Raw/Total", snapshot.raw.total.to_string());
        debug_log.add("Physics/Raw/Static", snapshot.raw.static_count.to_string());
        debug_log.add("Physics/Raw/NegRaw", snapshot.raw.neg_raw.to_string());
        debug_log.add("Physics/Raw/WarmUsed", snapshot.raw.warm_used.to_string());
        debug_log.add(
            "Physics/Raw/DepthRange",
            format!("[{:.5}, {:.5}]", snapshot.raw.min_raw, snapshot.raw.max_raw),
        );

        debug_log.add("Physics/Merged/Total", snapshot.merged.total.to_string());
        debug_log.add(
            "Physics/Merged/Static",
            snapshot.merged.static_count.to_string(),
        );
        debug_log.add(
            "Physics/Merged/WarmUsed",
            snapshot.merged.warm_used.to_string(),
        );

        debug_log.add("Physics/Active/Total", snapshot.active.total.to_string());
        debug_log.add(
            "Physics/Active/Static",
            snapshot.active.static_count.to_string(),
        );
        debug_log.add(
            "Physics/Active/WarmUsed",
            snapshot.active.warm_used.to_string(),
        );
        debug_log.add(
            "Physics/Config/ContactMargin",
            format!("{:.5}", self.config.contact_margin),
        );
        debug_log.add(
            "Physics/Config/WarmStartDepthSlop",
            format!("{:.5}", self.config.warm_start_depth_slop),
        );

        if let Some(body_stats) = &snapshot.primary_body {
            debug_log.add(
                "Physics/Body/Top/Handle",
                format!("{:?}", body_stats.handle),
            );
            debug_log.add(
                "Physics/Body/Top/Counts",
                format!(
                    "total={} static={}",
                    body_stats.total, body_stats.static_count
                ),
            );
            debug_log.add(
                "Physics/Body/Top/Depth",
                format!(
                    "min={:.5} max={:.5} avg={:.5}",
                    body_stats.min_depth, body_stats.max_depth, body_stats.avg_depth
                ),
            );
            debug_log.add(
                "Physics/Body/Top/RelN",
                format!(
                    "min={:.5} max={:.5} avg={:.5}",
                    body_stats.rel_n_min, body_stats.rel_n_max, body_stats.rel_n_avg
                ),
            );
            debug_log.add(
                "Physics/Body/Top/Normal",
                format!(
                    "avg=({:.3},{:.3},{:.3}) avgLen={:.3} minDot={:.3}",
                    body_stats.normal_avg.x,
                    body_stats.normal_avg.y,
                    body_stats.normal_avg.z,
                    body_stats.normal_avg_len,
                    body_stats.normal_min_dot
                ),
            );
            debug_log.add(
                "Physics/Body/Top/Speed",
                format!(
                    "linear={:.5} angular={:.5}",
                    body_stats.linear_speed, body_stats.angular_speed
                ),
            );
        } else {
            debug_log.add("Physics/Body/Top/Handle", "None");
        }

        if let Some(post) = &snapshot.post_solve_body {
            debug_log.add(
                "Physics/Body/Top/PostSolveRelN",
                format!(
                    "min={:.5} max={:.5} avg={:.5}",
                    post.rel_n_min, post.rel_n_max, post.rel_n_avg
                ),
            );
            debug_log.add(
                "Physics/Body/Top/PostSolveSpeed",
                format!(
                    "linear={:.5} angular={:.5}",
                    post.linear_speed, post.angular_speed
                ),
            );
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
            let Some(hit) = static_geometry.sweep_sphere(
                candidate.pre_center,
                candidate.post_center,
                candidate.radius,
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
        let triangles = static_geometry.query_triangles(&query);

        let mut contacts = Vec::new();
        for tri in &triangles {
            for c in obb_triangle_contacts(&obb, tri) {
                contacts.push(ContactConstraint {
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
                });
            }
        }

        if contacts.is_empty() {
            // Fallback: use the sweep hit directly
            if let Some(hit) = static_geometry.sweep_sphere(
                candidate.pre_center,
                candidate.post_center,
                candidate.radius,
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

fn relative_normal_velocity(bodies: &Arena<RigidBody>, contact: &ContactConstraint) -> Option<f32> {
    let body_b = bodies.get(contact.body_b.0)?;
    let (pos_a, vel_a, ang_a) = match contact.body_a {
        Some(handle) => {
            let body_a = bodies.get(handle.0)?;
            (
                body_a.position(),
                body_a.linear_velocity(),
                body_a.angular_velocity(),
            )
        }
        None => (contact.point, Vector3::zeros(), Vector3::zeros()),
    };
    let r_a = contact.point - pos_a;
    let r_b = contact.point - body_b.position();
    let vel_at_a = vel_a + ang_a.cross(&r_a);
    let vel_at_b = body_b.linear_velocity() + body_b.angular_velocity().cross(&r_b);
    let rel_vel = vel_at_b - vel_at_a;
    Some(rel_vel.dot(&contact.normal))
}

#[derive(Debug, Clone)]
struct ContactSample {
    depth: f32,
    raw_depth: f32,
    rel_n: f32,
    normal_dot_up: f32,
    warm_used: bool,
    point: Point3<f32>,
    normal: Vector3<f32>,
}

#[derive(Debug, Clone)]
struct ContactDebugStats {
    total: usize,
    static_count: usize,
    neg_raw: usize,
    steep_margin: usize,
    warm_used: usize,
    min_raw: f32,
    max_raw: f32,
    min_depth: f32,
    max_depth: f32,
    rel_n_min: f32,
    rel_n_max: f32,
    rel_n_avg: f32,
    samples: Vec<ContactSample>,
}

#[derive(Debug, Clone)]
struct BodyContactStats {
    handle: RigidBodyHandle,
    total: usize,
    static_count: usize,
    min_depth: f32,
    max_depth: f32,
    avg_depth: f32,
    rel_n_min: f32,
    rel_n_max: f32,
    rel_n_avg: f32,
    normal_avg: Vector3<f32>,
    normal_avg_len: f32,
    normal_min_dot: f32,
    linear_speed: f32,
    angular_speed: f32,
}

#[derive(Debug, Clone)]
struct BodyPostSolveStats {
    handle: RigidBodyHandle,
    rel_n_min: f32,
    rel_n_max: f32,
    rel_n_avg: f32,
    linear_speed: f32,
    angular_speed: f32,
}

#[derive(Clone)]
struct BodyContactAccum {
    total: usize,
    static_count: usize,
    depth_sum: f32,
    min_depth: f32,
    max_depth: f32,
    rel_n_sum: f32,
    rel_n_min: f32,
    rel_n_max: f32,
    rel_n_count: usize,
    normal_sum: Vector3<f32>,
}

impl BodyContactAccum {
    fn new() -> Self {
        Self {
            total: 0,
            static_count: 0,
            depth_sum: 0.0,
            min_depth: f32::INFINITY,
            max_depth: f32::NEG_INFINITY,
            rel_n_sum: 0.0,
            rel_n_min: f32::INFINITY,
            rel_n_max: f32::NEG_INFINITY,
            rel_n_count: 0,
            normal_sum: Vector3::zeros(),
        }
    }

    fn finish(
        self,
    ) -> (
        usize,
        usize,
        f32,
        f32,
        f32,
        f32,
        f32,
        f32,
        Vector3<f32>,
        f32,
    ) {
        let avg_depth = if self.total > 0 {
            self.depth_sum / self.total as f32
        } else {
            0.0
        };
        let rel_n_avg = if self.rel_n_count > 0 {
            self.rel_n_sum / self.rel_n_count as f32
        } else {
            0.0
        };
        let normal_avg_len = if self.total > 0 {
            self.normal_sum.magnitude() / self.total as f32
        } else {
            0.0
        };
        (
            self.total,
            self.static_count,
            self.min_depth,
            self.max_depth,
            avg_depth,
            self.rel_n_min,
            self.rel_n_max,
            rel_n_avg,
            self.normal_sum,
            normal_avg_len,
        )
    }
}

impl ContactDebugStats {
    fn empty() -> Self {
        Self {
            total: 0,
            static_count: 0,
            neg_raw: 0,
            steep_margin: 0,
            warm_used: 0,
            min_raw: 0.0,
            max_raw: 0.0,
            min_depth: 0.0,
            max_depth: 0.0,
            rel_n_min: 0.0,
            rel_n_max: 0.0,
            rel_n_avg: 0.0,
            samples: Vec::new(),
        }
    }

    fn log(&self, label: &str) {
        log::info!(
            "Contacts/{} Total={} Static={} NegRaw={} SteepMargin={} WarmUsed={}",
            label,
            self.total,
            self.static_count,
            self.neg_raw,
            self.steep_margin,
            self.warm_used
        );
        log::info!(
            "Contacts/{} RawDepth[min,max]=[{:.5},{:.5}] Depth[min,max]=[{:.5},{:.5}]",
            label,
            self.min_raw,
            self.max_raw,
            self.min_depth,
            self.max_depth
        );
        log::info!(
            "Contacts/{} RelN[min,max,avg]=[{:.5},{:.5},{:.5}]",
            label,
            self.rel_n_min,
            self.rel_n_max,
            self.rel_n_avg
        );
        for (i, sample) in self.samples.iter().enumerate() {
            log::info!(
                "Contacts/{} S{} depth={:.5} raw={:.5} relN={:.5} upDot={:.3} warm={} p=({:.3},{:.3},{:.3}) n=({:.3},{:.3},{:.3})",
                label,
                i,
                sample.depth,
                sample.raw_depth,
                sample.rel_n,
                sample.normal_dot_up,
                sample.warm_used as u8,
                sample.point.x,
                sample.point.y,
                sample.point.z,
                sample.normal.x,
                sample.normal.y,
                sample.normal.z
            );
        }
    }
}

#[derive(Debug, Clone)]
struct ContactEventStats {
    total: usize,
    ccd: usize,
    narrow: usize,
}

impl ContactEventStats {
    fn from_events(events: &[ContactEvent]) -> Self {
        let total = events.len();
        let ccd = events
            .iter()
            .filter(|e| e.source == ContactSource::Ccd)
            .count();
        let narrow = events
            .iter()
            .filter(|e| e.source == ContactSource::Narrowphase)
            .count();
        Self { total, ccd, narrow }
    }

    fn log(&self) {
        log::info!(
            "Contacts/Events Total={} CCD={} Narrowphase={}",
            self.total,
            self.ccd,
            self.narrow
        );
    }
}

#[derive(Debug, Clone)]
struct ContactDebugSnapshot {
    raw: ContactDebugStats,
    merged: ContactDebugStats,
    active: ContactDebugStats,
    events: ContactEventStats,
    primary_body: Option<BodyContactStats>,
    post_solve_body: Option<BodyPostSolveStats>,
}

impl ContactDebugSnapshot {
    fn new(
        bodies: &Arena<RigidBody>,
        raw: &[ContactConstraint],
        merged: &[ContactConstraint],
        active: &[ContactConstraint],
        events: &[ContactEvent],
    ) -> Self {
        Self {
            raw: compute_contact_stats(bodies, raw),
            merged: compute_contact_stats(bodies, merged),
            active: compute_contact_stats(bodies, active),
            events: ContactEventStats::from_events(events),
            primary_body: compute_primary_body_stats(bodies, active),
            post_solve_body: None,
        }
    }

    fn log(&self) {
        self.events.log();
        self.raw.log("Raw");
        self.merged.log("Merged");
        self.active.log("Active");
    }
}

fn compute_contact_stats(
    bodies: &Arena<RigidBody>,
    contacts: &[ContactConstraint],
) -> ContactDebugStats {
    if contacts.is_empty() {
        return ContactDebugStats::empty();
    }

    let total = contacts.len();
    let static_count = contacts.iter().filter(|c| c.body_a.is_none()).count();
    let neg_raw = contacts.iter().filter(|c| c.raw_depth < 0.0).count();
    let warm_used = contacts
        .iter()
        .filter(|c| c.warm_normal_impulse.abs() > 1e-6)
        .count();
    let up = Vector3::y();
    let steep_margin = contacts
        .iter()
        .filter(|c| c.raw_depth < 0.0 && c.normal.dot(&up).abs() < 0.9)
        .count();

    let mut min_raw = f32::INFINITY;
    let mut max_raw = f32::NEG_INFINITY;
    let mut min_depth = f32::INFINITY;
    let mut max_depth = f32::NEG_INFINITY;
    let mut rel_n_min = f32::INFINITY;
    let mut rel_n_max = f32::NEG_INFINITY;
    let mut rel_n_sum = 0.0;
    let mut rel_n_count = 0usize;

    for c in contacts {
        min_raw = min_raw.min(c.raw_depth);
        max_raw = max_raw.max(c.raw_depth);
        min_depth = min_depth.min(c.depth);
        max_depth = max_depth.max(c.depth);
        if let Some(rel_n) = relative_normal_velocity(bodies, c) {
            rel_n_min = rel_n_min.min(rel_n);
            rel_n_max = rel_n_max.max(rel_n);
            rel_n_sum += rel_n;
            rel_n_count += 1;
        }
    }

    let rel_n_avg = if rel_n_count > 0 {
        rel_n_sum / rel_n_count as f32
    } else {
        0.0
    };

    let mut indices: Vec<usize> = (0..contacts.len()).collect();
    indices.sort_by(|a, b| {
        contacts[*b]
            .depth
            .partial_cmp(&contacts[*a].depth)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut samples = Vec::new();
    for idx in indices.into_iter().take(3) {
        let c = &contacts[idx];
        let rel_n = relative_normal_velocity(bodies, c).unwrap_or(0.0);
        let normal_dot_up = c.normal.dot(&up);
        samples.push(ContactSample {
            depth: c.depth,
            raw_depth: c.raw_depth,
            rel_n,
            normal_dot_up,
            warm_used: c.warm_normal_impulse.abs() > 1e-6,
            point: c.point,
            normal: c.normal,
        });
    }

    ContactDebugStats {
        total,
        static_count,
        neg_raw,
        steep_margin,
        warm_used,
        min_raw,
        max_raw,
        min_depth,
        max_depth,
        rel_n_min,
        rel_n_max,
        rel_n_avg,
        samples,
    }
}

fn compute_primary_body_stats(
    bodies: &Arena<RigidBody>,
    contacts: &[ContactConstraint],
) -> Option<BodyContactStats> {
    if contacts.is_empty() {
        return None;
    }

    let mut per_body: HashMap<RigidBodyHandle, BodyContactAccum> = HashMap::new();
    for c in contacts {
        let entry = per_body
            .entry(c.body_b)
            .or_insert_with(BodyContactAccum::new);
        entry.total += 1;
        if c.body_a.is_none() {
            entry.static_count += 1;
        }
        entry.depth_sum += c.depth;
        entry.min_depth = entry.min_depth.min(c.depth);
        entry.max_depth = entry.max_depth.max(c.depth);
        if let Some(rel_n) = relative_normal_velocity(bodies, c) {
            entry.rel_n_sum += rel_n;
            entry.rel_n_min = entry.rel_n_min.min(rel_n);
            entry.rel_n_max = entry.rel_n_max.max(rel_n);
            entry.rel_n_count += 1;
        }
        entry.normal_sum += c.normal;
    }

    let (&handle, accum) = per_body.iter().max_by_key(|(_, stats)| stats.total)?;
    let (
        total,
        static_count,
        min_depth,
        max_depth,
        avg_depth,
        rel_n_min,
        rel_n_max,
        rel_n_avg,
        normal_sum,
        normal_avg_len,
    ) = accum.clone().finish();

    let normal_avg = if normal_sum.magnitude() > 1e-6 {
        normal_sum.normalize()
    } else {
        Vector3::zeros()
    };

    let mut normal_min_dot = 1.0f32;
    if normal_avg.magnitude() > 1e-6 {
        for c in contacts.iter().filter(|c| c.body_b == handle) {
            normal_min_dot = normal_min_dot.min(c.normal.dot(&normal_avg));
        }
    } else {
        normal_min_dot = 0.0;
    }

    let body = bodies.get(handle.0)?;
    let linear_speed = body.linear_velocity().magnitude();
    let angular_speed = body.angular_velocity().magnitude();

    Some(BodyContactStats {
        handle,
        total,
        static_count,
        min_depth,
        max_depth,
        avg_depth,
        rel_n_min,
        rel_n_max,
        rel_n_avg,
        normal_avg,
        normal_avg_len,
        normal_min_dot,
        linear_speed,
        angular_speed,
    })
}

fn compute_post_solve_body_stats(
    bodies: &Arena<RigidBody>,
    contacts: &[ContactConstraint],
    handle: RigidBodyHandle,
) -> Option<BodyPostSolveStats> {
    let mut rel_n_min = f32::INFINITY;
    let mut rel_n_max = f32::NEG_INFINITY;
    let mut rel_n_sum = 0.0;
    let mut rel_n_count = 0usize;

    for c in contacts.iter().filter(|c| c.body_b == handle) {
        if let Some(rel_n) = relative_normal_velocity(bodies, c) {
            rel_n_min = rel_n_min.min(rel_n);
            rel_n_max = rel_n_max.max(rel_n);
            rel_n_sum += rel_n;
            rel_n_count += 1;
        }
    }

    if rel_n_count == 0 {
        return None;
    }

    let rel_n_avg = rel_n_sum / rel_n_count as f32;
    let body = bodies.get(handle.0)?;
    let linear_speed = body.linear_velocity().magnitude();
    let angular_speed = body.angular_velocity().magnitude();

    Some(BodyPostSolveStats {
        handle,
        rel_n_min,
        rel_n_max,
        rel_n_avg,
        linear_speed,
        angular_speed,
    })
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
