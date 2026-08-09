//! Physics synchronization system.
//!
//! This system is the single point of integration between the physics engine
//! and the ECS world. It:
//! 1. Steps the physics simulation
//! 2. Syncs physics state back to ECS components

use nalgebra::{Point3, Vector3};
use rustc_hash::FxHashSet;
use specs::{Join, Read, ReadStorage, System, Write, WriteStorage};

use crate::animation::CharacterAnimator;
use crate::components::{Orientation, Position, RigidBodyComponent, Velocity, VelocityDriven};
use crate::debug::{DebugLines, DebugLog, DebugOverlays};
use crate::physics::{
    PhysicsImpulseQueue, PhysicsWorld, RigidBodyHandle, SequentialStepper, Stepper,
    SubstepForceProvider,
};
use crate::terrain::TerrainWorld;
use crate::time::Time;
use crate::water::buoyancy::BuoyancyForceProvider;
use crate::water::{WaterGrid, WaterSleepTracker, WaveGrid};

/// ECS resource wrapping the physics world and its stepping strategy.
pub struct PhysicsResource {
    pub world: PhysicsWorld,
    stepper: Option<Box<dyn Stepper>>,
}

impl PhysicsResource {
    pub fn new(world: PhysicsWorld, stepper: Box<dyn Stepper>) -> Self {
        Self {
            world,
            stepper: Some(stepper),
        }
    }

    pub fn stepper(&self) -> &dyn Stepper {
        self.stepper
            .as_ref()
            .expect("stepper taken during step")
            .as_ref()
    }
}

impl Default for PhysicsResource {
    fn default() -> Self {
        Self {
            world: PhysicsWorld::default(),
            // 12 × 1/240 = 0.05s of budget, so physics keeps real time down
            // to 20 FPS. At 8 the budget was 0.0333s — exactly 30 FPS — which
            // a 30Hz display sits right on, discarding ~2% of every frame.
            //
            // The cap only binds below its own frame rate, so this costs
            // nothing above 20 FPS: the substep count comes from the
            // accumulator, not the cap. Verified in `tests/substep_budget.rs`.
            stepper: Some(Box::new(SequentialStepper::new(1.0 / 240.0, 12))),
        }
    }
}

/// Steps physics simulation and syncs state to ECS.
pub struct PhysicsSyncSystem {
    water_sleep_tracker: WaterSleepTracker,
}

impl Default for PhysicsSyncSystem {
    fn default() -> Self {
        Self {
            water_sleep_tracker: WaterSleepTracker::new(),
        }
    }
}

impl PhysicsSyncSystem {
    fn sync_kinematics_from_ecs(
        physics: &mut PhysicsWorld,
        positions: &WriteStorage<Position>,
        velocities: &WriteStorage<Velocity>,
        orientations: &WriteStorage<Orientation>,
        bodies: &ReadStorage<RigidBodyComponent>,
    ) {
        let mut kinematic_updates = Vec::new();
        {
            let physics_ref = &*physics;
            for (pos, vel, orient, body) in
                ((&*positions), (&*velocities), (&*orientations), bodies).join()
            {
                if let Some(rb) = physics_ref.body(body.0) {
                    if rb.is_kinematic() {
                        kinematic_updates.push((body.0, pos.0, vel.0, orient.0));
                    }
                }
            }
        }
        for (handle, pos, vel, orient) in kinematic_updates {
            let position = Point3::new(pos.x, pos.y, pos.z);
            let _ = physics.set_kinematic_transform(handle, position, orient);
            let _ = physics.set_kinematic_velocity(handle, vel, Vector3::zeros());
        }
    }

    fn sync_velocity_driven_from_ecs(
        physics: &mut PhysicsWorld,
        velocities: &WriteStorage<Velocity>,
        bodies: &ReadStorage<RigidBodyComponent>,
        velocity_driven: &ReadStorage<VelocityDriven>,
    ) {
        let mut updates = Vec::new();
        for (vel, body, vd) in ((&*velocities), bodies, velocity_driven).join() {
            updates.push((
                body.0,
                vel.0,
                vd.angular_velocity,
                vd.max_accel,
                vd.angular_max_accel,
            ));
        }
        for (handle, vel, angular, max_accel, angular_max_accel) in updates {
            let _ =
                physics.set_body_velocity_drive(handle, vel, angular, max_accel, angular_max_accel);
        }
    }

    fn sync_physics_to_ecs(
        physics: &PhysicsWorld,
        positions: &mut WriteStorage<Position>,
        velocities: &mut WriteStorage<Velocity>,
        orientations: &mut WriteStorage<Orientation>,
        bodies: &ReadStorage<RigidBodyComponent>,
    ) {
        for (pos, vel, orient, body) in (positions, velocities, orientations, bodies).join() {
            if let Some(rb) = physics.body(body.0) {
                pos.0 = Vector3::new(rb.position().x, rb.position().y, rb.position().z);
                vel.0 = rb.linear_velocity();
                orient.0 = rb.rotation();
            }
        }
    }

    fn apply_grounded_state(
        grounded_handles: &FxHashSet<RigidBodyHandle>,
        bodies: &ReadStorage<RigidBodyComponent>,
        controllers: &mut WriteStorage<CharacterAnimator>,
    ) {
        for (body, controller) in (bodies, controllers).join() {
            controller.state.is_grounded = grounded_handles.contains(&body.0);
        }
    }
}

impl<'a> System<'a> for PhysicsSyncSystem {
    type SystemData = (
        Write<'a, PhysicsResource>,
        Option<Read<'a, TerrainWorld>>,
        Read<'a, Time>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, Velocity>,
        WriteStorage<'a, Orientation>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, VelocityDriven>,
        WriteStorage<'a, CharacterAnimator>,
        Write<'a, DebugLines>,
        Write<'a, DebugLog>,
        Write<'a, DebugOverlays>,
        Write<'a, PhysicsImpulseQueue>,
        Option<Read<'a, WaterGrid>>,
        Option<Read<'a, WaveGrid>>,
    );

    fn run(
        &mut self,
        (
            mut physics,
            terrain_opt,
            time,
            mut positions,
            mut velocities,
            mut orientations,
            bodies,
            velocity_driven,
            mut controllers,
            mut debug_lines,
            mut debug_log,
            mut debug_overlays,
            mut impulse_queue,
            flow_opt,
            wave_opt,
        ): Self::SystemData,
    ) {
        let frame_dt = time.delta_seconds();

        // Sync kinematic bodies from ECS into physics before stepping.
        Self::sync_kinematics_from_ecs(
            &mut physics.world,
            &positions,
            &velocities,
            &orientations,
            &bodies,
        );

        // Sync velocity-driven dynamic bodies (player, platforms, etc.)
        Self::sync_velocity_driven_from_ecs(
            &mut physics.world,
            &velocities,
            &bodies,
            &velocity_driven,
        );

        let impulses: Vec<_> = impulse_queue.drain().collect();

        // Wake sleeping bodies whose water surface has changed.
        if let Some(ref flow_grid) = flow_opt {
            let sleeping = physics.world.sleeping_bodies();
            for handle in &sleeping {
                if let Some(body) = physics.world.body(*handle) {
                    if self.water_sleep_tracker.should_wake(
                        *handle,
                        flow_grid,
                        wave_opt.as_deref(),
                        body.position(),
                    ) {
                        physics.world.wake_body(*handle);
                    }
                }
            }
        }

        // Build per-substep force providers.
        let buoyancy_provider = flow_opt.as_ref().map(|flow_grid| {
            let affected: Vec<_> = (&bodies)
                .join()
                .filter_map(|b| {
                    let body = physics.world.body(b.0)?;
                    body.is_dynamic().then_some(b.0)
                })
                .collect();
            BuoyancyForceProvider::new(flow_grid, wave_opt.as_deref(), affected)
        });
        let providers: Vec<&dyn SubstepForceProvider> = buoyancy_provider
            .as_ref()
            .map(|p| vec![p as &dyn SubstepForceProvider])
            .unwrap_or_default();

        // Step physics with terrain as static geometry
        let mut physics_substeps = 0u32;
        if let Some(ref terrain) = terrain_opt {
            if let Some(mut stepper) = physics.stepper.take() {
                let result = stepper.step(
                    &mut physics.world,
                    frame_dt,
                    &**terrain,
                    &impulses,
                    &providers,
                    &mut debug_lines,
                );
                physics_substeps = result.substeps;
                physics.stepper = Some(stepper);
            }
        }

        // Record water levels for awake buoyant bodies (used next frame
        // to detect surface changes under sleeping bodies).
        if let Some(ref flow_grid) = flow_opt {
            for body_comp in (&bodies).join() {
                let handle = body_comp.0;
                if physics.world.is_sleeping(handle) {
                    continue;
                }
                if let Some(body) = physics.world.body(handle) {
                    if body.is_dynamic() {
                        self.water_sleep_tracker.record(
                            handle,
                            flow_grid,
                            wave_opt.as_deref(),
                            body.position(),
                        );
                    }
                }
            }
        }

        let grounded_handles = physics.world.grounded_handles();

        // Debug visualization and logging
        physics.world.debugger().add_contact_overlays(
            physics.world.contact_events(),
            &mut debug_overlays,
            &mut debug_lines,
        );
        let sleeping_bodies = physics.world.sleeping_bodies();
        physics.world.debugger().add_sleep_overlays(
            physics.world.bodies(),
            physics.world.colliders_arena(),
            &sleeping_bodies,
            &mut debug_overlays,
        );
        if let Some(ref terrain) = terrain_opt {
            physics.world.debugger().add_broadphase_patch_overlays(
                physics.world.bodies(),
                physics.world.colliders_arena(),
                &**terrain,
                physics.world.config().contact_margin,
                &sleeping_bodies,
                &mut debug_overlays,
            );
        }
        physics.world.debugger().add_collider_shape_overlays(
            physics.world.bodies(),
            physics.world.colliders_arena(),
            &mut debug_overlays,
        );
        physics.world.debugger().write_debug_log(
            &mut debug_log,
            physics.world.config().contact_margin,
            physics.world.config().warm_start_depth_slop,
        );
        let fixed_dt = physics.stepper().fixed_dt();
        let gravity = physics.world.config().gravity;
        debug_log.add("Physics/Config/FrameDt", format!("{:.5}", frame_dt));
        debug_log.add("Physics/Config/FixedDt", format!("{:.5}", fixed_dt));
        debug_log.add("Physics/Config/Substeps", physics_substeps.to_string());
        debug_log.add(
            "Physics/Config/Gdt",
            format!("{:.5}", gravity.magnitude() * fixed_dt),
        );
        debug_log.add(
            "Physics/Config/GyDt",
            format!("{:.5}", gravity.y * fixed_dt),
        );

        // Sync physics state back to ECS components
        Self::sync_physics_to_ecs(
            &physics.world,
            &mut positions,
            &mut velocities,
            &mut orientations,
            &bodies,
        );

        Self::apply_grounded_state(&grounded_handles, &bodies, &mut controllers);
    }
}
