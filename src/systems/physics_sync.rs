//! Physics synchronization system.
//!
//! This system is the single point of integration between the physics engine
//! and the ECS world. It:
//! 1. Steps the physics simulation
//! 2. Syncs physics state back to ECS components

use nalgebra::{Point3, Vector3};
use specs::{Join, Read, ReadStorage, System, Write, WriteStorage};
use std::collections::HashSet;

use crate::biped::BipedController;
use crate::components::{Orientation, Position, RigidBodyComponent, Velocity, VelocityDriven};
use crate::debug::{DebugLines, DebugLog, DebugOverlays};
use crate::physics::{PhysicsImpulseQueue, PhysicsWorld, RigidBodyHandle};
use crate::terrain::TerrainManager;
use crate::time::Time;

/// Wrapper for PhysicsWorld to use as a specs resource.
#[derive(Default)]
pub struct PhysicsResource(pub PhysicsWorld);

/// Steps physics simulation and syncs state to ECS.
pub struct PhysicsSyncSystem;

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
        for (vel, body, _) in ((&*velocities), bodies, velocity_driven).join() {
            updates.push((body.0, vel.0));
        }
        for (handle, vel) in updates {
            let _ = physics.set_body_velocity(handle, vel, Vector3::zeros());
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
        grounded_handles: &HashSet<RigidBodyHandle>,
        bodies: &ReadStorage<RigidBodyComponent>,
        controllers: &mut WriteStorage<BipedController>,
    ) {
        for (body, controller) in (bodies, controllers).join() {
            controller.state.is_grounded = grounded_handles.contains(&body.0);
        }
    }

}

impl<'a> System<'a> for PhysicsSyncSystem {
    type SystemData = (
        Write<'a, PhysicsResource>,
        Option<Read<'a, TerrainManager>>,
        Read<'a, Time>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, Velocity>,
        WriteStorage<'a, Orientation>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, VelocityDriven>,
        WriteStorage<'a, BipedController>,
        Write<'a, DebugLines>,
        Write<'a, DebugLog>,
        Write<'a, DebugOverlays>,
        Write<'a, PhysicsImpulseQueue>,
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
        ): Self::SystemData,
    ) {
        let dt = time.delta_seconds();

        // Sync kinematic bodies from ECS into physics before stepping.
        Self::sync_kinematics_from_ecs(
            &mut physics.0,
            &positions,
            &velocities,
            &orientations,
            &bodies,
        );

        // Sync velocity-driven dynamic bodies (player, platforms, etc.)
        Self::sync_velocity_driven_from_ecs(
            &mut physics.0,
            &velocities,
            &bodies,
            &velocity_driven,
        );

        let impulses: Vec<_> = impulse_queue.drain().collect();

        // Step physics with terrain as static geometry
        if let Some(ref terrain) = terrain_opt {
            physics.0.step(dt, &**terrain, &impulses, &mut debug_lines);
        }

        let grounded_handles = physics.0.grounded_handles();

        // Debug visualization and logging
        physics.0.debugger().add_contact_overlays(
            physics.0.contact_events(),
            &mut debug_overlays,
            &mut debug_lines,
        );
        physics.0.debugger().add_sleep_overlays(
            physics.0.bodies(),
            &physics.0.sleeping_bodies(),
            &mut debug_overlays,
        );
        physics.0.debugger().write_debug_log(
            &mut debug_log,
            physics.0.config().contact_margin,
            physics.0.config().warm_start_depth_slop,
        );

        // Sync physics state back to ECS components
        Self::sync_physics_to_ecs(
            &physics.0,
            &mut positions,
            &mut velocities,
            &mut orientations,
            &bodies,
        );

        Self::apply_grounded_state(&grounded_handles, &bodies, &mut controllers);
    }
}
