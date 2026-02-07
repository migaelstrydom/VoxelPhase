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
use crate::components::{Orientation, Position, RigidBodyComponent, Velocity};
use crate::debug::DebugLines;
use crate::physics::{PhysicsWorld, RigidBodyHandle};
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

    fn collect_grounded_handles(physics: &PhysicsWorld) -> HashSet<RigidBodyHandle> {
        let mut grounded_handles = HashSet::new();
        for contact in physics.contact_events() {
            if contact.body_a.is_none() && contact.normal.y > 0.5 {
                grounded_handles.insert(contact.body_b);
            }
        }
        grounded_handles
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
        WriteStorage<'a, BipedController>,
        Write<'a, DebugLines>,
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
            mut controllers,
            mut debug_lines,
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

        // Step physics with terrain as static geometry
        if let Some(ref terrain) = terrain_opt {
            physics.0.step(dt, &**terrain, &mut debug_lines);
        }

        let grounded_handles = Self::collect_grounded_handles(&physics.0);

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
