//! Physics synchronization system.
//!
//! This system is the single point of integration between the physics engine
//! and the ECS world. It:
//! 1. Steps the physics simulation
//! 2. Syncs physics state back to ECS components

use nalgebra::Vector3;
use specs::{Join, Read, ReadStorage, System, Write, WriteStorage};

use crate::components::{Orientation, Position, RigidBodyComponent, Velocity};
use crate::debug::DebugLines;
use crate::physics::PhysicsWorld;
use crate::terrain::TerrainManager;
use crate::time::Time;

/// Wrapper for PhysicsWorld to use as a specs resource.
#[derive(Default)]
pub struct PhysicsResource(pub PhysicsWorld);

/// Steps physics simulation and syncs state to ECS.
pub struct PhysicsSyncSystem;

impl<'a> System<'a> for PhysicsSyncSystem {
    type SystemData = (
        Write<'a, PhysicsResource>,
        Option<Read<'a, TerrainManager>>,
        Read<'a, Time>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, Velocity>,
        WriteStorage<'a, Orientation>,
        ReadStorage<'a, RigidBodyComponent>,
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
            mut debug_lines,
        ): Self::SystemData,
    ) {
        let dt = time.delta_seconds();

        // Step physics with terrain as static geometry
        if let Some(ref terrain) = terrain_opt {
            physics.0.step(dt, &**terrain, &mut debug_lines);
        }

        // Sync physics state back to ECS components
        for (pos, vel, orient, body) in
            (&mut positions, &mut velocities, &mut orientations, &bodies).join()
        {
            if let Some(rb) = physics.0.body(body.0) {
                pos.0 = Vector3::new(rb.position().x, rb.position().y, rb.position().z);
                vel.0 = rb.linear_velocity();
                orient.0 = rb.rotation();
            }
        }
    }
}
