//! System for updating procedural character animation.

use nalgebra::Point3;
use specs::{Join, Read, ReadStorage, System, WriteStorage};

use crate::components::{Collider, OnGround, Position, Velocity};
use crate::player::Player;
use crate::skeleton::ProceduralCharacter;
use crate::time::Time;

/// Updates procedural character animation each frame.
pub struct ProceduralAnimationSystem;

impl<'a> System<'a> for ProceduralAnimationSystem {
    type SystemData = (
        Read<'a, Time>,
        ReadStorage<'a, Player>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Velocity>,
        ReadStorage<'a, OnGround>,
        ReadStorage<'a, Collider>,
        WriteStorage<'a, ProceduralCharacter>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (time, players, positions, velocities, on_grounds, colliders, mut characters) = data;

        let dt = time.delta_seconds();

        for (_player, pos, vel, on_ground, collider, character) in (
            &players,
            &positions,
            &velocities,
            &on_grounds,
            &colliders,
            &mut characters,
        )
            .join()
        {
            // Estimate ground height from player position and collision state
            // When grounded, the feet are at pos.y - collider radius
            let ground_height = if on_ground.grounded {
                pos.0.y - collider.shape.radius
            } else {
                // When not grounded, estimate ground is far below
                pos.0.y - collider.shape.radius - 1.0
            };

            // Pass velocity to locomotion for direction (used by gait planning)
            character.locomotion.velocity = vel.0;

            // Update the procedural animation
            let root_pos = Point3::new(pos.0.x, pos.0.y, pos.0.z);
            character.update(root_pos, ground_height, on_ground.grounded, dt);
        }
    }
}
