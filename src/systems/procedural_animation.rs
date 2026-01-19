//! System for updating procedural character animation.

use nalgebra::{Point3, Vector3};
use specs::{Join, Read, ReadStorage, System, WriteStorage};

use crate::components::{Collider, OnGround, Position, Rotation, Velocity};
use crate::player::Player;
use crate::skeleton::BipedCharacter;
use crate::time::Time;

/// Updates biped character animation each frame.
///
/// This is the Stage 2 animation system - pelvis + two legs with phase-based gait.
pub struct ProceduralAnimationSystem;

impl<'a> System<'a> for ProceduralAnimationSystem {
    type SystemData = (
        Read<'a, Time>,
        ReadStorage<'a, Player>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Velocity>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, OnGround>,
        ReadStorage<'a, Collider>,
        WriteStorage<'a, BipedCharacter>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (time, players, positions, velocities, rotations, on_grounds, colliders, mut characters) =
            data;

        let dt = time.delta_seconds();

        for (_player, pos, vel, rot, on_ground, collider, character) in (
            &players,
            &positions,
            &velocities,
            &rotations,
            &on_grounds,
            &colliders,
            &mut characters,
        )
            .join()
        {
            // The pelvis position - center of the physics body
            let pelvis_pos = Point3::new(pos.0.x, pos.0.y, pos.0.z);

            // Ground height is below the collider
            let ground_height = if on_ground.grounded {
                pos.0.y - collider.shape.radius
            } else {
                // When airborne, estimate ground is far below
                pos.0.y - collider.shape.radius - 2.0
            };

            // Calculate facing direction from rotation (yaw around Y axis)
            // rotation = 0 means facing +Z, rotation = π/2 means facing +X
            let facing = Vector3::new(rot.0.sin(), 0.0, rot.0.cos());

            // Update the biped animation
            character.update(
                pelvis_pos,
                ground_height,
                vel.0,
                facing,
                on_ground.grounded,
                dt,
            );
        }
    }
}
