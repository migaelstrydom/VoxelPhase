//! ECS systems for character animation.
//!
//! These are thin wrappers that call into the CharacterAnimator.

use specs::{Entities, Join, Read, ReadExpect, ReadStorage, System, Write, WriteStorage};

use super::animator::CharacterAnimator;
use super::debug_config::AnimationDebugConfig;
use super::foot_placer::{FootPhase, FootPlacer};
use crate::components::{Position, Rotation, Velocity};
use crate::debug::{DebugLines, DebugOverlays};
use crate::player::grab::GrabConfig;
use crate::player::{Player, PlayerState, PlayerTargetState};
use crate::rendering::Colour;
use crate::sensing::{ContactCandidates, SensorSet};
use crate::time::Time;

/// System that configures probes based on character animation state.
///
/// Runs BEFORE SensorProbeSystem.
/// Reads current position/rotation and writes probe configuration.
pub struct AnimationProbeConfigSystem;

impl<'a> System<'a> for AnimationProbeConfigSystem {
    type SystemData = (
        Entities<'a>,
        ReadStorage<'a, Player>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, CharacterAnimator>,
        WriteStorage<'a, SensorSet>,
    );

    fn run(
        &mut self,
        (entities, players, positions, rotations, animators, mut sensors): Self::SystemData,
    ) {
        for (entity, _player, pos, rot, animator) in
            (&entities, &players, &positions, &rotations, &animators).join()
        {
            let pelvis_pos = nalgebra::Point3::new(pos.0.x, pos.0.y, pos.0.z);
            let yaw = rot.0;

            let probes = animator.configure_probes(pelvis_pos, yaw);
            let sensor_set = SensorSet::new(probes);
            let _ = sensors.insert(entity, sensor_set);
        }
    }
}

/// System that updates character animation from probe results.
///
/// Runs AFTER SensorProbeSystem.
/// Reads probe results and updates the animator.
pub struct CharacterAnimationSystem;

impl<'a> System<'a> for CharacterAnimationSystem {
    type SystemData = (
        Read<'a, Time>,
        ReadExpect<'a, GrabConfig>,
        ReadExpect<'a, AnimationDebugConfig>,
        Entities<'a>,
        ReadStorage<'a, Player>,
        ReadStorage<'a, PlayerState>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, Velocity>,
        ReadStorage<'a, PlayerTargetState>,
        ReadStorage<'a, ContactCandidates>,
        WriteStorage<'a, CharacterAnimator>,
        Write<'a, DebugLines>,
        Write<'a, DebugOverlays>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            time,
            grab_config,
            debug_config,
            entities,
            players,
            player_states,
            positions,
            rotations,
            velocities,
            target_states,
            candidates,
            mut animators,
            mut _debug_lines,
            mut debug_overlays,
        ) = data;

        let dt = time.delta_seconds();

        for (entity, _player, player_state, pos, rot, vel, target, animator) in (
            &entities,
            &players,
            &player_states,
            &positions,
            &rotations,
            &velocities,
            &target_states,
            &mut animators,
        )
            .join()
        {
            let pelvis_pos = nalgebra::Point3::new(pos.0.x, pos.0.y, pos.0.z);
            let yaw = rot.0;

            let contacts = candidates
                .get(entity)
                .map(|c| c.candidates.as_slice())
                .unwrap_or(&[]);

            let velocity = nalgebra::Vector3::new(vel.0.x, vel.0.y, vel.0.z);
            animator.update(
                dt,
                pelvis_pos,
                yaw,
                velocity,
                player_state,
                target,
                &grab_config,
                contacts,
            );

            if debug_config.foot_placer_overlay {
                draw_foot_placer_overlay(&animator.foot_placer, &mut debug_overlays);
            }
        }
    }
}

/// Stage 1 debug: visualise what the foot placer *would* do if it were
/// driving the skeleton. Green = ideal target, yellow = planted,
/// orange = current swing position, magenta line = error vector.
fn draw_foot_placer_overlay(placer: &FootPlacer, overlays: &mut DebugOverlays) {
    for foot in [&placer.left, &placer.right] {
        overlays.add_sphere(foot.ideal_xz, 0.03, Colour::GREEN);
        overlays.add_sphere(foot.planted_position, 0.03, Colour::YELLOW);
        overlays.add_line(
            foot.planted_position,
            foot.ideal_xz,
            Colour::new(1.0, 0.0, 1.0, 1.0),
        );
        if let FootPhase::Stepping { from, to, .. } = foot.phase {
            overlays.add_sphere(foot.position, 0.035, Colour::new(1.0, 0.6, 0.0, 1.0));
            overlays.add_line(from, to, Colour::new(1.0, 0.6, 0.0, 1.0));
        }
    }
}
