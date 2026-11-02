//! ECS systems for character animation.
//!
//! These are thin wrappers that call into the CharacterAnimator.

use nalgebra::Vector3;
use specs::{Entities, Join, Read, ReadExpect, ReadStorage, System, Write, WriteStorage};

use super::animator::{BodyReading, CharacterAnimator};
use super::debug_config::AnimationDebugConfig;
use super::foot_placer::{FootPhase, FootPlacer};
use super::humanoid::hang::{HangPhase, LedgeReading};
use crate::character::grab::GrabConfig;
use crate::character::ledge::LedgeSensor;
use crate::character::locomotion::{LeanTowards, Ledge, LedgePhase};
use crate::character::{
    CharacterIntent, CharacterState, Grounding, Immersion, LocomotionConfig, LocomotionState,
    Throws,
};
use crate::components::{Orientation, Position, Rotation, Velocity};
use crate::debug::{DebugLines, DebugOverlays};
use crate::physics::PhysicsWorld;
use crate::rendering::Colour;
use crate::sensing::{ContactCandidates, SensorSet};
use crate::systems::PhysicsResource;
use crate::time::Time;

/// System that configures probes based on character animation state.
///
/// Runs BEFORE SensorProbeSystem.
/// Reads current position/rotation and writes probe configuration.
pub struct AnimationProbeConfigSystem;

impl<'a> System<'a> for AnimationProbeConfigSystem {
    type SystemData = (
        Entities<'a>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, Orientation>,
        ReadStorage<'a, CharacterAnimator>,
        WriteStorage<'a, SensorSet>,
    );

    fn run(
        &mut self,
        (entities, positions, rotations, orientations, animators, mut sensors): Self::SystemData,
    ) {
        for (entity, pos, rot, orientation, animator) in
            (&entities, &positions, &rotations, &orientations, &animators).join()
        {
            let body_pos = nalgebra::Point3::new(pos.0.x, pos.0.y, pos.0.z);
            let pelvis_pos = animator.pelvis_for(body_pos, orientation.0 * Vector3::y());
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
        ReadStorage<'a, CharacterState>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, Orientation>,
        ReadStorage<'a, Velocity>,
        ReadStorage<'a, CharacterIntent>,
        ReadStorage<'a, ContactCandidates>,
        ReadStorage<'a, Grounding>,
        ReadStorage<'a, Immersion>,
        ReadStorage<'a, Throws>,
        ReadStorage<'a, LedgeSensor>,
        ReadStorage<'a, LocomotionConfig>,
        Read<'a, PhysicsResource>,
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
            character_states,
            positions,
            rotations,
            orientations,
            velocities,
            intents,
            candidates,
            groundings,
            immersions,
            throws,
            sensors,
            configs,
            physics,
            mut animators,
            mut debug_lines,
            mut debug_overlays,
        ) = data;

        let dt = time.delta_seconds();

        for (entity, character_state, pos, rot, orientation, vel, target, grounding, animator) in (
            &entities,
            &character_states,
            &positions,
            &rotations,
            &orientations,
            &velocities,
            &intents,
            &groundings,
            &mut animators,
        )
            .join()
        {
            let body_pos = nalgebra::Point3::new(pos.0.x, pos.0.y, pos.0.z);
            let body_up = orientation.0 * Vector3::y();
            let immersion = immersions.get(entity).copied().unwrap_or_default();

            let contacts = candidates
                .get(entity)
                .map(|c| c.candidates.as_slice())
                .unwrap_or(&[]);

            let body = BodyReading {
                pelvis: animator.pelvis_for(body_pos, body_up),
                yaw: rot.0,
                velocity: Vector3::new(vel.0.x, vel.0.y, vel.0.z),
                up: body_up,
                grounding,
                immersion: &immersion,
                ledge: match (&character_state.locomotion, configs.get(entity)) {
                    (LocomotionState::Ledge(ledge), Some(config)) => ledge_reading(
                        ledge,
                        &physics.world,
                        sensors.get(entity).is_some_and(|s| s.wall_below),
                        config,
                    ),
                    _ => None,
                },
                reach_for: sensors
                    .get(entity)
                    .and_then(|s| s.ahead)
                    .map(|ahead| (ahead.hands, ahead.in_seconds)),
            };
            if let Some(throws) = throws.get(entity) {
                animator.observe_throws(throws.this_frame(), dt);
            }
            animator.update(dt, &body, character_state, target, &grab_config, contacts);

            if let Some(status) = animator.recording_status() {
                debug_lines.add("Placer rec", status);
            }

            if debug_config.foot_placer_overlay {
                draw_foot_placer_overlay(animator.foot_placer(), &mut debug_overlays);
            }
        }
    }
}

/// What the animation reads of `ledge` this frame: its edge where the edge's
/// body has taken it, the phase as the animation counts it, and how fast the
/// edge moves. `None` once the edge's body is gone.
fn ledge_reading(
    ledge: &Ledge,
    world: &PhysicsWorld,
    wall_below: bool,
    config: &LocomotionConfig,
) -> Option<LedgeReading> {
    let at = ledge.edge.in_world(world)?;
    let timing = &config.ledge;
    let phase = match ledge.phase {
        LedgePhase::Grab { elapsed } => HangPhase::Catch {
            progress: (elapsed / timing.settle).clamp(0.0, 1.0),
        },
        LedgePhase::Hang { lean } => HangPhase::Hang {
            lean: lean.map_or(0.0, |lean| {
                let held = (lean.held / timing.commit_hold).clamp(0.0, 1.0);
                match lean.towards {
                    LeanTowards::Drop => -held,
                    LeanTowards::PullUp => held,
                }
            }),
        },
        LedgePhase::PullUp { elapsed, duration } => HangPhase::Climb {
            progress: (elapsed / duration).clamp(0.0, 1.0),
        },
    };
    let grip = at.grip();
    let outward = at.outward.into_inner();
    let edge_velocity = ledge
        .edge
        .body
        .and_then(|body| world.body(body))
        .map_or(Vector3::zeros(), |body| body.velocity_at(grip));
    Some(LedgeReading {
        phase,
        hands: at.hands,
        outward,
        face: grip + outward * timing.grip_depth,
        stand: at.stand,
        edge_velocity,
        wall_below,
    })
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
        if let FootPhase::Stepping(swing) = foot.phase {
            overlays.add_sphere(foot.position, 0.035, Colour::new(1.0, 0.6, 0.0, 1.0));
            overlays.add_line(swing.from, swing.to, Colour::new(1.0, 0.6, 0.0, 1.0));
        }
    }
}
