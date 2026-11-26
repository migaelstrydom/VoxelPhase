//! ECS systems for peeper animation.
//!
//! Two thin wrappers around [`StiltAnimator`], mirroring the pair the
//! humanoid and the heart critter use: one to aim the probes before
//! sensing, one to animate after.
//!
//! This is the one place the rig meets the AI, and it meets it by reading
//! two components and turning them into a [`Mood`] — a pair of numbers.
//! The animator never sees a `Brain`, so the same rig can be posed by a
//! bench harness that has none.

use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use specs::{Entities, Join, Read, ReadStorage, System, Write, WriteStorage};

use crate::character::locomotion::{Down, DownPhase};
use crate::character::{
    CharacterIntent, CharacterState, Grounding, LocomotionConfig, LocomotionState,
};
use crate::components::{Orientation, PosedModel, Position, ProceduralMesh, Rotation, Velocity};
use crate::creature::{Behaviour, Brain, MeleeAttack, Perception};
use crate::debug::DebugLines;
use crate::sensing::{ContactCandidates, SensorSet};
use crate::time::Time;

use super::animator::{Mood, StiltAnimator};

/// Aims each peeper's foot probes. Runs BEFORE `SensorProbeSystem`.
/// Seconds a walker's eyes stay on where it last saw what it was watching.
const WATCH_MEMORY: f32 = 1.0;

pub struct StiltProbeConfigSystem;

impl<'a> System<'a> for StiltProbeConfigSystem {
    type SystemData = (
        Entities<'a>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, StiltAnimator>,
        WriteStorage<'a, SensorSet>,
    );

    fn run(&mut self, (entities, positions, rotations, animators, mut sensors): Self::SystemData) {
        for (entity, pos, rot, animator) in (&entities, &positions, &rotations, &animators).join() {
            let body_pos = nalgebra::Point3::new(pos.0.x, pos.0.y, pos.0.z);
            let pelvis = animator.pelvis_for(body_pos);
            let probes = animator.configure_probes(pelvis, rot.0);
            let _ = sensors.insert(entity, SensorSet::new(probes));
        }
    }
}

/// Animates each peeper from this frame's probe results. Runs AFTER
/// `SensorProbeSystem`.
pub struct StiltAnimationSystem;

impl<'a> System<'a> for StiltAnimationSystem {
    type SystemData = (
        Read<'a, Time>,
        Entities<'a>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, Velocity>,
        ReadStorage<'a, CharacterIntent>,
        ReadStorage<'a, Grounding>,
        ReadStorage<'a, ContactCandidates>,
        ReadStorage<'a, Brain>,
        ReadStorage<'a, MeleeAttack>,
        ReadStorage<'a, Perception>,
        WriteStorage<'a, StiltAnimator>,
        WriteStorage<'a, PosedModel>,
        ReadStorage<'a, Orientation>,
        WriteStorage<'a, ProceduralMesh>,
        Write<'a, DebugLines>,
        ReadStorage<'a, CharacterState>,
        ReadStorage<'a, LocomotionConfig>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            time,
            entities,
            positions,
            rotations,
            velocities,
            intents,
            groundings,
            candidates,
            brains,
            attacks,
            perceptions,
            mut animators,
            mut posed,
            orientations,
            mut procedural,
            mut debug_lines,
            states,
            configs,
        ) = data;

        let dt = time.delta_seconds();

        for (entity, pos, rot, vel, intent, grounding, animator) in (
            &entities,
            &positions,
            &rotations,
            &velocities,
            &intents,
            &groundings,
            &mut animators,
        )
            .join()
        {
            let body_pos = nalgebra::Point3::new(pos.0.x, pos.0.y, pos.0.z);
            let pelvis = animator.pelvis_for(body_pos);
            let contacts = candidates
                .get(entity)
                .map(|c| c.candidates.as_slice())
                .unwrap_or(&[]);

            let mood = Mood {
                alertness: brains
                    .get(entity)
                    .map_or(1.0, |brain| alertness_of(brain.behaviour)),
                thrust: attacks.get(entity).map_or(0.0, |attack| attack.thrust()),
                // Its eyes follow what it can see, and for a moment after.
                watching: perceptions
                    .get(entity)
                    .and_then(|perception| perception.target)
                    .filter(|target| target.since_seen < WATCH_MEMORY)
                    .map(|target| target.position),
            };

            let down = states.get(entity).and_then(|state| match state.locomotion {
                LocomotionState::Down(down) => Some(down),
                _ => None,
            });
            match down.zip(orientations.get(entity)) {
                Some((down, turned)) => {
                    let body = Isometry3::from_parts(Translation3::from(pos.0), turned.0);
                    let folded = configs
                        .get(entity)
                        .map_or(1.0, |config| folded(down, turned.0, config));
                    animator.lie(dt, body, folded);
                }
                None => animator.update(
                    dt,
                    pelvis,
                    rot.0,
                    nalgebra::Vector3::new(vel.0.x, vel.0.y, vel.0.z),
                    grounding,
                    intent,
                    contacts,
                    mood,
                ),
            }

            if let Some(posed) = posed.get_mut(entity) {
                posed.poses = animator.poses_from(&pos.0);
            }
            // A drape is drawn in the entity's own frame, as the renderer
            // places it.
            if let Some(mesh) = procedural.get_mut(entity) {
                let turn = orientations.get(entity).map_or_else(
                    || UnitQuaternion::from_axis_angle(&nalgebra::Vector3::y_axis(), rot.0),
                    |orientation| orientation.0,
                );
                animator.drape_into(&pos.0, &turn, &mut mesh.primitives);
            }

            // Every walker owns a recorder but only one can be armed, so
            // the idle ones stay off the screen rather than taking turns
            // overwriting the line.
            if animator.locomotion.is_recording() {
                if let Some(status) = animator.locomotion.recording_status() {
                    debug_lines.add("Stilt rec", status);
                }
            }
        }
    }
}

/// How far a knocked-down walker's legs are folded up, in `[0, 1]`: all the
/// way while it lies, and less as it comes upright getting up, straight by
/// the time it stands.
fn folded(down: Down, turned: UnitQuaternion<f32>, config: &LocomotionConfig) -> f32 {
    match down.phase {
        DownPhase::Rising { .. } => {
            let tilt = (turned * Vector3::y()).y.clamp(-1.0, 1.0).acos();
            tilt / config.down.lying_tilt
        }
        DownPhase::Limp { .. } | DownPhase::Gathered { .. } => 1.0,
    }
}

/// How wide the eye is for a given state of mind.
///
/// A dozing peeper is the point of the whole creature: with its eye shut
/// it is a landmark the player can creep past, and the snap to a wide eye
/// on first contact is the alert beat that other creatures express with a
/// hop and a shiver.
fn alertness_of(behaviour: Behaviour) -> f32 {
    match behaviour {
        Behaviour::Idle => 0.0,
        Behaviour::Wandering { .. } => 0.4,
        // Wide, and held wide, because this is the moment the player is
        // meant to notice.
        Behaviour::Alerted { .. } => 1.0,
        Behaviour::Chasing => 0.9,
        // Narrowed to a glare. Anything wider looks startled rather than
        // committed.
        Behaviour::Attacking { .. } => 0.65,
        Behaviour::Searching { .. } => 0.8,
        Behaviour::Fleeing { .. } => 1.0,
    }
}
