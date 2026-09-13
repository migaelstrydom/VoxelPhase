//! ECS systems for peeper animation.
//!
//! Two thin wrappers around [`PeeperAnimator`], mirroring the pair the
//! humanoid and the heart critter use: one to aim the probes before
//! sensing, one to animate after.
//!
//! This is the one place the rig meets the AI, and it meets it by reading
//! two components and turning them into a [`Mood`] — a pair of numbers.
//! The animator never sees a `Brain`, so the same rig can be posed by a
//! bench harness that has none.

use specs::{Entities, Join, Read, ReadStorage, System, WriteStorage};

use crate::character::{CharacterIntent, Grounding};
use crate::components::{Position, Rotation, Velocity};
use crate::creature::{Behaviour, Brain, MeleeAttack};
use crate::sensing::{ContactCandidates, SensorSet};
use crate::time::Time;

use super::animator::{Mood, PeeperAnimator};

/// Aims each peeper's foot probes. Runs BEFORE `SensorProbeSystem`.
pub struct PeeperProbeConfigSystem;

impl<'a> System<'a> for PeeperProbeConfigSystem {
    type SystemData = (
        Entities<'a>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, PeeperAnimator>,
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
pub struct PeeperAnimationSystem;

impl<'a> System<'a> for PeeperAnimationSystem {
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
        WriteStorage<'a, PeeperAnimator>,
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
            mut animators,
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
            };

            animator.update(
                dt,
                pelvis,
                rot.0,
                nalgebra::Vector3::new(vel.0.x, vel.0.y, vel.0.z),
                grounding,
                intent,
                contacts,
                mood,
            );
        }
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
