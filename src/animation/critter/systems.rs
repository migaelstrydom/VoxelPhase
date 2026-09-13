//! ECS systems for critter animation.
//!
//! Thin wrappers around [`CritterAnimator`], mirroring the pair the
//! humanoid uses: one to aim the probes before sensing, one to animate
//! after.

use specs::{Entities, Join, Read, ReadStorage, System, WriteStorage};

use crate::character::{CharacterIntent, Grounding};
use crate::components::{Position, Rotation, Velocity};
use crate::sensing::{ContactCandidates, SensorSet};
use crate::time::Time;

use super::animator::CritterAnimator;

/// Aims each critter's foot probes. Runs BEFORE `SensorProbeSystem`.
pub struct CritterProbeConfigSystem;

impl<'a> System<'a> for CritterProbeConfigSystem {
    type SystemData = (
        Entities<'a>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, CritterAnimator>,
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

/// Animates each critter from this frame's probe results. Runs AFTER
/// `SensorProbeSystem`.
pub struct CritterAnimationSystem;

impl<'a> System<'a> for CritterAnimationSystem {
    type SystemData = (
        Read<'a, Time>,
        Entities<'a>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, Velocity>,
        ReadStorage<'a, CharacterIntent>,
        ReadStorage<'a, Grounding>,
        ReadStorage<'a, ContactCandidates>,
        WriteStorage<'a, CritterAnimator>,
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

            animator.update(
                dt,
                pelvis,
                rot.0,
                nalgebra::Vector3::new(vel.0.x, vel.0.y, vel.0.z),
                grounding,
                intent,
                contacts,
            );
        }
    }
}
