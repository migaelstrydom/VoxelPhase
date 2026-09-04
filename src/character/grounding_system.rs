use specs::{Join, ReadStorage, System, Write, WriteStorage};

use super::grounding::Grounding;
use crate::components::RigidBodyComponent;
use crate::systems::PhysicsResource;

/// Derives [`Grounding`] from physics contacts, for every body that has one.
///
/// The Support Set is the single source of support in the engine, so this is
/// the only writer of `Grounding`: a rigged humanoid's answer comes from the
/// same contacts a roller's does. Foot probes remain a rangefinder for the
/// animator — where the ground is, and which way it faces at a landing target
/// nothing is touching yet — and no longer decide whether the character is on
/// it. See `docs/TRACTION_DRIVE_DESIGN.md` §6.6.
///
/// Contact grounding is truthful and chattery where reach-based grounding was
/// lenient and smooth; the forgiveness that costs is restored deliberately, by
/// `GroundForgiveness` on the FSM side.
pub struct ContactGroundingSystem;

impl<'a> System<'a> for ContactGroundingSystem {
    type SystemData = (
        Write<'a, PhysicsResource>,
        ReadStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, Grounding>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (mut physics_res, rigid_bodies, mut groundings) = data;

        let grounded = physics_res.world.grounded_bodies();

        for (rb, grounding) in (&rigid_bodies, &mut groundings).join() {
            *grounding = match grounded.get(&rb.0) {
                Some(normal) => Grounding::on(*normal),
                None => Grounding::airborne(),
            };
        }
    }
}
