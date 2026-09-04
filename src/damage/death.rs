use specs::{
    Component, DenseVecStorage, Entities, Join, Read, ReadStorage, System, Write, WriteStorage,
};

use super::health::{Dead, Health};
use crate::character::CharacterIntent;
use crate::components::RigidBodyComponent;
use crate::drive::Actuator;
use crate::physics::ConstraintHandle;
use crate::systems::PhysicsResource;
use crate::time::Time;

/// What holds a body up while it is alive, and must be let go of when it dies.
///
/// The cheapest convincing death in this engine is to stop fighting the physics
/// that is already running: release the KeepUpright constraint, stop driving the
/// velocity, and the body falls over on its own. No death animation required.
#[derive(Component, Debug, Default)]
#[storage(DenseVecStorage)]
pub struct Ragdoll {
    /// Constraints released on death — typically KeepUpright, plus any anchor
    /// pinning the body in place.
    pub constraints: Vec<ConstraintHandle>,
}

impl Ragdoll {
    pub fn new(constraints: Vec<ConstraintHandle>) -> Self {
        Self { constraints }
    }
}

/// Runs the one-time transition into a corpse, then despawns it.
///
/// Deliberately separate from `DamageApplySystem`: deciding something died is a
/// bookkeeping question, while going limp touches physics constraints and
/// character control. Splitting them keeps the queue drain free of both.
pub struct DeathSystem;

impl<'a> System<'a> for DeathSystem {
    type SystemData = (
        Entities<'a>,
        Read<'a, Time>,
        Write<'a, PhysicsResource>,
        WriteStorage<'a, Dead>,
        WriteStorage<'a, Health>,
        WriteStorage<'a, Ragdoll>,
        WriteStorage<'a, CharacterIntent>,
        ReadStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, Actuator>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            entities,
            time,
            mut physics_res,
            mut deads,
            healths,
            mut ragdolls,
            mut intents,
            rigid_bodies,
            mut actuators,
        ) = data;
        let dt = time.delta_seconds();

        let mut despawn = Vec::new();

        for (entity, dead) in (&entities, &mut deads).join() {
            if !dead.limp {
                dead.limp = true;

                // Let go of whatever was holding the body upright.
                if let Some(ragdoll) = ragdolls.get_mut(entity) {
                    for handle in ragdoll.constraints.drain(..) {
                        physics_res.world.remove_constraint(handle);
                    }
                }

                // Stop the corpse steering and turning itself. Zeroing the
                // intent is what actually silences it — `CharacterControlSystem`
                // is source-agnostic and would happily keep driving a dead
                // body's last-held direction otherwise.
                if let Some(intent) = intents.get_mut(entity) {
                    *intent = CharacterIntent::default();
                }
                // Out of service means gripping like any other body: the
                // actuator's non-support grip is tuning for a character that
                // jumps, and a corpse does not.
                if actuators.remove(entity).is_some() {
                    if let Some(rb) = rigid_bodies.get(entity) {
                        physics_res.world.set_body_non_support_grip(rb.0, 1.0);
                    }
                }
            }

            dead.elapsed += dt;

            let lifetime = healths.get(entity).and_then(|h| h.corpse_lifetime);
            if let Some(lifetime) = lifetime {
                if dead.elapsed >= lifetime {
                    despawn.push(entity);
                }
            }
        }

        for entity in despawn {
            // The rigid body outlives the entity unless it is removed here —
            // nothing else owns it, and a leaked body keeps colliding.
            if let Some(rb) = rigid_bodies.get(entity) {
                physics_res.world.remove_body(rb.0);
            }
            let _ = entities.delete(entity);
        }
    }
}
