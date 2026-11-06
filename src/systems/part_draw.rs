//! Drawing the pieces of breakable bodies from their parts' looks.

use specs::{Entities, Join, Read, ReadStorage, System, WriteStorage};

use crate::components::{ModelInstance, Orientation, Position, RigidBodyComponent, Velocity};
use crate::model::PartLooks;
use crate::systems::PhysicsResource;

/// Builds the model of every entity whose [`PartLooks`] changed: each look
/// at its collider's current offset, merged by material.
///
/// Runs after everything that splits, cuts or re-centres a body this frame
/// (fracture, cleave, glass and the owners' `PieceSplit` systems), and before
/// render. It also refreshes the entity's transform from the body, because
/// `PhysicsSyncSystem` ran before any of that: a model rebuilt about a
/// re-centred origin and drawn at last frame's origin blinks sideways for a
/// frame.
pub struct PartDrawSystem;

impl<'a> System<'a> for PartDrawSystem {
    type SystemData = (
        Entities<'a>,
        Read<'a, PhysicsResource>,
        WriteStorage<'a, PartLooks>,
        ReadStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, ModelInstance>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, Orientation>,
        WriteStorage<'a, Velocity>,
    );

    fn run(
        &mut self,
        (
            entities,
            physics,
            mut looks,
            bodies,
            mut models,
            mut positions,
            mut orientations,
            mut velocities,
        ): Self::SystemData,
    ) {
        for (entity, looks, body) in (&entities, &mut looks, &bodies).join() {
            if !looks.take_changed() {
                continue;
            }
            match looks.drawn(&physics.world, body.0) {
                Some(model) => match models.get_mut(entity) {
                    Some(instance) => instance.model = model,
                    None => {
                        let _ = models.insert(entity, ModelInstance::new(model));
                    }
                },
                None => {
                    models.remove(entity);
                }
            }
            let Some(rigid) = physics.world.body(body.0) else {
                continue;
            };
            if let Some(position) = positions.get_mut(entity) {
                position.0 = rigid.position().coords;
            }
            if let Some(orientation) = orientations.get_mut(entity) {
                orientation.0 = rigid.rotation();
            }
            if let Some(velocity) = velocities.get_mut(entity) {
                velocity.0 = rigid.linear_velocity();
            }
        }
    }
}
