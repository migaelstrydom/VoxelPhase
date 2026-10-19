//! Reads every shown suspension out of the physics world and turns it into
//! its field's look, kick-up and underglow.

use nalgebra::Vector3;
use specs::{Entities, Join, Read, ReadExpect, ReadStorage, System, Write, WriteStorage};

use crate::components::Position;
use crate::lighting::PointLight;
use crate::particles::ParticlePool;
use crate::physics::{ConstraintHandle, ConstraintKind, PhysicsWorld};
use crate::systems::PhysicsResource;
use crate::time::Time;

use super::pod::PodSample;
use super::visual::ForceFieldVisual;

/// Runs after the physics step, and before the frame's lights are chosen,
/// since it moves and brightens the underglow.
pub struct ForceFieldSystem;

impl<'a> System<'a> for ForceFieldSystem {
    type SystemData = (
        Entities<'a>,
        Read<'a, Time>,
        ReadExpect<'a, PhysicsResource>,
        Write<'a, ParticlePool>,
        ReadStorage<'a, Position>,
        WriteStorage<'a, ForceFieldVisual>,
        WriteStorage<'a, PointLight>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, time, physics, mut pool, positions, mut visuals, mut lights) = data;
        let dt = time.delta_seconds();

        for (entity, visual, position) in (&entities, &mut visuals, &positions).join() {
            let samples: Vec<Option<PodSample>> = visual
                .suspensions()
                .map(|handle| sample(&physics.world, handle))
                .collect();
            visual.update(&samples, dt);
            visual.kick_up(dt, &mut pool);

            if let Some(light) = lights.get_mut(entity) {
                match visual.underglow() {
                    Some(glow) => {
                        light.colour = glow.colour;
                        light.intensity = glow.intensity;
                        light.offset = glow.position.coords - position.0;
                    }
                    None => light.intensity = 0.0,
                }
            }
        }
    }
}

/// What the field reads of the suspension `handle`: where it is, which way
/// it pushes, its readout and how fast it sweeps over its ground. `None` if
/// it, or its body, is gone.
pub fn sample(world: &PhysicsWorld, handle: ConstraintHandle) -> Option<PodSample> {
    let ConstraintKind::Suspension(suspension) = &world.constraint(handle)?.kind else {
        return None;
    };
    let body = world.body(suspension.body())?;
    let readout = world.suspension(handle)?;
    let anchor = suspension.anchor_in_world(body);
    let ground_speed = readout.hit.map_or(0.0, |hit| {
        let ground = hit
            .partner
            .and_then(|partner| world.body(partner))
            .map_or(Vector3::zeros(), |partner| partner.velocity_at(hit.point));
        let relative = body.velocity_at(anchor) - ground;
        (relative - hit.normal * relative.dot(&hit.normal)).norm()
    });
    Some(PodSample {
        anchor,
        axis: suspension.desc().axis.in_world(&body.rotation()),
        rest_length: suspension.desc().rest_length,
        readout,
        ground_speed,
    })
}
