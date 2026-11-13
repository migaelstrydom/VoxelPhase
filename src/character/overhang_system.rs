use specs::{Join, Read, ReadStorage, System, WriteStorage};

use super::components::CharacterIntent;
use super::config::LocomotionConfig;
use super::grounding::Grounding;
use super::overhang::OverhangSensor;
use crate::components::RigidBodyComponent;
use crate::platform::MovingPlatform;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;
use crate::traversal::{carriers, WorldCaster};

/// Finds each sensing character standing over a drop, from the world as last
/// stepped, and hands the way off to the locomotion through the intent
/// (`CharacterIntent::overhang`): a grounded body whose support leans, with
/// no ground within reach under its centre.
pub struct OverhangSensorSystem;

impl<'a> System<'a> for OverhangSensorSystem {
    type SystemData = (
        ReadStorage<'a, OverhangSensor>,
        WriteStorage<'a, CharacterIntent>,
        ReadStorage<'a, Grounding>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, LocomotionConfig>,
        ReadStorage<'a, MovingPlatform>,
        Read<'a, PhysicsResource>,
        Option<Read<'a, TerrainWorld>>,
    );

    fn run(
        &mut self,
        (sensors, mut intents, groundings, bodies, configs, platforms, physics, terrain): Self::SystemData,
    ) {
        let Some(terrain) = terrain else {
            return;
        };
        let world = &physics.world;
        let Some(up) = world
            .config()
            .gravity_direction()
            .map(|down| -down.into_inner())
        else {
            return;
        };
        let carriers = carriers(&platforms, &bodies);
        for (sensor, intent, grounding, body, config) in
            (&sensors, &mut intents, &groundings, &bodies, &configs).join()
        {
            intent.overhang = None;
            let Some(normal) = grounding.normal.filter(|_| grounding.is_grounded) else {
                continue;
            };
            let Some(centre) = world.body(body.0).map(|b| b.position()) else {
                continue;
            };
            let caster = WorldCaster {
                world,
                scenery: &*terrain,
                exclude: &[body.0],
                filter: world.filter_of(body.0),
                carriers: &carriers,
            };
            intent.overhang =
                sensor.way_off(&caster, centre, normal, up, config.collider_half_height);
        }
    }
}
