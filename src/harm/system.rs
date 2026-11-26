use nalgebra::Vector3;
use specs::{Entities, Join, LendJoin, Read, ReadStorage, System, Write, WriteStorage};

use super::blow::{worst, Band, Blow};
use super::component::{Death, Harm};
use super::config::HarmConfig;
use crate::character::CharacterState;
use crate::components::RigidBodyComponent;
use crate::damage::Dead;
use crate::debug::DebugLog;
use crate::physics::{PhysicsWorld, RigidBodyHandle};
use crate::systems::{LevelRestart, PhysicsResource};
use crate::time::Time;

/// The worst blow `body` took in the last step, of its hits and shoves
/// ([`worst`]). A hit that continues a stop split across frames is the
/// rest of a blow already judged, and is passed over. `mass` is the body's,
/// `velocity` what it moved at before the step, `in_control` whether it
/// could brace.
pub fn worst_blow(
    world: &PhysicsWorld,
    body: RigidBodyHandle,
    mass: f32,
    velocity: Vector3<f32>,
    in_control: bool,
    config: &HarmConfig,
) -> Option<Blow> {
    let hits = world
        .hits_on(body)
        .filter(|hit| !hit.continues)
        .filter_map(|hit| Blow::of_hit(hit, body, mass, velocity, in_control, config));
    let shoves = world
        .shoves_on(body)
        .map(|shove| Blow::of_shove(shove, mass, config));
    worst(hits.chain(shoves), config)
}

/// Judges the blows each harmable body took in the step just run, and ends
/// whatever a deadly blow's beat runs out on: the room restarts for the
/// player, a creature dies.
///
/// Runs after the physics step; the knockdown it sets is taken by the
/// character's control on the next frame. A body that is no character
/// cannot brace, and nothing knocks it down.
pub struct HarmSystem;

impl<'a> System<'a> for HarmSystem {
    type SystemData = (
        Entities<'a>,
        Read<'a, Time>,
        Read<'a, PhysicsResource>,
        Write<'a, LevelRestart>,
        Write<'a, DebugLog>,
        WriteStorage<'a, Harm>,
        WriteStorage<'a, Dead>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, CharacterState>,
    );

    fn run(
        &mut self,
        (entities, time, physics, mut restart, mut log, mut harms, mut deads, bodies, states): Self::SystemData,
    ) {
        let dt = time.delta_seconds();
        for (entity, harm, body, state) in
            (&entities, &mut harms, &bodies, (&states).maybe()).join()
        {
            if harm.beat_out(dt) {
                match harm.death {
                    Death::RestartsRoom => restart.requested = true,
                    Death::Dies { lingers } => {
                        let _ = deads.insert(entity, Dead::lingering(lingers));
                    }
                }
            }
            let Some(rigid) = physics.world.body(body.0) else {
                continue;
            };
            let config = harm.config;
            let worst = worst_blow(
                &physics.world,
                body.0,
                rigid.mass(),
                harm.last_velocity,
                state.is_some_and(|state| state.locomotion.in_control()),
                &config,
            );
            if let Some(blow) = worst {
                harm.last_blow = Some((blow.sudden(&config), blow.band(&config)));
                harm.suffer(&blow);
            }
            harm.last_velocity = rigid.linear_velocity();
            if let Some((sudden, band)) = harm.last_blow.filter(|_| log.is_recording()) {
                log.add(
                    format!("Harm/LastBlow/{}", entity.id()),
                    format!("{sudden:.1} m/s, {}", label(band)),
                );
            }
        }
    }
}

fn label(band: Band) -> &'static str {
    match band {
        Band::Light => "light",
        Band::Heavy => "heavy",
        Band::Deadly => "deadly",
    }
}
