use nalgebra::Vector3;
use specs::{Join, Read, ReadStorage, System, Write, WriteStorage};

use super::blow::{Band, Blow};
use super::component::Harm;
use super::config::HarmConfig;
use crate::character::CharacterState;
use crate::components::RigidBodyComponent;
use crate::debug::DebugLog;
use crate::physics::{PhysicsWorld, RigidBodyHandle};
use crate::systems::{LevelRestart, PhysicsResource};
use crate::time::Time;

/// The worst blow `body` took in the last step, of its hits and shoves:
/// the most sudden. A hit that continues a stop split across frames is the
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
    hits.chain(shoves)
        .max_by(|a, b| a.sudden(config).total_cmp(&b.sudden(config)))
}

/// Judges the blows each harmable character took in the step just run, and
/// restarts the room when a deadly blow's beat runs out.
///
/// Runs after the physics step; the knockdown it sets is taken by the
/// character's control on the next frame.
pub struct HarmSystem;

impl<'a> System<'a> for HarmSystem {
    type SystemData = (
        Read<'a, Time>,
        Read<'a, PhysicsResource>,
        Write<'a, LevelRestart>,
        Write<'a, DebugLog>,
        WriteStorage<'a, Harm>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, CharacterState>,
    );

    fn run(
        &mut self,
        (time, physics, mut restart, mut log, mut harms, bodies, states): Self::SystemData,
    ) {
        let dt = time.delta_seconds();
        for (harm, body, state) in (&mut harms, &bodies, &states).join() {
            if harm.beat_out(dt) {
                restart.requested = true;
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
                state.locomotion.in_control(),
                &config,
            );
            if let Some(blow) = worst {
                let band = blow.band(&config);
                harm.last_blow = Some((blow.sudden(&config), band));
                harm.suffer(band);
            }
            harm.last_velocity = rigid.linear_velocity();
            if let Some((sudden, band)) = harm.last_blow {
                log.add("Harm/LastBlow", format!("{sudden:.1} m/s, {}", label(band)));
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
