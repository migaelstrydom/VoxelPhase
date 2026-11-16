//! Seeded random play: scripts of the stick, the jump and the crouch that no
//! scenario's author thought of, started from places nobody picked.
//!
//! A scenario is written about a case someone already saw. The legs' bugs
//! are the cases nobody has yet: a reversal a frame after a ledge, a jump
//! out of a crouch on a slope. Each seed is one run, the same on every
//! machine, named `soak:<level>:<seed>` so that a run the pose audit
//! (`pose_audit`) faults can be played again and filmed like any scenario.

use std::f32::consts::{PI, TAU};

use nalgebra::Vector3;
use traversal_planner::fuzz::Rng;

use crate::anim_viewer::{Beat, Script};

use super::scenario::{CameraRig, Judged, Scenario};

/// How long a soak run plays, in seconds.
const RUN_SECONDS: f32 = 7.0;
/// How long a run stands where it was dropped before its script begins, in
/// seconds: a random start may be inside a rock or over a crate, and the
/// body is shoved clear of it first.
const SETTLE_SECONDS: f32 = 0.6;
/// The label of that opening beat, which the pose audit does not judge.
pub const SETTLE: &str = "settle";

/// A level the soak plays in, and the ground it starts on.
#[derive(Clone, Copy, Debug)]
pub struct SoakLevel {
    /// The name a soak scenario gives it.
    pub name: &'static str,
    /// Level file, relative to the repository root.
    pub level: &'static str,
    /// The x range starts are drawn from.
    pub x: (f32, f32),
    /// The z range starts are drawn from.
    pub z: (f32, f32),
}

/// The levels the soak plays in: the gardens, whose every lane is an
/// obstacle, and the yards the game is played in.
pub const SOAK_LEVELS: [SoakLevel; 4] = [
    SoakLevel {
        name: "traversal_garden",
        level: "levels/traversal_garden.level.ron",
        x: (-12.0, 36.0),
        z: (-14.0, 92.0),
    },
    SoakLevel {
        name: "ledge_garden",
        level: "levels/ledge_garden.level.ron",
        x: (-6.0, 22.0),
        z: (-6.0, 78.0),
    },
    SoakLevel {
        name: "stack_yard",
        level: "levels/stack_yard.level.ron",
        x: (8.0, 120.0),
        z: (4.0, 120.0),
    },
    SoakLevel {
        name: "test_arena",
        level: "levels/test_arena.level.ron",
        x: (-40.0, 60.0),
        z: (-40.0, 60.0),
    },
];

/// The soak level named `name`.
pub fn soak_level(name: &str) -> Option<SoakLevel> {
    SOAK_LEVELS.into_iter().find(|level| level.name == name)
}

/// The scenario `soak:<level>:<seed>` names, if `name` is one.
pub fn find_soak(name: &str) -> Option<Scenario> {
    let mut parts = name.strip_prefix("soak:")?.split(':');
    let level = soak_level(parts.next()?)?;
    let seed = parts.next()?.parse().ok()?;
    parts.next().is_none().then(|| soak_scenario(&level, seed))
}

/// Seed `seed`'s run in `level`: a start and a facing drawn from the level's
/// ground, and a script of random beats.
pub fn soak_scenario(level: &SoakLevel, seed: u64) -> Scenario {
    // The level's name is folded into the stream so that the same seed is a
    // different run in each level.
    let salt = level
        .name
        .bytes()
        .fold(0u64, |hash, byte| hash.wrapping_mul(131) ^ u64::from(byte));
    let mut rng = Rng::new(seed.wrapping_mul(0x2545_f491_4f6c_dd1d) ^ salt);
    let start = (
        rng.range(level.x.0, level.x.1),
        rng.range(level.z.0, level.z.1),
    );
    let heading = rng.range(0.0, TAU);
    let name: &'static str = Box::leak(format!("soak:{}:{seed}", level.name).into_boxed_str());
    Scenario {
        name,
        summary: "seeded random play",
        level: level.level,
        start,
        yaw: heading,
        script: random_script(&mut rng, heading),
        camera: CameraRig::three_quarter().with_distance(3.5),
        judged: Judged::Motion,
    }
}

/// A script of random beats setting off along `heading` (radians about +y,
/// 0 facing +z): mostly travel, with the turns, stops, reversals, jumps and
/// crouches a player makes in the middle of it.
fn random_script(rng: &mut Rng, mut heading: f32) -> Script {
    let mut script = Script::new().then(Beat::stand(SETTLE_SECONDS).named(SETTLE));
    let mut sprint = rng.chance(0.5);
    while script.duration() < RUN_SECONDS {
        let towards = |heading: f32| Vector3::new(heading.sin(), 0.0, heading.cos());
        let travel = |seconds: f32, heading: f32, sprint: bool| {
            let beat = if sprint {
                Beat::run(seconds)
            } else {
                Beat::walk(seconds)
            };
            beat.towards(towards(heading))
        };
        match rng.next_u64() % 20 {
            0..=5 => {
                script = script.then(travel(rng.range(0.3, 1.2), heading, sprint));
            }
            6..=7 => {
                heading += rng.range(-0.8, 0.8);
                script = script.then(travel(rng.range(0.3, 1.0), heading, sprint).named("veer"));
            }
            8 => {
                heading += PI;
                script = script.then(travel(rng.range(0.1, 0.8), heading, sprint).named("back"));
            }
            // The stick flicked back and on again: in the air, if it follows
            // a jump or a ledge.
            9..=10 => {
                let flick = rng.range(0.05, 0.3);
                script = script
                    .then(travel(flick, heading + PI, sprint).named("flick"))
                    .then(travel(rng.range(0.3, 0.9), heading, sprint));
            }
            11 => {
                heading += rng.pick(&[-0.5 * PI, 0.5 * PI]);
                script = script.then(travel(rng.range(0.3, 0.9), heading, sprint).named("side"));
            }
            12 => {
                script = script.then(Beat::stand(rng.range(0.1, 0.6)));
            }
            13..=15 => {
                let held = rng.range(0.08, 0.5);
                let jump = if rng.chance(0.2) {
                    Beat::hop(held)
                } else {
                    Beat {
                        sprint,
                        ..Beat::jump(held).towards(towards(heading))
                    }
                };
                script = script.then(jump);
            }
            // A crouch pressed a moment before the jump: a long jump.
            16 => {
                script = script
                    .then(Beat::crouch_walk(0.08).towards(towards(heading)))
                    .then(Beat {
                        crouch: true,
                        ..Beat::jump(rng.range(0.2, 0.5)).towards(towards(heading))
                    });
            }
            17 => {
                let crouched = if rng.chance(0.3) {
                    Beat::crouch_still(rng.range(0.2, 0.5))
                } else {
                    Beat::crouch_walk(rng.range(0.3, 1.0)).towards(towards(heading))
                };
                script = script.then(crouched);
            }
            18 => {
                let rate = rng.range(1.0, 4.0) * rng.pick(&[-1.0, 1.0]);
                let seconds = rng.range(0.5, 1.5);
                script = script.then(travel(seconds, heading, sprint).turning(rate).named("turn"));
            }
            _ => sprint = !sprint,
        }
    }
    script.then(Beat::stand(0.5))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_seed_names_one_run_and_the_name_finds_it_again() {
        let level = soak_level("traversal_garden").expect("a soak level");
        let run = soak_scenario(&level, 7);
        assert_eq!(run.name, "soak:traversal_garden:7");
        let again = find_soak(run.name).expect("found by name");
        assert_eq!(again.start, run.start);
        assert_eq!(again.script.beats.len(), run.script.beats.len());
        assert!(run.script.duration() >= RUN_SECONDS);
        let other = soak_scenario(&level, 8);
        assert_ne!(other.start, run.start);
        assert!(find_soak("soak:nowhere:7").is_none());
        assert!(find_soak("soak:traversal_garden:x").is_none());
    }
}
