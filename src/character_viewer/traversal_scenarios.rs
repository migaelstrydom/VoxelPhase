//! The traversal catalogue: one scenario per station of
//! `levels/traversal_garden.level.ron`, walked and run where speed matters.
//!
//! Every lane runs along +x. The character starts at x = -5, has 12 m to come
//! up to speed, meets its station at x = 7 (the bridges at x = 3) and is
//! given enough script to run out several metres past it. The level file's
//! header draws the lanes.

use std::f32::consts::FRAC_PI_2;

use crate::anim_viewer::{Beat, Script};
use crate::character::LocomotionConfig;

use super::scenario::{CameraRig, Scenario};
use super::traversal::Station;

const GARDEN: &str = "levels/traversal_garden.level.ron";

/// Where every lane's run-up starts, in x.
const START_X: f32 = -5.0;
/// How far past the station the script carries the character, in metres.
const RUN_OUT: f32 = 5.0;
/// Script time allowed on top of the distance at the asked speed, for the
/// start from standing and whatever the station costs.
const SLACK: f32 = 1.5;

/// Walking or running.
#[derive(Clone, Copy)]
enum Gait {
    Walk,
    Run,
}

impl Gait {
    fn beat(self, duration: f32) -> Beat {
        match self {
            Gait::Walk => Beat::walk(duration),
            Gait::Run => Beat::run(duration),
        }
    }

    fn speed(self) -> f32 {
        let config = LocomotionConfig::default();
        match self {
            Gait::Walk => config.walk_speed,
            Gait::Run => config.walk_speed * config.sprint_speed_mul,
        }
    }
}

/// A run along the lane at `lane_z` across `enter..leave` of x.
fn crossing(
    name: &'static str,
    summary: &'static str,
    lane_z: f32,
    enter: f32,
    leave: f32,
    gait: Gait,
) -> Scenario {
    let duration = (leave + RUN_OUT - START_X) / gait.speed() + SLACK;
    Scenario {
        name,
        summary,
        level: GARDEN,
        start: (START_X, lane_z),
        yaw: FRAC_PI_2,
        script: Script::new()
            .then(Beat::stand(0.3))
            .then(gait.beat(duration))
            .then(Beat::stand(0.5)),
        camera: CameraRig::side().with_distance(5.0),
        station: Some(Station::along_x(enter, leave)),
    }
}

/// A lip's station: its face, and the two metres after it in which the body
/// should be back to speed.
fn lip(name: &'static str, summary: &'static str, lane_z: f32, gait: Gait) -> Scenario {
    crossing(name, summary, lane_z, 7.0, 9.0, gait)
}

pub fn catalogue() -> Vec<Scenario> {
    use Gait::{Run, Walk};
    vec![
        lip("lip3_walk", "Traversal: walk up a 3 cm lip", -12.0, Walk),
        lip("lip3_run", "Traversal: run up a 3 cm lip", -12.0, Run),
        lip("lip8_walk", "Traversal: walk up an 8 cm lip", -8.0, Walk),
        lip("lip8_run", "Traversal: run up an 8 cm lip", -8.0, Run),
        lip("lip15_walk", "Traversal: walk up a 15 cm lip", -4.0, Walk),
        lip("lip15_run", "Traversal: run up a 15 cm lip", -4.0, Run),
        lip("lip25_walk", "Traversal: walk up a 25 cm step", 0.0, Walk),
        lip("lip25_run", "Traversal: run up a 25 cm step", 0.0, Run),
        lip("lip40_walk", "Traversal: walk up a 40 cm step", 4.0, Walk),
        lip("lip40_run", "Traversal: run up a 40 cm step", 4.0, Run),
        crossing(
            "planks_walk",
            "Traversal: walk the Test Arena's plank bridge",
            10.0,
            3.0,
            13.0,
            Walk,
        ),
        crossing(
            "planks_run",
            "Traversal: run the Test Arena's plank bridge",
            10.0,
            3.0,
            13.0,
            Run,
        ),
        crossing(
            "planks_wide_walk",
            "Traversal: walk the plank bridge with wide gaps (7 planks)",
            18.0,
            3.0,
            13.0,
            Walk,
        ),
        crossing(
            "planks_wide_run",
            "Traversal: run the plank bridge with wide gaps (7 planks)",
            18.0,
            3.0,
            13.0,
            Run,
        ),
        crossing(
            "rough_walk",
            "Traversal: walk across the rough field",
            26.0,
            7.0,
            19.0,
            Walk,
        ),
        crossing(
            "rough_run",
            "Traversal: run across the rough field",
            26.0,
            7.0,
            19.0,
            Run,
        ),
        crossing(
            "slope40_walk",
            "Traversal: walk up the 40° slope",
            34.0,
            7.0,
            15.0,
            Walk,
        ),
        crossing(
            "slope40_run",
            "Traversal: run up the 40° slope",
            34.0,
            7.0,
            15.0,
            Run,
        ),
        crossing(
            "slope50_walk",
            "Traversal: walk up the 50° slope",
            42.0,
            7.0,
            13.0,
            Walk,
        ),
        crossing(
            "slope50_run",
            "Traversal: run up the 50° slope",
            42.0,
            7.0,
            13.0,
            Run,
        ),
        crossing(
            "scree_walk",
            "Traversal: walk up the 35° scree",
            50.0,
            7.0,
            15.0,
            Walk,
        ),
        crossing(
            "scree_run",
            "Traversal: run up the 35° scree",
            50.0,
            7.0,
            15.0,
            Run,
        ),
        crossing(
            "ledge_walk",
            "Traversal: walk into a 0.7 m ledge (no mantle yet)",
            58.0,
            7.0,
            9.0,
            Walk,
        ),
        Scenario {
            // From behind and above: the cave's walls hide it from the side.
            camera: CameraRig::three_quarter()
                .with_azimuth(250.0)
                .with_elevation(30.0),
            ..crossing(
                "cave_walk",
                "Traversal: walk into a roof 0.9 m up, below the top of the capsule",
                66.0,
                7.0,
                19.0,
                Walk,
            )
        },
    ]
}
