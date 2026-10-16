//! The catalogue.
//!
//! Every scenario is set in `levels/swim_test.level.ron`, whose beach runs
//! along z = 0 from dry land at x = -8 down to a 3 m sea floor at x = 12. The
//! water's edge is at x = -3 and the depth grows 0.2 m per metre east of it.
//! The pier stands 2 m over the deep end at x -8..8, z -20..-12.
//!
//! The exception is `climb_ramp`, in the Rubble Garden: the way up its cliff
//! to the lips, a ramp at x = 92 from the floor at z = 95 to the top (12 m) at
//! z = 61. And `climb_steep`, in the Water Park: the ramp down to the cavern
//! courtyard at x = -33, rising 11 m from z = -12 to z = -24 (about 43°). And
//! `contour` and `run_uphill`, on Island Sea's hillside: along it with the
//! ground falling away on the right, and straight up it at a run.
//!
//! The traversal scenarios, in the Traversal Garden, are catalogued in
//! `traversal_scenarios` and appended here.

use std::f32::consts::{FRAC_PI_2, PI};

use nalgebra::Vector3;

use crate::anim_viewer::{Beat, Script};

use super::scenario::{CameraRig, Scenario};
use super::traversal_scenarios;

const LEVEL: &str = "levels/swim_test.level.ron";
const GARDEN: &str = "levels/rubble_garden.level.ron";
const PARK: &str = "levels/water_park.level.ron";
const ISLAND: &str = "levels/island_sea.level.ron";

/// Facing +x, down the beach into the sea.
const SEAWARD: f32 = FRAC_PI_2;

pub fn catalogue() -> Vec<Scenario> {
    let mut scenarios = vec![
        Scenario {
            name: "wade_in",
            summary: "walk down the beach into the sea until it is too deep to stand",
            level: LEVEL,
            start: (-6.0, 0.0),
            yaw: SEAWARD,
            script: Script::new()
                .then(Beat::stand(0.5))
                .then(Beat::walk(4.0).named("wade"))
                .then(Beat::stand(2.0).named("tread")),
            camera: CameraRig::side().with_distance(4.5),
            station: None,
        },
        Scenario {
            name: "wade_stand",
            summary: "stand still waist deep, then walk on, then stop",
            level: LEVEL,
            start: (-1.0, 0.0),
            yaw: SEAWARD,
            script: Script::new()
                .then(Beat::stand(0.5))
                .then(Beat::walk(0.6).named("wade"))
                .then(Beat::stand(1.5))
                .then(Beat::walk(0.6).named("wade"))
                .then(Beat::stand(1.5)),
            camera: CameraRig::three_quarter(),
            station: None,
        },
        Scenario {
            name: "fall_in",
            summary: "walk off the end of the pier into deep water",
            level: LEVEL,
            start: (5.0, -16.0),
            yaw: SEAWARD,
            script: Script::new()
                .then(Beat::stand(0.5))
                .then(Beat::walk(1.2))
                .then(Beat::stand(3.0).named("tread")),
            camera: CameraRig::side().with_distance(6.0).with_elevation(8.0),
            station: None,
        },
        Scenario {
            name: "jump_in",
            summary: "run and jump off the end of the pier",
            level: LEVEL,
            start: (3.0, -16.0),
            yaw: SEAWARD,
            script: Script::new()
                .then(Beat::stand(0.3))
                .then(Beat::run(0.55))
                .then(Beat::jump(0.6))
                .then(Beat::stand(3.0).named("tread")),
            camera: CameraRig::side().with_distance(7.0).with_elevation(8.0),
            station: None,
        },
        Scenario {
            name: "tread",
            summary: "float in place in deep water",
            level: LEVEL,
            start: (18.0, 0.0),
            yaw: SEAWARD,
            script: Script::new().then(Beat::stand(4.0).named("tread")),
            camera: CameraRig::three_quarter(),
            station: None,
        },
        Scenario {
            name: "swim",
            summary: "swim out from treading, turn a half circle, stop",
            level: LEVEL,
            start: (14.0, 4.0),
            yaw: SEAWARD,
            script: Script::new()
                .then(Beat::stand(1.0).named("tread"))
                .then(Beat::walk(3.0).named("swim"))
                .then(Beat::walk(2.0).named("turn").turning(PI / 2.0))
                .then(Beat::stand(2.0).named("tread")),
            camera: CameraRig::three_quarter().with_distance(5.0),
            station: None,
        },
        Scenario {
            name: "swim_fast",
            summary: "sprint-swim in a straight line",
            level: LEVEL,
            start: (12.0, 4.0),
            yaw: SEAWARD,
            script: Script::new()
                .then(Beat::stand(0.5).named("tread"))
                .then(Beat::run(3.0).named("sprint")),
            camera: CameraRig::side(),
            station: None,
        },
        Scenario {
            name: "ashore",
            summary: "swim in to the beach and walk out of the water",
            level: LEVEL,
            start: (14.0, 0.0),
            yaw: -SEAWARD,
            script: Script::new()
                .then(Beat::stand(0.5).named("tread"))
                .then(Beat::walk(9.0).named("ashore").towards(-Vector3::x()))
                .then(Beat::stand(1.5)),
            camera: CameraRig::side().with_distance(5.0),
            station: None,
        },
        Scenario {
            name: "climb_ramp",
            summary: "Rubble Garden: run up the ramp onto the cliff top",
            level: GARDEN,
            start: (92.0, 94.0),
            yaw: PI,
            script: Script::new()
                .then(Beat::stand(0.5))
                .then(Beat::run(9.0).named("climb").towards(-Vector3::z()))
                .then(Beat::stand(1.0)),
            camera: CameraRig::side().with_azimuth(90.0).with_distance(8.0),
            station: None,
        },
        Scenario {
            name: "climb_steep",
            summary: "Water Park: walk up the 43° ramp from the cavern courtyard",
            level: PARK,
            start: (-33.0, -12.5),
            yaw: PI,
            script: Script::new()
                .then(Beat::stand(0.5))
                .then(Beat::walk(5.0).named("climb").towards(-Vector3::z()))
                .then(Beat::stand(1.0)),
            camera: CameraRig::side().with_azimuth(90.0).with_distance(6.0),
            station: None,
        },
        Scenario {
            name: "contour",
            summary: "Island Sea: walk along the hillside, the slope falling away to the right",
            level: ISLAND,
            start: (-5.91, 8.05),
            yaw: 2.04,
            script: Script::new()
                .then(Beat::stand(0.5))
                .then(
                    Beat::walk(3.0)
                        .named("contour")
                        .towards(Vector3::new(0.893, 0.0, -0.450)),
                )
                .then(Beat::stand(1.0)),
            camera: CameraRig::three_quarter().with_azimuth(296.0),
            station: None,
        },
        Scenario {
            name: "throw",
            summary: "throw standing, then walking, then two in quick succession",
            level: LEVEL,
            start: (-7.0, 6.0),
            yaw: SEAWARD,
            script: Script::new()
                .then(Beat::stand(0.3))
                .then(Beat::stand(0.9).throwing().named("throw"))
                .then(
                    Beat::walk(0.9)
                        .towards(Vector3::x())
                        .throwing()
                        .named("walk_throw"),
                )
                .then(Beat::stand(0.15).throwing().named("rapid"))
                .then(Beat::stand(0.9).throwing().named("rapid")),
            camera: CameraRig::side().with_distance(3.0),
            station: None,
        },
        Scenario {
            name: "throw_around",
            summary: "standing, throw to the left, then behind, then to the right, each turning \
                      from where the last left the body; then behind while walking",
            level: LEVEL,
            start: (-7.0, 6.0),
            yaw: SEAWARD,
            script: Script::new()
                .then(Beat::stand(0.3))
                .then(Beat::stand(1.0).throwing_at(-Vector3::z()).named("left"))
                .then(Beat::stand(1.0).throwing_at(Vector3::z()).named("behind"))
                .then(Beat::stand(1.0).throwing_at(-Vector3::x()).named("right"))
                .then(Beat::walk(1.2).towards(-Vector3::x()).named("turn"))
                .then(
                    Beat::walk(1.2)
                        .towards(-Vector3::x())
                        .throwing_at(Vector3::x())
                        .named("walk_behind"),
                ),
            camera: CameraRig::side().with_distance(2.2).with_elevation(10.0),
            station: None,
        },
        Scenario {
            name: "run_uphill",
            summary: "Island Sea: run up the hillside",
            level: ISLAND,
            start: (-3.74, 10.09),
            yaw: -2.72,
            script: Script::new()
                .then(Beat::stand(0.5))
                .then(
                    Beat::run(1.5)
                        .named("uphill")
                        .towards(Vector3::new(-0.413, 0.0, -0.911)),
                )
                .then(Beat::stand(1.0)),
            camera: CameraRig::side().with_azimuth(-2.72f32.to_degrees() + 180.0),
            station: None,
        },
    ];
    scenarios.extend(traversal_scenarios::catalogue());
    scenarios
}

pub fn find(name: &str) -> Option<Scenario> {
    catalogue().into_iter().find(|s| s.name == name)
}
