//! The catalogue.
//!
//! Every scenario is set in `levels/swim_test.level.ron`, whose beach runs
//! along z = 0 from dry land at x = -8 down to a 3 m sea floor at x = 12. The
//! water's edge is at x = -3 and the depth grows 0.2 m per metre east of it.
//! The pier stands 2 m over the deep end at x -8..8, z -20..-12.

use std::f32::consts::{FRAC_PI_2, PI};

use nalgebra::Vector3;

use crate::anim_viewer::{Beat, Script};

use super::scenario::{CameraRig, Scenario};

const LEVEL: &str = "levels/swim_test.level.ron";

/// Facing +x, down the beach into the sea.
const SEAWARD: f32 = FRAC_PI_2;

pub fn catalogue() -> Vec<Scenario> {
    vec![
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
        },
        Scenario {
            name: "tread",
            summary: "float in place in deep water",
            level: LEVEL,
            start: (18.0, 0.0),
            yaw: SEAWARD,
            script: Script::new().then(Beat::stand(4.0).named("tread")),
            camera: CameraRig::three_quarter(),
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
        },
    ]
}

pub fn find(name: &str) -> Option<Scenario> {
    catalogue().into_iter().find(|s| s.name == name)
}
