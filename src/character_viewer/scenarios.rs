//! The catalogue.
//!
//! Every scenario is set in `levels/swim_test.level.ron`, whose beach runs
//! along z = 0 from dry land at x = -8 down to a 3 m sea floor at x = 12. The
//! water's edge is at x = -3 and the depth grows 0.2 m per metre east of it.
//! The pier stands 2 m over the deep end at x -8..8, z -20..-12, and the
//! quay 0.3 m over it east of x = 20.
//!
//! The exception is `climb_ramp`, in the Rubble Garden: the way up its cliff
//! to the lips, a ramp at x = 92 from the floor at z = 95 to the top (12 m) at
//! z = 61. And `climb_steep`, in the Water Park: the ramp down to the cavern
//! courtyard at x = -33, rising 11 m from z = -12 to z = -24 (about 43°). And
//! `contour` and `run_uphill`, on Island Sea's hillside: along it with the
//! ground falling away on the right, and straight up it at a run, and
//! `swim_off_edge` off its ocean's south edge at z = 56, where the world
//! ends. And
//! `air_reverse`, `jump_straight`, `jump_standing`, `run_reverse` and
//! `crouch_walk`, on the
//! Traversal Garden's open ground,
//! `hop_reverse` at its 25 cm step, `drop_reverse` off its 40 cm step, and
//! `shelf_sprint_off` and `shelf_dither` at the side of its scree's shelf.
//! And `ride_ferry`, on the Traversal Garden's ferry: a moving platform
//! shuttling along z = 90 between two decks level with it.
//!
//! The traversal scenarios, in the Traversal Garden, are catalogued in
//! `traversal_scenarios` and appended here, and after them the turning runs,
//! on the garden's open ground, in `turning`.

use std::f32::consts::{FRAC_PI_2, PI};

use nalgebra::Vector3;

use crate::anim_viewer::{Beat, Script};

use super::scenario::{CameraRig, Judged, Scenario};
use super::{traversal_scenarios, turning};

const LEVEL: &str = "levels/swim_test.level.ron";
const GARDEN: &str = "levels/rubble_garden.level.ron";
const PARK: &str = "levels/water_park.level.ron";
const ISLAND: &str = "levels/island_sea.level.ron";
const YARD: &str = "levels/wrecking_yard.level.ron";
const TRAVERSAL_GARDEN: &str = "levels/traversal_garden.level.ron";

/// Facing +x, down the beach into the sea.
const SEAWARD: f32 = FRAC_PI_2;

pub fn catalogue() -> Vec<Scenario> {
    let mut scenarios = vec![
        Scenario {
            name: "trampoline",
            summary:
                "jump onto the Wrecking Yard's trampoline, ride it out, then pump it with hops",
            level: YARD,
            start: (71.4, 34.0),
            yaw: SEAWARD,
            script: Script::new()
                .then(Beat::stand(0.5))
                .then(Beat::jump(0.7).named("on"))
                .then(Beat::stand(3.0).named("ride"))
                .then(Beat::hop(1.0).named("pump"))
                .then(Beat::hop(1.0).named("pump"))
                .then(Beat::hop(1.0).named("pump"))
                .then(Beat::hop(1.2).named("pump"))
                .then(Beat::stand(2.0).named("ride")),
            camera: CameraRig::side().with_distance(7.0).with_elevation(10.0),
            judged: Judged::Motion,
        },
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
            judged: Judged::Motion,
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
            judged: Judged::Motion,
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
            judged: Judged::Motion,
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
            judged: Judged::Motion,
        },
        Scenario {
            name: "tread",
            summary: "float in place in deep water",
            level: LEVEL,
            start: (18.0, 0.0),
            yaw: SEAWARD,
            script: Script::new().then(Beat::stand(4.0).named("tread")),
            camera: CameraRig::three_quarter(),
            judged: Judged::Motion,
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
            judged: Judged::Motion,
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
            judged: Judged::Motion,
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
            judged: Judged::Motion,
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
            judged: Judged::Motion,
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
            judged: Judged::Motion,
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
            judged: Judged::Motion,
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
            judged: Judged::Motion,
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
            judged: Judged::Motion,
        },
        Scenario {
            name: "swim_off_edge",
            summary: "Island Sea: swim south off the edge of the world and fall",
            level: ISLAND,
            start: (0.0, 50.0),
            yaw: 0.0,
            script: Script::new()
                .then(Beat::stand(0.5).named("tread"))
                .then(Beat::walk(5.0).named("swim").towards(Vector3::z()))
                .then(Beat::stand(2.0).named("fall")),
            camera: CameraRig::side().with_distance(7.0).with_elevation(10.0),
            judged: Judged::Motion,
        },
        Scenario {
            name: "water_jump",
            summary: "tread water in the deep end, tap jump, then hold it: a short jump out \
                      of the water and a full one",
            level: LEVEL,
            start: (14.0, 0.0),
            yaw: SEAWARD,
            script: Script::new()
                .then(Beat::stand(1.5).named("tread"))
                .then(Beat::jump(0.05).named("tap"))
                .then(Beat::stand(2.5).named("back_in"))
                .then(Beat::jump(0.6).named("hold"))
                .then(Beat::stand(2.5).named("back_in")),
            camera: CameraRig::side().with_distance(5.0),
            judged: Judged::Motion,
        },
        Scenario {
            name: "water_jump_stroke",
            summary: "swim along the deep end and jump out of the water mid-stroke",
            level: LEVEL,
            start: (13.0, 4.0),
            yaw: 0.0,
            script: Script::new()
                .then(Beat::stand(1.0).named("tread"))
                .then(Beat::walk(1.5).named("stroke").towards(-Vector3::z()))
                .then(Beat::jump(0.6).named("jump").towards(-Vector3::z()))
                .then(Beat::walk(2.5).named("back_in").towards(-Vector3::z())),
            camera: CameraRig::side().with_distance(5.0),
            judged: Judged::Motion,
        },
        Scenario {
            name: "jump_ashore",
            summary: "swim up to the quay and jump out of the water onto it",
            level: LEVEL,
            start: (16.0, 0.0),
            yaw: SEAWARD,
            script: Script::new()
                .then(Beat::stand(1.0).named("tread"))
                .then(Beat::walk(1.2).named("swim").towards(Vector3::x()))
                .then(Beat::jump(0.6).named("jump").towards(Vector3::x()))
                .then(Beat::walk(1.5).named("ashore").towards(Vector3::x()))
                .then(Beat::stand(1.0)),
            camera: CameraRig::side().with_distance(5.0),
            judged: Judged::Motion,
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
            judged: Judged::Motion,
        },
        Scenario {
            name: "air_reverse",
            summary: "Traversal Garden: run, jump, and steer back the way it came in the air",
            level: TRAVERSAL_GARDEN,
            start: (-12.0, -15.0),
            yaw: SEAWARD,
            script: Script::new()
                .then(Beat::stand(0.3))
                .then(Beat::walk(1.2))
                .then(Beat::jump(0.1).named("jump"))
                .then(Beat::walk(0.8).towards(-Vector3::x()).named("back"))
                .then(Beat::stand(0.6)),
            camera: CameraRig::side().with_distance(3.0),
            judged: Judged::Motion,
        },
        Scenario {
            name: "jump_straight",
            summary: "Traversal Garden: run, jump, and keep the stick ahead through the jump",
            level: TRAVERSAL_GARDEN,
            start: (-15.0, -15.0),
            yaw: SEAWARD,
            script: Script::new()
                .then(Beat::stand(0.3))
                .then(Beat::walk(1.5))
                .then(Beat::jump(0.1).named("jump"))
                .then(Beat::walk(1.2).named("on"))
                .then(Beat::stand(0.5)),
            camera: CameraRig::side().with_distance(3.0),
            judged: Judged::Motion,
        },
        Scenario {
            name: "jump_standing",
            summary: "Traversal Garden: jump from standing, the stick ahead through the jump",
            level: TRAVERSAL_GARDEN,
            start: (-15.0, -15.0),
            yaw: SEAWARD,
            script: Script::new()
                .then(Beat::stand(0.3))
                .then(Beat::jump(1.5).named("jump"))
                .then(Beat::stand(0.5)),
            camera: CameraRig::side().with_distance(6.0),
            judged: Judged::Motion,
        },
        Scenario {
            name: "run_reverse",
            summary: "Traversal Garden: run, then turn the stick straight back",
            level: TRAVERSAL_GARDEN,
            start: (-15.0, -15.0),
            yaw: SEAWARD,
            script: Script::new()
                .then(Beat::stand(0.3))
                .then(Beat::walk(1.8))
                .then(Beat::walk(1.2).towards(-Vector3::x()).named("back"))
                .then(Beat::stand(0.5)),
            camera: CameraRig::side().with_distance(3.0),
            judged: Judged::Motion,
        },
        Scenario {
            name: "hop_reverse",
            summary: "Traversal Garden: walk at the 25 cm step, and steer back once the hop onto it has taken off (with --track)",
            level: TRAVERSAL_GARDEN,
            start: (-5.0, 0.0),
            yaw: SEAWARD,
            script: Script::new()
                .then(Beat::stand(0.3))
                .then(Beat::walk(2.3))
                .then(Beat::walk(0.8).towards(-Vector3::x()).named("back"))
                .then(Beat::stand(0.6)),
            camera: CameraRig::side().with_distance(3.0),
            judged: Judged::Motion,
        },
        Scenario {
            name: "drop_reverse",
            summary: "Traversal Garden: run west off the 40 cm step, and steer back east in the air",
            level: TRAVERSAL_GARDEN,
            start: (11.0, 4.0),
            yaw: -SEAWARD,
            script: Script::new()
                .then(Beat::stand(0.3))
                .then(Beat::run(0.7).towards(-Vector3::x()))
                .then(Beat::run(0.8).towards(Vector3::x()).named("back"))
                .then(Beat::stand(0.6)),
            camera: CameraRig::side().with_distance(3.0),
            judged: Judged::Motion,
        },
        Scenario {
            name: "shelf_sprint_off",
            summary: "Traversal Garden: sprint off the side of the scree's 5.6 m shelf and land",
            level: TRAVERSAL_GARDEN,
            start: (17.0, 50.0),
            yaw: 0.0,
            script: Script::new()
                .then(Beat::stand(0.3))
                .then(Beat::run(1.8).towards(Vector3::z()))
                .then(Beat::run(0.8).towards(Vector3::z()).named("landed"))
                .then(Beat::stand(0.6)),
            camera: CameraRig::side().with_distance(4.0),
            judged: Judged::Motion,
        },
        Scenario {
            name: "shelf_dither",
            summary: "Traversal Garden: run back and forth at the side of the scree's 5.6 m shelf, then off it",
            level: TRAVERSAL_GARDEN,
            start: (17.0, 50.5),
            yaw: 0.0,
            script: Script::new()
                .then(Beat::stand(0.3))
                .then(Beat::run(0.5).towards(Vector3::z()))
                .then(Beat::run(0.35).towards(-Vector3::z()).named("back"))
                .then(Beat::run(0.35).towards(Vector3::z()).named("forth"))
                .then(Beat::run(0.3).towards(-Vector3::z()).named("back"))
                .then(Beat::run(0.3).towards(Vector3::z()).named("forth"))
                .then(Beat::run(0.25).towards(-Vector3::z()).named("back"))
                .then(Beat::run(1.5).towards(Vector3::z()).named("off"))
                .then(Beat::stand(0.6)),
            camera: CameraRig::side().with_distance(4.0),
            judged: Judged::Motion,
        },
        Scenario {
            name: "crouch_walk",
            summary: "Traversal Garden: crouch and walk on level ground",
            level: TRAVERSAL_GARDEN,
            start: (-12.0, -15.0),
            yaw: SEAWARD,
            script: Script::new()
                .then(Beat::crouch_still(0.6))
                .then(Beat::crouch_walk(2.0))
                .then(Beat::crouch_still(0.5)),
            camera: CameraRig::side().with_distance(2.5).with_elevation(5.0),
            judged: Judged::Motion,
        },
        Scenario {
            name: "ride_ferry",
            summary: "Traversal Garden: walk onto the ferry, ride it across, walk off",
            level: TRAVERSAL_GARDEN,
            start: (2.4, 90.0),
            yaw: SEAWARD,
            script: Script::new()
                .then(Beat::stand(0.1))
                .then(Beat::walk(0.4).named("board"))
                .then(Beat::stand(3.8).named("ride"))
                .then(Beat::walk(1.2).named("alight"))
                .then(Beat::stand(1.0)),
            camera: CameraRig::side().with_distance(6.0).with_elevation(20.0),
            judged: Judged::Motion,
        },
    ];
    scenarios.extend(traversal_scenarios::catalogue());
    scenarios.extend(turning::catalogue());
    scenarios
}

pub fn find(name: &str) -> Option<Scenario> {
    catalogue().into_iter().find(|s| s.name == name)
}
