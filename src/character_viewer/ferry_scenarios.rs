//! The ferry fish (`levels/ferry_fish.level.ron`) ridden across its strait:
//! wait on the south cliff, hop onto its back as it passes, ride it round,
//! and hop off onto the north cliff; or walk back along it and ride on its
//! tail.

use nalgebra::Vector3;

use crate::anim_viewer::{Beat, Script};

use super::scenario::{CameraRig, Judged, Scenario};

const FERRY: &str = "levels/ferry_fish.level.ron";
const NORTH: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);
const WEST: Vector3<f32> = Vector3::new(-1.0, 0.0, 0.0);

pub fn catalogue() -> Vec<Scenario> {
    vec![ferry_ride(), ferry_tail()]
}

/// The fish sets off along the south cliff heading east and passes the
/// spot the player waits at after about four seconds; it comes back along
/// the north cliff heading west a dozen seconds after that.
fn ferry_ride() -> Scenario {
    Scenario {
        name: "ferry_ride",
        summary: "Ferry Fish: hop onto the fish from the south cliff, ride it round, hop off north",
        level: FERRY,
        start: (0.0, -11.2),
        yaw: 0.0,
        script: Script::new()
            .then(Beat::stand(3.2).named("wait"))
            .then(Beat::walk(0.3).towards(NORTH).named("run-up"))
            .then(Beat::jump(0.7).towards(NORTH).named("board"))
            .then(Beat::stand(12.0).named("ride"))
            .then(Beat::walk(0.3).towards(NORTH).named("run-up off"))
            .then(Beat::jump(0.9).towards(NORTH).named("alight"))
            .then(Beat::stand(1.5).named("ashore")),
        camera: CameraRig::three_quarter()
            .with_distance(9.0)
            .with_elevation(24.0),
        judged: Judged::Motion,
    }
}

/// Boarded as in `ferry_ride`, then walked back along the fish (it swims
/// east) onto the stalk its tail grows from, and ridden there round the
/// strait's east end: what is stood on reaches as far back as its body.
fn ferry_tail() -> Scenario {
    Scenario {
        name: "ferry_tail",
        summary: "Ferry Fish: hop onto the fish, walk back onto its tail stalk and ride there",
        level: FERRY,
        start: (0.0, -11.2),
        yaw: 0.0,
        script: Script::new()
            .then(Beat::stand(3.2).named("wait"))
            .then(Beat::walk(0.3).towards(NORTH).named("run-up"))
            .then(Beat::jump(0.7).towards(NORTH).named("board"))
            .then(Beat::stand(0.6).named("land"))
            .then(Beat::walk(0.45).towards(WEST).named("walk aft"))
            .then(Beat::stand(5.0).named("ride the tail")),
        camera: CameraRig::three_quarter()
            .with_distance(7.0)
            .with_elevation(30.0),
        judged: Judged::Motion,
    }
}
