//! The puzzle rooms of `levels/rooms/`, played through: the steps of each
//! room's way out that a script can be trusted with, done by the body the
//! player has.

use nalgebra::Vector3;

use crate::anim_viewer::{Beat, Script};

use super::scenario::{CameraRig, Judged, Scenario};

const E3: &str = "levels/rooms/e3_rubble_ramp.level.ron";
const NORTH: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);
const EAST: Vector3<f32> = Vector3::new(1.0, 0.0, 0.0);
const WEST: Vector3<f32> = Vector3::new(-1.0, 0.0, 0.0);

pub fn catalogue() -> Vec<Scenario> {
    vec![e3_legs()]
}

/// E3: blow the slab's four legs from the floor and watch it come down.
///
/// Thrown from 4.3 m back: a grenade arms 0.15 s after it leaves the hand,
/// 3 m at throw speed, and one that strikes a leg nearer than that bounces
/// back unarmed and goes off at the thrower's feet. The climb is left to
/// play: the slab cracks as it falls, and the pile it makes is not one a
/// fixed script can be trusted to find a way up.
fn e3_legs() -> Scenario {
    Scenario {
        name: "room_e3_legs",
        summary: "Room E3: blow the slab's four legs from 4 m back and watch it come down",
        level: E3,
        start: (13.0, 11.0),
        yaw: 0.0,
        script: Script::new()
            .then(Beat::stand(0.3))
            .then(Beat::stand(3.0).throwing_at(NORTH).named("front west"))
            .then(Beat::walk(1.2).towards(EAST).named("east"))
            .then(Beat::stand(3.0).throwing_at(NORTH).named("front east"))
            .then(Beat::stand(3.0).throwing_at(NORTH).named("back east"))
            .then(Beat::walk(1.2).towards(WEST).named("west"))
            .then(Beat::stand(3.0).throwing_at(NORTH).named("back west"))
            .then(Beat::stand(4.0).named("settle")),
        camera: CameraRig::side()
            .with_azimuth(-100.0)
            .with_distance(14.0)
            .with_elevation(30.0),
        judged: Judged::Motion,
    }
}
