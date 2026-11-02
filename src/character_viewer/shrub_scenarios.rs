//! The shrub scenarios, in the Shrub Garden (`levels/shrub_garden.level.ron`):
//! wading through the hedge row, a grenade thrown into a big shrub to tear
//! it loose, a loose clump kicked along the flat, and one shoved off the
//! terrace to tumble down the ramp.

use std::f32::consts::PI;

use nalgebra::Vector3;

use crate::anim_viewer::{Beat, Script};

use super::scenario::{CameraRig, Judged, Scenario};

const SHRUB_GARDEN: &str = "levels/shrub_garden.level.ron";
/// North, up the garden.
const NORTH: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);
const SOUTH: Vector3<f32> = Vector3::new(0.0, 0.0, -1.0);

pub fn catalogue() -> Vec<Scenario> {
    vec![
        Scenario {
            name: "shrub_wade",
            summary: "Shrub Garden: walk north through the hedge row's shrub at x = -3.8",
            level: SHRUB_GARDEN,
            start: (-3.8, 3.0),
            yaw: 0.0,
            script: Script::new()
                .then(Beat::stand(0.3))
                .then(Beat::walk(3.0).towards(NORTH).named("wade"))
                .then(Beat::stand(0.5)),
            camera: CameraRig::side()
                .with_azimuth(-35.0)
                .with_distance(3.5)
                .with_elevation(35.0),
            judged: Judged::Motion,
        },
        Scenario {
            name: "shrub_blast",
            summary: "Shrub Garden: throw a grenade at the big shrub at (0, 15) and watch it \
                      torn loose",
            level: SHRUB_GARDEN,
            start: (0.0, 1.0),
            yaw: 0.0,
            script: Script::new()
                .then(Beat::stand(0.3))
                .then(Beat::stand(5.0).throwing_at(NORTH).named("throw")),
            camera: CameraRig::side()
                .with_azimuth(115.0)
                .with_distance(17.0)
                .with_elevation(14.0),
            judged: Judged::Motion,
        },
        Scenario {
            name: "shrub_kick",
            summary: "Shrub Garden: run into the loose clump at (0, 23) and shove it along",
            level: SHRUB_GARDEN,
            start: (0.0, 19.5),
            yaw: 0.0,
            script: Script::new()
                .then(Beat::stand(0.3))
                .then(Beat::run(2.0).towards(NORTH).named("kick"))
                .then(Beat::stand(2.0)),
            camera: CameraRig::side().with_azimuth(90.0).with_distance(6.0),
            judged: Judged::Motion,
        },
        Scenario {
            name: "shrub_shove_off",
            summary: "Shrub Garden: walk the loose clump at (0, 47.5) off the terrace and let \
                      it tumble down the ramp",
            level: SHRUB_GARDEN,
            start: (0.0, 50.0),
            yaw: PI,
            script: Script::new()
                .then(Beat::stand(0.3))
                .then(Beat::walk(1.8).towards(SOUTH).named("shove"))
                .then(Beat::stand(5.0).named("watch")),
            camera: CameraRig::side()
                .with_azimuth(90.0)
                .with_distance(12.0)
                .with_elevation(15.0),
            judged: Judged::Motion,
        },
    ]
}
