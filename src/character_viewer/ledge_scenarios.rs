//! The ledge scenarios, in the Ledge Garden (`levels/ledge_garden.level.ron`):
//! jumping at a wall too high to clear and catching its lip, then each way
//! off it; the same jump pushing in, climbing it from the top; a mantle
//! onto a box at the chest; a lip under a roof, hung from with no room to
//! climb; light and heavy boxes, small to mantle and tall to hang from; and
//! platforms rising, going side to side, going diagonally and holding still,
//! caught by the rim and ridden.

use std::f32::consts::FRAC_PI_2;

use nalgebra::Vector3;

use crate::anim_viewer::{Beat, Script};

use super::scenario::{CameraRig, Judged, Scenario};

const LEDGE_GARDEN: &str = "levels/ledge_garden.level.ron";
/// The wall's face is at x = 7; the body starts with its capsule 15 cm off it.
const AT_THE_WALL: f32 = 6.6;
/// Into the wall.
const INTO: Vector3<f32> = Vector3::new(1.0, 0.0, 0.0);
const OUT: Vector3<f32> = Vector3::new(-1.0, 0.0, 0.0);
const ALONG: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);

/// A scenario facing the wall at lane `z`: `script` after a moment's stand.
fn at_wall(name: &'static str, summary: &'static str, z: f32, script: Script) -> Scenario {
    Scenario {
        name,
        summary,
        level: LEDGE_GARDEN,
        start: (AT_THE_WALL, z),
        yaw: FRAC_PI_2,
        script,
        camera: CameraRig::side().with_distance(6.0).with_elevation(15.0),
        judged: Judged::Motion,
    }
}

/// Stand, jump full height at the wall with the stick let go, and hang from
/// its lip; then `then`.
fn hang_then(then: &[Beat]) -> Script {
    jump_at(0.3, then)
}

/// Stand for `wait` seconds (for a platform to come round), jump full height
/// with the stick let go, and hang; then `then`.
fn jump_at(wait: f32, then: &[Beat]) -> Script {
    let mut script = Script::new()
        .then(Beat::stand(wait))
        .then(Beat::jump(0.8).named("jump").towards(Vector3::zeros()))
        .then(Beat::stand(1.0).named("hang"));
    for beat in then {
        script = script.then(beat.clone());
    }
    script.then(Beat::stand(1.0).named("after"))
}

/// Walk from 2 m back into whatever stands at lane `z`.
fn walk_in(name: &'static str, summary: &'static str, z: f32) -> Scenario {
    Scenario {
        start: (5.0, z),
        ..at_wall(
            name,
            summary,
            z,
            Script::new()
                .then(Beat::stand(0.3))
                .then(Beat::walk(2.0).named("mantle").towards(INTO))
                .then(Beat::stand(1.0).named("after")),
        )
    }
}

pub fn catalogue() -> Vec<Scenario> {
    vec![
        at_wall(
            "ledge_hang",
            "Ledge: jump at a wall too high to clear and hang from its lip",
            0.0,
            hang_then(&[]),
        ),
        at_wall(
            "ledge_jump_mantle",
            "Ledge: jump at the wall pushing into it, and climb it from the top of the jump",
            0.0,
            Script::new()
                .then(Beat::stand(0.3))
                .then(Beat::jump(0.8).named("jump").towards(INTO))
                .then(Beat::walk(1.0).named("on").towards(INTO))
                .then(Beat::stand(1.0).named("after")),
        ),
        at_wall(
            "ledge_drop",
            "Ledge: hang, then hold the stick away from the wall to drop",
            0.0,
            hang_then(&[Beat::walk(0.6).named("away").towards(OUT)]),
        ),
        at_wall(
            "ledge_crouch_drop",
            "Ledge: hang, then press crouch to drop at once",
            0.0,
            hang_then(&[Beat::crouch_still(0.3).named("crouch")]),
        ),
        at_wall(
            "ledge_pull_up",
            "Ledge: hang, then hold the stick into the wall to pull up onto it",
            0.0,
            hang_then(&[Beat::walk(1.2).named("pull").towards(INTO)]),
        ),
        at_wall(
            "ledge_back_jump",
            "Ledge: hang, then jump straight out from the wall",
            0.0,
            hang_then(&[
                Beat::jump(0.5).named("back"),
                Beat::stand(1.0).named("land"),
            ]),
        ),
        at_wall(
            "ledge_shimmy",
            "Ledge: hang, then shimmy along the lip with the stick",
            0.0,
            hang_then(&[Beat::walk(1.5).named("shimmy").towards(ALONG)]),
        ),
        at_wall(
            "ledge_no_room",
            "Ledge: hang from a lip under a roof; pushing in only strains",
            18.0,
            hang_then(&[Beat::walk(1.2).named("strain").towards(INTO)]),
        ),
        walk_in(
            "ledge_light_mantle",
            "Ledge: walk into a light box at the chest and climb onto it",
            24.0,
        ),
        walk_in(
            "ledge_heavy_mantle",
            "Ledge: walk into a heavy box at the chest and climb onto it",
            36.0,
        ),
        at_wall(
            "ledge_light_tall",
            "Ledge: jump at a light box too tall to clear, hang from it, and pull up",
            30.0,
            hang_then(&[
                Beat::stand(1.5).named("hang"),
                Beat::walk(1.2).named("pull").towards(INTO),
            ]),
        ),
        at_wall(
            "ledge_heavy_tall",
            "Ledge: jump at a heavy box too tall to clear, hang from it, and pull up",
            42.0,
            hang_then(&[
                Beat::stand(1.5).named("hang"),
                Beat::walk(1.2).named("pull").towards(INTO),
            ]),
        ),
        at_wall(
            "ledge_platform_lift",
            "Ledge: catch a rising platform's rim and ride it up and down",
            50.0,
            jump_at(2.0, &[Beat::stand(6.0).named("ride")]),
        ),
        at_wall(
            "ledge_platform_side",
            "Ledge: catch a platform going side to side and ride it",
            58.0,
            jump_at(1.8, &[Beat::stand(5.0).named("ride")]),
        ),
        at_wall(
            "ledge_platform_diagonal",
            "Ledge: catch a platform going diagonally to the side and up, and ride it",
            66.0,
            jump_at(2.0, &[Beat::stand(5.0).named("ride")]),
        ),
        at_wall(
            "ledge_platform_still",
            "Ledge: hang from a platform holding still, then pull up onto it",
            74.0,
            hang_then(&[
                Beat::stand(1.5).named("hang"),
                Beat::walk(1.2).named("pull").towards(INTO),
            ]),
        ),
        Scenario {
            start: (5.0, 12.0),
            ..at_wall(
                "ledge_mantle",
                "Ledge: walk into a box at the chest and climb onto it",
                12.0,
                Script::new()
                    .then(Beat::stand(0.3))
                    .then(Beat::walk(2.0).named("mantle").towards(INTO))
                    .then(Beat::stand(1.0).named("after")),
            )
        },
    ]
}
