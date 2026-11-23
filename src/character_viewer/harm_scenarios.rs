//! Harm on the player (`docs/GAME_DESIGN.md` *Harm*), in the drive-contract
//! level: a fall off the end of its highest plateau, which knocks the body
//! down, and a grenade thrown at the wall of the alley between two
//! plateaus, walked clear of to where its blast knocks the body down (on its
//! front, walking on; on its back, turned to face the grenade; over sideways,
//! turned across the alley), and stood by, where it knocks the body down
//! hardest and keeps it down longest. The report's `harm` column names each
//! blow's band and how sudden it was, and a death as `restart`; a knockdown
//! shows as `down`, `gathering` and `rising` in the locomotion column, with
//! the posture the figure is drawn in in the pose column.

use std::f32::consts::{FRAC_PI_2, PI};

use nalgebra::Vector3;

use crate::anim_viewer::{Beat, Script};

use super::scenario::{CameraRig, Judged, Scenario};

const CONTRACT: &str = "levels/drive_contract.level.ron";
const EAST: Vector3<f32> = Vector3::new(1.0, 0.0, 0.0);
const WEST: Vector3<f32> = Vector3::new(-1.0, 0.0, 0.0);
const SOUTH: Vector3<f32> = Vector3::new(0.0, 0.0, -1.0);
/// In the 3 m alley between the 9.6 m and 8.3 m plateaus, 1.5 m from the
/// higher one's wall: a grenade thrown at it strikes too soon to be armed,
/// drops at its foot, and lies there until its fuse burns out.
const ALLEY: (f32, f32) = (20.0, -20.0);

pub fn catalogue() -> Vec<Scenario> {
    vec![
        fall(),
        blast_knockdown(),
        blast_backward(),
        blast_sideways(),
        blast_close(),
    ]
}

/// Walk off the end of the 9.6 m plateau: landed on the feet, braced, at
/// about 13.7 m/s, which knocks the body down; it lies, then gets up.
fn fall() -> Scenario {
    Scenario {
        name: "harm_fall",
        summary: "Walk off the 9.6 m plateau and land: knocked down, then up again",
        level: CONTRACT,
        start: (27.0, -24.0),
        yaw: FRAC_PI_2,
        script: Script::new()
            .then(Beat::stand(0.3))
            .then(Beat::walk(0.8).towards(EAST).named("walk off"))
            .then(Beat::stand(6.0).named("lie")),
        camera: CameraRig::side().with_distance(9.0),
        judged: Judged::Motion,
    }
}

/// Throw a grenade at the alley's wall, then walk clear along the alley to
/// where its blast knocks the body down without killing it.
fn blast_knockdown() -> Scenario {
    Scenario {
        name: "harm_blast_knockdown",
        summary: "Throw a grenade at a wall close by, walk clear, and be knocked down by it",
        level: CONTRACT,
        start: ALLEY,
        yaw: PI,
        script: Script::new()
            .then(Beat::stand(0.3))
            .then(Beat::stand(0.4).throwing_at(SOUTH).named("throw"))
            .then(Beat::walk(0.4).towards(EAST).named("clear"))
            .then(Beat::stand(6.0).named("lie")),
        camera: CameraRig::side().with_distance(7.0),
        judged: Judged::Motion,
    }
}

/// Throw a grenade at the alley's wall, walk clear along the alley, and turn
/// back to face it: its blast knocks the body down onto its back, where it
/// sits up.
fn blast_backward() -> Scenario {
    Scenario {
        name: "harm_blast_backward",
        summary: "Grenade at a wall close by, walk clear, face it: knocked onto the back",
        level: CONTRACT,
        start: ALLEY,
        yaw: PI,
        script: Script::new()
            .then(Beat::stand(0.3))
            .then(Beat::stand(0.4).throwing_at(SOUTH).named("throw"))
            .then(Beat::walk(0.5).towards(EAST).named("clear"))
            .then(Beat::walk(0.25).towards(WEST).named("turn"))
            .then(Beat::stand(6.0).named("lie")),
        camera: CameraRig::side().with_distance(7.0),
        judged: Judged::Motion,
    }
}

/// Throw a grenade at the alley's wall, walk clear along the alley, and turn
/// across it: its blast knocks the body over sideways, rolling.
fn blast_sideways() -> Scenario {
    Scenario {
        name: "harm_blast_sideways",
        summary: "Grenade at a wall close by, walk clear, turn across: knocked over sideways",
        level: CONTRACT,
        start: ALLEY,
        yaw: PI,
        script: Script::new()
            .then(Beat::stand(0.3))
            .then(Beat::stand(0.4).throwing_at(SOUTH).named("throw"))
            .then(Beat::walk(0.3).towards(EAST).named("clear"))
            .then(Beat::walk(0.2).towards(SOUTH).named("turn"))
            .then(Beat::stand(6.0).named("lie")),
        camera: CameraRig::side().with_distance(7.0),
        judged: Judged::Motion,
    }
}

/// Throw a grenade at the alley's wall and stand by it.
fn blast_close() -> Scenario {
    Scenario {
        name: "harm_blast_close",
        summary: "Throw a grenade at a wall close by and stand by it: knocked down hard, it lies longest, then gets up",
        level: CONTRACT,
        start: ALLEY,
        yaw: PI,
        script: Script::new()
            .then(Beat::stand(0.3))
            .then(Beat::stand(0.4).throwing_at(SOUTH).named("throw"))
            .then(Beat::stand(9.0).named("stand by")),
        camera: CameraRig::side().with_azimuth(90.0).with_elevation(65.0).with_distance(5.0),
        judged: Judged::Motion,
    }
}
