//! The puzzle rooms of `levels/rooms/`, played through: the steps of each
//! room's way out that a script can be trusted with, done by the body the
//! player has.

use std::f32::consts::FRAC_PI_2;

use nalgebra::{Point3, Vector3};

use crate::anim_viewer::{Beat, Script};

use super::scenario::{CameraRig, Judged, Scenario};

const E3: &str = "levels/rooms/e3_rubble_ramp.level.ron";
const E5: &str = "levels/rooms/e5_bank_and_stair.level.ron";
const E7: &str = "levels/rooms/e7_crates_on_the_corner.level.ron";
const O1: &str = "levels/rooms/o1_crate_step.level.ron";
const O3: &str = "levels/rooms/o3_trampoline.level.ron";
const B2: &str = "levels/rooms/b2_shimmy.level.ron";
const M2: &str = "levels/rooms/m2_platform_and_column.level.ron";
const O2: &str = "levels/rooms/o2_raft.level.ron";
const NORTH: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);
const EAST: Vector3<f32> = Vector3::new(1.0, 0.0, 0.0);
const WEST: Vector3<f32> = Vector3::new(-1.0, 0.0, 0.0);
/// No stick: a jump straight up.
const STILL: Vector3<f32> = Vector3::new(0.0, 0.0, 0.0);

pub fn catalogue() -> Vec<Scenario> {
    vec![
        e3_legs(),
        e5_climb(),
        e7_push(),
        o1_push(),
        o3_carry(),
        b2_shimmy(),
        m2_cut_and_ride(),
        o2_raft(),
    ]
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

/// E5: climb the bank, catch the landing's lip from its top, and walk up
/// the stair to the exit.
fn e5_climb() -> Scenario {
    Scenario {
        name: "room_e5_climb",
        summary: "Room E5: up the bank, the landing and the stair to the exit",
        level: E5,
        start: (16.0, 9.5),
        yaw: 0.0,
        script: Script::new()
            .then(Beat::stand(0.3))
            .then(Beat::jump(0.9).towards(NORTH).named("onto the bank"))
            .then(Beat::stand(0.3))
            .then(Beat::jump(0.9).towards(NORTH).named("to the landing"))
            .then(Beat::walk(1.5).towards(NORTH).named("pull up"))
            .then(Beat::walk(2.5).towards(NORTH).named("up the stair"))
            .then(Beat::stand(0.5).named("at the exit")),
        camera: CameraRig::side().with_distance(14.0).with_elevation(20.0),
        judged: Judged::Motion,
    }
}

/// E7: cross onto the north arm, walk it to the crates and shove them off the
/// corner's open side, then go east to the exit.
fn e7_push() -> Scenario {
    Scenario {
        name: "room_e7_push",
        summary: "Room E7: shove the crates off the corner and walk on to the exit",
        level: E7,
        start: (15.0, 6.5),
        yaw: 0.0,
        script: Script::new()
            .then(Beat::stand(0.3))
            .then(Beat::jump(0.8).towards(NORTH).named("onto the arm"))
            .then(Beat::walk(5.3).towards(NORTH).named("along it and shove"))
            .then(Beat::stand(2.0).named("watch"))
            .then(Beat::walk(3.2).towards(EAST).named("east"))
            .then(Beat::walk(0.7).towards(NORTH).named("to the exit"))
            .then(Beat::stand(0.5).named("at the exit")),
        camera: CameraRig::side()
            .with_azimuth(-100.0)
            .with_distance(20.0)
            .with_elevation(35.0),
        judged: Judged::Motion,
    }
}

/// O1: push the crate to the cliff's foot, climb it and catch the lip.
fn o1_push() -> Scenario {
    Scenario {
        name: "room_o1_push",
        summary: "Room O1: push the crate to the cliff, climb it, catch the lip",
        level: O1,
        start: (10.0, 9.5),
        yaw: 0.0,
        script: Script::new()
            .then(Beat::stand(0.3))
            .then(Beat::walk(6.0).towards(NORTH).named("push"))
            .then(Beat::jump(0.9).towards(NORTH).named("onto the crate"))
            .then(Beat::stand(0.3))
            .then(Beat::jump(0.9).towards(NORTH).named("to the lip"))
            .then(Beat::walk(2.0).towards(NORTH).named("pull up")),
        camera: CameraRig::side().with_distance(12.0).with_elevation(20.0),
        judged: Judged::Motion,
    }
}

/// O3: carry the trampoline to the cliff's foot, bounce on it, pump it, and
/// lean into the cliff at the top of a bounce. Walked into, a trampoline is
/// not pushed: the walker goes in under its bed.
fn o3_carry() -> Scenario {
    Scenario {
        name: "room_o3_carry",
        summary: "Room O3: carry the trampoline to the cliff, bounce, and catch the lip",
        level: O3,
        start: (16.0, 9.0),
        yaw: 0.0,
        script: Script::new()
            .then(Beat::stand(0.3))
            .then(Beat::walk(0.3).towards(NORTH).named("up to it"))
            .then(Beat::stand(0.5).grabbing().named("grab"))
            .then(Beat::walk(3.5).towards(NORTH).grabbing().named("carry"))
            .then(Beat::stand(0.5).named("let go"))
            .then(Beat::jump(0.7).towards(NORTH).named("on"))
            .then(Beat::stand(2.0).named("ride"))
            .then(Beat::hop(1.0).named("pump"))
            .then(Beat::hop(1.0).named("pump"))
            .then(Beat::hop(1.0).named("pump"))
            .then(Beat::hop(1.2).towards(NORTH).named("for the lip"))
            .then(Beat::walk(2.0).towards(NORTH).named("pull up")),
        camera: CameraRig::side().with_distance(12.0).with_elevation(15.0),
        judged: Judged::Motion,
    }
}

/// B2: face the west wall, jump and catch its lip, shimmy north past the
/// chasm, and drop to the floor beyond.
///
fn b2_shimmy() -> Scenario {
    let into = WEST;
    let along = NORTH;
    Scenario {
        name: "room_b2_shimmy",
        summary: "Room B2: catch the west wall's lip, shimmy past the chasm, drop beyond",
        level: B2,
        start: (10.3, 8.0),
        yaw: -FRAC_PI_2,
        script: Script::new()
            .then(Beat::stand(0.3))
            .then(Beat::jump(0.8).towards(STILL).named("jump"))
            .then(Beat::stand(1.0).named("hang"))
            .then(Beat::walk(16.0).towards(along).named("shimmy"))
            .then(Beat::crouch_still(0.3).named("drop"))
            .then(Beat::walk(1.5).towards(along).named("to the exit"))
            .then(Beat::stand(0.5).towards(into)),
        camera: CameraRig::side().with_distance(14.0).with_elevation(25.0),
        judged: Judged::Motion,
    }
}

/// M2: cut the column from the ground, wait for the platform, set off the
/// moment the column went, to come back, step onto it, and ride it across.
fn m2_cut_and_ride() -> Scenario {
    const WAIT: f32 = 10.6;
    Scenario {
        name: "room_m2_cut_and_ride",
        summary: "Room M2: cut the column at the deck's height, board, and ride across",
        level: M2,
        start: (16.0, 9.0),
        yaw: 0.0,
        script: Script::new()
            .then(Beat::stand(0.3))
            .then(
                Beat::stand(2.5)
                    .throwing_at(Vector3::new(0.0, -0.3, 1.0))
                    .named("cut"),
            )
            .then(Beat::walk(0.4).towards(NORTH).named("to the edge"))
            .then(Beat::stand(WAIT).named("wait"))
            .then(Beat::walk(0.6).towards(NORTH).named("board"))
            .then(Beat::stand(7.0).named("ride"))
            .then(Beat::walk(2.2).towards(NORTH).named("off"))
            .then(Beat::stand(0.5).named("at the exit")),
        camera: CameraRig::side().with_distance(20.0).with_elevation(25.0),
        judged: Judged::Motion,
    }
}

/// O2: hole the dam from the floor, wait for the well to fill and float the
/// crate up, step onto it, and jump to the island.
fn o2_raft() -> Scenario {
    Scenario {
        name: "room_o2_raft",
        summary:
            "Room O2: hole the dam, let the well float the crate up, jump from it to the island",
        level: O2,
        start: (16.0, 13.0),
        yaw: 0.0,
        script: Script::new()
            .then(Beat::stand(0.3))
            .then(
                Beat::stand(60.0)
                    .throwing_at(Vector3::new(-7.5, 1.0, 7.0))
                    .named("throw and wait"),
            )
            .then(Beat::walk(0.9).towards(NORTH).named("to the well"))
            .then(Beat::jump(0.5).towards(NORTH).named("onto the crate"))
            .then(Beat::stand(0.5))
            .then(Beat::jump(0.9).towards(NORTH).named("to the island"))
            .then(Beat::stand(0.5).named("on the island")),
        camera: CameraRig::side()
            .with_distance(18.0)
            .with_elevation(30.0)
            .watching(Point3::new(14.0, 10.0, 20.0)),
        judged: Judged::Motion,
    }
}
