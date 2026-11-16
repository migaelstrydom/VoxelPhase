//! The puzzle rooms of `levels/rooms/` whose way out is something a blast
//! cuts loose: what each room promises a player, held as a scenario.

use nalgebra::Point3;

use super::scenario::{Blast, Scenario};
use super::scenarios::largest;

const E3: &str = include_str!("../../levels/rooms/e3_rubble_ramp.level.ron");
const E5: &str = include_str!("../../levels/rooms/e5_bank_and_stair.level.ron");
const E7: &str = include_str!("../../levels/rooms/e7_crates_on_the_corner.level.ron");
const M2: &str = include_str!("../../levels/rooms/m2_platform_and_column.level.ron");

/// Every room scenario, in the order `--list` prints them.
pub fn scenarios() -> Vec<Scenario> {
    vec![
        e3_one_leg(),
        e3_every_leg(),
        e5_bank(),
        e7_corner(),
        m2_column(),
    ]
}

fn grenade(x: f32, y: f32, z: f32) -> Blast {
    Blast::grenade(Point3::new(x, y, z))
}

/// Where a grenade thrown at a leg of E3's slab goes off: against its
/// spawn-side face, at chest height.
fn e3_leg(x: f32, z: f32) -> Blast {
    grenade(x, 3.0, z - 0.6)
}

fn e3_one_leg() -> Scenario {
    Scenario {
        name: "room_e3_one_leg",
        description: "E3: with one leg blown the slab stands on the other three",
        level: E3,
        blasts: vec![e3_leg(13.0, 15.8)],
        known_gap: None,
        expect: |run| match largest(run) {
            n if n < 50 => Ok(()),
            n => Err(format!("a fragment of {n} samples fell")),
        },
    }
}

fn e3_every_leg() -> Scenario {
    Scenario {
        name: "room_e3_every_leg",
        description: "E3: with every leg blown the slab drops whole",
        level: E3,
        blasts: vec![
            e3_leg(13.0, 15.8),
            e3_leg(19.0, 15.8),
            e3_leg(13.0, 18.2),
            e3_leg(19.0, 18.2),
        ],
        known_gap: None,
        expect: |run| match largest(run) {
            n if n >= 500 => Ok(()),
            n => Err(format!(
                "expected the slab to fall, largest fragment {n} samples"
            )),
        },
    }
}

fn e5_bank() -> Scenario {
    Scenario {
        name: "room_e5_bank",
        description: "E5: a grenade in the bank leaves the rock landing and stair standing",
        level: E5,
        blasts: vec![grenade(16.0, 3.0, 11.4)],
        known_gap: None,
        expect: |run| match largest(run) {
            n if n < 50 => Ok(()),
            n => Err(format!("a fragment of {n} samples fell")),
        },
    }
}

fn e7_corner() -> Scenario {
    Scenario {
        name: "room_e7_corner",
        description: "E7: a grenade at the crates on the corner drops the north arm",
        level: E7,
        blasts: vec![grenade(15.0, 11.0, 27.3)],
        known_gap: None,
        // The north arm is 19.5 m by 1.6 m by 1 m: 250 samples.
        expect: |run| match largest(run) {
            n if n >= 200 => Ok(()),
            n => Err(format!(
                "expected the north arm to fall, largest fragment {n} samples"
            )),
        },
    }
}

fn m2_column() -> Scenario {
    Scenario {
        name: "room_m2_column",
        description: "M2: a grenade at the deck's height takes the column's top, and nothing falls on the track",
        level: M2,
        blasts: vec![grenade(16.0, 10.4, 15.2)],
        known_gap: None,
        expect: |run| match largest(run) {
            n if n < 8 => Ok(()),
            n => Err(format!("a fragment of {n} samples fell")),
        },
    }
}
