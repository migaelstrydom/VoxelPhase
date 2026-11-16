//! The puzzle rooms of `levels/rooms/` whose way out is something a blast
//! cuts loose: what each room promises a player, held as a scenario.

use nalgebra::Point3;

use super::scenario::{Blast, Scenario};
use super::scenarios::largest;

const E3: &str = include_str!("../../levels/rooms/e3_rubble_ramp.level.ron");

/// Every room scenario, in the order `--list` prints them.
pub fn scenarios() -> Vec<Scenario> {
    vec![e3_one_leg(), e3_every_leg()]
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
