//! The puzzle rooms of `levels/rooms/` whose way out is water a blast sets
//! moving: how the room's water answers each breach a player might make.

use nalgebra::Point3;

use super::scenario::{Action, Beat, Probe, Scenario};

const W1: &str = include_str!("../../levels/rooms/w1_flood_the_hollow.level.ron");

/// Every room scenario, in the order `--list` prints them.
pub fn scenarios() -> Vec<Scenario> {
    vec![w1_breach(4.5), w1_breach(1.5)]
}

/// W1's dam holed from the pit's side, `height` above the pit's floor:
/// chest height is where a grenade thrown from the floor strikes it.
fn w1_breach(height: f32) -> Scenario {
    let (name, description) = if height > 2.0 {
        (
            "room_w1_breach_high",
            "W1: the dam holed above the island's height, where a lobbed throw strikes it",
        )
    } else {
        (
            "room_w1_breach_chest",
            "W1: the dam holed at chest height, low enough to drown its own outlet",
        )
    };
    Scenario {
        name,
        description,
        level: W1,
        duration: 240.0,
        beats: vec![Beat {
            at: 1.0,
            action: Action::Grenade {
                centre: Point3::new(11.2, 5.0 + height, 20.0),
            },
        }],
        probes: vec![
            Probe {
                name: "reservoir",
                at: Point3::new(6.0, 7.0, 20.0),
            },
            Probe {
                name: "pit",
                at: Point3::new(17.0, 6.0, 14.0),
            },
        ],
        camera: (Point3::new(21.0, 15.0, 13.0), Point3::new(11.0, 7.0, 20.0)),
    }
}
