//! The puzzle rooms of `levels/rooms/` whose way out is water a blast sets
//! moving: how the room's water answers each breach a player might make.

use nalgebra::Point3;

use super::scenario::{Action, Beat, Probe, Scenario};

const W1: &str = include_str!("../../levels/rooms/w1_flood_the_hollow.level.ron");

/// Every room scenario, in the order `--list` prints them.
pub fn scenarios() -> Vec<Scenario> {
    vec![w1_breach(1.5), w1_breach(0.0)]
}

/// W1's dam holed from the hollow's side, `height` above the room's floor:
/// chest height is where a grenade thrown from the floor strikes it.
fn w1_breach(height: f32) -> Scenario {
    let (name, description) = if height > 0.0 {
        (
            "room_w1_breach_chest",
            "W1: the dam holed at chest height, where a throw from the floor strikes it",
        )
    } else {
        (
            "room_w1_breach_foot",
            "W1: the dam holed at the room's floor, low against the ground",
        )
    };
    Scenario {
        name,
        description,
        level: W1,
        duration: 120.0,
        beats: vec![Beat {
            at: 1.0,
            action: Action::Grenade {
                centre: Point3::new(8.2, 10.0 + height, 16.0),
            },
        }],
        probes: vec![
            Probe {
                name: "reservoir",
                at: Point3::new(4.0, 11.0, 15.0),
            },
            Probe {
                name: "hollow",
                at: Point3::new(16.0, 7.0, 16.0),
            },
            Probe {
                name: "far floor",
                at: Point3::new(16.0, 10.5, 27.0),
            },
        ],
        camera: (Point3::new(18.0, 16.0, 0.0), Point3::new(14.0, 8.0, 16.0)),
    }
}
