use crate::physics::bench_harness::scenarios::HoneycombWallScenario;

use super::{run, Case, Projectile, RunOptions, Structure, ENERGY_GAIN_LIMIT};

/// A honeycomb wall struck low by a steel ball regains nothing (seed 495).
///
/// Its blocks rest on one another along joints leaning 60° off the
/// horizontal. Shock propagation moves bodies exactly as physical impulses
/// would that carry each withheld share of a push down through the supports
/// to the ground; struck low, blocks tipped off the blocks under them while
/// still scaled, so those impulses would have had a support pull or push past
/// its friction, and the wall regained 2.9 J/kg as if it rose 30 cm.
#[test]
fn a_honeycomb_wall_struck_low_by_a_steel_ball_regains_nothing() {
    let mut wall = HoneycombWallScenario::new();
    wall.columns = 3;
    wall.rows = 7;
    let case = Case {
        seed: 495,
        structure: Structure::HoneycombWall(wall),
        frame_dt: 1.0 / 60.0,
        ordered_contacts: false,
        knocks: Vec::new(),
        projectile: Some(Projectile {
            radius: 0.215,
            density: 7622.0,
            bearing: 1.578,
            height: 0.118,
            speed: 9.67,
        }),
        walker: None,
    };
    let verdict = run(&case, RunOptions::default());
    assert!(
        verdict.worst_gain < ENERGY_GAIN_LIMIT,
        "the wall regained {:.2} J/kg, as if it rose {:.0} cm on its own",
        verdict.worst_gain,
        verdict.worst_gain / 9.81 * 100.0
    );
}
