use nalgebra::Vector3;

use crate::physics::bench_harness::scenarios::{HoneycombWallScenario, JengaTowerScenario};

use super::{
    run, Case, Knock, Projectile, RunOptions, Structure, Verdict, WalkerScript, ENERGY_GAIN_LIMIT,
};

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
    assert_regains_nothing(&run(&case, RunOptions::default()));
}

/// A honeycomb wall shoved at its foot by a sprinting walker regains nothing
/// (seed 253).
///
/// The walker stands on the ground, so it is one of the carriers of the
/// block it strikes. Last frame's impulses held that block, but this frame
/// the walker's push does not fit them, and the blocks above it were scaled
/// against it as it was driven into them: the wall regained 1.9 J/kg.
#[test]
fn a_honeycomb_wall_shoved_by_a_sprinting_walker_regains_nothing() {
    let mut wall = HoneycombWallScenario::new();
    wall.columns = 6;
    wall.rows = 7;
    let case = Case {
        seed: 253,
        structure: Structure::HoneycombWall(wall),
        frame_dt: 1.0 / 60.0,
        ordered_contacts: true,
        knocks: vec![Knock {
            body: 0.9636215,
            velocity: Vector3::new(-2.352112, 0.41167784, 1.9072752),
        }],
        projectile: None,
        walker: Some(WalkerScript {
            bearing: 0.3049738,
            run_up: 5.19283,
            aim_across: 0.11963093,
            sprint: true,
            jump_from_edge: Some(1.1728119),
            run_seconds: 3.8936193,
        }),
    };
    assert_regains_nothing(&run(&case, RunOptions::default()));
}

/// A jenga tower pushed along the floor by a sprinting walker regains
/// nothing (seed 1742).
///
/// Its bottom block slid under the walker's push with the tower above it
/// scaled, and every joint up the tower pushed it along: 1.35 J/kg. It
/// passes now that a driven body about to strike a block ungrounds it and the
/// cone's tolerance is 2 %, but it fails with either undone, so it is kept as
/// a sample of the collapse rather than a proof of one rule.
#[test]
fn a_jenga_tower_pushed_along_the_floor_regains_nothing() {
    let mut jenga = JengaTowerScenario::new(9);
    jenga.friction = 0.38659745;
    let case = Case {
        seed: 1742,
        structure: Structure::Jenga(jenga),
        frame_dt: 1.0 / 60.0,
        ordered_contacts: true,
        knocks: Vec::new(),
        projectile: None,
        walker: Some(WalkerScript {
            bearing: 5.9622855,
            run_up: 4.262816,
            aim_across: -0.027383685,
            sprint: true,
            jump_from_edge: None,
            run_seconds: 3.8659787,
        }),
    };
    assert_regains_nothing(&run(&case, RunOptions::default()));
}

/// A tall jenga tower a walker runs into and then leans on regains nothing
/// (seed 636).
///
/// The walker stands on the ground, so it counted as a support of the block
/// it leaned on, and being held up by its drive rather than by its contacts
/// it was never grounded itself: every layer above that block lost shock
/// propagation, the eighteen layers sank 4 mm at each joint, and rising back
/// out of it read as 0.55 J/kg regained.
#[test]
fn a_tall_jenga_tower_a_walker_leans_on_regains_nothing() {
    let mut jenga = JengaTowerScenario::new(18);
    jenga.friction = 0.5121753;
    let case = Case {
        seed: 636,
        structure: Structure::Jenga(jenga),
        frame_dt: 1.0 / 30.0,
        ordered_contacts: true,
        knocks: vec![
            Knock {
                body: 0.11363542,
                velocity: Vector3::new(0.6457479, 2.1202192, -0.41163182),
            },
            Knock {
                body: 0.5373614,
                velocity: Vector3::new(-1.7532771, -0.7176199, -2.7617533),
            },
        ],
        projectile: None,
        walker: Some(WalkerScript {
            bearing: 3.3593543,
            run_up: 3.2934012,
            aim_across: 0.03777659,
            sprint: true,
            jump_from_edge: None,
            run_seconds: 3.5319872,
        }),
    };
    assert_regains_nothing(&run(&case, RunOptions::default()));
}

fn assert_regains_nothing(verdict: &Verdict) {
    assert!(
        verdict.worst_gain < ENERGY_GAIN_LIMIT,
        "the structure regained {:.2} J/kg, as if it rose {:.0} cm on its own",
        verdict.worst_gain,
        verdict.worst_gain / 9.81 * 100.0
    );
}
