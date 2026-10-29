//! A body turning steadily as it runs, the path laid afresh every few
//! frames: the plan is carried onto each new path, so the footfall the legs
//! are swinging to stays where it was, and nothing is refused that a straight
//! run crosses.

use std::path::PathBuf;

use super::run::{run, PerfRun, PerfSpec};

const GARDEN: &str = "levels/traversal_garden.level.ron";
/// How far the committed footfall may move from one frame to the next before
/// it lands, in metres: the few centimetres a 2° turn of the path moves a
/// footfall a step ahead, and some.
const FOOTFALL_MOVE: f32 = 0.08;

fn turning(at: (f32, f32), turn: f32, speed: f32, frames: usize) -> PerfRun {
    run(&PerfSpec {
        level: PathBuf::from(GARDEN),
        at,
        below: None,
        heading_degrees: 90.0,
        turn_degrees_per_second: turn,
        speed,
        frames,
        blasts: Vec::new(),
        verify: false,
    })
    .expect("runs")
}

/// The frames the path was laid afresh on, and those on which the
/// committed footfall, not yet landed the frame before, moved further than
/// [`FOOTFALL_MOVE`].
fn relays_and_moves(run: &PerfRun) -> (usize, Vec<(usize, f32)>) {
    let relays = run
        .frames
        .iter()
        .filter(|frame| frame.cost.laid_afresh)
        .count();
    let moves = run
        .frames
        .windows(2)
        .enumerate()
        .filter_map(|(index, pair)| {
            let (before, now) = (pair[0].next_footfall?, pair[1].next_footfall?);
            // A footfall landing within a frame has landed: the next is
            // another foot's.
            (before.0 > 1.5 / 60.0).then(|| (index + 1, (now.1 - before.1).norm()))
        })
        .filter(|&(_, moved)| moved > FOOTFALL_MOVE)
        .collect();
    (relays, moves)
}

#[test]
fn a_steady_turn_keeps_the_footfall_the_legs_swing_to() {
    for (at, turn, speed, frames) in [
        ((-15.0, -14.0), -45.0, 5.0, 180),
        ((-15.0, -15.0), -60.0, 8.0, 120),
    ] {
        let run = turning(at, turn, speed, frames);
        let (relays, moves) = relays_and_moves(&run);
        assert!(
            relays > frames / 5,
            "{turn}°/s at {speed} m/s: {relays} relays"
        );
        assert!(
            moves.is_empty(),
            "{turn}°/s at {speed} m/s: committed footfall moved {moves:?}"
        );
        assert!(
            run.frames
                .iter()
                .all(|frame| frame.crosses && !frame.diverged),
            "{turn}°/s at {speed} m/s: refused or dropped a commitment"
        );
    }
}

/// The 25 cm and 40 cm lips, each met turning gently, from the lane's start.
#[test]
fn a_lip_met_turning_is_crossed_as_one_met_straight() {
    for (lip, lane_z) in [("25 cm", 0.0), ("40 cm", 4.0)] {
        for speed in [5.0, 8.0] {
            let run = turning((-5.0, lane_z), -8.0, speed, (750.0 / speed) as usize);
            assert!(
                run.frames.iter().all(|frame| frame.crosses),
                "the {lip} lip turning at {speed} m/s: refused"
            );
        }
    }
}
