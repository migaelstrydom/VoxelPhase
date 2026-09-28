//! Shapes that start partly inside static geometry: pushed out by the side
//! their centre is on, never on through, and never let into solid they only
//! touch.

use super::super::framework::{run_scenario, BenchRunConfig, BenchRunResult};
use super::super::scenarios::*;
use super::assertions::*;
use super::write_exports;

// ── A shape set too low comes up on top ─────────────────────────────

/// A shape whose centre is a little under the ground — a centimetre, or most
/// of its reach — is pushed up onto it, not dropped through it: with nothing
/// on the far side to compete, a face pushes a shape whose centre has
/// crossed it.
#[test]
fn a_shape_with_its_centre_under_the_ground_comes_up_on_top() {
    let mut failures = Vec::new();
    for shape in ProbeShape::ALL {
        for depth in [0.01, 0.4] {
            let scenario = ShapeIntoSolidScenario::buried(shape, depth);
            let run = run(&scenario, "shape_buried");
            let rest = shape.reach().y;
            let end = final_sample(&run).y;
            if (end - rest).abs() > 0.03 {
                failures.push(format!(
                    "{shape:?} {depth} m under: ended at y = {end:.3}, not resting at {rest:.3}"
                ));
            }
        }
    }
    assert_none(failures);
}

/// A plank four centimetres thick has only two centimetres of reach below
/// its centre: the case where a light press, not a mis-authored level, is
/// enough to put the centre under the surface.
#[test]
fn a_plank_with_its_centre_just_under_the_ground_comes_up_on_top() {
    let run = run_scenario(&BuriedBoxScenario::plank(), BenchRunConfig::default());
    write_exports(&run, "buried_box_plank");

    assert_above_floor(&run, -0.05);
    assert_settled(&run, 1.0, 0.02, 0.05);
    assert_final_y_near(&run, 0.02, 0.01);
}

// ── A shape inside solid leaves by the side its centre is on ────────

fn run(scenario: &ShapeIntoSolidScenario, export: &str) -> BenchRunResult {
    let run = run_scenario(scenario, BenchRunConfig::default());
    write_exports(
        &run,
        &format!("{export}_{:?}", scenario.shape).to_lowercase(),
    );
    run
}

fn furthest_x(run: &BenchRunResult) -> f32 {
    run.samples.iter().map(|s| s.x).fold(f32::MIN, f32::max)
}

fn widest_x(run: &BenchRunResult) -> f32 {
    run.samples.iter().map(|s| s.x.abs()).fold(0.0, f32::max)
}

fn final_sample(run: &BenchRunResult) -> &super::super::framework::BenchSample {
    run.samples.last().expect("the run recorded samples")
}

/// Every shape's failures at once, so one run shows the whole picture.
fn assert_none(failures: Vec<String>) {
    assert!(failures.is_empty(), "\n{}", failures.join("\n"));
}

/// A shape pressed into a wall comes back out of the near face for as long
/// as its centre is short of the wall's midline, however thin the wall — and
/// out of the far face once it is past it, since by then it is more through
/// than not. The midline is `p = reach + t/2`.
///
/// It must end clear of the wall on that side, not flush with it: a ball
/// pushed out rolls on, and a standing capsule tips over.
#[test]
fn a_shape_in_a_thin_wall_leaves_by_the_side_its_centre_is_on() {
    let mut failures = Vec::new();
    for shape in ProbeShape::ALL {
        let reach = shape.reach().x;
        for thickness in [0.01, 0.1] {
            let near = -thickness * 0.5 - reach;
            let midline = reach + thickness * 0.5;
            for penetration in [0.05, 0.25, 0.45, midline - 0.05] {
                let scenario = ShapeIntoSolidScenario::thin_wall(shape, thickness, penetration);
                let run = run(&scenario, "shape_into_thin_wall");
                let end = final_sample(&run).x;
                if furthest_x(&run) >= 0.0 || end > near + 0.03 {
                    failures.push(format!(
                        "{shape:?} t = {thickness}, p = {penetration}: reached x = {:.3}, ended at \
                         {end:.3}; should end clear of the near face at {near:.3}",
                        furthest_x(&run)
                    ));
                }
            }

            let far = thickness * 0.5 + reach;
            let scenario = ShapeIntoSolidScenario::thin_wall(shape, thickness, midline + 0.05);
            let run = run(&scenario, "shape_into_thin_wall_past_midline");
            let end = final_sample(&run).x;
            if end < far - 0.03 {
                failures.push(format!(
                    "{shape:?} t = {thickness}, past the midline: ended at x = {end:.3}, not \
                     clear of the far face at {far:.3}"
                ));
            }
        }
    }
    assert_none(failures);
}

/// A shape wedged in a slot narrower than itself is pushed by both walls, and
/// passes through neither. The two walls face each other, so neither is the
/// far side of the other.
#[test]
fn a_shape_wedged_in_a_slot_passes_through_neither_wall() {
    let half_width = 0.4;
    let mut failures = Vec::new();
    for shape in ProbeShape::ALL {
        let run = run(
            &ShapeIntoSolidScenario::slot(shape, half_width, 0.05),
            "shape_into_slot",
        );
        if widest_x(&run) >= half_width {
            failures.push(format!(
                "{shape:?}: centre reached x = ±{:.3}, inside a wall at ±{half_width}",
                widest_x(&run)
            ));
        }
    }
    assert_none(failures);
}

/// With its centre already past one wall of a narrow slot, the shape is still
/// pushed back out of that wall, not on into it. The far wall's push is the
/// shorter one, but the walls face each other: had the one the centre is
/// behind been dropped for it, the shape would be driven deeper.
#[test]
fn a_shape_with_its_centre_inside_a_slot_wall_is_not_driven_deeper() {
    let mut failures = Vec::new();
    for shape in ProbeShape::ALL {
        let scenario = ShapeIntoSolidScenario::slot(shape, 0.1, 0.2);
        let run = run(&scenario, "shape_into_slot_deep");
        if furthest_x(&run) > scenario.start.x + 0.02 {
            failures.push(format!(
                "{shape:?}: driven into the wall from x = {:.3} to x = {:.3}",
                scenario.start.x,
                furthest_x(&run)
            ));
        }
    }
    assert_none(failures);
}

/// A shape pressed into the side of a blade whose faces are 160° apart, not
/// 180°, still leaves by its own side.
#[test]
fn a_shape_pressed_into_a_knife_edge_is_pushed_back_out_of_its_side() {
    let mut failures = Vec::new();
    for shape in ProbeShape::ALL {
        for penetration in [0.1, 0.3] {
            let scenario = ShapeIntoSolidScenario::knife_edge(shape, penetration);
            let run = run(&scenario, "shape_into_knife_edge");
            if furthest_x(&run) >= 0.0 {
                failures.push(format!(
                    "{shape:?} p = {penetration}: pushed across the blade to x = {:.3}",
                    furthest_x(&run)
                ));
            }
        }
    }
    assert_none(failures);
}

/// A shape pressed onto a steep ridge is lifted straight off it. Its centre
/// is in front of both slopes — ordinary contact — so both push, and their
/// sideways halves cancel. Picking one would shove it off to the side.
#[test]
fn a_shape_pressed_onto_a_steep_ridge_is_lifted_straight_up() {
    let mut failures = Vec::new();
    for shape in ProbeShape::ALL {
        let scenario = ShapeIntoSolidScenario::ridge(shape, 0.1);
        let run = run(&scenario, "shape_into_ridge");
        let end = final_sample(&run).y;
        if widest_x(&run) >= 0.01 || end <= scenario.start.y + 0.05 {
            failures.push(format!(
                "{shape:?}: pushed sideways to x = ±{:.3}, y went from {:.3} to {end:.3}",
                widest_x(&run),
                scenario.start.y
            ));
        }
    }
    assert_none(failures);
}

/// A shape set down across a steep ridge rests on its apex. Balanced there,
/// it neither sinks onto the ridge nor is pushed off it.
#[test]
fn a_shape_set_down_on_a_steep_ridge_rests_on_its_apex() {
    let mut failures = Vec::new();
    for shape in ProbeShape::ALL {
        let scenario = ShapeIntoSolidScenario::resting_on_ridge(shape);
        let run = run(&scenario, "shape_resting_on_ridge");
        let lowest = run.samples.iter().map(|s| s.y).fold(f32::MAX, f32::min);
        if lowest <= scenario.start.y - 0.03 || widest_x(&run) >= 0.01 {
            failures.push(format!(
                "{shape:?}: sank from y = {:.3} to {lowest:.3}, pushed to x = ±{:.3}",
                scenario.start.y,
                widest_x(&run)
            ));
        }
    }
    assert_none(failures);
}

/// A shape pressed diagonally into a pillar thinner than itself leaves by the
/// corner it came in at: each pair of opposite faces is decided on its own.
#[test]
fn a_shape_pressed_into_a_thin_pillar_is_pushed_back_the_way_it_came() {
    let mut failures = Vec::new();
    for shape in ProbeShape::ALL {
        for penetration in [0.1, 0.3] {
            let scenario = ShapeIntoSolidScenario::pillar(shape, penetration);
            let run = run(&scenario, "shape_into_pillar");
            if furthest_x(&run) >= 0.0 {
                failures.push(format!(
                    "{shape:?} p = {penetration}: pushed through the pillar to x = {:.3}",
                    furthest_x(&run)
                ));
            }
        }
    }
    assert_none(failures);
}

/// A pillar thinner than the shape is behind the shape's centre on every side
/// at once when the centre is inside it, so no one side of it is the way
/// out. The shape must still leave it, and come to rest beside it.
#[test]
fn a_shape_skewered_on_a_thin_round_pillar_comes_off_it() {
    let mut failures = Vec::new();
    for shape in round_pillar_probes() {
        for offset in [0.0, 0.05, 0.12] {
            let scenario = ShapeIntoSolidScenario::round_pillar(shape, offset);
            let run = run(&scenario, "shape_on_round_pillar");
            if let Some(failure) = check_clear_of_round_pillar(&run) {
                failures.push(format!(
                    "{shape:?} started {offset} m off the axis: {failure}"
                ));
            }
        }
    }
    assert_none(failures);
}

/// A shape skewered on a thin round pillar leaves it sideways, the shortest
/// way out: the pillar's upright edges come in short pieces, and sliding up
/// along one clears that piece, but only onto the next.
#[test]
fn a_shape_skewered_on_a_thin_round_pillar_leaves_it_sideways() {
    let mut failures = Vec::new();
    for shape in round_pillar_probes() {
        for offset in [0.0, 0.05, 0.12] {
            let scenario = ShapeIntoSolidScenario::round_pillar(shape, offset);
            let run = run(&scenario, "shape_on_round_pillar");
            let highest = run.samples.iter().map(|s| s.y).fold(f32::MIN, f32::max);
            let rise = highest - scenario.start.y;
            if rise > 0.1 {
                failures.push(format!(
                    "{shape:?} started {offset} m off the axis: rose {rise:.3} m up the pillar"
                ));
            }
        }
    }
    assert_none(failures);
}

/// A shape knocked against a thin round pillar glances off or stops against
/// it; however it tumbles, it never works its way onto it.
#[test]
fn a_shape_thrown_at_a_thin_round_pillar_never_ends_up_on_it() {
    let mut failures = Vec::new();
    for shape in round_pillar_probes() {
        for speed in [1.0, 3.0, 6.0] {
            for miss in [0.0, 0.2, 0.4] {
                let scenario = ShapeIntoSolidScenario::thrown_at_round_pillar(shape, speed, miss);
                let run = run(&scenario, "shape_thrown_at_round_pillar");
                if let Some(failure) = check_clear_of_round_pillar(&run) {
                    failures.push(format!(
                        "{shape:?} thrown at {speed} m/s, {miss} m wide: {failure}"
                    ));
                }
            }
        }
    }
    assert_none(failures);
}

fn round_pillar_probes() -> impl Iterator<Item = ProbeShape> {
    ProbeShape::ALL
        .into_iter()
        .chain([ProbeShape::Dodecahedron])
}

/// Every probe is at least 45 cm from its centre to its side, however it
/// lies: a centre nearer the pillar's axis than that and the pillar's radius
/// is a centre with the pillar through the shape.
fn check_clear_of_round_pillar(run: &BenchRunResult) -> Option<String> {
    let clear = ROUND_PILLAR_RADIUS + 0.45;
    let from_axis = |s: &super::super::framework::BenchSample| s.x.hypot(s.z);
    let end = final_sample(run);
    (from_axis(end) < clear).then(|| {
        format!(
            "ended {:.3} m from the axis at y = {:.3}, less than {clear:.2}",
            from_axis(end),
            end.y
        )
    })
}
