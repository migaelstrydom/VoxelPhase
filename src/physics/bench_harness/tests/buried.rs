//! Boxes that start partly inside static geometry: pushed out by the side
//! their centre is on, never on through, and never let into solid they only
//! touch.

use super::super::framework::{run_scenario, BenchRunConfig, BenchRunResult};
use super::super::scenarios::*;
use super::assertions::*;
use super::write_exports;

// ── A box set too low comes up on top ───────────────────────────────

/// A cube whose centre is a centimetre under the ground is pushed up onto it,
/// not dropped through it: with nothing on the far side to compete, a face
/// pushes a box whose centre has crossed it.
#[test]
fn a_cube_with_its_centre_just_under_the_ground_comes_up_on_top() {
    let run = run_scenario(&BuriedBoxScenario::cube(), BenchRunConfig::default());
    write_exports(&run, "buried_box_cube");

    assert_above_floor(&run, -0.5);
    assert_settled(&run, 1.0, 0.02, 0.05);
    assert_final_y_near(&run, 0.5, 0.03);
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

/// Most of the way under, a cube still comes up.
#[test]
fn a_cube_buried_almost_half_its_reach_still_comes_up() {
    let scenario = BuriedBoxScenario::new(nalgebra::Vector3::repeat(0.5), 0.4);
    let run = run_scenario(&scenario, BenchRunConfig::default());
    write_exports(&run, "buried_box_deep");

    assert_above_floor(&run, -0.5);
    assert_final_y_near(&run, 0.5, 0.03);
}

// ── A box inside solid leaves by the side its centre is on ──────────

const H: f32 = BoxIntoSolidScenario::HALF_EXTENT;

fn run(scenario: &BoxIntoSolidScenario, export: &str) -> BenchRunResult {
    let run = run_scenario(scenario, BenchRunConfig::default());
    write_exports(&run, export);
    run
}

fn furthest_x(run: &BenchRunResult) -> f32 {
    run.samples.iter().map(|s| s.x).fold(f32::MIN, f32::max)
}

fn final_x(run: &BenchRunResult) -> f32 {
    run.samples.last().expect("the run recorded samples").x
}

/// A cube pressed into a wall comes back out of the near face for as long as
/// its centre is short of the wall's midline, however thin the wall — and out
/// of the far face once it is past it, since by then it is more through than
/// not. The midline is `p = h + t/2`.
#[test]
fn a_cube_in_a_thin_wall_leaves_by_the_side_its_centre_is_on() {
    for thickness in [0.01, 0.1] {
        let near = -thickness * 0.5 - H;
        let midline = H + thickness * 0.5;
        for penetration in [0.05, 0.25, 0.45, midline - 0.05] {
            let scenario = BoxIntoSolidScenario::thin_wall(thickness, penetration);
            let run = run(&scenario, "box_into_thin_wall");
            assert!(
                furthest_x(&run) < 0.0,
                "t = {thickness}, p = {penetration}: pushed through the wall to x = {:.3}",
                furthest_x(&run)
            );
            assert!(
                (final_x(&run) - near).abs() < 0.03,
                "t = {thickness}, p = {penetration}: ended at x = {:.3}, not flush with the near \
                 face at {near:.3}",
                final_x(&run)
            );
        }

        let far = thickness * 0.5 + H;
        let scenario = BoxIntoSolidScenario::thin_wall(thickness, midline + 0.05);
        let run = run(&scenario, "box_into_thin_wall_past_midline");
        assert!(
            (final_x(&run) - far).abs() < 0.03,
            "t = {thickness}: a cube past the midline ended at x = {:.3}, not flush with the far \
             face at {far:.3}",
            final_x(&run)
        );
    }
}

/// A cube wedged in a slot narrower than itself is pushed by both walls, and
/// passes through neither. The two walls face each other, so neither is the
/// far side of the other.
#[test]
fn a_cube_wedged_in_a_slot_passes_through_neither_wall() {
    let half_width = 0.4;
    let run = run(
        &BoxIntoSolidScenario::slot(half_width, 0.05),
        "box_into_slot",
    );

    let widest = run.samples.iter().map(|s| s.x.abs()).fold(0.0, f32::max);
    assert!(
        widest < half_width,
        "the cube's centre reached x = ±{widest:.3}, inside a wall at ±{half_width}"
    );
}

/// With its centre already past one wall of a narrow slot, the cube is still
/// pushed back out of that wall, not on into it. The far wall's push is the
/// shorter one, but the walls face each other: had the one the centre is
/// behind been dropped for it, the cube would be driven deeper.
#[test]
#[ignore = "the manifold reducer keeps points by spread alone, so with more of its four \
            points on one wall than the other, that wall outvotes the deeper one"]
fn a_cube_with_its_centre_inside_a_slot_wall_is_not_driven_deeper() {
    let scenario = BoxIntoSolidScenario::slot(0.1, 0.2);
    let run = run(&scenario, "box_into_slot_deep");

    assert!(
        furthest_x(&run) <= scenario.start.x + 0.02,
        "the cube was driven into the wall from x = {:.3} to x = {:.3}",
        scenario.start.x,
        furthest_x(&run)
    );
}

/// A cube pressed into the side of a blade whose faces are 160° apart, not
/// 180°, still leaves by its own side.
#[test]
fn a_cube_pressed_into_a_knife_edge_is_pushed_back_out_of_its_side() {
    for penetration in [0.1, 0.3] {
        let run = run(
            &BoxIntoSolidScenario::knife_edge(penetration),
            "box_into_knife_edge",
        );
        assert!(
            furthest_x(&run) < 0.0,
            "p = {penetration}: pushed across the blade to x = {:.3}",
            furthest_x(&run)
        );
    }
}

/// A cube pressed onto a steep ridge is lifted straight off it. Its centre
/// is in front of both slopes — ordinary contact — so both push, and their
/// sideways halves cancel. Picking one would shove it off to the side.
#[test]
#[ignore = "an OBB gets no contact from a convex mesh edge that runs through one of its \
            faces: face clipping misses it and the edge fallback only matches OBB edges"]
fn a_cube_pressed_onto_a_steep_ridge_is_lifted_straight_up() {
    let scenario = BoxIntoSolidScenario::ridge(0.1);
    let run = run(&scenario, "box_into_ridge");

    let widest = run.samples.iter().map(|s| s.x.abs()).fold(0.0, f32::max);
    assert!(
        widest < 0.01,
        "the cube was pushed sideways to x = ±{widest:.3}"
    );
    let last = run.samples.last().expect("the run recorded samples");
    assert!(
        last.y > scenario.start.y + 0.05,
        "the cube was not lifted: y went from {:.3} to {:.3}",
        scenario.start.y,
        last.y
    );
}

/// A cube pressed diagonally into a pillar thinner than itself leaves by the
/// corner it came in at: each pair of opposite faces is decided on its own.
#[test]
fn a_cube_pressed_into_a_thin_pillar_is_pushed_back_the_way_it_came() {
    for penetration in [0.1, 0.3] {
        let run = run(
            &BoxIntoSolidScenario::pillar(penetration),
            "box_into_pillar",
        );
        assert!(
            furthest_x(&run) < 0.0,
            "p = {penetration}: pushed through the pillar to x = {:.3}",
            furthest_x(&run)
        );
    }
}
