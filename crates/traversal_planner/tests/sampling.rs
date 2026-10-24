//! The strip sampler against authored ground, each case against a number
//! worked out by hand.

use traversal_planner::plan::{BodyState, Limb};
use traversal_planner::scene::{AuthoredGround, Form, Ground, Intent, Shape};
use traversal_planner::section::{Footing, Profile, Section};
use traversal_planner::{GaitParams, TraversalScene};

const BODY_RADIUS: f32 = 0.25;
const PLANK_TOP: f32 = 0.56;

fn block(from: f32, to: f32, top: f32) -> Shape {
    Shape {
        form: Form::Block {
            from,
            to,
            top,
            tilt: 0.0,
        },
        across: None,
        yaw: 0.0,
        footing: Footing::Fixed,
    }
}

fn section_of(base: Option<f32>, shapes: Vec<Shape>) -> Section {
    TraversalScene {
        ground: Ground::Authored(AuthoredGround { base, shapes }),
        body: BodyState {
            s: 0.0,
            z: 0.0,
            speed: 0.0,
            climb: 0.0,
        },
        lead: Limb::LeftFoot,
        intent: Intent { speed: 0.0 },
        params: GaitParams::default(),
        layout: Default::default(),
        expect: None,
    }
    .section()
}

/// Every sample of a profile within `s_range`, as `(s, z, normal z)`.
fn samples(profile: &Profile, s_range: (f32, f32)) -> Vec<(f32, f32, f32)> {
    profile
        .surfaces
        .iter()
        .flat_map(|surface| surface.points.iter())
        .filter(|point| point.s >= s_range.0 && point.s <= s_range.1)
        .map(|point| (point.s, point.z, point.normal.z))
        .collect()
}

fn lane<'a>(section: &'a Section, d: f32) -> &'a Profile {
    &section
        .foothold_lanes
        .iter()
        .find(|lane| (lane.d - d).abs() < 1e-6)
        .expect("lane")
        .profile
}

/// Two planks flush with their tops at `PLANK_TOP`, the gap between them
/// `width` wide from `s = 0.5`, over ground at 0.
fn plank_gap(width: f32) -> Section {
    section_of(
        Some(0.0),
        vec![
            block(-1.0, 0.5, PLANK_TOP),
            block(0.5 + width, 3.5, PLANK_TOP),
        ],
    )
}

#[test]
fn a_flat_top_holds_the_body_a_radius_above_it() {
    let section = section_of(Some(0.0), vec![]);
    for (s, z, normal_z) in samples(&section.body_floor, (-0.5, 3.0)) {
        assert!((z - BODY_RADIUS).abs() < 1e-4, "s = {s}: z = {z}");
        assert!(normal_z > 0.9999, "s = {s}: normal z = {normal_z}");
    }
}

#[test]
fn the_body_floor_dips_over_a_narrow_gap_as_a_sphere_on_two_lips() {
    for width in [0.22_f32, 0.30, 0.40] {
        let section = plank_gap(width);
        let lips = [0.5, 0.5 + width];
        let gap = samples(&section.body_floor, (lips[0], lips[1]));
        assert!(!gap.is_empty());
        for (s, z, normal_z) in gap {
            let nearer = lips
                .iter()
                .map(|lip| (s - lip).abs())
                .fold(f32::INFINITY, f32::min);
            let on_lip = PLANK_TOP + (BODY_RADIUS * BODY_RADIUS - nearer * nearer).sqrt();
            assert!(
                (z - on_lip).abs() < 1e-4,
                "gap {width} m, s = {s}: z = {z}, on the nearer lip {on_lip}"
            );
            let expected_normal_z = (on_lip - PLANK_TOP) / BODY_RADIUS;
            assert!(
                (normal_z - expected_normal_z).abs() < 1e-4,
                "gap {width} m, s = {s}: normal z = {normal_z}"
            );
        }
    }
}

#[test]
fn the_body_floor_falls_through_a_gap_wider_than_the_body() {
    let section = plank_gap(0.7);
    let gap = samples(&section.body_floor, (0.5, 1.2));
    let lowest = gap
        .iter()
        .map(|sample| sample.1)
        .fold(f32::INFINITY, f32::min);
    assert!(
        (lowest - BODY_RADIUS).abs() < 1e-4,
        "rests on the ground at {lowest}"
    );
}

#[test]
fn footholds_break_at_every_plank_edge() {
    let width = 0.22;
    let section = plank_gap(width);
    let surfaces = &lane(&section, 0.0).surfaces;
    let tops: Vec<(f32, f32, f32)> = surfaces
        .iter()
        .map(|surface| {
            let first = surface.points.first().expect("points");
            let last = surface.points.last().expect("points");
            let middle = surface.points[surface.points.len() / 2];
            (first.s, last.s, middle.z)
        })
        .collect();
    assert_eq!(tops.len(), 3, "plank, ground in the gap, plank: {tops:?}");
    let foot_radius = GaitParams::default().foot.radius;
    assert!((tops[0].2 - (PLANK_TOP + foot_radius)).abs() < 1e-4);
    assert!(
        (tops[1].2 - foot_radius).abs() < 1e-4,
        "the gap's floor is the ground"
    );
    assert!(
        tops[1].0 > 0.5 && tops[1].1 < 0.5 + width,
        "the gap's footholds lie within it"
    );
    assert!((tops[2].2 - (PLANK_TOP + foot_radius)).abs() < 1e-4);
}

#[test]
fn a_stone_off_the_line_is_only_in_the_lanes_over_it() {
    let stone = Shape {
        across: Some((0.1, 0.3)),
        ..block(1.0, 1.3, 0.0)
    };
    let section = section_of(None, vec![block(-1.0, 0.4, 0.0), stone]);
    for (d, sees_it) in [
        (-0.3, false),
        (-0.15, false),
        (0.0, false),
        (0.15, true),
        (0.3, true),
    ] {
        let on_stone = samples(lane(&section, d), (1.0, 1.3));
        assert_eq!(
            !on_stone.is_empty(),
            sees_it,
            "lane {d}: {} samples on the stone",
            on_stone.len()
        );
    }
}

#[test]
fn a_stone_tilted_across_the_path_keeps_its_tilt_in_the_normal() {
    let tilt = 0.4_f32;
    let stone = Shape {
        form: Form::Block {
            from: -1.0,
            to: 3.5,
            top: 0.0,
            tilt,
        },
        across: None,
        yaw: 0.0,
        footing: Footing::Fixed,
    };
    let section = section_of(None, vec![stone]);
    let expected = (-tilt, 1.0);
    let length = (expected.0 * expected.0 + expected.1 * expected.1).sqrt();
    for lane in &section.foothold_lanes {
        for point in lane
            .profile
            .surfaces
            .iter()
            .flat_map(|surface| &surface.points)
        {
            assert!(
                (point.normal.d - expected.0 / length).abs() < 1e-4
                    && (point.normal.z - expected.1 / length).abs() < 1e-4,
                "lane {}, s = {}: normal {:?}",
                lane.d,
                point.s,
                point.normal
            );
            assert!(
                point.normal.s.abs() < 1e-4,
                "no slope along the path: {:?}",
                point.normal
            );
        }
    }
}

#[test]
fn a_seam_between_loose_stones_splits_the_surface_and_fixed_ground_does_not() {
    let loose = |from, to| Shape {
        footing: Footing::Loose {
            mass: 30.0,
            resting: true,
        },
        ..block(from, to, 0.2)
    };
    let loose_section = section_of(Some(0.0), vec![loose(0.5, 1.0), loose(1.0, 1.5)]);
    let fixed_section = section_of(Some(0.0), vec![block(0.5, 1.0, 0.2), block(1.0, 1.5, 0.2)]);
    let on_top = |section: &Section| {
        lane(section, 0.0)
            .surfaces
            .iter()
            .filter(|surface| {
                surface
                    .points
                    .iter()
                    .any(|point| (point.s - 0.75).abs() < 0.3 && point.z > 0.2)
            })
            .count()
    };
    assert_eq!(
        on_top(&loose_section),
        2,
        "two loose stones are two surfaces"
    );
    assert_eq!(on_top(&fixed_section), 1, "fixed blocks are one ground");
}

#[test]
fn a_roof_is_the_ceiling_a_radius_below_its_underside() {
    let roof = Shape {
        form: Form::Roof {
            from: 1.0,
            to: 2.0,
            bottom: 1.2,
        },
        across: None,
        yaw: 0.0,
        footing: Footing::Fixed,
    };
    let section = section_of(Some(0.0), vec![roof]);
    let under = samples(&section.ceiling, (1.3, 1.7));
    assert!(!under.is_empty());
    for (s, z, normal_z) in under {
        assert!((z - (1.2 - BODY_RADIUS)).abs() < 1e-4, "s = {s}: z = {z}");
        assert!(normal_z < -0.9999, "faces down: {normal_z}");
    }
    assert!(
        samples(&section.ceiling, (-0.5, 0.7)).is_empty(),
        "open sky before the roof"
    );
}

#[test]
fn a_ramp_holds_the_body_its_radius_out_along_the_slope_normal() {
    let slope = 0.5_f32;
    let ramp = Shape {
        form: Form::Ramp {
            from: -2.0,
            to: 5.0,
            start_top: -2.0 * slope,
            end_top: 5.0 * slope,
        },
        across: None,
        yaw: 0.0,
        footing: Footing::Fixed,
    };
    let section = section_of(None, vec![ramp]);
    let stretch = (1.0 + slope * slope).sqrt();
    for (s, z, normal_z) in samples(&section.body_floor, (-0.5, 3.0)) {
        let expected = slope * s + BODY_RADIUS * stretch;
        assert!(
            (z - expected).abs() < 1e-4,
            "s = {s}: z = {z}, expected {expected}"
        );
        assert!(
            (normal_z - 1.0 / stretch).abs() < 1e-4,
            "s = {s}: normal z = {normal_z}"
        );
    }
}

#[test]
fn a_plank_laid_askew_meets_each_lane_at_its_turned_edge() {
    let yaw = 0.18_f32;
    let plank = Shape {
        across: Some((-0.6, 0.6)),
        yaw,
        ..block(1.0, 1.4, PLANK_TOP)
    };
    let section = section_of(Some(0.0), vec![plank]);
    let foot_radius = GaitParams::default().foot.radius;
    let (cos, sin) = (yaw.cos(), yaw.sin());
    for lane in &section.foothold_lanes {
        // The near edge, `s = 1.0` before the turn about the centre
        // (1.2, 0), crosses this lane where the unturned `d` is
        // `(lane + 0.2 sin) / cos`.
        let unturned_d = (lane.d + 0.2 * sin) / cos;
        let edge = 1.2 - 0.2 * cos - sin * unturned_d;
        let first_on_top = lane
            .profile
            .surfaces
            .iter()
            .flat_map(|surface| &surface.points)
            .find(|point| {
                // On the top face itself, not the rounding over its edge.
                (point.z - (PLANK_TOP + foot_radius)).abs() < 1e-4 && point.normal.z > 1.0 - 1e-6
            })
            .expect("the plank's top");
        assert!(
            first_on_top.s >= edge - 1e-4 && first_on_top.s <= edge + 0.02 + 1e-4,
            "lane {}: top starts at s = {}, edge at {edge}",
            lane.d,
            first_on_top.s
        );
    }
}

#[test]
fn a_tilt_turns_with_its_stone() {
    let tilt = 0.4_f32;
    let stone = Shape {
        form: Form::Block {
            from: -1.0,
            to: 3.5,
            top: 0.0,
            tilt,
        },
        across: Some((-2.0, 2.0)),
        yaw: std::f32::consts::FRAC_PI_2,
        footing: Footing::Fixed,
    };
    let section = section_of(None, vec![stone]);
    // Turned a quarter counterclockwise, a top rising toward `+d` rises
    // toward `-s`.
    let length = (1.0 + tilt * tilt).sqrt();
    assert!(!samples(lane(&section, 0.0), (0.0, 2.5)).is_empty());
    for lane in &section.foothold_lanes {
        for point in lane
            .profile
            .surfaces
            .iter()
            .flat_map(|surface| &surface.points)
            .filter(|point| point.s > -0.5 && point.s < 3.0)
        {
            assert!(
                (point.normal.s - tilt / length).abs() < 1e-4 && point.normal.d.abs() < 1e-4,
                "lane {}, s = {}: normal {:?}",
                lane.d,
                point.s,
                point.normal
            );
        }
    }
}
