//! End-to-end tests for the traversal primitives: authored RON in, meshed
//! geometry out.
//!
//! These deliberately go through the whole path — level parsing, generation,
//! marching cubes — rather than testing the distance functions directly. What
//! matters about a route is where its surface *ends up*, and the two things
//! that could put it somewhere else (the lattice the rasteriser samples on, and
//! the density encoding marching cubes interpolates) both live between the
//! distance function and the mesh.

use nalgebra::Point3;

use crate::level::{build_segments, resolve_placements, Level};
use crate::terrain::{SegmentFrame, TerrainWorld};

/// Build a one-segment world holding nothing but the given volume features.
///
/// The heightfield is pushed far below the segment's extent so no ground is
/// generated: whatever the mesh contains is the primitives and nothing else.
fn world_of(volumes: &str, voxel_size: f32, yaw: f32) -> TerrainWorld {
    let ron = format!(
        r#"Level(
            name: "Traversal fixture",
            segments: [(
                name: "main",
                terrain: Terrain(
                    voxel_size: {voxel_size},
                    bounds: (min: (0.0, -16.0, 0.0), max: (64.0, 48.0, 64.0)),
                    base_height: -1000.0,
                    features: [],
                    volumes: [{volumes}],
                ),
            )],
            placements: [Root(segment: "main", yaw: {yaw})],
            player_spawn: (8.0, 12.0, 8.0),
        )"#
    );
    let mut level: Level = ron::from_str(&ron).expect("fixture level should parse");
    level.frames = resolve_placements(&level).expect("fixture placement should resolve");
    TerrainWorld::from_segments_headless(build_segments(&level).expect("fixture should build"))
}

/// The frame the fixture's single segment ends up in.
fn fixture_frame(yaw: f32) -> SegmentFrame {
    SegmentFrame::new(Point3::origin(), (yaw / 90.0).round() as i32)
}

/// Highest meshed surface in a segment-local column, or `None` for a void.
fn surface_at(world: &TerrainWorld, frame: &SegmentFrame, local: (f32, f32)) -> Option<f32> {
    let p = frame.to_world(Point3::new(local.0, 0.0, local.1));
    world.mesh_surface_height_at(p.x, p.z)
}

/// Every meshed surface in a segment-local column, highest first.
fn surfaces_at(world: &TerrainWorld, frame: &SegmentFrame, local: (f32, f32)) -> Vec<f32> {
    let p = frame.to_world(Point3::new(local.0, 0.0, local.1));
    world.mesh_surface_heights_at(p.x, p.z)
}

/// Whether any surface in a column sits within a tread's thickness of `y`.
fn has_surface_near(world: &TerrainWorld, frame: &SegmentFrame, local: (f32, f32), y: f32) -> bool {
    surfaces_at(world, frame, local)
        .iter()
        .any(|s| (s - y).abs() < 0.3)
}

/// Assert a column's meshed surface sits at `expected`.
///
/// The tolerance is a *fraction of a voxel*, which is the point: naive
/// rasterisation snaps a surface to the lattice, so anything under half a voxel
/// of error can only come from the sub-voxel density encoding working.
fn assert_surface(
    world: &TerrainWorld,
    frame: &SegmentFrame,
    local: (f32, f32),
    expected: f32,
    what: &str,
) {
    let got = surface_at(world, frame, local)
        .unwrap_or_else(|| panic!("{what}: no surface at local {local:?}"));
    assert!(
        (got - expected).abs() < 0.05,
        "{what}: surface at local {local:?} is {got:.3}, authored {expected:.3}"
    );
}

const CATWALK: &str = r#"Path(
    points: [(8.0, 12.0, 8.0), (40.0, 12.0, 8.0), (40.0, 20.0, 40.0)],
    width: 4.0,
    thickness: 2.0,
    material: Sand,
)"#;

/// A path that turns a corner and climbs must still be one closed surface.
/// Every leg is swept independently, so a gap at a joint is the failure mode
/// this rules out — and an open edge is how it would show.
#[test]
fn a_path_that_turns_and_climbs_is_watertight() {
    let world = world_of(CATWALK, 0.5, 0.0);
    assert!(
        world.triangle_count() > 0,
        "the path generated no geometry at all"
    );
    assert_eq!(
        world.open_edge_count(),
        0,
        "the swept path has open edges: {} of {} triangles",
        world.open_edge_count(),
        world.triangle_count()
    );
}

/// The corner itself is covered. A polyline swept leg-by-leg with square ends
/// would leave a notch on the outside of every turn.
#[test]
fn a_path_corner_is_filled() {
    let world = world_of(CATWALK, 0.5, 0.0);
    let frame = fixture_frame(0.0);
    // On the diagonal outside the corner, where neither leg's centreline
    // reaches: covered only by the half-round cap the sweep leaves there.
    let corner = surface_at(&world, &frame, (41.0, 9.0)).expect("corner is not filled");
    assert!(
        (12.0..12.4).contains(&corner),
        "corner surface at {corner:.2}, expected the deck's 12 m level"
    );
}

/// The guarantee that stops a route being resolution-dependent: the deck's top
/// lands on its authored height whether the segment runs metre or
/// quarter-metre voxels.
#[test]
fn a_path_lands_at_its_authored_height_at_two_resolutions() {
    for voxel in [1.0, 0.25] {
        let world = world_of(CATWALK, voxel, 0.0);
        let frame = fixture_frame(0.0);
        assert_surface(&world, &frame, (20.0, 8.0), 12.0, "flat leg");
        // Half way up the climbing leg: the leg runs 32 m in z while gaining
        // 8 m, so z = 24 is at y = 16.
        assert_surface(&world, &frame, (40.0, 24.0), 16.0, "climbing leg");
    }
}

/// A rounded path is still flat where the player walks; only its edges and
/// underside curve away.
#[test]
fn a_rounded_path_keeps_a_flat_walking_surface() {
    let world = world_of(
        r#"Path(
            points: [(8.0, 10.0, 32.0), (56.0, 10.0, 32.0)],
            width: 4.0,
            thickness: 1.5,
            profile: Rounded,
        )"#,
        0.25,
        0.0,
    );
    let frame = fixture_frame(0.0);
    assert_surface(&world, &frame, (32.0, 32.0), 10.0, "centreline");
    assert_surface(&world, &frame, (32.0, 31.0), 10.0, "one metre off centre");
}

#[test]
fn a_platform_lands_at_its_authored_height_at_two_resolutions() {
    for voxel in [1.0, 0.25] {
        let world = world_of(
            r#"Platform(center: (24.0, 9.0, 24.0), half_extents: (4.0, 3.0), thickness: 2.0)"#,
            voxel,
            0.0,
        );
        let frame = fixture_frame(0.0);
        assert_surface(&world, &frame, (24.0, 24.0), 9.0, "platform centre");
        assert_surface(&world, &frame, (27.0, 26.0), 9.0, "platform corner");
        assert!(
            surface_at(&world, &frame, (32.0, 24.0)).is_none(),
            "a free-standing platform grew past its half-extents"
        );
    }
}

/// Each tread lands on its own authored height. A staircase that interpolated
/// between densities across a riser would put the treads at heights nobody
/// authored, which is precisely what makes a step rise uncheckable.
#[test]
fn staircase_treads_land_at_their_authored_heights() {
    for voxel in [1.0, 0.25] {
        let world = world_of(
            r#"Staircase(
                from: (16.0, 8.0, 32.0),
                to: (24.0, 12.0, 32.0),
                width: 4.0,
                steps: 4,
                thickness: 2.0,
            )"#,
            voxel,
            0.0,
        );
        let frame = fixture_frame(0.0);
        // Four treads over an 8 m run: each is 2 m long and 1 m higher.
        for i in 0..4 {
            let x = 16.0 + 2.0 * i as f32 + 1.0;
            assert_surface(&world, &frame, (x, 32.0), 9.0 + i as f32, "tread");
        }
    }
}

/// The bore is empty and the ledge is inside it. Without the ledge the shaft is
/// a hole; without the bore the ledge is buried.
#[test]
fn a_shaft_ledge_spirals_inside_an_empty_bore() {
    let world = world_of(
        r#"Platform(center: (32.0, 20.0, 32.0), half_extents: (14.0, 14.0), thickness: 24.0),
           Shaft(
                center: (32.0, 32.0),
                from_y: 0.0,
                to_y: 20.0,
                radius: 8.0,
                ledge: Some((width: 2.5, thickness: 0.6, pitch: 6.0)),
           )"#,
        0.5,
        0.0,
    );
    let frame = fixture_frame(0.0);

    // The middle of the bore is open all the way down: the deepest surface
    // under the axis is the platform's underside, not a floor part way up.
    let axis = surface_at(&world, &frame, (32.0, 32.0));
    assert!(
        axis.is_none() || axis.unwrap() < 0.5,
        "the bore is blocked at {axis:?}"
    );

    // The helix passes through from_y at angle 0 — the +x side — and gains a
    // pitch per turn, so the +x column holds a tread every 6 m.
    assert!(
        has_surface_near(&world, &frame, (39.0, 32.0), 6.0),
        "no ledge one turn above the floor on the +x side: {:?}",
        surfaces_at(&world, &frame, (39.0, 32.0))
    );
    // Half a turn round, on the -x side, the treads are offset by half a pitch.
    assert!(
        has_surface_near(&world, &frame, (25.0, 32.0), 9.0),
        "no ledge half a turn further up on the -x side: {:?}",
        surfaces_at(&world, &frame, (25.0, 32.0))
    );
}

/// Every primitive placed in a segment turned a quarter turn must land in the
/// rotated position with the rotated shape.
///
/// Comparing against the same fixture at yaw 0, sampled through the frame, is
/// the strong form: it catches a primitive that rotated its position but not
/// its geometry, which a spot-check of one column would not.
#[test]
fn every_primitive_turns_with_its_segment() {
    let volumes = format!(
        "{CATWALK},
         Platform(center: (24.0, 9.0, 48.0), half_extents: (5.0, 2.0), thickness: 1.0),
         Staircase(from: (8.0, 4.0, 56.0), to: (20.0, 10.0, 56.0), width: 4.0, steps: 6),
         Platform(center: (52.0, 20.0, 52.0), half_extents: (10.0, 10.0), thickness: 16.0),
         Shaft(
            center: (52.0, 52.0),
            from_y: 6.0,
            to_y: 20.0,
            radius: 6.0,
            ledge: Some((width: 2.5, thickness: 0.6, pitch: 5.0)),
         )"
    );

    let flat = world_of(&volumes, 0.5, 0.0);
    let turned = world_of(&volumes, 0.5, 90.0);
    let flat_frame = fixture_frame(0.0);
    let turned_frame = fixture_frame(90.0);

    // Sample the whole footprint rather than the features individually: a
    // primitive that failed to turn would disagree somewhere on this grid.
    let mut compared = 0;
    for i in 0..64 {
        for j in 0..64 {
            let local = (i as f32 + 0.5, j as f32 + 0.5);
            let a = surface_at(&flat, &flat_frame, local);
            let b = surface_at(&turned, &turned_frame, local);
            match (a, b) {
                (None, None) => {}
                (Some(a), Some(b)) => {
                    assert!(
                        (a - b).abs() < 0.05,
                        "local {local:?}: yaw 0 has a surface at {a:.2}, yaw 90 at {b:.2}"
                    );
                    compared += 1;
                }
                _ => panic!("local {local:?}: yaw 0 gives {a:?}, yaw 90 gives {b:?}"),
            }
        }
    }
    assert!(
        compared > 200,
        "only {compared} columns held geometry; the fixture is not exercising much"
    );
}
