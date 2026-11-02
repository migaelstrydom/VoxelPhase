//! The edge test on voxel terrain, meshed as the game meshes it: the roof
//! over the Traversal Garden's cave lip, 2.5 m up, and the Ledge Garden's
//! bevelled box.

use std::path::Path;

use nalgebra::{Point3, Vector3};

use crate::level::load_level;
use crate::level_check::build_terrain;
use crate::physics::{PhysicsConfig, PhysicsWorld};
use crate::terrain::TerrainWorld;
use crate::traversal::WorldCaster;

use super::{edge_at_hands, Edge, LedgeConfig, Reach};

fn garden() -> TerrainWorld {
    let level = load_level(Path::new("levels/traversal_garden.level.ron")).expect("the garden");
    build_terrain(&level)
}

/// The edge at the hands of a body facing +x at `centre`.
fn edge_at(terrain: &TerrainWorld, world: &PhysicsWorld, centre: Point3<f32>) -> Option<Edge> {
    let caster = WorldCaster {
        world,
        scenery: terrain,
        exclude: &[],
        carriers: &[],
    };
    let reach = Reach {
        centre,
        heading: Vector3::x(),
        radius: 0.25,
        half_height: 0.5,
    };
    edge_at_hands(&caster, &reach, &LedgeConfig::default())
}

/// Hands on a lip `top` metres up whose face stands at x = 7, from a body
/// `below` metres under it, 5 cm off the face.
fn assert_held(terrain: &TerrainWorld, z: f32, top: f32, below: f32) {
    let world = PhysicsWorld::new(PhysicsConfig::default());
    let centre = Point3::new(6.7, top - below, z);
    let edge = edge_at(terrain, &world, centre)
        .unwrap_or_else(|| panic!("no edge {below} m over a body at z {z}"));
    assert_eq!(edge.body, None);
    for hand in edge.hands {
        assert!(
            (hand.y - top).abs() < 0.05,
            "hand at {hand}, the top at {top}"
        );
        assert!(
            (hand.x - 7.04).abs() < 0.05,
            "hand at {hand}, the face at 7"
        );
    }
    assert!(
        edge.outward.x < -0.99,
        "outward {}",
        edge.outward.into_inner()
    );
    assert!(edge.climbable(), "the top at z {z} is wide and open");
}

#[test]
fn the_cave_roof_is_held_through_the_catch_band() {
    let terrain = garden();
    for below in [0.25, 0.4, 0.6] {
        assert_held(&terrain, 66.0, 2.5, below);
    }
}

/// The Ledge Garden's 0.72 m box, whose lip the voxel mesh bevels over a
/// voxel: held at its top wherever on the bevel's slope a body pushing into
/// it has been lifted to. Found from where the bevel enters the band, the
/// lip moved with the body, and a body riding up the bevel lost it.
#[test]
fn a_bevelled_lip_is_held_at_its_top_wherever_the_body_rides() {
    let level = load_level(Path::new("levels/ledge_garden.level.ron")).expect("the ledge garden");
    let terrain = build_terrain(&level);
    let world = PhysicsWorld::new(PhysicsConfig::default());
    for x in [6.5, 6.6, 6.73] {
        for y in [0.5, 0.55, 0.6, 0.65] {
            let edge = edge_at(&terrain, &world, Point3::new(x, y, 12.16))
                .unwrap_or_else(|| panic!("no edge from ({x}, {y})"));
            for hand in edge.hands {
                assert!(
                    (hand.y - 0.72).abs() < 0.02,
                    "held at {} from ({x}, {y})",
                    hand.y
                );
            }
            assert!(edge.climbable());
        }
    }
}

#[test]
fn scratch_roof_side_climbable() {
    let level = load_level(Path::new("levels/ledge_garden.level.ron")).unwrap();
    let terrain = build_terrain(&level);
    let world = PhysicsWorld::new(PhysicsConfig::default());
    let caster = WorldCaster {
        world: &world,
        scenery: &terrain,
        exclude: &[],
        carriers: &[],
    };
    for x in [8.0, 10.0, 12.9, 14.5, 14.9] {
        for y in [2.67, 2.9, 3.0] {
            let reach = Reach {
                centre: Point3::new(x, y, 15.74),
                heading: Vector3::z(),
                radius: 0.25,
                half_height: 0.5,
            };
            let edge = edge_at_hands(&caster, &reach, &LedgeConfig::default());
            eprintln!("x {x} y {y}: {:?}", edge.map(|e| (e.hands[0].y, e.stand)));
        }
    }
}
