//! The edge test on voxel terrain: the roof over the Traversal Garden's cave
//! lip, its lip 2.5 m up, meshed as the game meshes it.

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
