//! The overhang sensor on voxel terrain, meshed as the game meshes it: the
//! Traversal Garden's 0.7 m ledge, whose brink is a bevel, against the
//! garden's slopes and lips, which a body must be able to stand on.

use std::path::Path;

use nalgebra::{Point3, Vector3};

use crate::level::load_level;
use crate::level_check::build_terrain;
use crate::physics::{CollisionFilter, PhysicsConfig, PhysicsWorld};
use crate::terrain::TerrainWorld;
use crate::traversal::WorldCaster;

use super::{LocomotionConfig, OverhangSensor};

/// Lanes of `levels/traversal_garden.level.ron`, in z.
const LEDGE_LANE: f32 = 58.0;
const SLOPE_40_LANE: f32 = 34.0;
const SLOPE_50_LANE: f32 = 42.0;
/// The 3, 8, 15 and 25 cm lips.
const LOW_LIP_LANES: [f32; 4] = [-12.0, -8.0, -4.0, 0.0];

struct Garden {
    terrain: TerrainWorld,
    world: PhysicsWorld,
    config: LocomotionConfig,
}

impl Garden {
    fn new() -> Self {
        let level = load_level(Path::new("levels/traversal_garden.level.ron")).expect("the garden");
        Self {
            terrain: build_terrain(&level),
            world: PhysicsWorld::new(PhysicsConfig::default()),
            config: LocomotionConfig::player(),
        }
    }

    fn caster(&self) -> WorldCaster<'_> {
        WorldCaster {
            world: &self.world,
            scenery: &self.terrain,
            exclude: &[],
            filter: CollisionFilter::DEFAULT,
            carriers: &[],
        }
    }

    /// What the sensor says of the player's capsule at rest over `(x, z)`:
    /// its bottom sphere let down onto the ground there, held up along the
    /// normal it lands on.
    fn way_off(&self, x: f32, z: f32) -> Option<Vector3<f32>> {
        let (radius, half_height) = (
            self.config.collider_radius,
            self.config.collider_half_height,
        );
        let caster = self.caster();
        let landed = caster
            .cast(Point3::new(x, 20.0, z), -Vector3::y(), 25.0, radius)
            .unwrap_or_else(|| panic!("no ground at ({x}, {z})"));
        let centre = landed.centre + Vector3::y() * (half_height - radius);
        OverhangSensor::default().way_off(
            &caster,
            centre,
            landed.contact.normal,
            Vector3::y(),
            half_height,
        )
    }
}

/// Edged off the ledge's top until its centre is over the drop, the body
/// rests on the bevel with nothing under it: steered off, down the lane.
#[test]
fn a_body_past_the_ledges_brink_is_over_a_drop() {
    let garden = Garden::new();
    let outward = garden
        .way_off(6.87, LEDGE_LANE)
        .expect("the brink's bevel holds the body over nothing");
    assert!(outward.x < -0.95, "steered off along {outward}");
}

#[test]
fn a_body_on_the_ledges_top_is_not() {
    let garden = Garden::new();
    assert_eq!(garden.way_off(7.3, LEDGE_LANE), None);
}

/// A walkable slope goes on under the body wherever it stands on it.
#[test]
fn no_body_on_a_slope_is_over_a_drop() {
    let garden = Garden::new();
    for lane in [SLOPE_40_LANE, SLOPE_50_LANE] {
        for step in 0..=60 {
            let x = 6.5 + step as f32 * 0.1;
            assert_eq!(garden.way_off(x, lane), None, "at ({x}, {lane})");
        }
    }
}

/// A lip lower than the probe's reach is ground underfoot, brink and all.
#[test]
fn no_body_on_a_low_lip_is_over_a_drop() {
    let garden = Garden::new();
    for lane in LOW_LIP_LANES {
        for step in 0..=30 {
            let x = 6.0 + step as f32 * 0.05;
            assert_eq!(garden.way_off(x, lane), None, "at ({x}, {lane})");
        }
    }
}
