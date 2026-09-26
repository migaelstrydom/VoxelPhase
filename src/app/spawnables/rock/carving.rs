//! Knapping a rock out of a block: the seeded shape generator.
//!
//! ```text
//!   seed ──▶ block of random proportions ──▶ cuts, each through ──▶ scaled to size,
//!            (length × breadth × height)     a corner, tilted and    centred on its
//!                                            bitten a random depth   centre of volume
//!                                                  │
//!                                                  └─ refused if it leaves a stub,
//!                                                     a knife edge or a near-flat one
//! ```
//!
//! A field stone is a block that has had its corners knocked off, many times
//! over. Each cut is aimed out through one of the rock's corners as it stands,
//! tilted at random, and set a random depth in from wherever the rock reaches
//! furthest along it, so every cut takes a corner off. Few cuts leave a
//! chunky, angular stone; many leave a rounded one. The weathered drawing
//! rounds the arrises either way, so the collider can stay the plain faceted
//! hull.
//!
//! A cut that leaves a stub of an edge, a knife edge or two faces meeting
//! almost flat is refused: none of them reads as a knocked-off corner, and
//! the first two are finer than the wear cut into them.

use std::f32::consts::TAU;

use nalgebra::Vector3;

use crate::app::spawnables::shared::textures::TextureRng;
use crate::collision::convex_hull::{cube_hull, ConvexHull};
use crate::collision::hull_split::{HullDraft, Plane};

/// Shortest edge a cut may leave, as a share of the block's length. A cut that
/// just misses a corner leaves a facet a few millimetres across: too small to
/// read as a facet, and smaller than a chip, so the wear cuts it away from
/// every side at once.
const MIN_EDGE: f32 = 0.08;

/// Sharpest an edge may be, in degrees between its two faces. A cut that
/// meets a face at a knife angle is a flake, not a knocked-off corner, and a
/// wedge that thin is thinner than the wear cuts into it from either side.
/// Square, the block's own edges, is as sharp as it gets.
const MIN_EDGE_ANGLE: f32 = 90.0;

/// Flattest an edge may be, in degrees between its two faces. Two faces this
/// close to one plane read as one bent face once the arris between them is
/// rounded over, so the cut is wasted.
const MAX_EDGE_ANGLE: f32 = 150.0;

/// How many planes may be tried for each cut before the rock is left with
/// fewer cuts than it drew.
const ATTEMPTS_PER_CUT: u32 = 4;

/// How a family of rocks is cut. Every range is sampled per rock from its
/// seed, so one carving gives a whole rockery of stones that are alike but
/// never the same.
pub struct RockCarving {
    /// Breadth of the starting block as a share of its length.
    pub breadth: (f32, f32),
    /// Height of the starting block as a share of its length.
    pub height: (f32, f32),
    /// How many cuts are made. Fewer is more angular.
    pub cuts: (u32, u32),
    /// How deep one cut bites, as a share of the block's length, measured in
    /// from the furthest point of the rock along the cut's normal.
    pub depth: (f32, f32),
    /// Most a cut's normal is tilted off the direction to the corner it
    /// takes, as the length of a random vector added to that unit direction.
    pub tilt: f32,
}

impl RockCarving {
    /// Rockery stones: chunky, longer than they are tall, with a few big
    /// facets and an occasional rounder one among them.
    pub const GARDEN: Self = Self {
        breadth: (0.6, 0.95),
        height: (0.45, 0.8),
        cuts: (6, 14),
        depth: (0.1, 0.3),
        tilt: 1.0,
    };

    /// A rock `size` metres along its longest side, cut as `seed` says.
    /// Centred on its centre of volume, so it can be a body's hull as it is.
    pub fn carve(&self, size: f32, seed: u32) -> ConvexHull {
        let mut rng = TextureRng::new(seed);

        let half_extents = Vector3::new(
            0.5,
            0.5 * rng.range(self.height.0, self.height.1),
            0.5 * rng.range(self.breadth.0, self.breadth.1),
        );
        let mut rock = cube_hull(half_extents);

        let cuts = self.cuts.0 + rng.pick(self.cuts.1 - self.cuts.0 + 1);
        let mut made = 0;
        for _ in 0..cuts * ATTEMPTS_PER_CUT {
            if made == cuts {
                break;
            }
            let normal = self.aim(&rock, &mut rng);
            let depth = rng.range(self.depth.0, self.depth.1);
            let plane = Plane {
                normal,
                offset: reach(&rock, normal) - depth,
            };
            // A cut the draft refuses would have left a sliver; one that
            // leaves a stub of an edge or a knife edge, a stone that does not
            // look like one. Either way the rock keeps that corner and another
            // cut is tried.
            if let Some(cut) = HullDraft::of(&rock).trim(plane).map(HullDraft::build) {
                if well_formed(&cut) {
                    rock = cut;
                    made += 1;
                }
            }
        }

        let centred = rock.translated(-rock.centroid());
        let length = longest_side(&centred);
        centred.scaled(size / length)
    }

    /// Which way a cut faces: out through one of the rock's corners, tilted at
    /// random. Aimed at a corner so that every cut knocks one off — a plane
    /// drawn from anywhere at all mostly shaves a face or grazes an edge,
    /// which reads as a block, not a rock.
    fn aim(&self, rock: &ConvexHull, rng: &mut TextureRng) -> Vector3<f32> {
        let centre = rock.centroid();
        let corner = rock.vertices[rng.pick(rock.vertices.len() as u32) as usize];
        let outward = (corner - centre).normalize();
        (outward + random_direction(rng) * rng.range(0.0, self.tilt)).normalize()
    }
}

/// How far `hull` reaches along `direction`.
fn reach(hull: &ConvexHull, direction: Vector3<f32>) -> f32 {
    hull.vertices
        .iter()
        .map(|v| direction.dot(v))
        .fold(f32::NEG_INFINITY, f32::max)
}

/// Whether every edge of `hull` is long enough to read as one and blunt
/// enough to weather as stone does.
fn well_formed(hull: &ConvexHull) -> bool {
    // A hair under, so the block's own square edges pass.
    let blunt = (180.0 - MIN_EDGE_ANGLE).to_radians().cos() - 1e-4;
    let distinct = (180.0 - MAX_EDGE_ANGLE).to_radians().cos();
    hull.edges.iter().all(|edge| {
        let length =
            (hull.vertices[edge.v0 as usize] - hull.vertices[edge.v1 as usize]).magnitude();
        let turn = edge.normal_a.dot(&edge.normal_b);
        length >= MIN_EDGE && turn >= blunt && turn <= distinct
    })
}

/// A direction drawn evenly over the sphere.
fn random_direction(rng: &mut TextureRng) -> Vector3<f32> {
    let z = rng.range(-1.0, 1.0);
    let azimuth = rng.range(0.0, TAU);
    let ring = (1.0 - z * z).sqrt();
    Vector3::new(ring * azimuth.cos(), z, ring * azimuth.sin())
}

/// The longest side of the hull's axis-aligned box.
fn longest_side(hull: &ConvexHull) -> f32 {
    let (lo, hi) = bounds(hull);
    (hi - lo).max()
}

/// The shortest side of the hull's axis-aligned box: how thick the rock is,
/// which sets how large its texture is laid on.
pub fn thickness(hull: &ConvexHull) -> f32 {
    let (lo, hi) = bounds(hull);
    (hi - lo).min()
}

fn bounds(hull: &ConvexHull) -> (Vector3<f32>, Vector3<f32>) {
    hull.vertices.iter().fold(
        (
            Vector3::repeat(f32::INFINITY),
            Vector3::repeat(f32::NEG_INFINITY),
        ),
        |(lo, hi), v| (lo.inf(v), hi.sup(v)),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_rock_is_as_long_as_its_size_and_centred() {
        for seed in 0..200 {
            let hull = RockCarving::GARDEN.carve(0.4, seed);
            assert!((longest_side(&hull) - 0.4).abs() < 1e-4, "seed {seed}");
            assert!(hull.centroid().norm() < 1e-4, "seed {seed}");
        }
    }

    #[test]
    fn the_same_seed_cuts_the_same_rock() {
        let a = RockCarving::GARDEN.carve(0.4, 7);
        let b = RockCarving::GARDEN.carve(0.4, 7);
        assert_eq!(a.vertices, b.vertices);
    }

    #[test]
    fn different_seeds_cut_different_rocks() {
        let a = RockCarving::GARDEN.carve(0.4, 7);
        let b = RockCarving::GARDEN.carve(0.4, 8);
        assert_ne!(a.vertices, b.vertices);
    }

    /// No rock is left a block: it has more faces than the box it was cut
    /// from, and clearly less volume than the box around it.
    #[test]
    fn every_rock_is_cut() {
        for seed in 0..200 {
            let hull = RockCarving::GARDEN.carve(1.0, seed);
            assert!(
                hull.faces.len() > 6,
                "seed {seed}: {} faces",
                hull.faces.len()
            );
            let (lo, hi) = bounds(&hull);
            let extent = hi - lo;
            let fill = hull.compute_volume() / (extent.x * extent.y * extent.z);
            assert!(fill < 0.98, "seed {seed}: fills {fill} of its box");
        }
    }
}
