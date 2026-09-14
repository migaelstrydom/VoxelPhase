//! Where the cracks go: the seed layout of an impact's crack web.
//!
//! ```text
//!            ·      ·                 rings of seeds around the hit, each
//!        ·   ·  ·  ·   ·              ring wider and more populous than
//!       ·  ·  · x ·  ·  ·             the last: Voronoi cells between rings
//!        ·   ·  ·  ·   ·              are the concentric cracks, cells
//!            ·      ·                 between neighbours the radial ones
//!
//!    ·          ·          ·          background seeds far from the hit,
//!         ·          ·                so no shard is wider than the sheet
//!                                     can afford
//! ```
//!
//! Real glass breaks in radial cracks running out from the impact and
//! concentric rings around it, tight near the hit and coarse far away. Seeds
//! on growing rings give exactly that once they are turned into Voronoi
//! cells, and jittering them keeps the web from looking drawn with a
//! compass. The background seeds are a concession to the collision engine
//! rather than to realism: a shard is a thin convex hull, and a hull the
//! engine accepts cannot be too wide for its thickness.

use std::f32::consts::TAU;

use nalgebra::Vector2;

use super::polygon::ConvexPolygon;

/// The shape of a crack web.
#[derive(Debug, Clone, Copy)]
pub struct CrackWeb {
    /// Radius of the innermost ring, in metres: the size of the smallest
    /// shard around the impact.
    pub core_radius: f32,
    /// Ratio between one ring's radius and the next.
    pub ring_growth: f32,
    /// How many rings at most; fewer when the cell runs out first.
    pub rings: u32,
    /// Spacing of the background seeds that cap shard size far from the hit.
    pub background_spacing: f32,
}

impl CrackWeb {
    /// The seeds of a web around `hit` within `cell`, in a layout chosen by
    /// `salt`. The hit itself is always the first seed.
    pub fn seeds(&self, cell: &ConvexPolygon, hit: Vector2<f32>, salt: u32) -> Vec<Vector2<f32>> {
        let mut seeds = vec![hit];
        let reach = cell.extent();
        let mut outer = 0.0f32;
        let mut noise = Jitter::new(salt);

        for ring in 0..self.rings {
            let radius = self.core_radius * self.ring_growth.powi(ring as i32);
            if radius > reach {
                break;
            }
            outer = radius;
            let count = 6 + 2 * ring;
            let step = TAU / count as f32;
            let phase = if ring % 2 == 1 { 0.5 } else { 0.0 };
            for k in 0..count {
                let angle = (k as f32 + phase + 0.35 * (noise.unit() - 0.5)) * step;
                let r = radius * (1.0 + 0.3 * (noise.unit() - 0.5));
                let seed = hit + Vector2::new(angle.cos(), angle.sin()) * r;
                if self.admits(cell, &seeds, seed) {
                    seeds.push(seed);
                }
            }
        }

        let spacing = self.background_spacing;
        if spacing > 0.0 {
            let (min, max) = cell.bounds();
            let columns = ((max.x - min.x) / spacing).ceil().max(1.0) as u32;
            let rows = ((max.y - min.y) / spacing).ceil().max(1.0) as u32;
            for row in 0..rows {
                for column in 0..columns {
                    let seed = Vector2::new(
                        min.x + (column as f32 + 0.2 + 0.6 * noise.unit()) * spacing,
                        min.y + (row as f32 + 0.2 + 0.6 * noise.unit()) * spacing,
                    );
                    if (seed - hit).magnitude() > outer + spacing * 0.5
                        && self.admits(cell, &seeds, seed)
                    {
                        seeds.push(seed);
                    }
                }
            }
        }

        seeds
    }

    /// Whether a seed may join the layout: inside the cell, clear of its
    /// boundary, and clear of every seed already placed.
    ///
    /// A seed hugging the boundary owns a sliver between itself and the edge,
    /// and two seeds on top of each other own a sliver between them. Both are
    /// shards too thin to be worth a collider, and the width rule would
    /// reject the whole layout for them.
    fn admits(&self, cell: &ConvexPolygon, placed: &[Vector2<f32>], seed: Vector2<f32>) -> bool {
        let clearance = self.core_radius * 0.4;
        cell.contains(seed)
            && cell.boundary_distance(seed) >= clearance
            && placed
                .iter()
                .all(|other| (other - seed).magnitude() >= clearance)
    }
}

/// A small deterministic noise source, so a web depends only on where the
/// sheet was hit and not on anything that happened before.
struct Jitter {
    state: u32,
}

impl Jitter {
    fn new(salt: u32) -> Self {
        Self {
            state: salt.wrapping_mul(0x9E37_79B9) ^ 0x5bd1_e995,
        }
    }

    /// Uniform in `[0, 1)`.
    fn unit(&mut self) -> f32 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 17;
        self.state ^= self.state << 5;
        (self.state >> 8) as f32 / (1u32 << 24) as f32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn web() -> CrackWeb {
        CrackWeb {
            core_radius: 0.08,
            ring_growth: 1.7,
            rings: 5,
            background_spacing: 0.4,
        }
    }

    #[test]
    fn the_hit_is_the_first_seed_and_the_web_is_densest_around_it() {
        let cell = ConvexPolygon::rectangle(Vector2::zeros(), 1.0, 0.75);
        let hit = Vector2::new(0.3, -0.2);
        let seeds = web().seeds(&cell, hit, 7);
        assert_eq!(seeds[0], hit);
        let near = seeds
            .iter()
            .filter(|s| (*s - hit).magnitude() < 0.3)
            .count();
        let far = seeds
            .iter()
            .filter(|s| (*s - hit).magnitude() > 0.9)
            .count();
        assert!(near > far, "near {near} far {far}");
    }

    #[test]
    fn the_far_corner_still_gets_seeds() {
        let cell = ConvexPolygon::rectangle(Vector2::zeros(), 1.5, 1.0);
        let hit = Vector2::new(-1.4, -0.9);
        let seeds = web().seeds(&cell, hit, 1);
        assert!(
            seeds.iter().any(|s| s.x > 0.8 && s.y > 0.4),
            "no background seed near the far corner"
        );
    }

    #[test]
    fn the_same_hit_gives_the_same_web() {
        let cell = ConvexPolygon::rectangle(Vector2::zeros(), 1.0, 1.0);
        let a = web().seeds(&cell, Vector2::new(0.1, 0.1), 3);
        let b = web().seeds(&cell, Vector2::new(0.1, 0.1), 3);
        assert_eq!(a, b);
    }
}
