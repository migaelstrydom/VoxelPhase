//! When a pane crazes, and into what.
//!
//! ```text
//!   hit (spike, blast or fatigue)
//!     │
//!     ▼
//!   CrazeRule::craze(cell, hit) ──▶ CrackWeb seeds ──▶ voronoi ──▶ cells
//!     │                                                          │
//!     └── hole_radius(spike) ──────────▶ cells_within(hit) ──────┴──▶ severed
//! ```
//!
//! Two decisions live here and both are pure geometry over a load number:
//! whether a cell breaks into a web at all, and which of the new cells were
//! under the hit hard enough to be knocked clean out.

use nalgebra::Vector2;

use super::polygon::ConvexPolygon;
use super::voronoi::voronoi_cells;
use super::web::CrackWeb;

/// Widest a shard may be for its thickness. The collision engine rejects a
/// hull thinner than 1% of its width outright; this keeps a margin above it.
const MAX_EXTENT_PER_THICKNESS: f32 = 60.0;

/// How many seed layouts to try before deciding a cell cannot be crazed.
const LAYOUT_ATTEMPTS: u32 = 4;

/// The impact response of one substance of sheet.
#[derive(Debug, Clone, Copy)]
pub struct CrazeRule {
    /// Contact spike or blast, in N·s, above which a cell cracks into a web.
    pub threshold: f32,
    /// The shape of that web.
    pub web: CrackWeb,
    /// A cell smaller than this, in m², is a shard and not a pane: a hit
    /// knocks it out whole rather than cracking it further.
    pub min_area: f32,
}

impl CrazeRule {
    /// The cells `cell` cracks into around `hit`, or `None` if it is too
    /// small to crack or no layout produced shards the engine can take.
    pub fn craze(
        &self,
        cell: &ConvexPolygon,
        hit: Vector2<f32>,
        thickness: f32,
        salt: u32,
    ) -> Option<Vec<ConvexPolygon>> {
        if cell.area() < self.min_area * 4.0 {
            return None;
        }
        let hit = cell.closest_point(hit);
        (0..LAYOUT_ATTEMPTS).find_map(|attempt| {
            let seeds = self.web.seeds(cell, hit, salt.wrapping_add(attempt));
            // An edge shorter than the glass is thick makes a side face the
            // hull builder rejects as flat, so such edges are collapsed.
            let cells: Vec<ConvexPolygon> = voronoi_cells(&seeds, cell)
                .into_iter()
                .flatten()
                .filter_map(|shard| shard.simplified(thickness))
                .collect();
            let sound = cells.len() > 1
                && cells.iter().all(|shard| {
                    shard.width() >= thickness
                        && shard.extent() <= thickness * MAX_EXTENT_PER_THICKNESS
                });
            sound.then_some(cells)
        })
    }

    /// How far around a hit of `spike` N·s the shards are knocked out.
    ///
    /// Zero at the threshold — a hit that only just crazes leaves every
    /// shard in place — and growing with the square root of the excess, so
    /// the hole's *area* grows with the energy that went in.
    pub fn hole_radius(&self, spike: f32) -> f32 {
        let excess = (spike / self.threshold - 1.0).max(0.0);
        self.web.core_radius * excess.sqrt()
    }
}

/// The cells under a hit: every cell that reaches within `radius` of it.
///
/// Reaches, not is centred: the radius is the outline of what came through,
/// and a crate rests as happily on the tip of a wedge as on a whole pane.
/// What it presses on, it takes with it.
pub fn cells_within(cells: &[ConvexPolygon], hit: Vector2<f32>, radius: f32) -> Vec<usize> {
    if radius <= 0.0 {
        return Vec::new();
    }
    cells
        .iter()
        .enumerate()
        .filter(|(_, cell)| cell.distance_to(hit) <= radius)
        .map(|(index, _)| index)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rule() -> CrazeRule {
        CrazeRule {
            threshold: 10.0,
            web: CrackWeb {
                core_radius: 0.08,
                ring_growth: 1.7,
                rings: 5,
                background_spacing: 0.4,
            },
            min_area: 0.002,
        }
    }

    /// The point of the whole system: a hit in one corner leaves the far
    /// side in large pieces, and every piece is one the engine accepts.
    #[test]
    fn a_corner_hit_is_fine_near_the_corner_and_coarse_far_from_it() {
        let sheet = ConvexPolygon::rectangle(Vector2::zeros(), 1.0, 0.75);
        let hit = Vector2::new(-0.9, -0.65);
        let cells = rule().craze(&sheet, hit, 0.015, 1).expect("a sheet crazes");

        // Collapsing hairline edges gives away slivers along the cracks;
        // the loss must stay invisible.
        let total: f32 = cells.iter().map(ConvexPolygon::area).sum();
        assert!(
            (total - sheet.area()).abs() < sheet.area() * 0.01,
            "cells cover the sheet: {total}"
        );

        let near: Vec<f32> = cells
            .iter()
            .filter(|c| (c.centroid() - hit).magnitude() < 0.2)
            .map(ConvexPolygon::area)
            .collect();
        let far: Vec<f32> = cells
            .iter()
            .filter(|c| (c.centroid() - hit).magnitude() > 1.2)
            .map(ConvexPolygon::area)
            .collect();
        assert!(!near.is_empty() && !far.is_empty());
        let mean = |v: &[f32]| v.iter().sum::<f32>() / v.len() as f32;
        assert!(
            mean(&far) > mean(&near) * 5.0,
            "near {:?} far {:?}",
            mean(&near),
            mean(&far)
        );

        for cell in &cells {
            assert!(cell.width() >= 0.03);
            assert!(cell.extent() <= 0.9);
        }
    }

    #[test]
    fn a_shard_too_small_to_craze_says_so() {
        let shard = ConvexPolygon::rectangle(Vector2::zeros(), 0.04, 0.04);
        assert!(rule().craze(&shard, Vector2::zeros(), 0.015, 1).is_none());
    }

    #[test]
    fn the_hole_grows_with_the_excess_over_the_threshold() {
        let rule = rule();
        assert_eq!(rule.hole_radius(5.0), 0.0);
        assert_eq!(rule.hole_radius(10.0), 0.0);
        assert!((rule.hole_radius(50.0) - 0.16).abs() < 1e-6);
    }

    #[test]
    fn only_the_cells_around_the_hit_are_under_it() {
        let sheet = ConvexPolygon::rectangle(Vector2::zeros(), 1.0, 1.0);
        let hit = Vector2::new(0.5, 0.5);
        let cells = rule().craze(&sheet, hit, 0.015, 2).expect("crazes");
        let under = cells_within(&cells, hit, 0.15);
        assert!(!under.is_empty());
        assert!(under.len() < cells.len() / 2);
        assert!(cells_within(&cells, hit, 0.0).is_empty());
    }
}
