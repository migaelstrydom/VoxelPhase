//! Voronoi partition of a convex polygon.
//!
//! Each seed owns the part of the boundary closer to it than to any other
//! seed. That region is the boundary cut by one half-plane per rival seed,
//! and cutting a convex polygon by half-planes leaves a convex polygon —
//! which is the whole reason a crack pattern is a Voronoi diagram here rather
//! than anything more physical: every piece it makes is a shape the collision
//! engine can take as it is.
//!
//! Quadratic in the seed count, which is fine for the few dozen seeds a
//! crack web uses and would not be for a thousand.

use nalgebra::Vector2;

use super::polygon::ConvexPolygon;

/// Seeds closer than this are the same seed.
const COINCIDENT: f32 = 1e-4;

/// The cells of the seeds inside `boundary`, in seed order.
///
/// A seed may lie outside the boundary and still own part of it; a seed whose
/// region is entirely outside owns nothing and yields `None`.
pub fn voronoi_cells(
    seeds: &[Vector2<f32>],
    boundary: &ConvexPolygon,
) -> Vec<Option<ConvexPolygon>> {
    seeds
        .iter()
        .enumerate()
        .map(|(i, seed)| {
            let mut cell = boundary.clone();
            for (j, rival) in seeds.iter().enumerate() {
                if i == j {
                    continue;
                }
                let towards = rival - seed;
                let distance = towards.magnitude();
                if distance <= COINCIDENT {
                    // Two seeds on top of each other: the first keeps the
                    // region, the second gets nothing, so the partition
                    // still covers the boundary exactly once.
                    if j < i {
                        return None;
                    }
                    continue;
                }
                let normal = towards / distance;
                let midpoint = (seed + rival) * 0.5;
                cell = cell.clip(normal, normal.dot(&midpoint))?;
            }
            Some(cell)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_cells_partition_the_boundary() {
        let boundary = ConvexPolygon::rectangle(Vector2::zeros(), 1.0, 0.5);
        let seeds = [
            Vector2::new(-0.5, 0.1),
            Vector2::new(0.4, -0.2),
            Vector2::new(0.1, 0.3),
            Vector2::new(0.8, 0.4),
        ];
        let cells = voronoi_cells(&seeds, &boundary);
        let total: f32 = cells.iter().flatten().map(ConvexPolygon::area).sum();
        assert!((total - boundary.area()).abs() < 1e-4, "total {total}");
        for (seed, cell) in seeds.iter().zip(&cells) {
            let cell = cell.as_ref().expect("every inside seed owns a cell");
            assert!(cell.contains(*seed));
        }
    }

    #[test]
    fn a_seed_far_outside_owns_nothing() {
        let boundary = ConvexPolygon::rectangle(Vector2::zeros(), 1.0, 1.0);
        let seeds = [Vector2::zeros(), Vector2::new(50.0, 0.0)];
        let cells = voronoi_cells(&seeds, &boundary);
        assert!(cells[0].is_some());
        assert!(cells[1].is_none());
    }

    #[test]
    fn coincident_seeds_do_not_double_count() {
        let boundary = ConvexPolygon::rectangle(Vector2::zeros(), 1.0, 1.0);
        let seeds = [
            Vector2::new(0.2, 0.2),
            Vector2::new(0.2, 0.2),
            Vector2::new(-0.5, 0.0),
        ];
        let cells = voronoi_cells(&seeds, &boundary);
        let total: f32 = cells.iter().flatten().map(ConvexPolygon::area).sum();
        assert!((total - 4.0).abs() < 1e-4);
        assert!(cells[1].is_none());
    }
}
