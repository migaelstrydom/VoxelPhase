//! Lips: where water leaves the upper store of a link (§7.9).
//!
//! ```text
//!   crest cell, reach end, hole, source ──▶ Lip { at, direction }
//!                                            │
//!        separates(graph, thickness)? ◀──────┘  the ground within FALL_RUN
//!                                               past it lies more than the
//!                                               jet's thickness below it
//! ```
//!
//! A lip is fixed when its link is made. Its position and direction are kept
//! so the link's arc can be traced, and traced again after an edit.

use nalgebra::{Point3, Vector2, Vector3};

use crate::water::geometry::{Column, SpanGraph, COLUMN_SIZE};

use super::fall_tracer::{FALL_RUN, GRAVITY};

/// Spacing of the ground samples past a lip, m. Each is taken mid-step, so
/// none falls on a column edge.
const SAMPLE_STEP: f32 = 0.25;

/// Where water leaves the upper store of a link.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Lip {
    /// On the floor where the water leaves.
    pub at: Point3<f32>,
    /// Unit horizontal direction it leaves in; zero for straight down.
    pub direction: Vector2<f32>,
}

impl Lip {
    /// Across the edge between two columns, from `inside` towards `outside`,
    /// at `height`. Two columns that are one (the world's edge) give a lip
    /// with no direction at the inside column's centre.
    pub fn across(inside: Column, outside: Column, height: f32) -> Self {
        let (ix, iz) = inside.centre();
        let (ox, oz) = outside.centre();
        let direction = Vector2::new(ox - ix, oz - iz)
            .try_normalize(1e-6)
            .unwrap_or_else(Vector2::zeros);
        let edge = direction * (0.5 * COLUMN_SIZE);
        Self {
            at: Point3::new(ix + edge.x, height, iz + edge.y),
            direction,
        }
    }

    /// Water leaving straight down, through a hole.
    pub fn down(at: Point3<f32>) -> Self {
        Self {
            at,
            direction: Vector2::zeros(),
        }
    }

    pub fn height(&self) -> f32 {
        self.at.y
    }

    /// Where water leaving at `surface` with horizontal `speed` starts its
    /// arc, and its velocity there.
    pub fn launch(&self, surface: f32, speed: f32) -> (Point3<f32>, Vector3<f32>) {
        let d = self.direction * speed;
        (
            Point3::new(self.at.x, surface.max(self.at.y), self.at.z),
            Vector3::new(d.x, 0.0, d.y),
        )
    }

    /// Whether a jet `thickness` deep leaving here parts from the ground: a
    /// sample of the ground within `FALL_RUN` past the lip lies more than
    /// `thickness` below it, or the world ends there. Water thicker than the
    /// drop runs down it as a steep bed instead. Straight down always parts.
    pub fn separates(&self, graph: &SpanGraph, thickness: f32) -> bool {
        if self.direction == Vector2::zeros() {
            return true;
        }
        let steps = (FALL_RUN / SAMPLE_STEP).round() as i32;
        (1..=steps).any(|i| {
            let p = self.direction * ((i as f32 - 0.5) * SAMPLE_STEP);
            let column = Column::containing(self.at.x + p.x, self.at.z + p.y);
            if !graph.contains_column(column) {
                return true;
            }
            graph
                .span_at(column, self.at.y)
                .is_some_and(|span| graph.span(span).floor_c < self.at.y - thickness)
        })
    }
}

/// Critical depth of `q` m³/s over a lip `width` m wide: the thickness of the
/// jet that leaves it, m.
pub fn critical_depth(q: f64, width: f32) -> f32 {
    let per_metre = q as f32 / width.max(COLUMN_SIZE);
    (per_metre * per_metre / GRAVITY).cbrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::geometry::{Span, SpanChunk, SpanChunkCoord, COLUMNS_PER_CHUNK};

    /// One 8 m chunk whose floor at column `i` is `floor(i)`.
    fn ground(floor: impl Fn(i32) -> f32) -> SpanGraph {
        let coord = SpanChunkCoord { x: 0, z: 0 };
        let mut graph = SpanGraph::new(coord, coord);
        let columns: Vec<Vec<Span>> = (0..COLUMNS_PER_CHUNK)
            .map(|local| {
                let f = floor(coord.column(local).i);
                vec![Span {
                    floor_c: f,
                    floor_min: f,
                    floor_max: f,
                    ceiling: f32::INFINITY,
                }]
            })
            .collect();
        graph.replace_chunk(coord, SpanChunk::from_columns(&columns, 1));
        graph
    }

    #[test]
    fn a_jet_parts_from_a_riser_but_runs_down_a_steep_bed() {
        let lip = Lip::across(Column::new(3, 4), Column::new(4, 4), 5.0);
        // A riser: 5 m, then 3.75 m past the lip.
        let riser = ground(|i| if i <= 3 { 5.0 } else { 3.75 });
        assert!(lip.separates(&riser, 0.3));
        // Water thicker than the step runs down it.
        assert!(!lip.separates(&riser, 1.5));
        // A 1-in-4 bed falls 0.25 m over the metre past the lip.
        let steep = ground(|i| 5.0 - 0.125 * (i - 3).max(0) as f32);
        assert!(!lip.separates(&steep, 0.3));
    }

    #[test]
    fn water_leaving_straight_down_or_off_the_world_always_parts() {
        let flat = ground(|_| 5.0);
        assert!(Lip::down(Point3::new(2.0, 5.0, 2.0)).separates(&flat, 10.0));
        let edge = Lip::across(Column::new(15, 4), Column::new(16, 4), 5.0);
        assert!(edge.separates(&flat, 10.0));
    }

    #[test]
    fn a_lip_across_a_column_edge_sits_on_the_edge() {
        let lip = Lip::across(Column::new(0, 0), Column::new(1, 0), 2.0);
        let (x, _) = Column::new(0, 0).centre();
        assert!((lip.at.x - (x + 0.5 * COLUMN_SIZE)).abs() < 1e-6);
        assert_eq!(lip.direction, Vector2::x());
        assert_eq!(lip.height(), 2.0);
    }

    #[test]
    fn two_cumecs_over_four_metres_leave_a_jet_of_critical_depth() {
        // (0.5² / 9.81)^⅓ = 0.294 m.
        assert!((critical_depth(2.0, 4.0) - 0.294).abs() < 1e-3);
    }
}
