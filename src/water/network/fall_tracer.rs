//! The fall tracer: where water that leaves a lip comes down (§7.6).
//!
//! ```text
//!   lip or source ──parabola, 0.25 m steps──▶ first span it cannot stay in:
//!                                              ground under it, a wall beside it,
//!                                              or off the world
//!                  noting on the way ─────────▶ the first span where a store could
//!                                              hold water at the arc's height
//! ```
//!
//! The arc is swept through the span graph, not the terrain mesh: a point is
//! in air when some span's column range holds it above that span's floor.
//! It runs on through water to the ground, so one arc serves every level the
//! water below may stand at (§7.9); where the water is caught is a question
//! of geometry, answered by `holds`, not of the level now.

use nalgebra::{Point3, Vector3};

use crate::water::geometry::{Column, SpanGraph, SpanRef};

use super::link::FallPath;

/// A step down the drainage path deeper than this, within `FALL_RUN`, is a
/// fall rather than a steep bed, m.
pub const FALL_THRESHOLD: f32 = 0.75;

/// Horizontal run over which a fall's drop is measured, m. A marching-cubes
/// edge rounds a cliff lip over about a voxel, so a drop measured one column
/// at a time would split every riser into two steep steps.
pub const FALL_RUN: f32 = 1.0;

pub const GRAVITY: f32 = 9.81;

/// Arc length per step, m.
const STEP: f32 = 0.25;

/// Longest a fall is followed, s. An arc still falling after this has left
/// the world downwards.
const MAX_FALL_TIME: f32 = 12.0;

/// How far a source placed inside rock is moved along its direction to find
/// air, m, before it is judged buried.
const EMERGE_DISTANCE: f32 = 2.0;

/// Where a traced arc comes down.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Landing {
    /// On the floor of this span, or on water standing over it.
    Span(SpanRef),
    /// Off the edge of the world.
    Void,
}

/// A traced fall: its arc and where it lands.
#[derive(Debug, Clone, PartialEq)]
pub struct Trace {
    pub path: FallPath,
    /// Where the arc meets the ground.
    pub landing: Landing,
    /// The first span along the arc over which a store could hold water at
    /// the arc's height, and that height: where the water is caught, at any
    /// level.
    pub caught: Option<(SpanRef, f32)>,
}

/// What a point of the arc is in.
enum Cell {
    Air(SpanRef),
    Solid,
    Outside,
}

/// Sweeps fall arcs through the span graph.
pub struct FallTracer<'a> {
    pub graph: &'a SpanGraph,
    /// Whether a store could hold water at this height over this span.
    pub holds: &'a dyn Fn(SpanRef, f32) -> bool,
}

impl FallTracer<'_> {
    /// Follow water launched from `from` at `velocity` until it lands. `None`
    /// when `from` is in rock with no air within reach along the launch
    /// direction: a source sealed in.
    pub fn trace(&self, from: Point3<f32>, velocity: Vector3<f32>) -> Option<Trace> {
        let start = self.emerge(from, velocity)?;
        let mut path = FallPath {
            points: vec![start],
            times: vec![0.0],
            velocity,
        };
        let mut caught = None;
        let mut span = match self.cell(start) {
            Cell::Air(span) => span,
            _ => {
                return Some(Trace {
                    path,
                    landing: Landing::Void,
                    caught,
                })
            }
        };
        let gravity = Vector3::new(0.0, -GRAVITY, 0.0);
        let mut t = 0.0f32;
        let mut previous = start;
        while t < MAX_FALL_TIME {
            let speed = (velocity + gravity * t).norm().max(1.0);
            t += STEP / speed;
            let p = start + velocity * t + gravity * (0.5 * t * t);
            match self.cell(p) {
                Cell::Outside => {
                    path.points.push(p);
                    path.times.push(t);
                    return Some(Trace {
                        path,
                        landing: Landing::Void,
                        caught,
                    });
                }
                Cell::Solid => {
                    // Ground under the arc, or a wall across it: the water
                    // comes down in the span it was falling through.
                    let floor = self.graph.span(span).floor_c;
                    let at = if Column::containing(p.x, p.z) == span.column {
                        Point3::new(p.x, floor, p.z)
                    } else {
                        Point3::new(previous.x, floor, previous.z)
                    };
                    path.points.push(at);
                    path.times.push(t);
                    return Some(Trace {
                        path,
                        landing: Landing::Span(span),
                        caught: caught
                            .or(Some((span, floor)).filter(|(s, y)| (self.holds)(*s, *y))),
                    });
                }
                Cell::Air(here) => {
                    span = here;
                    previous = p;
                    if caught.is_none() && (self.holds)(here, p.y) {
                        caught = Some((here, p.y));
                    }
                    path.points.push(p);
                    path.times.push(t);
                }
            }
        }
        Some(Trace {
            path,
            landing: Landing::Void,
            caught,
        })
    }

    /// The first point in air along the launch direction, from `from`.
    fn emerge(&self, from: Point3<f32>, velocity: Vector3<f32>) -> Option<Point3<f32>> {
        if !matches!(self.cell(from), Cell::Solid) {
            return Some(from);
        }
        let direction = velocity.try_normalize(1e-6)?;
        let steps = (EMERGE_DISTANCE / 0.1).ceil() as i32;
        (1..=steps)
            .map(|i| from + direction * (i as f32 * 0.1))
            .find(|p| !matches!(self.cell(*p), Cell::Solid))
    }

    fn cell(&self, p: Point3<f32>) -> Cell {
        let column = Column::containing(p.x, p.z);
        if !self.graph.contains_column(column) {
            return Cell::Outside;
        }
        match self.graph.span_at(column, p.y) {
            Some(span) if self.graph.span(span).floor_c <= p.y => Cell::Air(span),
            _ => Cell::Solid,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::geometry::{Span, SpanChunk, SpanChunkCoord, COLUMNS_PER_CHUNK};

    fn open(floor: f32) -> Span {
        Span {
            floor_c: floor,
            floor_min: floor,
            floor_max: floor,
            ceiling: f32::INFINITY,
        }
    }

    /// One 8 m chunk: a cliff top at 10 m for columns i < 4 (x < 2 m), and
    /// ground at 0 beyond, with a wall of rock to 20 m at i = 14.
    fn cliff() -> SpanGraph {
        let coord = SpanChunkCoord { x: 0, z: 0 };
        let mut graph = SpanGraph::new(coord, coord);
        let columns: Vec<Vec<Span>> = (0..COLUMNS_PER_CHUNK)
            .map(|local| {
                let c = coord.column(local);
                let floor = match c.i {
                    i if i < 4 => 10.0,
                    i if i >= 14 => 20.0,
                    _ => 0.0,
                };
                vec![open(floor)]
            })
            .collect();
        graph.replace_chunk(coord, SpanChunk::from_columns(&columns, 1));
        graph
    }

    #[test]
    fn water_off_a_cliff_lands_where_the_parabola_meets_the_ground() {
        let graph = cliff();
        let dry = |_: SpanRef, _: f32| false;
        let tracer = FallTracer {
            graph: &graph,
            holds: &dry,
        };
        // 10 m at 1.4 m/s: 1.43 s in the air, 2 m out.
        let trace = tracer
            .trace(Point3::new(2.0, 10.0, 4.0), Vector3::new(1.4, 0.0, 0.0))
            .unwrap();
        let landing = trace.path.landing().unwrap();
        assert!((landing.x - 4.0).abs() < 0.25, "{landing:?}");
        assert_eq!(landing.y, 0.0);
        assert!((trace.path.times.last().unwrap() - 1.43).abs() < 0.05);
        let Landing::Span(span) = trace.landing else {
            panic!("landed nowhere");
        };
        assert_eq!(span.column, Column::containing(landing.x, landing.z));
    }

    #[test]
    fn a_jet_into_a_wall_comes_down_at_its_foot() {
        let graph = cliff();
        let dry = |_: SpanRef, _: f32| false;
        let tracer = FallTracer {
            graph: &graph,
            holds: &dry,
        };
        let trace = tracer
            .trace(Point3::new(2.0, 10.0, 4.0), Vector3::new(8.0, 0.0, 0.0))
            .unwrap();
        let landing = trace.path.landing().unwrap();
        assert!(landing.x < 7.0 && landing.x > 6.5, "{landing:?}");
        assert_eq!(landing.y, 0.0);
    }

    #[test]
    fn water_runs_on_to_the_ground_through_a_pond_that_catches_it() {
        let graph = cliff();
        // A pond whose banks could hold it to 3 m, from x = 2 on.
        let pond = |span: SpanRef, y: f32| span.column.i >= 4 && y <= 3.0;
        let tracer = FallTracer {
            graph: &graph,
            holds: &pond,
        };
        let trace = tracer
            .trace(Point3::new(2.0, 10.0, 4.0), Vector3::new(1.4, 0.0, 0.0))
            .unwrap();
        assert_eq!(trace.path.landing().unwrap().y, 0.0);
        let (caught, _) = trace.caught.expect("the pond catches it");
        let Landing::Span(landing) = trace.landing else {
            panic!("landed nowhere");
        };
        // Caught where it crosses 3 m, a little short of where it lands.
        assert!(
            caught.column.i <= landing.column.i,
            "{caught:?} vs {landing:?}"
        );
    }

    #[test]
    fn a_source_in_rock_emerges_along_its_direction_or_is_sealed() {
        let graph = cliff();
        let dry = |_: SpanRef, _: f32| false;
        let tracer = FallTracer {
            graph: &graph,
            holds: &dry,
        };
        // 0.5 m into the cliff face, pointing out of it.
        let out = tracer.trace(Point3::new(1.5, 5.0, 4.0), Vector3::new(1.0, 0.0, 0.0));
        assert!(out.unwrap().path.points[0].x >= 2.0);
        // Pointing down into the rock under it.
        let sealed = tracer.trace(Point3::new(1.5, 5.0, 4.0), Vector3::new(0.0, -1.0, 0.0));
        assert!(sealed.is_none());
    }
}
