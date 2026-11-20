//! Where water enters and leaves the field other than across its faces:
//! springs and sky sources pour into the cells their water lands in, and sink
//! boxes swallow whatever stands in the cells they hold. Pumps, which only
//! move water within the field, are [`super::pumps`].

use nalgebra::{Point3, Vector3};

use crate::water::geometry::{Column, SpanGraph, SpanIndex, SpanRef};

use super::cells::{FieldCells, CELL_AREA};

/// Gravity, m/s².
const G: f32 = 9.81;

/// Time step of the landing trace, s.
const TRACE_STEP: f32 = 0.01;

/// Farthest a neighbour's floor may stand below the floor the arc meets and
/// still take a share of its water, m.
const FOOTPRINT_DROP: f32 = 1.0;

/// Farthest it may stand above, m: a jet spreads where it lands and down,
/// not up a bank beside it.
const FOOTPRINT_RISE: f32 = 0.1;

/// Longest a trace runs before it gives up: water that has not landed by
/// then has fallen out of the world, s.
const TRACE_LIMIT: f32 = 20.0;

/// An endless source of water: a spring or a sky source.
#[derive(Debug, Clone)]
pub struct Source {
    /// Where the water comes out.
    pub origin: Point3<f32>,
    /// Its launch velocity, m/s.
    pub velocity: Vector3<f32>,
    /// m³/s, forever.
    pub discharge: f32,
    /// The cells its water lands in, as the ground stands: the cell the arc
    /// meets and those around it on much the same floor, a jet's footprint
    /// rather than a point. Empty when it falls out of the world.
    pub landing: Vec<SpanIndex>,
}

impl Source {
    pub fn new(
        origin: Point3<f32>,
        velocity: Vector3<f32>,
        discharge: f32,
        graph: &SpanGraph,
        cells: &FieldCells,
    ) -> Self {
        let mut source = Self {
            origin,
            velocity,
            discharge,
            landing: Vec::new(),
        };
        source.retrace(graph, cells);
        source
    }

    /// Find where the water lands now: the first span whose floor the arc
    /// reaches, and its neighbours on much the same floor or below it.
    pub fn retrace(&mut self, graph: &SpanGraph, cells: &FieldCells) {
        self.landing = landing_cells(graph, cells, self.origin, self.velocity);
    }

    /// Pour `dt` seconds of its water into the cells it lands in, each an
    /// equal share, but none deeper than its roof: a source whose water
    /// lands in a pocket filled to its roof is choked, and pours only as
    /// fast as the pocket lets water out. Returns the volume that left the
    /// source, m³: all of it where it falls out of the world.
    pub fn pour(&self, cells: &mut FieldCells, dt: f32) -> f64 {
        let volume = self.discharge as f64 * dt as f64;
        if self.landing.is_empty() {
            return volume;
        }
        let held_back = cells.pour_into(&self.landing, volume);
        for &at in &self.landing {
            cells.wake(at.slot as usize);
        }
        volume - held_back
    }
}

/// The cells water thrown from `origin` at `velocity` lands in, as the
/// ground stands: the cell the arc meets and those around it on much the
/// same floor. Empty when it falls out of the world.
pub fn landing_cells(
    graph: &SpanGraph,
    cells: &FieldCells,
    origin: Point3<f32>,
    velocity: Vector3<f32>,
) -> Vec<SpanIndex> {
    let Some((column, ordinal)) = trace(graph, origin, velocity) else {
        return Vec::new();
    };
    let hit = SpanRef::key(column, ordinal);
    let floor = graph.span(hit).bed;
    let around = graph
        .neighbours(hit)
        .into_iter()
        .map(|n| SpanRef::key(n.span.column, n.span.ordinal))
        .filter(|s| {
            let rise = graph.span(*s).bed - floor;
            rise < FOOTPRINT_RISE && -rise < FOOTPRINT_DROP
        });
    std::iter::once(hit)
        .chain(around)
        .filter_map(|s| cells.index(graph, s.column, s.ordinal))
        .collect()
}

/// Follow a ballistic arc until it meets a floor.
fn trace(graph: &SpanGraph, origin: Point3<f32>, velocity: Vector3<f32>) -> Option<(Column, u8)> {
    let mut t = 0.0;
    while t < TRACE_LIMIT {
        let p = origin + velocity * t + Vector3::new(0.0, -0.5 * G * t * t, 0.0);
        let column = Column::containing(p.x, p.z);
        if !graph.contains_column(column) {
            return None;
        }
        let span = graph.span_at(column, p.y)?;
        if graph.span(span).bed >= p.y {
            return Some((column, span.ordinal));
        }
        t += TRACE_STEP;
    }
    None
}

/// A box that swallows water.
#[derive(Debug, Clone)]
pub struct Sink {
    pub min: Point3<f32>,
    pub max: Point3<f32>,
    /// The cells whose floor lies inside the box, as the ground stands.
    pub cells: Vec<SpanIndex>,
}

impl Sink {
    pub fn new(min: Point3<f32>, max: Point3<f32>, graph: &SpanGraph, cells: &FieldCells) -> Self {
        let mut sink = Self {
            min,
            max,
            cells: Vec::new(),
        };
        sink.relay(graph, cells);
        sink
    }

    /// Find the cells inside the box again.
    pub fn relay(&mut self, graph: &SpanGraph, cells: &FieldCells) {
        self.cells.clear();
        let from = Column::containing(self.min.x, self.min.z);
        let to = Column::containing(self.max.x, self.max.z);
        for i in from.i..=to.i {
            for k in from.k..=to.k {
                let column = Column::new(i, k);
                if !graph.contains_column(column) {
                    continue;
                }
                for (ordinal, span) in graph.spans(column).iter().enumerate() {
                    if (self.min.y..=self.max.y).contains(&span.bed) {
                        if let Some(at) = cells.index(graph, column, ordinal as u8) {
                            self.cells.push(at);
                        }
                    }
                }
            }
        }
    }

    /// Swallow the water in the box's cells. Returns the volume taken, m³.
    pub fn drain(&self, cells: &mut FieldCells) -> f64 {
        let mut taken = 0.0;
        for &at in &self.cells {
            let depth = cells.depth_mut(at);
            taken += *depth * CELL_AREA;
            *depth = 0.0;
        }
        taken
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::field::testing::{cell, chunk_of, open, roofed};

    #[test]
    fn a_spring_into_a_pocket_filled_to_its_roof_is_choked() {
        // A pocket two cells wide under a roof a metre over its floor.
        let (_, mut cells) = chunk_of(|i, _| {
            if i == 5 || i == 6 {
                roofed(0.0, 1.0)
            } else {
                open(0.0)
            }
        });
        let pocket = [cell(5, 5), cell(6, 5)];
        let spring = Source {
            origin: Point3::new(2.75, 0.5, 2.75),
            velocity: Vector3::zeros(),
            discharge: 1.0,
            landing: pocket.to_vec(),
        };
        let mut sent = 0.0;
        for _ in 0..60 {
            sent += spring.pour(&mut cells, 1.0 / 60.0);
        }
        let room = pocket.len() as f64 * CELL_AREA;
        assert!((sent - room).abs() < 1.0e-9, "sent {sent} m³");
        for c in pocket {
            assert!(cells.room(c).abs() < 1.0e-9, "{c:?} over its roof");
        }
    }
}
