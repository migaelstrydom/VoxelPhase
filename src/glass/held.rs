//! A sheet held along its edges: welds to the world, one per stretch of
//! held edge, that follow the cells along it as the sheet cracks.
//!
//! ```text
//!   ═══════════════════  held edge, welded at its middle
//!   ┌──────────────────┐
//!   │       pane       │      whole: one weld per held edge
//!   └──────────────────┘
//!            │ crazes
//!            ▼
//!   ═══╤═════╤══════╤═══  each cell along the edge welded at the middle
//!   ┌──┴─┬───┴──┬───┴──┐     of its own stretch of it
//!   │    │      │      │
//! ```
//!
//! A weld holds a body by a point, not a cell, so a pane crazed under one
//! weld per edge would hang from whichever cell inherited it, and a crack
//! across the pane would drop the half that did not. Welding every edge cell
//! is what lets a floor cracked across stay up as two pieces.

use nalgebra::{Isometry3, Point3, Vector2};
use rustc_hash::FxHashMap;

use super::pane::SheetFrame;
use super::polygon::ConvexPolygon;
use crate::physics::{ConstraintHandle, ConstraintKind, PhysicsWorld, RigidBodyHandle};

/// How far a cell's edge may lie off a held edge and still be held by it, m:
/// cells are cut from the pane exactly, so this only has to absorb rounding.
const ON_EDGE: f32 = 0.005;

/// Shortest stretch of held edge worth a weld of its own, m: a cell that
/// touches the edge at a corner is held by its neighbours.
const MIN_HELD: f32 = 0.01;

/// A stretch of a sheet's edge welded to the world, in world space, where
/// the weld holds it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HeldEdge {
    pub from: Point3<f32>,
    pub to: Point3<f32>,
}

/// Which stretch of edge each of a sheet's welds holds.
#[derive(Debug, Clone, Default)]
pub struct HeldEdges {
    edges: FxHashMap<ConstraintHandle, HeldEdge>,
}

impl HeldEdges {
    pub fn insert(&mut self, weld: ConstraintHandle, edge: HeldEdge) {
        self.edges.insert(weld, edge);
    }

    /// Forget `weld`, returning what it held.
    pub fn take(&mut self, weld: ConstraintHandle) -> Option<HeldEdge> {
        self.edges.remove(&weld)
    }

    /// What `welds` hold, for a piece that took them.
    pub fn share(&self, welds: &[ConstraintHandle]) -> Self {
        Self {
            edges: welds
                .iter()
                .filter_map(|w| Some((*w, *self.edges.get(w)?)))
                .collect(),
        }
    }

    /// Forget every weld but `welds`.
    pub fn retain(&mut self, welds: &[ConstraintHandle]) {
        self.edges.retain(|w, _| welds.contains(w));
    }

    pub fn is_empty(&self) -> bool {
        self.edges.is_empty()
    }
}

/// Weld `cell` of the sheet on `body` to the world along the stretch of
/// `edge` it lies on, at that stretch's middle. `None` if the cell does not
/// lie along the edge.
pub fn weld_along(
    world: &mut PhysicsWorld,
    body: RigidBodyHandle,
    frame: &SheetFrame,
    cell: &ConvexPolygon,
    edge: &HeldEdge,
) -> Option<(ConstraintHandle, HeldEdge)> {
    let rigid = world.body(body)?;
    let rotation = rigid.rotation();
    let pose = Isometry3::from_parts(rigid.position().coords.into(), rotation);
    let in_plane = |p: &Point3<f32>| frame.to_plane((pose.inverse() * p).coords);
    let (a, b) = (in_plane(&edge.from), in_plane(&edge.to));
    let (start, end) = held_stretch(cell, a, b)?;

    let lift = |p: Vector2<f32>| Point3::from(frame.lift(p, 0.0));
    let middle = lift((start + end) * 0.5);
    let weld = world.create_constraint(ConstraintKind::world_fixed(
        body,
        pose * middle,
        middle.coords,
        &rotation,
        0.0,
        f32::INFINITY,
    ));
    Some((
        weld,
        HeldEdge {
            from: pose * lift(start),
            to: pose * lift(end),
        },
    ))
}

/// The longest stretch of the segment `a`–`b` that an edge of `cell` lies
/// along, as its two ends in the plane.
fn held_stretch(
    cell: &ConvexPolygon,
    a: Vector2<f32>,
    b: Vector2<f32>,
) -> Option<(Vector2<f32>, Vector2<f32>)> {
    let along = b - a;
    let length = along.norm();
    if length <= MIN_HELD {
        return None;
    }
    let direction = along / length;
    let off_line = |p: Vector2<f32>| {
        let d = p - a;
        (d.x * direction.y - d.y * direction.x).abs()
    };
    let corners = cell.vertices();
    (0..corners.len())
        .filter_map(|i| {
            let (p, q) = (corners[i], corners[(i + 1) % corners.len()]);
            if off_line(p) > ON_EDGE || off_line(q) > ON_EDGE {
                return None;
            }
            let (s, t) = ((p - a).dot(&direction), (q - a).dot(&direction));
            let (low, high) = (s.min(t).max(0.0), s.max(t).min(length));
            (high - low > MIN_HELD).then_some((low, high))
        })
        .max_by(|x, y| (x.1 - x.0).total_cmp(&(y.1 - y.0)))
        .map(|(low, high)| (a + direction * low, a + direction * high))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A cell along the edge is held by the stretch it covers, a cell off it
    /// is not, and one touching at a corner is not.
    #[test]
    fn a_cell_is_held_along_the_stretch_of_edge_it_lies_on() {
        let (a, b) = (Vector2::new(-1.0, 1.0), Vector2::new(1.0, 1.0));
        let along = ConvexPolygon::rectangle(Vector2::new(0.5, 0.75), 0.25, 0.25);
        let (start, end) = held_stretch(&along, a, b).expect("the cell lies on the edge");
        assert!((start - Vector2::new(0.25, 1.0)).norm() < 1e-5);
        assert!((end - Vector2::new(0.75, 1.0)).norm() < 1e-5);

        let inside = ConvexPolygon::rectangle(Vector2::new(0.0, 0.0), 0.25, 0.25);
        assert!(held_stretch(&inside, a, b).is_none());

        let corner = ConvexPolygon::new(vec![
            Vector2::new(0.0, 1.0),
            Vector2::new(-0.2, 0.8),
            Vector2::new(0.2, 0.8),
        ])
        .unwrap();
        assert!(held_stretch(&corner, a, b).is_none());
    }
}
