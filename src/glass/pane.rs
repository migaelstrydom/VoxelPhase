//! The sheet's own coordinate frame, and the lift from a flat cell to a
//! solid shard.
//!
//! ```text
//!          normal
//!            ▲       ┌──────────┐  ┐
//!            │      ╱   cell   ╱   │ thickness
//!            │     └──────────┘    ┘
//!            └─────▶ u          (v into the page)
//! ```
//!
//! A sheet has one frame for its whole life: `u` and `v` span it, `normal`
//! is the way through it, all three in the body's own axes. Every shard that
//! ever comes off it is a polygon in the `u v` plane extruded by the sheet's
//! thickness along `normal`, so the frame is the one thing that has to be
//! remembered about the sheet — the polygons themselves are read back off
//! the colliders whenever they are needed.

use nalgebra::{Vector2, Vector3};

use super::polygon::ConvexPolygon;
use crate::collision::convex_hull::{ConvexHull, HullFace};
use crate::physics::ColliderShape;

/// The axes of a sheet in its body's frame, and how thick it is.
#[derive(Debug, Clone, Copy)]
pub struct SheetFrame {
    /// First in-plane axis. `u × v` must be `normal`, which is what keeps a
    /// counter-clockwise polygon's top face facing along `normal`.
    pub u: Vector3<f32>,
    /// Second in-plane axis.
    pub v: Vector3<f32>,
    /// Through the sheet.
    pub normal: Vector3<f32>,
    /// Full thickness along `normal`, in metres.
    pub thickness: f32,
}

impl SheetFrame {
    /// A sheet lying flat: a floor, a bridge deck.
    pub fn lying(thickness: f32) -> Self {
        Self {
            u: Vector3::z(),
            v: Vector3::x(),
            normal: Vector3::y(),
            thickness,
        }
    }

    /// A sheet standing up, facing along the body's `+Z`: a window.
    pub fn upright(thickness: f32) -> Self {
        Self {
            u: Vector3::x(),
            v: Vector3::y(),
            normal: Vector3::z(),
            thickness,
        }
    }

    /// A body-frame point as `(u, v)` in the plane.
    pub fn to_plane(&self, point: Vector3<f32>) -> Vector2<f32> {
        Vector2::new(self.u.dot(&point), self.v.dot(&point))
    }

    /// A plane point back into the body's frame, `height` along the normal.
    pub fn lift(&self, point: Vector2<f32>, height: f32) -> Vector3<f32> {
        self.u * point.x + self.v * point.y + self.normal * height
    }

    /// The half-extents of a box collider along this frame's in-plane axes,
    /// for a box the frame was built for: a rectangle covering the sheet.
    pub fn box_half_extents(&self, half_u: f32, half_v: f32) -> Vector3<f32> {
        (self.u * half_u + self.v * half_v + self.normal * (self.thickness * 0.5)).abs()
    }

    /// The footprint of one child in the plane, and how far along the normal
    /// its mid-plane sits. `None` for a shape that is not a piece of sheet.
    pub fn polygon_of(
        &self,
        shape: &ColliderShape,
        offset: Vector3<f32>,
    ) -> Option<(ConvexPolygon, f32)> {
        let height = self.normal.dot(&offset);
        match shape {
            ColliderShape::Box { half_extents } => {
                let half_u = half_extents.dot(&self.u.abs());
                let half_v = half_extents.dot(&self.v.abs());
                Some((
                    ConvexPolygon::rectangle(self.to_plane(offset), half_u, half_v),
                    height,
                ))
            }
            ColliderShape::ConvexHull { hull } => {
                // The top face's corners, whichever order the hull keeps them
                // in: a convex polygon's vertices sort by angle about its
                // centre.
                let mut top: Vec<Vector2<f32>> = hull
                    .vertices
                    .iter()
                    .filter(|vertex| self.normal.dot(vertex) > 0.0)
                    .map(|vertex| self.to_plane(offset + vertex))
                    .collect();
                let centre = top.iter().sum::<Vector2<f32>>() / top.len().max(1) as f32;
                top.sort_by(|a, b| {
                    let angle = |p: &Vector2<f32>| (p.y - centre.y).atan2(p.x - centre.x);
                    angle(a).total_cmp(&angle(b))
                });
                ConvexPolygon::new(top).map(|polygon| (polygon, height))
            }
            _ => None,
        }
    }

    /// The solid shard of a cell: a convex prism centred on the cell's
    /// centroid, with the centroid's plane position returned beside it so the
    /// caller can place the collider.
    pub fn prism(&self, cell: &ConvexPolygon) -> (ConvexHull, Vector2<f32>) {
        let centroid = cell.centroid();
        let corners = cell.vertices();
        let n = corners.len();
        let half = self.thickness * 0.5;

        let mut vertices = Vec::with_capacity(2 * n);
        for corner in corners {
            vertices.push(self.lift(corner - centroid, half));
        }
        for corner in corners {
            vertices.push(self.lift(corner - centroid, -half));
        }

        let mut faces = Vec::with_capacity(n + 2);
        faces.push(HullFace {
            vertex_indices: (0..n as u16).collect(),
            normal: self.normal,
        });
        faces.push(HullFace {
            vertex_indices: (0..n as u16).rev().map(|i| i + n as u16).collect(),
            normal: -self.normal,
        });
        for i in 0..n {
            let j = (i + 1) % n;
            let edge = self.lift(corners[j] - corners[i], 0.0);
            let (top_i, top_j) = (i as u16, j as u16);
            let (bottom_i, bottom_j) = (top_i + n as u16, top_j + n as u16);
            faces.push(HullFace {
                vertex_indices: [bottom_i, bottom_j, top_j, top_i].into_iter().collect(),
                normal: edge.cross(&self.normal).normalize(),
            });
        }

        (ConvexHull::new(vertices, faces), centroid)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector2;

    #[test]
    fn a_prism_has_the_cell_area_times_the_thickness_for_a_volume() {
        let frame = SheetFrame::upright(0.02);
        let cell = ConvexPolygon::new(vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(0.3, 0.0),
            Vector2::new(0.35, 0.2),
            Vector2::new(0.1, 0.25),
        ])
        .expect("a quad");
        let (hull, centroid) = frame.prism(&cell);
        assert!((hull.compute_volume() - cell.area() * 0.02).abs() < 1e-6);
        assert!((centroid - cell.centroid()).magnitude() < 1e-6);
        for face in &hull.faces {
            // Every face's normal points away from the centre.
            let corner = hull.vertices[face.vertex_indices[0] as usize];
            assert!(
                face.normal.dot(&corner) > 0.0,
                "face {:?} faces inward",
                face.normal
            );
        }
    }

    #[test]
    fn a_shard_reads_back_as_the_cell_it_was_cut_from() {
        let frame = SheetFrame::lying(0.015);
        let cell = ConvexPolygon::new(vec![
            Vector2::new(-0.2, 0.1),
            Vector2::new(0.4, 0.0),
            Vector2::new(0.3, 0.5),
        ])
        .expect("a triangle");
        let (hull, centroid) = frame.prism(&cell);
        let offset = frame.lift(centroid, 1.5);
        let (read_back, height) = frame
            .polygon_of(&ColliderShape::ConvexHull { hull: hull.into() }, offset)
            .expect("a prism is a piece of sheet");
        assert!((height - 1.5).abs() < 1e-6);
        assert!((read_back.area() - cell.area()).abs() < 1e-5);
        assert!((read_back.centroid() - cell.centroid()).magnitude() < 1e-5);
    }

    /// A crate resting on a shard must be pushed *up* by it. The hull-vs-box
    /// manifold has to agree about which way is out of a plate a centimetre
    /// and a half thick.
    #[test]
    fn a_box_on_a_shard_is_pushed_away_from_it() {
        use crate::collision::dispatch::generate_manifold;
        use crate::collision::shape_view::ShapeView;
        use nalgebra::{Point3, UnitQuaternion};

        let frame = SheetFrame::lying(0.015);
        let cell = ConvexPolygon::new(vec![
            Vector2::new(-0.2, -0.15),
            Vector2::new(0.25, -0.1),
            Vector2::new(0.2, 0.2),
            Vector2::new(-0.1, 0.22),
        ])
        .expect("a quad");
        let (hull, _) = frame.prism(&cell);
        let shard = ColliderShape::ConvexHull { hull: hull.into() };
        let crate_shape = ColliderShape::Box {
            half_extents: Vector3::repeat(0.15),
        };
        let shard_view = ShapeView {
            center: Point3::new(0.0, 1.0, 0.0),
            rotation: UnitQuaternion::identity(),
            shape: &shard,
        };
        let crate_view = ShapeView {
            center: Point3::new(0.02, 1.0 + 0.0075 + 0.15 - 0.004, 0.03),
            rotation: UnitQuaternion::identity(),
            shape: &crate_shape,
        };
        let manifold = generate_manifold(&shard_view, &crate_view, 0.02, None, None);
        assert!(!manifold.is_empty(), "resting contact expected");
        for contact in &manifold.points {
            assert!(
                contact.normal.y > 0.9,
                "normal {:?} should point from the shard up into the crate",
                contact.normal
            );
        }
    }

    #[test]
    fn a_box_reads_back_as_a_rectangle() {
        let frame = SheetFrame::upright(0.01);
        let (rect, height) = frame
            .polygon_of(
                &ColliderShape::Box {
                    half_extents: frame.box_half_extents(1.0, 0.5),
                },
                Vector3::new(0.0, 2.0, 0.3),
            )
            .expect("a box is a piece of sheet");
        assert!((rect.area() - 2.0).abs() < 1e-5);
        assert!((rect.centroid() - Vector2::new(0.0, 2.0)).magnitude() < 1e-6);
        assert!((height - 0.3).abs() < 1e-6);
    }
}
