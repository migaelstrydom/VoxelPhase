//! Convex polygons in the plane of a sheet.
//!
//! Everything the glass system reasons about is flat: a pane is a polygon,
//! a crack pattern is a partition of a polygon into smaller polygons, and a
//! shard is a polygon with thickness. Keeping the geometry two-dimensional
//! until the last moment is what makes the partition cheap and — since every
//! operation here preserves convexity — what guarantees the physics engine is
//! only ever handed convex shapes.

use nalgebra::Vector2;

/// Vertices closer than this are one vertex; a polygon thinner than this
/// has collapsed.
const EPS: f32 = 1e-5;

/// A convex polygon with its vertices in counter-clockwise order.
#[derive(Debug, Clone, PartialEq)]
pub struct ConvexPolygon {
    vertices: Vec<Vector2<f32>>,
}

impl ConvexPolygon {
    /// A polygon from vertices given in either winding, with coincident
    /// neighbours merged. `None` if fewer than three distinct vertices are
    /// left or the polygon has no area.
    pub fn new(vertices: Vec<Vector2<f32>>) -> Option<Self> {
        let mut distinct: Vec<Vector2<f32>> = Vec::with_capacity(vertices.len());
        for vertex in vertices {
            if distinct
                .last()
                .is_none_or(|last| (last - vertex).magnitude() > EPS)
            {
                distinct.push(vertex);
            }
        }
        while distinct.len() > 1 && (distinct[0] - distinct[distinct.len() - 1]).magnitude() <= EPS
        {
            distinct.pop();
        }
        if distinct.len() < 3 {
            return None;
        }
        let mut polygon = Self { vertices: distinct };
        let area = polygon.signed_area();
        if area.abs() <= EPS * EPS {
            return None;
        }
        if area < 0.0 {
            polygon.vertices.reverse();
        }
        Some(polygon)
    }

    /// An axis-aligned rectangle centred on `centre`.
    pub fn rectangle(centre: Vector2<f32>, half_u: f32, half_v: f32) -> Self {
        Self {
            vertices: vec![
                centre + Vector2::new(-half_u, -half_v),
                centre + Vector2::new(half_u, -half_v),
                centre + Vector2::new(half_u, half_v),
                centre + Vector2::new(-half_u, half_v),
            ],
        }
    }

    /// The polygon with every edge shorter than `min_edge` collapsed.
    ///
    /// Dropping a vertex of a convex polygon leaves a convex polygon inside
    /// the old one, so this only ever gives away a sliver along the edge.
    /// `None` if fewer than three vertices survive.
    pub fn simplified(&self, min_edge: f32) -> Option<Self> {
        let mut kept: Vec<Vector2<f32>> = Vec::with_capacity(self.vertices.len());
        for vertex in &self.vertices {
            if kept
                .last()
                .is_none_or(|last| (last - vertex).magnitude() >= min_edge)
            {
                kept.push(*vertex);
            }
        }
        while kept.len() > 2 && (kept[0] - kept[kept.len() - 1]).magnitude() < min_edge {
            kept.pop();
        }
        Self::new(kept)
    }

    pub fn vertices(&self) -> &[Vector2<f32>] {
        &self.vertices
    }

    fn signed_area(&self) -> f32 {
        let n = self.vertices.len();
        (0..n)
            .map(|i| {
                let a = self.vertices[i];
                let b = self.vertices[(i + 1) % n];
                a.x * b.y - b.x * a.y
            })
            .sum::<f32>()
            * 0.5
    }

    pub fn area(&self) -> f32 {
        self.signed_area().abs()
    }

    pub fn centroid(&self) -> Vector2<f32> {
        let n = self.vertices.len();
        let mut weighted = Vector2::zeros();
        let mut total = 0.0;
        for i in 0..n {
            let a = self.vertices[i];
            let b = self.vertices[(i + 1) % n];
            let cross = a.x * b.y - b.x * a.y;
            weighted += (a + b) * cross;
            total += cross;
        }
        if total.abs() <= EPS * EPS {
            return self.vertices.iter().sum::<Vector2<f32>>() / n as f32;
        }
        weighted / (3.0 * total)
    }

    /// Whether `point` is inside or on the boundary.
    pub fn contains(&self, point: Vector2<f32>) -> bool {
        let n = self.vertices.len();
        (0..n).all(|i| {
            let a = self.vertices[i];
            let b = self.vertices[(i + 1) % n];
            let edge = b - a;
            edge.x * (point.y - a.y) - edge.y * (point.x - a.x) >= -EPS
        })
    }

    /// The part of this polygon on the near side of a line: every point `p`
    /// with `normal · p <= offset`. `None` if nothing is left.
    pub fn clip(&self, normal: Vector2<f32>, offset: f32) -> Option<Self> {
        let n = self.vertices.len();
        let mut kept = Vec::with_capacity(n + 1);
        for i in 0..n {
            let a = self.vertices[i];
            let b = self.vertices[(i + 1) % n];
            let da = normal.dot(&a) - offset;
            let db = normal.dot(&b) - offset;
            if da <= 0.0 {
                kept.push(a);
            }
            if (da < 0.0 && db > 0.0) || (da > 0.0 && db < 0.0) {
                let t = da / (da - db);
                kept.push(a + (b - a) * t);
            }
        }
        Self::new(kept)
    }

    /// The polygon with every edge moved inward by `distance`. `None` if
    /// nothing is left.
    pub fn inset(&self, distance: f32) -> Option<Self> {
        let n = self.vertices.len();
        let mut result = self.clone();
        for i in 0..n {
            let a = self.vertices[i];
            let b = self.vertices[(i + 1) % n];
            let edge = b - a;
            let length = edge.magnitude();
            if length <= EPS {
                continue;
            }
            let outward = Vector2::new(edge.y, -edge.x) / length;
            result = result.clip(outward, outward.dot(&a) - distance)?;
        }
        Some(result)
    }

    /// The point of the polygon nearest to `point`: the point itself when it
    /// is inside, otherwise the nearest point on the boundary.
    pub fn closest_point(&self, point: Vector2<f32>) -> Vector2<f32> {
        if self.contains(point) {
            return point;
        }
        let n = self.vertices.len();
        let mut best = self.vertices[0];
        let mut best_distance = f32::MAX;
        for i in 0..n {
            let a = self.vertices[i];
            let b = self.vertices[(i + 1) % n];
            let edge = b - a;
            let length_sq = edge.magnitude_squared();
            let t = if length_sq > 0.0 {
                ((point - a).dot(&edge) / length_sq).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let candidate = a + edge * t;
            let distance = (point - candidate).magnitude_squared();
            if distance < best_distance {
                best_distance = distance;
                best = candidate;
            }
        }
        best
    }

    /// Distance from an inside point to the nearest edge; zero outside.
    pub fn boundary_distance(&self, point: Vector2<f32>) -> f32 {
        let n = self.vertices.len();
        (0..n)
            .map(|i| {
                let a = self.vertices[i];
                let b = self.vertices[(i + 1) % n];
                let edge = b - a;
                let length = edge.magnitude();
                if length <= EPS {
                    return f32::MAX;
                }
                (edge.x * (point.y - a.y) - edge.y * (point.x - a.x)) / length
            })
            .fold(f32::MAX, f32::min)
            .max(0.0)
    }

    pub fn distance_to(&self, point: Vector2<f32>) -> f32 {
        (self.closest_point(point) - point).magnitude()
    }

    /// The polygon's bounding box as `(min, max)`.
    pub fn bounds(&self) -> (Vector2<f32>, Vector2<f32>) {
        let mut min = Vector2::repeat(f32::MAX);
        let mut max = Vector2::repeat(f32::MIN);
        for v in &self.vertices {
            min = min.inf(v);
            max = max.sup(v);
        }
        (min, max)
    }

    /// The larger side of the bounding box.
    pub fn extent(&self) -> f32 {
        let (min, max) = self.bounds();
        (max.x - min.x).max(max.y - min.y)
    }

    /// How thin the polygon is: the smallest distance between two parallel
    /// lines that enclose it. For a convex polygon one of those lines runs
    /// along an edge, so this is the smallest over edges of the polygon's
    /// height above that edge.
    pub fn width(&self) -> f32 {
        let n = self.vertices.len();
        (0..n)
            .map(|i| {
                let a = self.vertices[i];
                let b = self.vertices[(i + 1) % n];
                let edge = b - a;
                let length = edge.magnitude();
                if length <= EPS {
                    return f32::MAX;
                }
                let inward = Vector2::new(-edge.y, edge.x) / length;
                self.vertices
                    .iter()
                    .map(|v| inward.dot(&(v - a)))
                    .fold(0.0f32, f32::max)
            })
            .fold(f32::MAX, f32::min)
    }

    /// Whether the two polygons share a stretch of boundary at least
    /// `min_overlap` long — the test for whether two shards are joined.
    ///
    /// Two cells of one partition meet along a segment of a common line;
    /// two cells that merely touch at a corner do not hold each other up.
    pub fn touches(&self, other: &Self, min_overlap: f32) -> bool {
        let tolerance = 1e-3;
        let n = self.vertices.len();
        let m = other.vertices.len();
        for i in 0..n {
            let a = self.vertices[i];
            let b = self.vertices[(i + 1) % n];
            let edge = b - a;
            let length = edge.magnitude();
            if length <= EPS {
                continue;
            }
            let direction = edge / length;
            let normal = Vector2::new(-direction.y, direction.x);
            for j in 0..m {
                let c = other.vertices[j];
                let d = other.vertices[(j + 1) % m];
                if normal.dot(&(c - a)).abs() > tolerance || normal.dot(&(d - a)).abs() > tolerance
                {
                    continue;
                }
                let (lo, hi) = {
                    let s = direction.dot(&(c - a));
                    let t = direction.dot(&(d - a));
                    (s.min(t), s.max(t))
                };
                let overlap = hi.min(length) - lo.max(0.0);
                if overlap >= min_overlap {
                    return true;
                }
            }
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square() -> ConvexPolygon {
        ConvexPolygon::rectangle(Vector2::zeros(), 1.0, 1.0)
    }

    #[test]
    fn a_rectangle_knows_its_area_and_centre() {
        let rect = ConvexPolygon::rectangle(Vector2::new(2.0, 3.0), 1.5, 0.5);
        assert!((rect.area() - 3.0).abs() < 1e-6);
        assert!((rect.centroid() - Vector2::new(2.0, 3.0)).magnitude() < 1e-6);
        assert!((rect.width() - 1.0).abs() < 1e-6);
        assert!((rect.extent() - 3.0).abs() < 1e-6);
    }

    #[test]
    fn simplifying_drops_short_edges_and_keeps_the_shape() {
        let nearly_square = ConvexPolygon::new(vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(1.0, 0.001),
            Vector2::new(1.0, 1.0),
            Vector2::new(0.0, 1.0),
            Vector2::new(0.0, 0.999),
        ])
        .expect("a polygon");
        let simple = nearly_square.simplified(0.01).expect("a square remains");
        assert_eq!(simple.vertices().len(), 4);
        assert!((simple.area() - 1.0).abs() < 0.01);
    }

    #[test]
    fn an_inset_square_is_a_smaller_square_and_a_deep_inset_is_nothing() {
        let inner = square().inset(0.1).expect("a smaller square");
        assert!((inner.area() - 3.24).abs() < 1e-5);
        assert!(!inner.contains(Vector2::new(0.95, 0.0)));
        assert!(square().inset(1.5).is_none());
    }

    #[test]
    fn clockwise_input_comes_out_counter_clockwise() {
        let cw = ConvexPolygon::new(vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(0.0, 1.0),
            Vector2::new(1.0, 1.0),
            Vector2::new(1.0, 0.0),
        ])
        .expect("a square is a polygon");
        assert!(cw.contains(Vector2::new(0.5, 0.5)));
        assert!(!cw.contains(Vector2::new(1.5, 0.5)));
    }

    #[test]
    fn clipping_keeps_the_near_side() {
        let half = square()
            .clip(Vector2::x(), 0.0)
            .expect("half a square remains");
        assert!((half.area() - 2.0).abs() < 1e-5);
        assert!(half.contains(Vector2::new(-0.5, 0.0)));
        assert!(!half.contains(Vector2::new(0.5, 0.0)));
        assert!(square().clip(Vector2::x(), -2.0).is_none());
    }

    #[test]
    fn the_closest_point_is_on_the_boundary_for_an_outside_point() {
        let p = square().closest_point(Vector2::new(3.0, 0.2));
        assert!((p - Vector2::new(1.0, 0.2)).magnitude() < 1e-6);
        assert_eq!(
            square().closest_point(Vector2::new(0.1, 0.1)),
            Vector2::new(0.1, 0.1)
        );
    }

    #[test]
    fn neighbours_share_an_edge_and_diagonal_cells_do_not() {
        let left = ConvexPolygon::rectangle(Vector2::new(-0.5, 0.0), 0.5, 0.5);
        let right = ConvexPolygon::rectangle(Vector2::new(0.5, 0.0), 0.5, 0.5);
        // Meets `right` at the single corner (1, 0.5).
        let diagonal = ConvexPolygon::rectangle(Vector2::new(1.5, 1.0), 0.5, 0.5);
        assert!(left.touches(&right, 0.1));
        assert!(!right.touches(&diagonal, 0.1));
        assert!(!left.touches(&diagonal, 0.1));
    }
}
