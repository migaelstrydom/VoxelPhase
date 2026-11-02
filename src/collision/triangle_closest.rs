//! The point of a triangle closest to a query point, as barycentric weights
//! and the feature it lies on: Ericson's Voronoi-region walk (Real-Time
//! Collision Detection, 5.1.5).
//!
//! GJK reduces its triangle simplex to the feature and interpolates its
//! witness points with the weights; a sphere against a triangle wants only
//! the point.

use nalgebra::Point3;

/// Below this, the triangle's barycentric denominator counts as zero: the
/// triangle has no area to be inside of.
const DEGENERATE_DENOMINATOR: f32 = 1e-10;

/// The triangle feature holding the closest point, by corner index (0, 1, 2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriangleFeature {
    Vertex(usize),
    /// The edge between two corners, the lower index first.
    Edge(usize, usize),
    Face,
}

/// The closest point of a triangle to a query point.
#[derive(Debug, Clone, Copy)]
pub struct ClosestOnTriangle {
    /// Barycentric weights of the three corners; they sum to one, and are
    /// zero for corners off `feature`.
    pub weights: [f32; 3],
    /// The feature the closest point lies on.
    pub feature: TriangleFeature,
}

impl ClosestOnTriangle {
    /// The closest point of `corners`, the triangle it was found for.
    pub fn point(&self, corners: [Point3<f32>; 3]) -> Point3<f32> {
        let [wa, wb, wc] = self.weights;
        Point3::from(corners[0].coords * wa + corners[1].coords * wb + corners[2].coords * wc)
    }

    fn vertex(index: usize) -> Self {
        let mut weights = [0.0; 3];
        weights[index] = 1.0;
        Self {
            weights,
            feature: TriangleFeature::Vertex(index),
        }
    }

    /// `t` of the way from corner `i` to corner `j`.
    fn edge(i: usize, j: usize, t: f32) -> Self {
        let mut weights = [0.0; 3];
        weights[i] = 1.0 - t;
        weights[j] = t;
        Self {
            weights,
            feature: TriangleFeature::Edge(i, j),
        }
    }
}

/// The point of triangle `corners` closest to `query`.
///
/// A degenerate triangle whose interior region the query falls in answers
/// its first corner.
pub fn closest_on_triangle(query: Point3<f32>, corners: [Point3<f32>; 3]) -> ClosestOnTriangle {
    let [a, b, c] = corners;
    let ab = b - a;
    let ac = c - a;

    let ap = query - a;
    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return ClosestOnTriangle::vertex(0);
    }

    let bp = query - b;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return ClosestOnTriangle::vertex(1);
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return ClosestOnTriangle::edge(0, 1, d1 / (d1 - d3));
    }

    let cp = query - c;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return ClosestOnTriangle::vertex(2);
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return ClosestOnTriangle::edge(0, 2, d2 / (d2 - d6));
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return ClosestOnTriangle::edge(1, 2, (d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }

    let denom = va + vb + vc;
    if denom.abs() < DEGENERATE_DENOMINATOR {
        return ClosestOnTriangle::vertex(0);
    }
    let (wb, wc) = (vb / denom, vc / denom);
    ClosestOnTriangle {
        weights: [1.0 - wb - wc, wb, wc],
        feature: TriangleFeature::Face,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CORNERS: [Point3<f32>; 3] = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, 2.0),
    ];

    #[test]
    fn each_region_finds_its_feature_and_point() {
        let cases = [
            (
                Point3::new(-1.0, 1.0, -1.0),
                TriangleFeature::Vertex(0),
                Point3::new(0.0, 0.0, 0.0),
            ),
            (
                Point3::new(3.0, 1.0, -1.0),
                TriangleFeature::Vertex(1),
                Point3::new(2.0, 0.0, 0.0),
            ),
            (
                Point3::new(-1.0, 1.0, 3.0),
                TriangleFeature::Vertex(2),
                Point3::new(0.0, 0.0, 2.0),
            ),
            (
                Point3::new(1.0, 1.0, -1.0),
                TriangleFeature::Edge(0, 1),
                Point3::new(1.0, 0.0, 0.0),
            ),
            (
                Point3::new(-1.0, 1.0, 1.0),
                TriangleFeature::Edge(0, 2),
                Point3::new(0.0, 0.0, 1.0),
            ),
            (
                Point3::new(2.0, 1.0, 2.0),
                TriangleFeature::Edge(1, 2),
                Point3::new(1.0, 0.0, 1.0),
            ),
            (
                Point3::new(0.5, 1.0, 0.5),
                TriangleFeature::Face,
                Point3::new(0.5, 0.0, 0.5),
            ),
        ];
        for (query, feature, point) in cases {
            let closest = closest_on_triangle(query, CORNERS);
            assert_eq!(closest.feature, feature, "query {query}");
            assert!(
                (closest.point(CORNERS) - point).norm() < 1e-6,
                "query {query}"
            );
            assert!((closest.weights.iter().sum::<f32>() - 1.0).abs() < 1e-6);
        }
    }

    #[test]
    fn a_degenerate_triangle_answers_a_corner() {
        let line = [
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ];
        let closest = closest_on_triangle(Point3::new(1.0, 1.0, 0.0), line);
        assert!(closest.point(line).coords.norm() <= 2.0 + 1e-6);
        assert!(closest.weights.iter().all(|w| w.is_finite()));
    }
}
