//! A channel's centreline: the D8 path smoothed into a curve.
//!
//! A path that follows drainage directions steps between column centres in
//! eight directions, so a river running at 30° zig-zags, and one running at
//! 45° measures √2 too wide if cut across its steps. Two passes of Chaikin's
//! corner cutting turn the steps into a curve whose tangent a cross-section
//! can be cut perpendicular to.

use nalgebra::{Point3, Vector2};

/// Corner-cutting passes applied to a D8 path.
pub const SMOOTHING_PASSES: usize = 2;

/// A smoothed channel path, with each point's tangent and distance along it.
#[derive(Debug, Clone, Default)]
pub struct Centreline {
    /// Points along the channel bed, upstream first.
    pub points: Vec<Point3<f32>>,
    /// Unit horizontal direction of travel at each point.
    pub tangents: Vec<Vector2<f32>>,
    /// Horizontal distance from the first point, in metres.
    pub distance: Vec<f32>,
}

impl Centreline {
    /// Smooth a stepped path and measure it.
    pub fn from_path(path: &[Point3<f32>]) -> Self {
        let mut points = path.to_vec();
        for _ in 0..SMOOTHING_PASSES {
            points = chaikin(&points);
        }
        let tangents = tangents(&points);
        let mut distance = Vec::with_capacity(points.len());
        let mut total = 0.0;
        for (i, p) in points.iter().enumerate() {
            if i > 0 {
                total += horizontal(p - points[i - 1]).norm();
            }
            distance.push(total);
        }
        Self {
            points,
            tangents,
            distance,
        }
    }

    /// Horizontal length, in metres.
    pub fn length(&self) -> f32 {
        self.distance.last().copied().unwrap_or(0.0)
    }
}

/// One pass of Chaikin's corner cutting, keeping both endpoints.
pub fn chaikin(points: &[Point3<f32>]) -> Vec<Point3<f32>> {
    if points.len() < 3 {
        return points.to_vec();
    }
    let mut out = Vec::with_capacity(points.len() * 2);
    out.push(points[0]);
    for pair in points.windows(2) {
        let (a, b) = (pair[0].coords, pair[1].coords);
        out.push(Point3::from(a * 0.75 + b * 0.25));
        out.push(Point3::from(a * 0.25 + b * 0.75));
    }
    out.push(*points.last().expect("at least three points"));
    out
}

/// Unit horizontal tangent at each point, by central differences.
fn tangents(points: &[Point3<f32>]) -> Vec<Vector2<f32>> {
    (0..points.len())
        .map(|i| {
            let before = points[i.saturating_sub(1)];
            let after = points[(i + 1).min(points.len() - 1)];
            let d = horizontal(after - before);
            if d.norm() > 1e-6 {
                d.normalize()
            } else {
                Vector2::x()
            }
        })
        .collect()
}

fn horizontal(v: nalgebra::Vector3<f32>) -> Vector2<f32> {
    Vector2::new(v.x, v.z)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_staircase_path_smooths_into_a_diagonal() {
        // A D8 path zig-zagging along a 30° line.
        let path: Vec<Point3<f32>> = (0..20)
            .map(|i| Point3::new(i as f32 * 0.5, 0.0, (i / 2) as f32 * 0.5))
            .collect();
        let line = Centreline::from_path(&path);
        let n = line.tangents.len();
        let mean = line.tangents[n / 4..3 * n / 4]
            .iter()
            .fold(nalgebra::Vector2::zeros(), |acc, t| acc + t);
        let angle = mean.y.atan2(mean.x).to_degrees();
        assert!((angle - 26.6).abs() < 3.0, "mean tangent at {angle}°");
        // Smoothing pulls the steps towards the line they approximate.
        let worst = |points: &[Point3<f32>]| {
            points
                .iter()
                .map(|p| (p.z - (p.x * 0.5 - 0.125)).abs())
                .fold(0.0f32, f32::max)
        };
        assert!(worst(&line.points[4..line.points.len() - 4]) < worst(&path));
        assert_eq!(line.points.first(), path.first());
        assert_eq!(line.points.last(), path.last());
    }
}
