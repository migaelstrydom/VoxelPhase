//! Tube mesh generation: a circle swept along a path.
//!
//! One generator for every bent rod: a hoop is a closed circle of points, a
//! coil spring a helix, a leg a few points with a bend. The cross-section is
//! carried along the path by parallel transport, so it does not twist where
//! the path turns.

use std::f32::consts::TAU;

use nalgebra::{Point3, Vector2, Vector3};

use crate::rendering::colour::Colour;
use crate::rendering::vertex::Vertex;

/// Sweep a circle of `radius` along `path`.
///
/// `closed` joins the last point back to the first, for a hoop; an open tube
/// is left open at its ends. `sides` is the number of facets around the
/// tube. Texture coordinates run around the tube in `u` and along its length,
/// in metres, in `v`.
///
/// Returns (vertices, indices), empty for a path of fewer than two points.
pub fn generate_tube(
    path: &[Point3<f32>],
    radius: f32,
    sides: u32,
    closed: bool,
    colour: Colour,
) -> (Vec<Vertex>, Vec<u32>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    append_tube(
        &mut vertices,
        &mut indices,
        path,
        radius,
        sides,
        closed,
        colour,
    );
    (vertices, indices)
}

/// [`generate_tube`], appended to existing buffers so many tubes can share
/// one mesh.
pub fn append_tube(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<u32>,
    path: &[Point3<f32>],
    radius: f32,
    sides: u32,
    closed: bool,
    colour: Colour,
) {
    if path.len() < 2 || sides < 3 {
        return;
    }
    let colour = colour.to_vec4();
    let base = vertices.len() as u32;
    let count = path.len();
    let tangent_at = |i: usize| -> Vector3<f32> {
        let (prev, next) = if closed {
            (path[(i + count - 1) % count], path[(i + 1) % count])
        } else {
            (path[i.saturating_sub(1)], path[(i + 1).min(count - 1)])
        };
        (next - prev)
            .try_normalize(1.0e-9)
            .unwrap_or_else(Vector3::y)
    };

    let first_tangent = tangent_at(0);
    let seed = if first_tangent.y.abs() < 0.9 {
        Vector3::y()
    } else {
        Vector3::x()
    };
    let mut normal = first_tangent.cross(&seed).normalize();
    let mut previous_tangent = first_tangent;
    let mut along = 0.0;

    for (i, point) in path.iter().enumerate() {
        let tangent = tangent_at(i);
        normal = transported(normal, previous_tangent, tangent);
        previous_tangent = tangent;
        let binormal = tangent.cross(&normal);
        if i > 0 {
            along += (point - path[i - 1]).norm();
        }
        for side in 0..=sides {
            let angle = side as f32 / sides as f32 * TAU;
            let outward = normal * angle.cos() + binormal * angle.sin();
            vertices.push(Vertex {
                pos: (point + outward * radius).coords,
                color: colour,
                tex_coords: Vector2::new(side as f32 / sides as f32, along),
                normal: outward,
                ao: 1.0,
            });
        }
    }

    let ring = sides + 1;
    let segments = if closed { count } else { count - 1 };
    for segment in 0..segments {
        let a = base + (segment as u32) * ring;
        let b = base + ((segment + 1) % count) as u32 * ring;
        for side in 0..sides {
            indices.extend_from_slice(&[a + side, a + side + 1, b + side]);
            indices.extend_from_slice(&[b + side, a + side + 1, b + side + 1]);
        }
    }
}

/// The centreline of a coil spring from `start` to `end`: a straight hook of
/// `hook` metres at each end and a helix of `turns` turns of `coil_radius`
/// between them, `points_per_turn` points to a turn. However far apart the
/// ends are, the coil keeps its turns and stretches or squeezes between them.
pub fn coil_path(
    start: Point3<f32>,
    end: Point3<f32>,
    coil_radius: f32,
    hook: f32,
    turns: f32,
    points_per_turn: u32,
) -> Vec<Point3<f32>> {
    let span = end - start;
    let length = span.norm();
    if length < 1.0e-6 {
        return vec![start, end];
    }
    let axis = span / length;
    let seed = if axis.y.abs() < 0.9 {
        Vector3::y()
    } else {
        Vector3::x()
    };
    let across = axis.cross(&seed).normalize();
    let other = axis.cross(&across);
    let hook = hook.min(length * 0.25);
    let coil_start = start + axis * hook;
    let coil_length = length - 2.0 * hook;
    let steps = ((turns * points_per_turn as f32).ceil() as u32).max(1);

    let mut path = Vec::with_capacity(steps as usize + 3);
    path.push(start);
    for step in 0..=steps {
        let t = step as f32 / steps as f32;
        let angle = t * turns * TAU;
        // The coil swells from the axis over its first and last quarter turn,
        // so the hooks run into it rather than meeting it side-on.
        let swell = (t * turns * 4.0).min((1.0 - t) * turns * 4.0).min(1.0);
        let offset = (across * angle.cos() + other * angle.sin()) * coil_radius * swell;
        path.push(coil_start + axis * (coil_length * t) + offset);
    }
    path.push(end);
    path
}

/// Carry `normal` from a path's `from` tangent to its `to` tangent by the
/// smallest rotation between them.
fn transported(normal: Vector3<f32>, from: Vector3<f32>, to: Vector3<f32>) -> Vector3<f32> {
    let axis = from.cross(&to);
    let sin = axis.norm();
    let cos = from.dot(&to);
    if sin < 1.0e-6 {
        return normal;
    }
    let axis = axis / sin;
    let turned = normal * cos + axis.cross(&normal) * sin + axis * axis.dot(&normal) * (1.0 - cos);
    (turned - to * to.dot(&turned)).normalize()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hoop(points: usize) -> Vec<Point3<f32>> {
        (0..points)
            .map(|i| {
                let a = i as f32 / points as f32 * TAU;
                Point3::new(a.cos() * 2.0, 0.0, a.sin() * 2.0)
            })
            .collect()
    }

    #[test]
    fn every_vertex_is_a_radius_from_the_path() {
        let path = hoop(24);
        let (vertices, _) = generate_tube(&path, 0.1, 8, true, Colour::WHITE);
        for (i, vertex) in vertices.iter().enumerate() {
            let centre = path[i / 9];
            let offset = Point3::from(vertex.pos) - centre;
            assert!((offset.norm() - 0.1).abs() < 1.0e-4, "vertex {i}");
            assert!((offset.normalize() - vertex.normal).norm() < 1.0e-4);
        }
    }

    #[test]
    fn a_closed_tube_joins_its_ends_and_an_open_one_does_not() {
        let path = hoop(24);
        let (_, closed) = generate_tube(&path, 0.1, 8, true, Colour::WHITE);
        let (_, open) = generate_tube(&path, 0.1, 8, false, Colour::WHITE);
        assert_eq!(closed.len(), 24 * 8 * 6);
        assert_eq!(open.len(), 23 * 8 * 6);
    }

    #[test]
    fn a_coil_runs_from_end_to_end_within_its_radius() {
        let start = Point3::new(0.0, 0.0, 0.0);
        let end = Point3::new(0.4, -0.1, 0.0);
        let path = coil_path(start, end, 0.03, 0.04, 8.0, 8);
        assert_eq!(path.first(), Some(&start));
        assert_eq!(path.last(), Some(&end));
        let axis = (end - start).normalize();
        for point in &path {
            let along = (point - start).dot(&axis);
            let off = ((point - start) - axis * along).norm();
            assert!(off <= 0.03 + 1.0e-5, "{off}");
        }
    }

    #[test]
    fn triangles_face_outward() {
        let path = hoop(24);
        let (vertices, indices) = generate_tube(&path, 0.1, 8, true, Colour::WHITE);
        for triangle in indices.chunks(3) {
            let [a, b, c] = [0, 1, 2].map(|k| vertices[triangle[k] as usize].pos);
            let face = (b - a).cross(&(c - a));
            let outward = vertices[triangle[0] as usize].normal;
            assert!(face.dot(&outward) > 0.0, "a triangle faces inward");
        }
    }
}
