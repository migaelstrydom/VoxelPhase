//! An ellipsoid meshed as rings round its longest axis, its vertices about
//! evenly spaced on the ellipsoid itself.
//!
//! A geodesic sphere stretched into an ellipsoid keeps its sphere's even
//! spacing only in proportion: sized for the long axis, it is oversampled
//! across the short ones by their ratio. Rings follow the ellipsoid instead:
//! they are laid at equal arc lengths down its longest meridian, and each
//! holds as many vertices as its own perimeter needs, at equal arc lengths
//! round it. The two ends of the long axis are single vertices.
//!
//! ```text
//!        pole                    rings by arc length down the meridian,
//!       /|||\                    vertices by arc length round each ring,
//!      ( ring )  ── zipped ──    neighbouring rings zipped into triangles
//!      ( ring )                  by how far round each vertex is
//!       \|||/
//!        pole
//! ```

use std::f32::consts::{PI, TAU};

use nalgebra::Vector3;

use super::icosphere::Icosphere;

/// Steps an arc length is integrated over: fine enough that spacing errors
/// are well under a vertex.
const ARC_STEPS: usize = 256;

/// A mesh of the ellipsoid with semi-axes `radii`, its vertices about
/// `spacing` apart over its surface.
///
/// Positions are on the unit sphere: scaled by `radii` they lie on the
/// ellipsoid, so a caller maps them as it would a geodesic sphere's.
/// Triangles are counter-clockwise seen from outside.
pub fn generate_ellipsoid_rings(radii: Vector3<f32>, spacing: f32) -> Icosphere {
    let long = radii.imax();
    let (u, v) = ((long + 1) % 3, (long + 2) % 3);
    let meridian = radii[u].max(radii[v]);
    let polar = even_angles(radii[long], meridian, spacing, PI);
    let rings = polar.len() - 1;

    let direction = |theta: f32, phi: f32| {
        let mut d = Vector3::zeros();
        d[u] = theta.sin() * phi.cos();
        d[v] = theta.sin() * phi.sin();
        d[long] = theta.cos();
        d
    };
    let mut positions = vec![direction(0.0, 0.0)];
    // Each ring's first vertex and its vertices' fractions of a turn.
    let mut ring_rows: Vec<(u32, Vec<f32>)> = Vec::with_capacity(rings);
    for &theta in &polar[1..rings] {
        let (a, b) = (radii[u] * theta.sin(), radii[v] * theta.sin());
        let azimuths = even_angles(a, b, spacing, TAU);
        let azimuths = &azimuths[..azimuths.len() - 1];
        let first = positions.len() as u32;
        positions.extend(azimuths.iter().map(|&phi| direction(theta, phi)));
        ring_rows.push((first, azimuths.iter().map(|phi| phi / TAU).collect()));
    }
    let south = positions.len() as u32;
    positions.push(direction(PI, 0.0));

    let mut indices = Vec::new();
    let mut triangle = |a: u32, b: u32, c: u32, positions: &[Vector3<f32>]| {
        let [pa, pb, pc] = [a, b, c].map(|i| positions[i as usize]);
        if (pb - pa).cross(&(pc - pa)).dot(&(pa + pb + pc)) > 0.0 {
            indices.extend_from_slice(&[a, b, c]);
        } else {
            indices.extend_from_slice(&[a, c, b]);
        }
    };
    let fan =
        |pole: u32, (first, turns): &(u32, Vec<f32>), triangle: &mut dyn FnMut(u32, u32, u32)| {
            let n = turns.len() as u32;
            for j in 0..n {
                triangle(pole, first + j, first + (j + 1) % n);
            }
        };
    {
        let mut emit = |a, b, c| triangle(a, b, c, &positions);
        fan(0, &ring_rows[0], &mut emit);
        for pair in ring_rows.windows(2) {
            zip(&pair[0], &pair[1], &mut emit);
        }
        fan(south, &ring_rows[ring_rows.len() - 1], &mut emit);
    }
    Icosphere { positions, indices }
}

/// Angles from 0 to `span` round the ellipse with semi-axes `a` (at angle
/// 0) and `b`, at equal arc lengths about `spacing` apart, both ends
/// included. At least three steps, so even a tiny ring is a triangle.
fn even_angles(a: f32, b: f32, spacing: f32, span: f32) -> Vec<f32> {
    let point = |t: f32| Vector3::new(a * t.cos(), b * t.sin(), 0.0);
    let step = span / ARC_STEPS as f32;
    let mut arc = Vec::with_capacity(ARC_STEPS + 1);
    let mut length = 0.0;
    arc.push(0.0);
    for k in 1..=ARC_STEPS {
        length += (point(step * k as f32) - point(step * (k - 1) as f32)).magnitude();
        arc.push(length);
    }
    let count = ((length / spacing).ceil() as usize).max(3);
    let mut angles = Vec::with_capacity(count + 1);
    let mut k = 0;
    for i in 0..=count {
        let target = length * i as f32 / count as f32;
        while k + 1 < ARC_STEPS && arc[k + 1] < target {
            k += 1;
        }
        let within = (target - arc[k]) / (arc[k + 1] - arc[k]).max(f32::EPSILON);
        angles.push(step * (k as f32 + within.clamp(0.0, 1.0)));
    }
    angles
}

/// Triangles between two neighbouring rings, walking both round together
/// and always advancing the one whose next vertex is less far round.
fn zip(
    (first_a, turns_a): &(u32, Vec<f32>),
    (first_b, turns_b): &(u32, Vec<f32>),
    triangle: &mut dyn FnMut(u32, u32, u32),
) {
    let (n_a, n_b) = (turns_a.len(), turns_b.len());
    let turn = |turns: &[f32], i: usize| {
        if i < turns.len() {
            turns[i]
        } else {
            1.0 + turns[i - turns.len()]
        }
    };
    let index = |first: u32, n: usize, i: usize| first + (i % n) as u32;
    let (mut i, mut j) = (0, 0);
    while i < n_a || j < n_b {
        let advance_a = j == n_b || (i < n_a && turn(turns_a, i + 1) <= turn(turns_b, j + 1));
        let (a, b) = (index(*first_a, n_a, i), index(*first_b, n_b, j));
        if advance_a {
            triangle(a, b, index(*first_a, n_a, i + 1));
            i += 1;
        } else {
            triangle(a, b, index(*first_b, n_b, j + 1));
            j += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use super::*;

    fn on_ellipsoid(mesh: &Icosphere, radii: Vector3<f32>) -> Vec<Vector3<f32>> {
        mesh.positions
            .iter()
            .map(|p| p.component_mul(&radii))
            .collect()
    }

    /// Every edge is shared by exactly two triangles, and every triangle
    /// faces out: the mesh is closed and wound as a geodesic sphere is.
    #[test]
    fn the_rings_close_and_face_out() {
        for radii in [
            Vector3::new(0.3, 1.4, 0.35),
            Vector3::new(1.0, 0.35, 0.9),
            Vector3::new(0.5, 0.5, 0.5),
        ] {
            let mesh = generate_ellipsoid_rings(radii, 0.025);
            let mut edges: HashMap<(u32, u32), i32> = HashMap::new();
            for face in mesh.indices.chunks_exact(3) {
                let [a, b, c] = [0, 1, 2].map(|i| mesh.positions[face[i] as usize]);
                assert!((b - a).cross(&(c - a)).dot(&(a + b + c)) > 0.0);
                for k in 0..3 {
                    let (p, q) = (face[k], face[(k + 1) % 3]);
                    *edges.entry((p.min(q), p.max(q))).or_default() += 1;
                }
            }
            assert!(edges.values().all(|&n| n == 2), "{radii:?} is not closed");
        }
    }

    /// On a column, edges are about the spacing long whichever way they
    /// run, so there are about as many vertices as its area needs.
    #[test]
    fn a_column_is_meshed_evenly() {
        let radii = Vector3::new(0.3, 1.4, 0.3);
        let spacing = 0.025;
        let mesh = generate_ellipsoid_rings(radii, spacing);
        let points = on_ellipsoid(&mesh, radii);
        let mut lengths: Vec<f32> = mesh
            .indices
            .chunks_exact(3)
            .flat_map(|f| (0..3).map(move |k| (f[k], f[(k + 1) % 3])))
            .map(|(p, q)| (points[p as usize] - points[q as usize]).magnitude())
            .collect();
        lengths.sort_by(f32::total_cmp);
        let median = lengths[lengths.len() / 2];
        assert!(
            (0.8 * spacing..1.3 * spacing).contains(&median),
            "median edge {median}"
        );
        // A prolate spheroid's area, against a vertex per spacing squared.
        let e = (1.0 - (radii.x / radii.y).powi(2)).sqrt();
        let area = 2.0 * PI * radii.x * radii.x * (1.0 + radii.y / (radii.x * e) * e.asin());
        let per_vertex = area / mesh.positions.len() as f32;
        assert!(
            (0.5 * spacing * spacing..1.2 * spacing * spacing).contains(&per_vertex),
            "{} vertices for {area} m²",
            mesh.positions.len()
        );
    }
}
