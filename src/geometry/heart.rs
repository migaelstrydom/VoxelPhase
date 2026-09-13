//! A plump, three-dimensional heart.
//!
//! The outline is the classic parametric heart curve, swept front-to-back
//! and puffed out by a circular profile so the cross-section through the
//! middle is an ellipse. The result is a plush heart rather than an
//! extruded flat one.
//!
//! ```text
//!        outline (right, up)          sweep (forward)
//!            ,-.   ,-.                 back ──────► front
//!           (   \ /   )                 .-'‾‾‾‾‾‾'-.
//!            \       /                 (  scale(u)  )
//!             \     /                   '-.______.-'
//!              \   /                   u: 0 ──────► 1
//!               `-'                    scale = sqrt(1 - (2u-1)^2)
//! ```
//!
//! A sweep narrower than the full `0..1` yields an open cap. Inflating
//! that cap along its own normals lays it on the surface of a larger
//! heart, which is how a heart gets a heart drawn on its front.

use nalgebra::{Point2, Vector3};

use crate::rendering::{colour::Colour, vertex::Vertex};

/// What a heart looks like.
#[derive(Clone, Copy, Debug)]
pub struct HeartSpec {
    /// Full width, lobe to lobe, along local +X.
    pub width: f32,
    /// Full height, cleft to point, along local +Y.
    pub height: f32,
    /// Full depth, back to front, along local +Z.
    pub depth: f32,
    /// Vertices around the outline. The curve has two lobes and a cleft
    /// to resolve, so this wants to be generous.
    pub outline_segments: u32,
    /// Rings across the sweep.
    pub depth_segments: u32,
    /// Portion of the sweep to emit, from back (`0.0`) to front (`1.0`).
    /// The full range closes the surface with a pole at each end; a
    /// partial range leaves an open cap.
    pub sweep: (f32, f32),
    /// Distance to push every vertex out along its own normal, after the
    /// surface is built. Lifts a cap clear of the body it sits on.
    pub inflate: f32,
    pub colour: Colour,
}

impl HeartSpec {
    /// A closed heart of the given size, tessellated for a small prop.
    pub fn new(width: f32, height: f32, depth: f32, colour: Colour) -> Self {
        Self {
            width,
            height,
            depth,
            outline_segments: 72,
            depth_segments: 10,
            sweep: (0.0, 1.0),
            inflate: 0.0,
            colour,
        }
    }

    /// The patch of this heart's front whose silhouette is a heart
    /// `scale` times its own size, lifted `inflate` clear of the surface.
    ///
    /// This is what makes a marking: the patch *is* part of the body's
    /// surface, so it sits flat on it at every point and cannot be buried
    /// inside it. A smaller heart of the same proportions placed at the
    /// front would be — every ring of it is narrower than the body ring
    /// at the same depth.
    pub fn front_marking(mut self, scale: f32, inflate: f32) -> Self {
        self.sweep = (depth_at_scale(scale), 1.0);
        self.inflate = inflate;
        self
    }

    /// The same marking on the back.
    pub fn back_marking(mut self, scale: f32, inflate: f32) -> Self {
        self.sweep = (0.0, 1.0 - depth_at_scale(scale));
        self.inflate = inflate;
        self
    }
}

/// The depth fraction at which the sweep has narrowed to `scale`, on the
/// front half. Inverts [`sweep_scale`].
fn depth_at_scale(scale: f32) -> f32 {
    0.5 * (1.0 + (1.0 - scale.clamp(0.0, 1.0).powi(2)).max(0.0).sqrt())
}

/// How wide the sweep is at depth fraction `u`. Zero at both poles, one
/// at the widest plane.
#[inline]
fn sweep_scale(u: f32) -> f32 {
    (1.0 - (2.0 * u - 1.0).powi(2)).max(0.0).sqrt()
}

/// The heart curve, normalised into a unit box centred on the origin.
///
/// `x = 16 sin³t`, `y = 13 cos t − 5 cos 2t − 2 cos 3t − cos 4t`, scaled
/// so the bounding box is exactly `[-0.5, 0.5]²`.
fn outline(segments: u32) -> Vec<Point2<f32>> {
    let raw: Vec<Point2<f32>> = (0..segments)
        .map(|i| {
            let t = std::f32::consts::TAU * i as f32 / segments as f32;
            let x = 16.0 * t.sin().powi(3);
            let y =
                13.0 * t.cos() - 5.0 * (2.0 * t).cos() - 2.0 * (3.0 * t).cos() - (4.0 * t).cos();
            Point2::new(x, y)
        })
        .collect();

    let (mut min, mut max) = (raw[0], raw[0]);
    for p in &raw {
        min = Point2::new(min.x.min(p.x), min.y.min(p.y));
        max = Point2::new(max.x.max(p.x), max.y.max(p.y));
    }
    let centre = Point2::new((min.x + max.x) * 0.5, (min.y + max.y) * 0.5);
    let extent = Point2::new(max.x - min.x, max.y - min.y);

    raw.iter()
        .map(|p| Point2::new((p.x - centre.x) / extent.x, (p.y - centre.y) / extent.y))
        .collect()
}

/// Build a heart.
pub fn generate_heart(spec: &HeartSpec) -> (Vec<Vertex>, Vec<u32>) {
    let outline = outline(spec.outline_segments.max(12));
    let ring_len = outline.len();
    let rings = spec.depth_segments.max(2) as usize;
    let (u0, u1) = spec.sweep;

    let mut positions: Vec<Vector3<f32>> = Vec::new();
    let mut normals: Vec<Vector3<f32>> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();

    // A ring collapses to a single vertex where the sweep reaches a pole.
    let mut ring_starts: Vec<(u32, bool)> = Vec::new();
    for k in 0..=rings {
        let u = u0 + (u1 - u0) * k as f32 / rings as f32;
        let start = positions.len() as u32;

        if sweep_scale(u) <= 1e-5 {
            positions.push(surface(spec, &outline, 0, u));
            normals.push(Vector3::z() * (u - 0.5).signum());
            ring_starts.push((start, true));
            continue;
        }
        for i in 0..ring_len {
            positions.push(surface(spec, &outline, i, u));
            normals.push(surface_normal(spec, &outline, i, u));
        }
        ring_starts.push((start, false));
    }

    for k in 0..rings {
        let (a, a_pole) = ring_starts[k];
        let (b, b_pole) = ring_starts[k + 1];
        for i in 0..ring_len {
            let j = (i + 1) % ring_len;
            let (a0, a1) = (a + i as u32, a + j as u32);
            let (b0, b1) = (b + i as u32, b + j as u32);
            match (a_pole, b_pole) {
                (true, true) => {}
                (true, false) => indices.extend([a, b1, b0]),
                (false, true) => indices.extend([a0, a1, b]),
                (false, false) => indices.extend([a0, a1, b1, a0, b1, b0]),
            }
        }
    }

    let vertices = positions
        .iter()
        .zip(&normals)
        .map(|(pos, normal)| Vertex {
            pos: pos + normal * spec.inflate,
            color: spec.colour.into(),
            tex_coords: nalgebra::Vector2::zeros(),
            normal: *normal,
            ao: 1.0,
        })
        .collect();

    (vertices, indices)
}

/// A point on the swept surface: outline vertex `i` at depth fraction `u`.
fn surface(spec: &HeartSpec, outline: &[Point2<f32>], i: usize, u: f32) -> Vector3<f32> {
    let scale = sweep_scale(u);
    let p = outline[i];
    Vector3::new(
        p.x * spec.width * scale,
        p.y * spec.height * scale,
        (u - 0.5) * spec.depth,
    )
}

/// The surface normal at an outline vertex, from the two tangents of the
/// parametric surface.
///
/// Taken from the surface rather than accumulated from the faces that
/// happen to be present. On a closed heart the two agree; on a cap they
/// do not, because the rim's faces all lie to one side of it, and a rim
/// inflated along a lopsided normal peels away from the body it is meant
/// to be lying on.
fn surface_normal(spec: &HeartSpec, outline: &[Point2<f32>], i: usize, u: f32) -> Vector3<f32> {
    let ring_len = outline.len();
    let previous = (i + ring_len - 1) % ring_len;
    let next = (i + 1) % ring_len;

    let along_outline = surface(spec, outline, next, u) - surface(spec, outline, previous, u);

    const DU: f32 = 1e-3;
    let (lower, upper) = ((u - DU).max(0.0), (u + DU).min(1.0));
    let across_sweep = surface(spec, outline, i, upper) - surface(spec, outline, i, lower);

    let normal = along_outline
        .cross(&across_sweep)
        .try_normalize(1e-9)
        .unwrap_or_else(Vector3::z);

    // Point it away from the sweep axis. The outline winds one way and the
    // sweep the other, so the raw cross product's sign is not obvious by
    // inspection — this settles it at every vertex instead.
    let position = surface(spec, outline, i, u);
    let outward = Vector3::new(position.x, position.y, 0.0);
    if normal.dot(&outward) < 0.0 {
        -normal
    } else {
        normal
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bounds(verts: &[Vertex]) -> (Vector3<f32>, Vector3<f32>) {
        let mut min = verts[0].pos;
        let mut max = verts[0].pos;
        for v in verts {
            min = min.inf(&v.pos);
            max = max.sup(&v.pos);
        }
        (min, max)
    }

    /// A heart is exactly as big as it was asked to be, and centred.
    #[test]
    fn a_heart_fills_the_box_it_was_given() {
        let spec = HeartSpec::new(0.3, 0.26, 0.16, Colour::RED);
        let (verts, _) = generate_heart(&spec);
        let (min, max) = bounds(&verts);

        assert!(
            (max.x - min.x - 0.3).abs() < 1e-3,
            "width {}",
            max.x - min.x
        );
        assert!(
            (max.y - min.y - 0.26).abs() < 1e-3,
            "height {}",
            max.y - min.y
        );
        assert!(
            (max.z - min.z - 0.16).abs() < 1e-3,
            "depth {}",
            max.z - min.z
        );
        assert!(min.x.abs() - max.x.abs() < 1e-3, "not centred laterally");
    }

    /// The lobes are at the top and the point is at the bottom — a heart
    /// upside down is a very different piece of clip art.
    #[test]
    fn a_heart_points_downward() {
        let spec = HeartSpec::new(1.0, 1.0, 0.5, Colour::RED);
        let (verts, _) = generate_heart(&spec);

        let widest_ring = verts
            .iter()
            .filter(|v| v.pos.z.abs() < 1e-4)
            .collect::<Vec<_>>();
        let lowest = widest_ring
            .iter()
            .min_by(|a, b| a.pos.y.total_cmp(&b.pos.y))
            .unwrap();
        assert!(
            lowest.pos.x.abs() < 0.02,
            "the point of a heart is on its centreline, got x={}",
            lowest.pos.x
        );

        // The widest part of a heart is above its middle.
        let widest = widest_ring
            .iter()
            .max_by(|a, b| a.pos.x.total_cmp(&b.pos.x))
            .unwrap();
        assert!(widest.pos.y > 0.0, "lobes below the middle");
    }

    /// A marking is a patch of the body's own front, so it reaches the
    /// nose and never the back.
    #[test]
    fn a_marking_covers_only_the_front() {
        let spec = HeartSpec::new(0.3, 0.26, 0.16, Colour::RED).front_marking(0.55, 0.0);
        let (verts, _) = generate_heart(&spec);
        let (min, max) = bounds(&verts);

        assert!(
            min.z > 0.0,
            "a front marking should not reach the back, {min}"
        );
        assert!((max.z - 0.08).abs() < 1e-3, "it should reach the nose");
    }

    /// The point of a marking: its silhouette is a heart of the size it
    /// was asked for, and every vertex lies on the body it marks.
    #[test]
    fn a_marking_is_a_smaller_heart_lying_on_the_body() {
        let body = HeartSpec::new(0.3, 0.26, 0.16, Colour::RED);
        let (body_verts, _) = generate_heart(&body);
        let (mark_verts, _) = generate_heart(&body.front_marking(0.5, 0.0));

        let (_, mark_max) = bounds(&mark_verts);
        let (_, body_max) = bounds(&body_verts);
        assert!(
            (mark_max.x - body_max.x * 0.5).abs() < 3e-3,
            "marking half the size should be half as wide: {} vs {}",
            mark_max.x,
            body_max.x * 0.5
        );

        // The marking's rim sits on the body surface rather than inside
        // it: at the depth where the body has narrowed to half its width,
        // and nowhere else.
        let rim_z = mark_verts
            .iter()
            .map(|v| v.pos.z)
            .fold(f32::INFINITY, f32::min);
        let expected_z = (depth_at_scale(0.5) - 0.5) * 0.16;
        assert!(
            (rim_z - expected_z).abs() < 1e-4,
            "rim at z={rim_z}, body is half-width at z={expected_z}"
        );
    }
}
