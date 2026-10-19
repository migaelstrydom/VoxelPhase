//! The bed and its springs, drawn where the bed's body is this frame.
//!
//! The bed's collider is a flat disc riding the spring. The drawn bed is a
//! dish over it: flat and level with the collider's top across the middle,
//! where a jumper lands, and rising toward the edge, where the springs hold
//! it up. The springs run from the hoop to that edge and stretch as it sinks.
//!
//! ```text
//!   hoop ◯~~~~~~~~╮                        ╭~~~~~~~~◯ hoop
//!                  ╲______________________╱
//!                   ▲ collider top (the sag below the hoop)
//! ```

use std::f32::consts::TAU;

use nalgebra::{Vector2, Vector3};

use super::shape::TrampolineShape;
use crate::geometry::{append_tube, coil_path};
use crate::rendering::colour::Colour;
use crate::rendering::vertex::Vertex;

/// The share of the sag the bed's edge sinks by: the dish's rim, where the
/// springs hold it, stays nearer the hoop than its middle.
const EDGE_SAG_SHARE: f32 = 0.3;
/// The flat middle of the dish, as a share of the bed's radius.
const FLAT_SHARE: f32 = 0.55;
/// Rings from the bed's centre to its edge.
const RINGS: u32 = 14;
/// Points around each ring.
const SEGMENTS: u32 = 48;
/// Thickness of the drawn bed, between its top and its underside.
const BED_THICKNESS: f32 = 0.012;

/// The springs' and legs' bright steel. Colour rides on the vertices, since
/// a material's base colour is not applied.
pub const STEEL: Colour = Colour::rgb(0.86, 0.87, 0.9);

/// Radius of a spring's coil.
const COIL_RADIUS: f32 = 0.028;
/// Radius of a spring's wire.
const WIRE_RADIUS: f32 = 0.008;
/// Length of the straight hook at each end of a spring.
const HOOK_LENGTH: f32 = 0.035;
/// Turns in each spring.
const COIL_TURNS: f32 = 9.0;
/// Points along a coil per turn.
const POINTS_PER_TURN: u32 = 7;
/// Facets around a spring's wire.
const WIRE_SIDES: u32 = 4;

/// How far below the hoop's centreline the dish's top is at `radius` from
/// its centre, for a bed whose collider top has sunk `sag` below its rest.
fn dish_depth(shape: &TrampolineShape, sag: f32, radius: f32) -> f32 {
    let flat = shape.bed_radius * FLAT_SHARE;
    if radius <= flat {
        return sag;
    }
    let t = ((radius - flat) / (shape.bed_radius - flat)).clamp(0.0, 1.0);
    let rise = t * t * (3.0 - 2.0 * t);
    sag * (1.0 - rise * (1.0 - EDGE_SAG_SHARE))
}

/// The slope of [`dish_depth`] with radius: how fast the top falls away
/// toward the middle.
fn dish_slope(shape: &TrampolineShape, sag: f32, radius: f32) -> f32 {
    let step = shape.bed_radius * 1.0e-3;
    (dish_depth(shape, sag, radius + step) - dish_depth(shape, sag, radius - step)) / (2.0 * step)
}

/// The drawn bed for a collider top `sag` below its rest, moved by `offset`
/// from the ground frame into the frame it is drawn in: a dish with its top
/// and underside, textured by a planar map over its disc.
pub fn bed_mesh(
    shape: &TrampolineShape,
    sag: f32,
    offset: Vector3<f32>,
) -> (Vec<Vertex>, Vec<u32>) {
    let colour = Colour::WHITE.to_vec4();
    let mut vertices = Vec::with_capacity((2 * (RINGS + 1) * (SEGMENTS + 1)) as usize);
    let mut indices = Vec::new();

    for (face, lift, facing) in [(0u32, 0.0, 1.0f32), (1, -BED_THICKNESS, -1.0)] {
        let base = face * (RINGS + 1) * (SEGMENTS + 1);
        for ring in 0..=RINGS {
            let radius = shape.bed_radius * ring as f32 / RINGS as f32;
            let depth = dish_depth(shape, sag, radius);
            let slope = dish_slope(shape, sag, radius);
            for segment in 0..=SEGMENTS {
                let bearing = segment as f32 / SEGMENTS as f32 * TAU;
                let outward = Vector3::new(bearing.cos(), 0.0, bearing.sin());
                let point = shape.on_circle(radius, bearing, lift - depth);
                // The top rises outward by -slope per metre; its normal
                // leans back toward the middle by as much.
                let normal = (Vector3::y() + outward * slope).normalize() * facing;
                vertices.push(Vertex {
                    pos: point.coords + offset,
                    color: colour,
                    tex_coords: Vector2::new(
                        0.5 + 0.5 * point.x / shape.bed_radius,
                        0.5 + 0.5 * point.z / shape.bed_radius,
                    ),
                    normal,
                    ao: 1.0,
                });
            }
        }
        let row = SEGMENTS + 1;
        for ring in 0..RINGS {
            for segment in 0..SEGMENTS {
                let a = base + ring * row + segment;
                let b = a + row;
                if face == 0 {
                    indices.extend_from_slice(&[a, a + 1, b, a + 1, b + 1, b]);
                } else {
                    indices.extend_from_slice(&[a, b, a + 1, a + 1, b, b + 1]);
                }
            }
        }
    }

    // The edge: a band joining the top's outer ring to the underside's.
    let rim = RINGS * (SEGMENTS + 1);
    let underside_rim = (RINGS + 1) * (SEGMENTS + 1) + rim;
    for segment in 0..SEGMENTS {
        let (top, bottom) = (rim + segment, underside_rim + segment);
        indices.extend_from_slice(&[top, top + 1, bottom, top + 1, bottom + 1, bottom]);
    }

    (vertices, indices)
}

/// Every spring, from its hook on the hoop to the drawn bed's edge, for a
/// collider top `sag` below its rest, moved by `offset` like [`bed_mesh`].
pub fn springs_mesh(
    shape: &TrampolineShape,
    sag: f32,
    offset: Vector3<f32>,
) -> (Vec<Vertex>, Vec<u32>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let edge_depth = dish_depth(shape, sag, shape.bed_radius) + BED_THICKNESS * 0.5;
    for index in 0..shape.spring_count {
        let hoop_end = shape.spring_hoop_end(index) + offset;
        let bed_end =
            shape.on_circle(shape.bed_radius, shape.spring_bearing(index), -edge_depth) + offset;
        let path = coil_path(
            hoop_end,
            bed_end,
            COIL_RADIUS,
            HOOK_LENGTH,
            COIL_TURNS,
            POINTS_PER_TURN,
        );
        append_tube(
            &mut vertices,
            &mut indices,
            &path,
            WIRE_RADIUS,
            WIRE_SIDES,
            false,
            STEEL,
        );
    }
    (vertices, indices)
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;

    /// The top of the drawn bed at `radius` from its centre, in the ground
    /// frame: what a test compares with the collider.
    fn dish_top(shape: &TrampolineShape, sag: f32, radius: f32) -> Point3<f32> {
        shape.on_circle(radius, 0.0, -dish_depth(shape, sag, radius))
    }

    #[test]
    fn the_middle_of_the_dish_is_level_with_the_collider_top() {
        let shape = TrampolineShape::with_radius(1.8);
        for sag in [-0.1, 0.0, 0.3, 0.6] {
            let collider_top = shape.bed_rest_centre().y + shape.bed_thickness * 0.5 - sag;
            for share in [0.0, 0.3, FLAT_SHARE] {
                let top = dish_top(&shape, sag, shape.bed_radius * share);
                assert!(
                    (top.y - collider_top).abs() < 1.0e-5,
                    "sag {sag}, share {share}"
                );
            }
        }
    }

    #[test]
    fn the_edge_sinks_by_its_share_of_the_sag() {
        let shape = TrampolineShape::with_radius(1.8);
        let edge = dish_top(&shape, 0.5, shape.bed_radius);
        assert!((shape.hoop_height - edge.y - 0.5 * EDGE_SAG_SHARE).abs() < 1.0e-5);
    }

    #[test]
    fn the_bed_faces_up_on_top_and_down_beneath() {
        let shape = TrampolineShape::with_radius(1.8);
        let (vertices, indices) = bed_mesh(&shape, 0.4, Vector3::zeros());
        let (mut up, mut down) = (0, 0);
        for triangle in indices.chunks(3) {
            let [a, b, c] = [0, 1, 2].map(|k| vertices[triangle[k] as usize].pos);
            let face = (b - a).cross(&(c - a));
            if face.norm() < 1.0e-9 {
                continue;
            }
            // The edge band stands on end; only the top and underside are
            // judged.
            if face.y.abs() < 0.5 * face.norm() {
                continue;
            }
            let normal = vertices[triangle[0] as usize].normal;
            assert!(face.dot(&normal) > 0.0, "a face winds against its normal");
            if face.y > 0.0 {
                up += 1;
            } else if face.y < 0.0 {
                down += 1;
            }
        }
        assert!(up > 0 && down > 0);
    }
}
