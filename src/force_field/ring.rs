//! The field's glowing rings: on each pod's face, on the ground under it,
//! and thrown outward by a load spike.
//!
//! They are opaque and emissive, so they are what blooms: only opaque
//! surfaces reach the bright pass, and the column is drawn after it as light.
//! One unit torus serves them all, placed and sized by a transform and
//! restyled every frame by its surface.

use std::sync::{Arc, OnceLock};

use nalgebra::{Matrix4, Point3, Rotation3, Vector2, Vector3, Vector4};

use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::rendering::colour::Colour;
use crate::rendering::material::{Emission, MaterialId, SurfaceFinish, SurfaceParams};
use crate::rendering::vertex::Vertex;

use super::pod::{PodLook, RingLook};
use super::style::FieldStyle;

/// Thickness of a ring's tube, as a fraction of its radius.
const TUBE: f32 = 0.07;
/// Vertices around a ring, and around its tube.
const SEGMENTS: u32 = 48;
const TUBE_SEGMENTS: u32 = 8;
/// The ring's own colour under its glow: dark, so the glow is all it shows.
const BODY: Colour = Colour::rgb(0.06, 0.08, 0.1);
/// Its finish: glossy, so it still catches a highlight when it glows dimly.
const FINISH: SurfaceFinish = SurfaceFinish {
    roughness: 0.25,
    metallic: 0.0,
};

/// One ring to draw: the unit torus, placed, and how it glows.
#[derive(Debug, Clone, Copy)]
pub struct RingDraw {
    pub transform: Matrix4<f32>,
    pub surface: SurfaceParams,
}

/// The unit torus every ring is drawn from: radius 1, lying in the `xz`
/// plane. Its material is never read: a ring is drawn with the surface its
/// [`RingDraw`] carries.
pub fn ring_model() -> &'static Arc<Model> {
    static MODEL: OnceLock<Arc<Model>> = OnceLock::new();
    MODEL.get_or_init(|| {
        let (vertices, indices) = torus();
        Arc::new(Model::flat(vec![ModelPart::new(vec![MeshPrimitive {
            vertices,
            indices,
            material: MaterialId(0),
        }])]))
    })
}

/// Every ring among `looks`, styled by `style`: one on each pod's face, and
/// one on the ground and a pulse wherever the look has them.
pub fn ring_draws(looks: &[PodLook], style: &FieldStyle) -> Vec<RingDraw> {
    let mut draws = Vec::with_capacity(looks.len() * 3);
    for look in looks {
        let colour = Colour::rgb(look.colour.x, look.colour.y, look.colour.z);
        let pod = RingLook {
            centre: look.anchor,
            normal: -look.axis,
            radius: style.pod_ring_radius,
            glow: look.pod_glow,
        };
        let rings = [Some(pod), look.ground_ring, look.pulse_ring];
        draws.extend(rings.into_iter().flatten().map(|ring| RingDraw {
            transform: placed(&ring),
            surface: SurfaceParams::glowing(FINISH, Emission::glow(colour, ring.glow)),
        }));
    }
    draws
}

/// The unit torus, lying on the plane of `ring` rather than through it.
fn placed(ring: &RingLook) -> Matrix4<f32> {
    let lift = ring.normal * (TUBE * ring.radius);
    let turn = Rotation3::rotation_between(&Vector3::y(), &ring.normal)
        .unwrap_or_else(|| Rotation3::from_axis_angle(&Vector3::x_axis(), std::f32::consts::PI));
    Matrix4::new_translation(&(ring.centre + lift).coords)
        * turn.to_homogeneous()
        * Matrix4::new_scaling(ring.radius)
}

/// A torus of radius 1 and tube `TUBE`, about `y`.
fn torus() -> (Vec<Vertex>, Vec<u32>) {
    let mut vertices = Vec::with_capacity(((SEGMENTS + 1) * (TUBE_SEGMENTS + 1)) as usize);
    for i in 0..=SEGMENTS {
        let around = i as f32 / SEGMENTS as f32 * std::f32::consts::TAU;
        let outward = Vector3::new(around.cos(), 0.0, around.sin());
        for j in 0..=TUBE_SEGMENTS {
            let tube = j as f32 / TUBE_SEGMENTS as f32 * std::f32::consts::TAU;
            let normal = outward * tube.cos() + Vector3::y() * tube.sin();
            let pos = Point3::from(outward) + normal * TUBE;
            vertices.push(Vertex {
                pos: pos.coords,
                color: Vector4::new(BODY.r, BODY.g, BODY.b, 1.0),
                tex_coords: Vector2::new(
                    i as f32 / SEGMENTS as f32,
                    j as f32 / TUBE_SEGMENTS as f32,
                ),
                normal,
                ao: 1.0,
            });
        }
    }
    let stride = TUBE_SEGMENTS + 1;
    let mut indices = Vec::with_capacity((SEGMENTS * TUBE_SEGMENTS * 6) as usize);
    for i in 0..SEGMENTS {
        for j in 0..TUBE_SEGMENTS {
            let a = i * stride + j;
            let b = a + stride;
            indices.extend_from_slice(&[a, a + 1, b, a + 1, b + 1, b]);
        }
    }
    (vertices, indices)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Wound so the outside faces out: each triangle's right-hand normal
    /// agrees with its vertices', as a cube's does. The pipeline culls back
    /// faces, so a ring wound the other way would show only its inside.
    #[test]
    fn the_torus_is_wound_outward() {
        let (vertices, indices) = torus();
        for triangle in indices.chunks(3) {
            let [a, b, c] = [0, 1, 2].map(|k| vertices[triangle[k] as usize]);
            let face = (b.pos - a.pos).cross(&(c.pos - a.pos));
            if face.norm() < 1e-9 {
                continue;
            }
            let normal = a.normal + b.normal + c.normal;
            assert!(face.dot(&normal) > 0.0);
        }
    }

    /// A ring on the ground lies on it: its lowest point is on the plane.
    #[test]
    fn a_ground_ring_rests_on_its_plane() {
        let ring = RingLook {
            centre: Point3::new(1.0, 2.0, 3.0),
            normal: Vector3::new(0.0, 1.0, 1.0).normalize(),
            radius: 0.3,
            glow: 1.0,
        };
        let transform = placed(&ring);
        let (vertices, _) = torus();
        let lowest = vertices
            .iter()
            .map(|v| {
                (transform.transform_point(&Point3::from(v.pos)) - ring.centre).dot(&ring.normal)
            })
            .fold(f32::MAX, f32::min);
        assert!(lowest.abs() < 1e-4, "{lowest}");
    }
}
