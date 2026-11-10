//! Paint baked into the vertices of a form built from ellipsoids
//! ([`Ellipsoid`]), so that it reads as a creature or a fruit rather than a
//! toy moulded in one colour per piece.
//!
//! Both work in the body's frame, so a patch runs on across the seam
//! between two parts and the paint moves with the body.

use nalgebra::Vector3;

use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::utils::noise::fbm_perlin_3d;

use super::parts::{Ellipsoid, SculptedPart};
use super::pattern::{Spots, Veins};

/// Salt that makes the warmth field independent of the value field.
const WARMTH_SALT: u32 = 0x9e37;

/// Patches of lighter and darker, warmer and cooler paint over a body:
/// the uneven colour of skin, shell or a painted glaze.
#[derive(Clone, Copy, Debug)]
pub struct Mottle {
    /// Patches per metre.
    pub frequency: f32,
    /// How far a patch lightens or darkens the paint, as a share of it.
    pub value: f32,
    /// How far a patch shifts the paint warmer (red up, blue down) or
    /// cooler, as a share of it.
    pub warmth: f32,
    /// Seed of the patches.
    pub seed: u32,
}

impl Mottle {
    /// Paint `mesh`'s vertex colours with the patches where its vertices
    /// sit.
    pub fn paint(&self, mesh: &mut MeshPrimitive) {
        for vertex in &mut mesh.vertices {
            let at = vertex.pos * self.frequency;
            let value = 1.0 + self.value * fbm_perlin_3d(at.x, at.y, at.z, 3, 0.5, self.seed);
            let warmth = self.warmth
                * fbm_perlin_3d(
                    at.x,
                    at.y,
                    at.z,
                    2,
                    0.5,
                    self.seed.wrapping_add(WARMTH_SALT),
                );
            let tint = Vector3::new(1.0 + warmth, 1.0, 1.0 - warmth) * value;
            vertex.color.x *= tint.x;
            vertex.color.y *= tint.y;
            vertex.color.z *= tint.z;
        }
    }
}

/// Marks painted on a mesh's vertices where they sit: for a form with no
/// texture coordinates to carry a texture ([`super::clay`]). `frequency`
/// and the like are then per metre, and a mark shows only where the mesh
/// is finer than it is.
pub trait VertexPaint {
    /// The colour the mark multiplies the paint by at its heart (linear),
    /// and how much of it covers `point`, 0 to 1.
    fn mark(&self, point: Vector3<f32>) -> (Vector3<f32>, f32);

    /// Paint `mesh`'s vertex colours with the marks where its vertices sit.
    fn paint(&self, mesh: &mut MeshPrimitive) {
        for vertex in &mut mesh.vertices {
            let (tint, cover) = self.mark(vertex.pos);
            let wash = Vector3::repeat(1.0).lerp(&tint, cover.clamp(0.0, 1.0));
            vertex.color.x *= wash.x;
            vertex.color.y *= wash.y;
            vertex.color.z *= wash.z;
        }
    }
}

impl VertexPaint for Spots {
    fn mark(&self, point: Vector3<f32>) -> (Vector3<f32>, f32) {
        (self.tint, self.cover(point))
    }
}

impl VertexPaint for Veins {
    fn mark(&self, point: Vector3<f32>) -> (Vector3<f32>, f32) {
        (self.tint, self.cover(point))
    }
}

/// A stain round one spot, fading out from it: the flush of a knee or a
/// knuckle, a bruise, the rim of an eye.
#[derive(Clone, Copy, Debug)]
pub struct Blush {
    /// Where its heart is.
    pub at: Vector3<f32>,
    /// How far out it reaches, m.
    pub radius: f32,
    /// The colour it multiplies the paint by at its heart (linear).
    pub tint: Vector3<f32>,
}

impl VertexPaint for Blush {
    fn mark(&self, point: Vector3<f32>) -> (Vector3<f32>, f32) {
        let t = ((point - self.at).norm() / self.radius).min(1.0);
        (self.tint, 1.0 - t * t * (3.0 - 2.0 * t))
    }
}

/// Dirt taken up from the ground: everything below a height stained, the
/// stain fading out above it.
#[derive(Clone, Copy, Debug)]
pub struct Grime {
    /// Height below which the stain is whole, and how far above that it
    /// fades out over, m.
    pub below: f32,
    pub fade: f32,
    /// The colour it multiplies the paint by (linear).
    pub tint: Vector3<f32>,
}

impl VertexPaint for Grime {
    fn mark(&self, point: Vector3<f32>) -> (Vector3<f32>, f32) {
        let t = ((point.y - self.below) / self.fade).clamp(0.0, 1.0);
        (self.tint, 1.0 - t * t * (3.0 - 2.0 * t))
    }
}

/// Shade baked where one part of a body meets another: the soft shadow in
/// the crease where a neck goes into a hood or a stud sits in a shell,
/// which is what makes the parts read as one body.
#[derive(Clone, Copy, Debug)]
pub struct ContactShade {
    /// How far from another part the shade reaches, m.
    pub reach: f32,
    /// How dark it is right against another part, as a share of the light
    /// taken away.
    pub depth: f32,
    /// The darkest any vertex gets, however many parts crowd it.
    pub floor: f32,
}

impl ContactShade {
    /// Bake the shade into the occlusion of `meshes`, each the mesh of the
    /// part of `parts` at the same index.
    pub fn shade(&self, meshes: &mut [MeshPrimitive], parts: &[Ellipsoid]) {
        for (index, mesh) in meshes.iter_mut().enumerate() {
            for vertex in &mut mesh.vertices {
                let open: f32 = parts
                    .iter()
                    .enumerate()
                    .filter(|&(other, _)| other != index)
                    .map(|(_, part)| self.openness(part.distance_outside(vertex.pos)))
                    .product();
                vertex.ao *= open.max(self.floor);
            }
        }
    }

    /// How open a point `distance` outside another part is, 1 when out of
    /// its reach.
    fn openness(&self, distance: f32) -> f32 {
        let t = (distance / self.reach).clamp(0.0, 1.0);
        let near = 1.0 - t * t * (3.0 - 2.0 * t);
        1.0 - self.depth * near
    }
}

/// `parts` as one model, mottled and shaded where they meet.
pub fn painted_model(parts: &[SculptedPart], mottle: &Mottle, shade: &ContactShade) -> Model {
    Model::flat(vec![ModelPart::new(painted_meshes(parts, mottle, shade))])
}

/// Each of `parts` as its own mesh, in their order, mottled and shaded
/// where they meet: for a form whose parts are drawn apart (a compound that
/// can break).
pub fn painted_meshes(
    parts: &[SculptedPart],
    mottle: &Mottle,
    shade: &ContactShade,
) -> Vec<MeshPrimitive> {
    let mut meshes: Vec<MeshPrimitive> = parts
        .iter()
        .map(|part| {
            let mut mesh = part.mesh();
            mottle.paint(&mut mesh);
            mesh
        })
        .collect();
    let shapes: Vec<Ellipsoid> = parts.iter().map(|part| part.shape).collect();
    shade.shade(&mut meshes, &shapes);
    meshes
}

/// A number in `-1..1` that stays the same for the same `key` and `salt`
/// and looks unrelated to its neighbours': for scattering repeated parts
/// (a shell's studs) so that they are not set out by machine.
pub fn scatter(key: u32, salt: u32) -> f32 {
    let mut n = key.wrapping_mul(0x27d4_eb2d) ^ salt.wrapping_mul(0x1656_67b1);
    n ^= n >> 15;
    n = n.wrapping_mul(0x85eb_ca6b);
    n ^= n >> 13;
    n = n.wrapping_mul(0xc2b2_ae35);
    n ^= n >> 16;
    n as f32 / u32::MAX as f32 * 2.0 - 1.0
}
