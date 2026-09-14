//! Ice cube — a block of frozen water, and the engine's first see-through prop.
//!
//! Two things about it are deliberate and neither is the transparency, which
//! it gets for free from [`substance::ICE`].
//!
//! The first is that it is *bevelled* rather than a plain cuboid. A polished
//! surface shows almost nothing on a flat face — a mirror-finish plane reflects
//! the sun in one direction and is dull from every other — so a glossy cube
//! with square edges reads as a grey slab. The twelve narrow bevels catch the
//! key light at whatever angle the cube has tumbled to, and the eight corner
//! facets do the same for the parts of the sky the bevels miss. The shine is
//! geometry; the material only decides what colour it is.
//!
//! The second is that it barely grips anything. Ice is by a wide margin the
//! slipperiest substance in the library, and a block of it on a slope is
//! supposed to be a problem.
//!
//! ```text
//!   IceCubeDef ──▶ substance::ICE ──┬──▶ ColliderDesc  (light, slippery)
//!                                   └──▶ Material      (glassy, blended)
//!              └──▶ ice_cube_mesh ──────▶ Model
//! ```

use std::sync::Arc;

use nalgebra::{Point3, Vector2, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::orientation::Yaw;
use super::shared::textures::seed_from_position;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::colour::Colour;
use crate::rendering::material::MaterialId;
use crate::rendering::pattern;
use crate::rendering::substance::{self, ColliderSubstance, Substance};
use crate::rendering::vertex::Vertex;
use crate::systems::PhysicsResource;

const TEXTURE_SIZE: u32 = 256;

/// How far each edge is cut back, as a fraction of the cube's half-extent.
///
/// Small enough that the cube still reads as a cube and that treating the
/// collider as a full box stays honest, large enough that the bevels are a
/// visible band of highlight rather than a shading artefact at the silhouette.
const BEVEL_FRACTION: f32 = 0.14;

/// The mesh an ice cube of the given half-extent is drawn with.
///
/// Exposed so the visual bench can look at the real geometry rather than a
/// stand-in — a bench that draws its own approximation of a prop is judging
/// the wrong object.
pub fn ice_cube_mesh(half: f32) -> (Vec<Vertex>, Vec<u32>) {
    bevelled_cube(half, half * BEVEL_FRACTION)
}

#[derive(Deserialize)]
pub struct IceCubeDef {
    pub pos: (f32, f32, f32),

    /// Half-extent of the cube, before the edges are cut back.
    #[serde(default = "IceCubeDef::default_size")]
    pub size: f32,

    /// Rotation about `+Y`, in degrees.
    #[serde(default)]
    pub yaw: f32,
}

impl IceCubeDef {
    pub fn default_size() -> f32 {
        0.4
    }

    /// The one declaration of what this object is made of: the collider takes
    /// the coefficients, the material takes the finish and the transparency.
    fn substance(&self) -> Substance {
        substance::ICE
    }
}

impl Spawnable for IceCubeDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(vec![ctx.patterned(
            &self.substance(),
            &pattern::ICE,
            seed_from_position(self.pos, 0),
            TEXTURE_SIZE,
        )?])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let substance = self.substance();
        let pos = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let yaw = Yaw::degrees(self.yaw);
        let half_extents = Vector3::repeat(self.size);

        let model = ice_cube_model(self.size, materials[0]);

        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body_desc = RigidBodyDesc::dynamic()
                .position(pos)
                .rotation(yaw.rotation())
                .gravity_scale(1.0)
                .linear_damping(0.01)
                .angular_damping(0.005);

            let body_handle = physics.world.create_body(body_desc);

            // A box rather than the bevelled hull. The six faces of the mesh
            // sit exactly on the box, so a cube resting on a face is resting
            // where it looks like it is; only the cut edges are a shade inside
            // their collider, and no gameplay reads that. In exchange the cube
            // takes the engine's best-exercised collision path.
            physics.world.attach_collider(
                body_handle,
                ColliderDesc::box_shape(half_extents).of(&substance),
            );

            body_handle
        };

        // Deliberately not `Flammable`. Everything else box-shaped in the
        // library is, and this one is made of water.
        vec![world
            .create_entity()
            .with(Position(Vector3::new(pos.x, pos.y, pos.z)))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(yaw.rotation()))
            .with(RigidBodyComponent(body_handle))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .build()]
    }
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

/// A cube of half-extent `half` with every edge cut back by `bevel`.
///
/// The solid is the cube intersected with its twelve edge planes
/// (`|a| + |b| <= 2 * half - bevel` for each axis pair), which gives 24
/// vertices — every permutation of `(±half, ±inner, ±inner)` with `inner =
/// half - bevel` — arranged as six square faces, twelve rectangular bevels and
/// eight corner triangles.
///
/// Flat-shaded: each face carries its own copies of its vertices with the
/// face's own normal, because a shared normal would round the bevels off into
/// exactly the soft edge they exist to avoid.
fn bevelled_cube(half: f32, bevel: f32) -> (Vec<Vertex>, Vec<u32>) {
    let inner = half - bevel;
    let mut builder = FaceBuilder::new(half);

    // Six faces, each a square inset to the bevel.
    for axis in 0..3 {
        for sign in [1.0_f32, -1.0] {
            let normal = axis_vector(axis, sign);
            let (u_axis, v_axis) = tangent_axes(axis);
            // Wound so the face is counter-clockwise seen from outside, which
            // is what flips with the sign of the normal.
            let (u, v) = if sign > 0.0 {
                (axis_vector(u_axis, 1.0), axis_vector(v_axis, 1.0))
            } else {
                (axis_vector(v_axis, 1.0), axis_vector(u_axis, 1.0))
            };

            let centre = normal * half;
            builder.quad(
                normal,
                [
                    centre - u * inner - v * inner,
                    centre + u * inner - v * inner,
                    centre + u * inner + v * inner,
                    centre - u * inner + v * inner,
                ],
            );
        }
    }

    // Twelve bevels, one per cube edge: a rectangle spanning the shared axis.
    for axis in 0..3 {
        let (a, b) = tangent_axes(axis);
        for sign_a in [1.0_f32, -1.0] {
            for sign_b in [1.0_f32, -1.0] {
                let dir_a = axis_vector(a, sign_a);
                let dir_b = axis_vector(b, sign_b);
                let along = axis_vector(axis, 1.0);

                // The two vertices on each end of the cut edge, one lying on
                // the `a` face and one on the `b` face.
                let on_a = dir_a * half + dir_b * inner;
                let on_b = dir_a * inner + dir_b * half;

                let normal = (dir_a + dir_b).normalize();
                let corners = [
                    on_a - along * inner,
                    on_b - along * inner,
                    on_b + along * inner,
                    on_a + along * inner,
                ];
                builder.quad_facing(normal, corners);
            }
        }
    }

    // Eight corner triangles, one per cube corner.
    for sx in [1.0_f32, -1.0] {
        for sy in [1.0_f32, -1.0] {
            for sz in [1.0_f32, -1.0] {
                let sign = Vector3::new(sx, sy, sz);
                let normal = sign.normalize();
                let corners = [
                    Vector3::new(sx * half, sy * inner, sz * inner),
                    Vector3::new(sx * inner, sy * half, sz * inner),
                    Vector3::new(sx * inner, sy * inner, sz * half),
                ];
                builder.triangle_facing(normal, corners);
            }
        }
    }

    (builder.vertices, builder.indices)
}

/// The cube as a single-primitive model, ready for a `ModelInstance`.
fn ice_cube_model(half: f32, material: MaterialId) -> Arc<Model> {
    let (vertices, indices) = ice_cube_mesh(half);
    let parts = vec![ModelPart::new(vec![MeshPrimitive {
        vertices,
        indices,
        material,
    }])];
    Arc::new(Model::flat(parts))
}

/// Accumulates flat-shaded faces into one mesh.
struct FaceBuilder {
    vertices: Vec<Vertex>,
    indices: Vec<u32>,
    /// Half-extent the texture coordinates are normalised against, so that the
    /// pattern is the same size on a big cube as on a small one.
    half: f32,
}

impl FaceBuilder {
    fn new(half: f32) -> Self {
        Self {
            vertices: Vec::new(),
            indices: Vec::new(),
            half,
        }
    }

    /// Texture coordinates for a point, projected down the face's dominant
    /// axis.
    ///
    /// Both the pattern's fracture planes and the substance's grain are
    /// directional and both follow `v`, so this projection is what decides
    /// which way the ice looks like it froze — per face, rather than per
    /// block. That is the right answer for a cube: a real one shows a
    /// different section of the same structure on each face, and a single
    /// direction carried across all six would read as a printed wrapper.
    fn uv(&self, pos: Vector3<f32>, normal: Vector3<f32>) -> Vector2<f32> {
        let axis = dominant_axis(normal);
        let (u_axis, v_axis) = tangent_axes(axis);
        let scale = 0.5 / self.half;
        Vector2::new(pos[u_axis] * scale + 0.5, pos[v_axis] * scale + 0.5)
    }

    fn push_vertex(&mut self, pos: Vector3<f32>, normal: Vector3<f32>) -> u32 {
        let index = self.vertices.len() as u32;
        self.vertices.push(Vertex {
            pos,
            color: Colour::WHITE.to_vec4(),
            tex_coords: self.uv(pos, normal),
            normal,
            ao: 1.0,
        });
        index
    }

    /// A quad with the given winding, as two triangles.
    fn quad(&mut self, normal: Vector3<f32>, corners: [Vector3<f32>; 4]) {
        let base = self.push_vertex(corners[0], normal);
        for corner in &corners[1..] {
            self.push_vertex(*corner, normal);
        }
        self.indices
            .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }

    /// A quad whose corners may be wound either way, flipped if needed so that
    /// it faces outwards.
    ///
    /// The bevel and corner loops build their corners from sign combinations,
    /// and half of those combinations come out clockwise. Rather than case out
    /// which, the winding is checked against the normal the face is supposed to
    /// have — the one piece of information that is unambiguous.
    fn quad_facing(&mut self, normal: Vector3<f32>, corners: [Vector3<f32>; 4]) {
        let corners = if faces_outward(normal, corners[0], corners[1], corners[2]) {
            corners
        } else {
            [corners[3], corners[2], corners[1], corners[0]]
        };
        self.quad(normal, corners);
    }

    /// A triangle, flipped if needed so that it faces outwards.
    fn triangle_facing(&mut self, normal: Vector3<f32>, corners: [Vector3<f32>; 3]) {
        let corners = if faces_outward(normal, corners[0], corners[1], corners[2]) {
            corners
        } else {
            [corners[2], corners[1], corners[0]]
        };

        let base = self.push_vertex(corners[0], normal);
        self.push_vertex(corners[1], normal);
        self.push_vertex(corners[2], normal);
        self.indices.extend_from_slice(&[base, base + 1, base + 2]);
    }
}

/// Whether a triangle wound `a, b, c` has its geometric normal on the same
/// side as `normal`.
fn faces_outward(normal: Vector3<f32>, a: Vector3<f32>, b: Vector3<f32>, c: Vector3<f32>) -> bool {
    (b - a).cross(&(c - a)).dot(&normal) > 0.0
}

/// The unit vector along `axis`, times `sign`.
fn axis_vector(axis: usize, sign: f32) -> Vector3<f32> {
    let mut v = Vector3::zeros();
    v[axis] = sign;
    v
}

/// The two axes that are not `axis`, in cyclic order so that
/// `axis × u = v` — which is what makes the face windings come out consistent.
fn tangent_axes(axis: usize) -> (usize, usize) {
    ((axis + 1) % 3, (axis + 2) % 3)
}

/// Which axis a normal points most nearly along.
fn dominant_axis(normal: Vector3<f32>) -> usize {
    let abs = normal.abs();
    if abs.x >= abs.y && abs.x >= abs.z {
        0
    } else if abs.y >= abs.z {
        1
    } else {
        2
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mesh(half: f32, bevel: f32) -> (Vec<Vertex>, Vec<u32>) {
        bevelled_cube(half, bevel)
    }

    /// Six squares, twelve bevel rectangles and eight corner triangles:
    /// 2 + 2 + 2 triangles' worth per quad and one per corner.
    #[test]
    fn the_cube_has_every_face_a_bevelled_cube_should() {
        let (_, indices) = mesh(0.5, 0.07);
        let triangles = indices.len() / 3;
        assert_eq!(triangles, 6 * 2 + 12 * 2 + 8);
    }

    /// The property a bevel exists for: no vertex sits on a cube corner any
    /// more. A vertex with all three coordinates at the half-extent would mean
    /// the chamfer never happened.
    #[test]
    fn no_vertex_is_left_on_a_sharp_corner() {
        let half = 0.5;
        let (vertices, _) = mesh(half, 0.07);
        for vertex in &vertices {
            let at_extent = [vertex.pos.x, vertex.pos.y, vertex.pos.z]
                .iter()
                .filter(|c| (c.abs() - half).abs() < 1e-5)
                .count();
            assert_eq!(
                at_extent, 1,
                "vertex {:?} lies on {} faces of the cube",
                vertex.pos, at_extent
            );
        }
    }

    /// Every face must point away from the centre. A single inverted winding
    /// is invisible on an opaque object under back-face culling and glaring on
    /// a transparent one, where both sides are drawn — which is exactly why it
    /// is worth a test here and was never worth one on the crate.
    #[test]
    fn every_triangle_faces_outwards() {
        let (vertices, indices) = mesh(0.5, 0.07);
        for triangle in indices.chunks_exact(3) {
            let a = vertices[triangle[0] as usize].pos;
            let b = vertices[triangle[1] as usize].pos;
            let c = vertices[triangle[2] as usize].pos;
            let centroid = (a + b + c) / 3.0;
            let geometric = (b - a).cross(&(c - a));
            assert!(
                geometric.dot(&centroid) > 0.0,
                "triangle at {centroid:?} is wound inside out"
            );
        }
    }

    /// And every vertex normal must agree with the face it belongs to, or the
    /// flat shading the bevels depend on has been smoothed away.
    #[test]
    fn each_face_carries_its_own_normal() {
        let (vertices, indices) = mesh(0.5, 0.07);
        for triangle in indices.chunks_exact(3) {
            let normals: Vec<Vector3<f32>> = triangle
                .iter()
                .map(|&i| vertices[i as usize].normal)
                .collect();
            assert!((normals[0] - normals[1]).norm() < 1e-5);
            assert!((normals[0] - normals[2]).norm() < 1e-5);
            assert!((normals[0].norm() - 1.0).abs() < 1e-5);
        }
    }

    /// The cube must not outgrow the box collider spawned alongside it, or it
    /// would visibly sink into whatever it lands on.
    #[test]
    fn nothing_pokes_out_past_the_collider() {
        let half = 0.5;
        let (vertices, _) = mesh(half, 0.07);
        for vertex in &vertices {
            assert!(vertex.pos.amax() <= half + 1e-5, "{:?}", vertex.pos);
        }
    }
}
