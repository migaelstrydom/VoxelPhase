//! Ice block — one rectangular prism of frozen water, and the piece every
//! other ice object in the library is built out of.
//!
//! Two things about it are deliberate and neither is the transparency, which
//! it gets for free from [`substance::ICE`].
//!
//! The first is that it is *bevelled* rather than a plain cuboid. A polished
//! surface shows almost nothing on a flat face — a mirror-finish plane reflects
//! the sun in one direction and is dull from every other — so a glossy block
//! with square edges reads as a grey slab. The twelve narrow bevels catch the
//! key light at whatever angle the block has tumbled to, and the eight corner
//! facets do the same for the parts of the sky the bevels miss. The shine is
//! geometry; the material only decides what colour it is.
//!
//! The second is that it barely grips anything. Ice is by a wide margin the
//! slipperiest substance in the library, and a block of it on a slope is
//! supposed to be a problem.
//!
//! ```text
//!   IceBoxDef ──┐
//!   IceWallDef ─┼──▶ IceBlock ──▶ substance::ICE ──┬──▶ ColliderDesc  (slippery)
//!   IglooDef ───┘        │                         └──▶ Material      (glassy)
//!                        └──────▶ ice_block_mesh ──────▶ Model
//! ```

use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector2, Vector3};
use specs::{Builder, Entity, World, WorldExt};

use super::super::shared::models::PiecePlacement;
use super::super::shared::textures::seed_from_position;
use super::super::MaterialCtx;
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

pub const TEXTURE_SIZE: u32 = 256;

/// How far each edge is cut back, as a fraction of the block's *smallest*
/// half-extent.
///
/// Against the smallest rather than each axis in turn for two reasons: the cut
/// has to be the same width on both faces meeting at an edge or the bevel
/// stops being a 45° facet, and a thin slab must not have its thin axis
/// chamfered away entirely. Small enough that the block still reads as a
/// block and that treating the collider as a full box stays honest, large
/// enough that the bevels are a visible band of highlight rather than a
/// shading artefact at the silhouette.
const BEVEL_FRACTION: f32 = 0.14;

/// Half-extent the ice texture is normalised against, in metres.
///
/// A constant rather than the block's own size, which is what makes every
/// piece of ice in a level look like the same substance: fracture planes and
/// frost come out the same size on a brick of an igloo as on a loose cube, and
/// a block larger than this tiles the pattern instead of magnifying it. The
/// texture is periodic, so tiling is seamless.
const TEXTURE_HALF: f32 = 0.4;

/// The material every ice object asks for: the ice substance, marked with the
/// ice pattern.
///
/// `seed` separates one block's markings from another's; identical seeds share
/// a baked texture through the level's texture cache, which is what keeps a
/// wall of forty bricks down to a handful of textures.
pub fn ice_material(ctx: &mut MaterialCtx, seed: u32) -> EngineResult<MaterialId> {
    ctx.patterned(&ice(), &pattern::ICE, seed, TEXTURE_SIZE)
}

/// A spread of distinct ice textures for an assembly of `count` blocks.
///
/// The blocks of a wall or a dome want to differ from each other, but not by
/// one bespoke 256² texture each — that is megabytes of variation nobody can
/// see. `variants` textures, cycled, is the whole of the effect.
pub fn ice_materials(
    ctx: &mut MaterialCtx,
    origin: (f32, f32, f32),
    variants: usize,
) -> EngineResult<Vec<MaterialId>> {
    (0..variants)
        .map(|variant| ice_material(ctx, seed_from_position(origin, variant as u32)))
        .collect()
}

/// The one declaration of what these objects are made of: the collider takes
/// the coefficients, the material takes the finish and the transparency.
pub fn ice() -> Substance {
    substance::ICE
}

/// One rectangular prism of ice, ready to be put into the world.
pub struct IceBlock {
    /// Centre of the block, in world space.
    pub centre: Point3<f32>,
    /// Half-extents along the block's own axes, before the edges are cut back.
    pub half_extents: Vector3<f32>,
    /// Orientation of those axes.
    pub rotation: UnitQuaternion<f32>,
    /// Which of the caller's materials this block wears.
    pub material: MaterialId,
}

impl IceBlock {
    pub fn new(centre: Point3<f32>, half_extents: Vector3<f32>, material: MaterialId) -> Self {
        Self {
            centre,
            half_extents,
            rotation: UnitQuaternion::identity(),
            material,
        }
    }

    pub fn rotated(mut self, rotation: UnitQuaternion<f32>) -> Self {
        self.rotation = rotation;
        self
    }

    /// Create the body, the collider and the entity.
    pub fn spawn(&self, world: &mut World) -> Entity {
        let substance = ice();
        let model = ice_block_model(self.half_extents, self.material);

        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body_handle = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .gravity_scale(1.0)
                    .linear_damping(0.01)
                    .angular_damping(0.005)
                    .position(self.centre)
                    .rotation(self.rotation),
            );

            // A box rather than the bevelled hull. The six faces of the mesh
            // sit exactly on the box, so a block resting on a face is resting
            // where it looks like it is; only the cut edges are a shade inside
            // their collider, and no gameplay reads that. In exchange the block
            // takes the engine's best-exercised collision path.
            physics.world.attach_collider(
                body_handle,
                ColliderDesc::box_shape(self.half_extents).of(&substance),
            );

            body_handle
        };

        // Deliberately not `Flammable`. Everything else box-shaped in the
        // library is, and this one is made of water.
        world
            .create_entity()
            .with(Position(self.centre.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(self.rotation))
            .with(RigidBodyComponent(body_handle))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .build()
    }
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

/// The mesh an ice block of the given half-extents is drawn with.
///
/// Exposed so the visual bench can look at the real geometry rather than a
/// stand-in — a bench that draws its own approximation of a prop is judging
/// the wrong object.
pub fn ice_block_mesh(half_extents: Vector3<f32>) -> (Vec<Vertex>, Vec<u32>) {
    let bevel = half_extents.min() * BEVEL_FRACTION;
    bevelled_box(half_extents, bevel)
}

/// The same mesh, for a block that is one piece of a larger object.
///
/// A compound object is many blocks of the same size, and a mesh that depends
/// only on that size gives every one of them identical markings — a wall of
/// forty bricks with the same crack in each. Offsetting the texture by where
/// the piece sits makes the object read as ice that was carved into blocks
/// rather than as one block printed forty times.
pub fn ice_piece_mesh(piece: &PiecePlacement) -> (Vec<Vertex>, Vec<u32>) {
    let (mut vertices, indices) = ice_block_mesh(piece.half_extents);
    let offset = texture_offset(piece.offset);
    for vertex in &mut vertices {
        vertex.tex_coords += offset;
    }
    (vertices, indices)
}

/// How far through the pattern a piece at this offset starts.
///
/// Any injective-enough function of the offset would do; this one keeps
/// neighbours a whole feature apart rather than a hair, which is what makes
/// the difference visible.
fn texture_offset(offset: Vector3<f32>) -> Vector2<f32> {
    let scale = 0.5 / TEXTURE_HALF;
    Vector2::new(
        (offset.x + offset.y * 0.5) * scale,
        (offset.z + offset.y * 0.5) * scale,
    )
}

/// The block as a single-primitive model, ready for a `ModelInstance`.
pub fn ice_block_model(half_extents: Vector3<f32>, material: MaterialId) -> Arc<Model> {
    let (vertices, indices) = ice_block_mesh(half_extents);
    let parts = vec![ModelPart::new(vec![MeshPrimitive {
        vertices,
        indices,
        material,
    }])];
    Arc::new(Model::flat(parts))
}

/// A rectangular prism of the given half-extents with every edge cut back by
/// `bevel`.
///
/// The solid is the box intersected with its twelve edge planes, which gives
/// 24 vertices — every permutation of one coordinate at its half-extent and
/// the other two at `half - bevel` — arranged as six rectangular faces, twelve
/// rectangular bevels and eight corner triangles.
///
/// The cut-back is one distance rather than one fraction per axis, which is
/// what keeps every bevel a true 45° facet between the two faces it joins.
///
/// Flat-shaded: each face carries its own copies of its vertices with the
/// face's own normal, because a shared normal would round the bevels off into
/// exactly the soft edge they exist to avoid.
fn bevelled_box(half: Vector3<f32>, bevel: f32) -> (Vec<Vertex>, Vec<u32>) {
    let inner = half.map(|h| h - bevel);
    let mut builder = FaceBuilder::new();

    // Six faces, each a rectangle inset to the bevel.
    for axis in 0..3 {
        for sign in [1.0_f32, -1.0] {
            let normal = axis_vector(axis, sign);
            let (u_axis, v_axis) = tangent_axes(axis);
            // Wound so the face is counter-clockwise seen from outside, which
            // is what flips with the sign of the normal.
            let (first, second) = if sign > 0.0 {
                (u_axis, v_axis)
            } else {
                (v_axis, u_axis)
            };
            let u = axis_vector(first, 1.0) * inner[first];
            let v = axis_vector(second, 1.0) * inner[second];

            let centre = normal * half[axis];
            builder.quad(
                normal,
                [
                    centre - u - v,
                    centre + u - v,
                    centre + u + v,
                    centre - u + v,
                ],
            );
        }
    }

    // Twelve bevels, one per box edge: a rectangle spanning the shared axis.
    for axis in 0..3 {
        let (a, b) = tangent_axes(axis);
        for sign_a in [1.0_f32, -1.0] {
            for sign_b in [1.0_f32, -1.0] {
                let dir_a = axis_vector(a, sign_a);
                let dir_b = axis_vector(b, sign_b);
                let along = axis_vector(axis, 1.0);

                // The two vertices on each end of the cut edge, one lying on
                // the `a` face and one on the `b` face.
                let on_a = dir_a * half[a] + dir_b * inner[b];
                let on_b = dir_a * inner[a] + dir_b * half[b];

                let normal = (dir_a + dir_b).normalize();
                let corners = [
                    on_a - along * inner[axis],
                    on_b - along * inner[axis],
                    on_b + along * inner[axis],
                    on_a + along * inner[axis],
                ];
                builder.quad_facing(normal, corners);
            }
        }
    }

    // Eight corner triangles, one per box corner. The three cut points are one
    // bevel apart along each pair of axes, so the facet they span is the same
    // 45°-from-every-face plane a cube's corner facet is, whatever the box's
    // proportions.
    for sx in [1.0_f32, -1.0] {
        for sy in [1.0_f32, -1.0] {
            for sz in [1.0_f32, -1.0] {
                let sign = Vector3::new(sx, sy, sz);
                let normal = sign.normalize();
                let corners = [
                    Vector3::new(sx * half.x, sy * inner.y, sz * inner.z),
                    Vector3::new(sx * inner.x, sy * half.y, sz * inner.z),
                    Vector3::new(sx * inner.x, sy * inner.y, sz * half.z),
                ];
                builder.triangle_facing(normal, corners);
            }
        }
    }

    (builder.vertices, builder.indices)
}

/// Accumulates flat-shaded faces into one mesh.
struct FaceBuilder {
    vertices: Vec<Vertex>,
    indices: Vec<u32>,
}

impl FaceBuilder {
    fn new() -> Self {
        Self {
            vertices: Vec::new(),
            indices: Vec::new(),
        }
    }

    /// Texture coordinates for a point, projected down the face's dominant
    /// axis at a fixed world scale.
    ///
    /// Both the pattern's fracture planes and the substance's grain are
    /// directional and both follow `v`, so this projection is what decides
    /// which way the ice looks like it froze — per face, rather than per
    /// block. That is the right answer for a block: a real one shows a
    /// different section of the same structure on each face, and a single
    /// direction carried across all six would read as a printed wrapper.
    fn uv(&self, pos: Vector3<f32>, normal: Vector3<f32>) -> Vector2<f32> {
        let axis = dominant_axis(normal);
        let (u_axis, v_axis) = tangent_axes(axis);
        let scale = 0.5 / TEXTURE_HALF;
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

    /// A quad, flipped if needed so that it faces outwards.
    ///
    /// The bevels and corner facets are generated by sign permutation, which
    /// gets the geometry right and the winding right only half the time; the
    /// faces are generated by hand, where the winding is the more direct
    /// statement. Both end up with an outward face, which is the only
    /// information that is unambiguous.
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

    /// A cube and a slab, because every property here is one a generalisation
    /// from a cube can lose.
    fn shapes() -> [Vector3<f32>; 2] {
        [cube(), slab()]
    }

    fn cube() -> Vector3<f32> {
        Vector3::new(0.5, 0.5, 0.5)
    }

    fn slab() -> Vector3<f32> {
        Vector3::new(2.5, 0.5, 1.2)
    }

    fn mesh(half: Vector3<f32>) -> (Vec<Vertex>, Vec<u32>) {
        bevelled_box(half, half.min() * BEVEL_FRACTION)
    }

    /// Six rectangles, twelve bevel rectangles and eight corner triangles:
    /// 2 + 2 + 2 triangles' worth per quad and one per corner.
    #[test]
    fn the_block_has_every_face_a_bevelled_box_should() {
        for half in shapes() {
            let (_, indices) = mesh(half);
            let triangles = indices.len() / 3;
            assert_eq!(triangles, 6 * 2 + 12 * 2 + 8);
        }
    }

    /// The property a bevel exists for: no vertex sits on a box corner any
    /// more. A vertex with all three coordinates at a half-extent would mean
    /// the chamfer never happened.
    #[test]
    fn no_vertex_is_left_on_a_sharp_corner() {
        for half in shapes() {
            let (vertices, _) = mesh(half);
            for vertex in &vertices {
                let at_extent = (0..3)
                    .filter(|&axis| (vertex.pos[axis].abs() - half[axis]).abs() < 1e-5)
                    .count();
                assert_eq!(
                    at_extent, 1,
                    "vertex {:?} lies on {} faces of the box",
                    vertex.pos, at_extent
                );
            }
        }
    }

    /// Every face must point away from the centre. A single inverted winding
    /// is invisible on an opaque object under back-face culling and glaring on
    /// a transparent one, where both sides are drawn — which is exactly why it
    /// is worth a test here and was never worth one on the crate.
    #[test]
    fn every_triangle_faces_outwards() {
        for half in shapes() {
            let (vertices, indices) = mesh(half);
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
    }

    /// And every vertex normal must agree with the face it belongs to, or the
    /// flat shading the bevels depend on has been smoothed away.
    #[test]
    fn each_face_carries_its_own_normal() {
        for half in shapes() {
            let (vertices, indices) = mesh(half);
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
    }

    /// The block must not outgrow the box collider spawned alongside it, or it
    /// would visibly sink into whatever it lands on.
    #[test]
    fn nothing_pokes_out_past_the_collider() {
        for half in shapes() {
            let (vertices, _) = mesh(half);
            for vertex in &vertices {
                for axis in 0..3 {
                    assert!(
                        vertex.pos[axis].abs() <= half[axis] + 1e-5,
                        "{:?}",
                        vertex.pos
                    );
                }
            }
        }
    }

    /// A slab must actually reach its authored extents on every axis — the
    /// failure mode of a generalisation from a cube is a block that is drawn
    /// as a cube of the smallest half-extent inside its own collider.
    #[test]
    fn a_slab_fills_its_half_extents() {
        let (vertices, _) = mesh(slab());
        for axis in 0..3 {
            let reach = vertices
                .iter()
                .fold(0.0_f32, |acc, v| acc.max(v.pos[axis].abs()));
            assert!(
                (reach - slab()[axis]).abs() < 1e-5,
                "axis {axis} reaches {reach}, wanted {}",
                slab()[axis]
            );
        }
    }

    /// Every bevel facet is a 45° cut between the two faces it joins, on a
    /// slab as much as on a cube. This is the property that fails the moment
    /// the cut-back is authored per axis rather than as one distance.
    #[test]
    fn bevels_are_true_forty_five_degree_facets() {
        let (vertices, indices) = mesh(slab());
        for triangle in indices.chunks_exact(3) {
            let normal = vertices[triangle[0] as usize].normal;
            let components: Vec<f32> = (0..3).map(|axis| normal[axis].abs()).collect();
            let off_axis = components.iter().filter(|c| **c > 1e-4).count();
            // A face normal has one component, a bevel two, a corner three;
            // and whenever there is more than one they must be equal.
            if off_axis > 1 {
                let first = components.iter().find(|c| **c > 1e-4).unwrap();
                for component in components.iter().filter(|c| **c > 1e-4) {
                    assert!((component - first).abs() < 1e-4, "normal {normal:?}");
                }
            }
        }
    }
}
