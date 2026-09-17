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

use super::super::shared::models::{
    dominant_axis, hull_mesh, tangent_axes, PieceHull, PiecePlacement, SurfaceUvs,
};
use super::super::shared::textures::seed_from_position;
use super::super::MaterialCtx;
use crate::cleave::{BrittleSolid, CleaveRule};
use crate::collision::convex_hull::ConvexHull;
use crate::collision::hull_bevel::{bevel_hull_by_face, extent_of};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::fracture::CompoundFracture;
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::colour::Colour;
use crate::rendering::material::MaterialId;
use crate::rendering::pattern::{self, Spread};
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

/// Half-extent one tile of the ice texture is normalised against, in metres.
///
/// A constant rather than the block's own size, which is what makes every
/// piece of ice in a level look like the same substance: fracture planes and
/// frost come out the same size on a brick of an igloo as on a loose cube, and
/// a block larger than this tiles the pattern instead of magnifying it. The
/// texture is periodic, so tiling is seamless.
///
/// Seamless is not the same as unnoticed, though — see [`texture_spread`].
const TEXTURE_HALF: f32 = 0.4;

/// How many tiles of pattern a block of this size is given.
///
/// The tile is 0.8m across and it is the *same* 0.8m on a paving slab five
/// metres wide, which is right for the feature size and wrong for everything
/// else: the slab wears a six-by-six grid of the same frost bloom, and a grid
/// is the easiest thing in a picture to see. Asking for a [`Spread`] instead
/// keeps the features at 0.8m and gives the block up to four tiles of distinct
/// pattern before anything comes round again.
///
/// Measured against the largest half-extent, because that is the face with the
/// most surface to fill and the texture is square: sizing to a thin axis would
/// leave the broad face tiling exactly as before.
pub fn texture_spread(half_extents: Vector3<f32>) -> Spread {
    Spread::covering(half_extents.max() / TEXTURE_HALF)
}

/// Texture coordinates per metre for a block at this spread.
///
/// One number, derived in one place, because the mesh's UVs and the texture
/// the material baked have to agree: a spread that reaches one and not the
/// other does not tile wrongly, it changes the size of every feature on the
/// block.
fn uv_scale(spread: Spread) -> f32 {
    0.5 / (TEXTURE_HALF * spread.tiles() as f32)
}

/// How ice is laid out on its texture, at this spread.
///
/// The one thing every piece of an ice object has to agree on. A wedge is
/// drawn from the hull it broke into, which has faces the block never had and
/// is smaller than the block besides; mapping the texture onto each of those
/// faces in turn would magnify the frost by whatever each face happened to
/// measure. Fixing the density instead keeps the markings the size they were,
/// so a block that cracks reads as one piece of ice in two parts rather than
/// as two smaller pieces of some finer ice.
pub fn ice_uvs(spread: Spread) -> SurfaceUvs {
    SurfaceUvs::PerMetre(uv_scale(spread))
}

/// The material every ice object asks for: the ice substance, marked with the
/// ice pattern.
///
/// `seed` separates one block's markings from another's; identical seeds share
/// a baked texture through the level's texture cache, which is what keeps a
/// wall of forty bricks down to a handful of textures.
pub fn ice_material(ctx: &mut MaterialCtx, seed: u32, spread: Spread) -> EngineResult<MaterialId> {
    ctx.patterned_spread(&ice(), &pattern::ICE, seed, TEXTURE_SIZE, spread)
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
    spread: Spread,
) -> EngineResult<Vec<MaterialId>> {
    (0..variants)
        .map(|variant| ice_material(ctx, seed_from_position(origin, variant as u32), spread))
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
    /// How hard the block has to be struck to come apart, and into what.
    pub cleaving: CleaveRule,
    /// How many tiles of pattern `material`'s texture holds. Carried rather
    /// than re-derived, because an assembly bakes one texture for blocks that
    /// are not all the same size — a wall's end bricks are half-length — and
    /// every block wearing that texture has to address it the same way.
    pub spread: Spread,
}

impl IceBlock {
    pub fn new(
        centre: Point3<f32>,
        half_extents: Vector3<f32>,
        material: MaterialId,
        spread: Spread,
    ) -> Self {
        Self {
            centre,
            half_extents,
            rotation: UnitQuaternion::identity(),
            material,
            cleaving: ice_cleaving(),
            spread,
        }
    }

    pub fn rotated(mut self, rotation: UnitQuaternion<f32>) -> Self {
        self.rotation = rotation;
        self
    }

    /// Create the body, the collider and the entity.
    pub fn spawn(&self, world: &mut World) -> Entity {
        let substance = ice();
        let model = ice_block_model(self.half_extents, self.material, self.spread);

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
        //
        // A compound of one block, which is all a block needs to be able to
        // break: `SolidCleaveSystem` replaces the one child with the wedges
        // it cleaved into, and from then on the block is a compound like any
        // other.
        //
        // Deliberately not `shedding_debris`, unlike glass. A pane crazes into
        // dozens of slivers that are scenery the moment they land, and taking
        // the small ones away is the whole reason the budget exists. Ice
        // cleaves into two or three wedges the size of the block, and a wedge
        // that large glittering out of existence in front of the player reads
        // as a bug rather than as settling debris. The cost is that ice piles
        // up: the ceiling on the pieces in a level is the cleave depth, not
        // the budget.
        world
            .create_entity()
            .with(Position(self.centre.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(self.rotation))
            .with(RigidBodyComponent(body_handle))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .with(
                CompoundFracture::boxes(Vec::new(), 1, self.material)
                    .with_piece_mesh(ice_piece_mesh)
                    .with_hull_mesh(ice_hull_mesh)
                    .with_uvs(ice_uvs(self.spread)),
            )
            .with(BrittleSolid::new(self.cleaving, self.material, 1))
            .build()
    }
}

/// How ice comes apart: into a few stout wedges, along surfaces tilted off
/// the block's own faces.
///
/// The threshold is roughly the blow a block of this size takes falling two
/// metres onto rock — hard enough that handling one does not shatter it, soft
/// enough that throwing one at something does. It has not been play-tested.
pub fn ice_cleaving() -> CleaveRule {
    CleaveRule {
        threshold: 400.0,
        pieces: 3,
        tilt: 14.0,
        min_volume: 0.004,
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
pub fn ice_block_mesh(half_extents: Vector3<f32>, spread: Spread) -> (Vec<Vertex>, Vec<u32>) {
    let bevel = half_extents.min() * BEVEL_FRACTION;
    bevelled_box(half_extents, bevel, uv_scale(spread))
}

/// The spread a piece of a compound ice object falls back to.
///
/// [`PieceMesh`](super::super::shared::models::PieceMesh) is a bare function
/// pointer — it is stored in a fracture component that outlives the spawn —
/// so a piece mesh cannot carry a spread of its own. It is told one through
/// [`SurfaceUvs`] instead, and this is what it uses when it is not: walls and
/// domes are built out of bricks, and a brick is smaller than a tile. See
/// [`texture_spread`] for the case this is not.
const PIECE_SPREAD: Spread = Spread::ONE;

/// The same mesh, for a block that is one piece of a larger object.
///
/// A compound object is many blocks of the same size, and a mesh that depends
/// only on that size gives every one of them identical markings — a wall of
/// forty bricks with the same crack in each. Offsetting the texture by where
/// the piece sits makes the object read as ice that was carved into blocks
/// rather than as one block printed forty times.
pub fn ice_piece_mesh(piece: &PiecePlacement, uvs: SurfaceUvs) -> (Vec<Vertex>, Vec<u32>) {
    let scale = uvs.per_metre_or(uv_scale(PIECE_SPREAD));
    let bevel = piece.half_extents.min() * BEVEL_FRACTION;
    let (mut vertices, indices) = bevelled_box(piece.half_extents, bevel, scale);
    let offset = texture_offset(piece.offset, scale);
    for vertex in &mut vertices {
        vertex.tex_coords += offset;
    }
    (vertices, indices)
}

/// The mesh one cleaved wedge of ice is drawn with: its hull, chamfered.
///
/// A wedge is drawn from the hull it collides as, and a hull cut by a plane
/// has square edges — so without this a block that had twelve bevels and eight
/// corner facets lost every one of them at the instant it cracked, which is
/// the instant the player is watching it. The chamfer goes on every edge of
/// the wedge rather than only the ones that were on the block, because a wedge
/// does not know which of its faces it was born with; the cost is that the
/// fracture surfaces catch the light too, which on ice reads as the crack it
/// is.
///
/// Drawing only. The collider keeps the full hull, the same bargain the whole
/// block makes with its box.
pub fn ice_hull_mesh(piece: &PieceHull, uvs: SurfaceUvs) -> (Vec<Vertex>, Vec<u32>) {
    // `extent_of` is a full dimension and [`BEVEL_FRACTION`] is authored
    // against a half-extent, so that a wedge wears the same width of facet
    // the block it came out of does.
    hull_mesh(&PieceHull::new(&chamfered(piece.hull), piece.offset), uvs)
}

/// A wedge's hull as it is drawn: the block's own faces cut back in full, the
/// surfaces it broke along barely at all.
fn chamfered(hull: &ConvexHull) -> ConvexHull {
    let bevel = extent_of(hull) * 0.5 * BEVEL_FRACTION;
    let widths: Vec<f32> = hull
        .faces
        .iter()
        .map(|face| {
            if was_cut(face.normal) {
                bevel * CUT_FACE_BEVEL
            } else {
                bevel
            }
        })
        .collect();
    bevel_hull_by_face(hull, &widths)
}

/// How much of the block's chamfer a fracture surface gets.
///
/// Not zero, because a cut face meeting a cut face at a square edge is a
/// hairline the eye reads as an artefact; not the full width, because two
/// wedges still holding together each cut back by that much leave a groove
/// wide enough to read as a gap rather than a crack.
const CUT_FACE_BEVEL: f32 = 0.2;

/// How far off an axis a face may point and still be one the block was
/// authored with, in degrees.
///
/// A block is a box, so in a wedge's own frame — which is the block's frame,
/// translated — its six original faces point exactly along an axis. Every
/// fracture surface is a face normal tilted by [`CleaveRule::tilt`], and the
/// rule never tilts by less than 60% of it, so for ice the nearest a cut can
/// come to an axis is 8.4°. This sits well inside that gap; the test
/// `a_cut_face_is_never_mistaken_for_one_of_the_blocks_own` is what keeps the
/// two from drifting together.
const FACE_OF_THE_BLOCK: f32 = 4.0;

/// Whether a face of a wedge is a surface it was broken along rather than one
/// of the block's own.
fn was_cut(normal: Vector3<f32>) -> bool {
    let nearest = (0..3).fold(0.0f32, |acc, axis| acc.max(normal[axis].abs()));
    nearest < FACE_OF_THE_BLOCK.to_radians().cos()
}

/// How far through the pattern a piece at this offset starts.
///
/// Any injective-enough function of the offset would do; this one keeps
/// neighbours a whole feature apart rather than a hair, which is what makes
/// the difference visible.
fn texture_offset(offset: Vector3<f32>, scale: f32) -> Vector2<f32> {
    Vector2::new(
        (offset.x + offset.y * 0.5) * scale,
        (offset.z + offset.y * 0.5) * scale,
    )
}

/// The block as a single-primitive model, ready for a `ModelInstance`.
pub fn ice_block_model(
    half_extents: Vector3<f32>,
    material: MaterialId,
    spread: Spread,
) -> Arc<Model> {
    let (vertices, indices) = ice_block_mesh(half_extents, spread);
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
fn bevelled_box(half: Vector3<f32>, bevel: f32, uv_scale: f32) -> (Vec<Vertex>, Vec<u32>) {
    let inner = half.map(|h| h - bevel);
    let mut builder = FaceBuilder::new(uv_scale);

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
    /// Texture coordinates per metre, from [`uv_scale`].
    uv_scale: f32,
}

impl FaceBuilder {
    fn new(uv_scale: f32) -> Self {
        Self {
            vertices: Vec::new(),
            indices: Vec::new(),
            uv_scale,
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
        Vector2::new(
            pos[u_axis] * self.uv_scale + 0.5,
            pos[v_axis] * self.uv_scale + 0.5,
        )
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
        bevelled_box(
            half,
            half.min() * BEVEL_FRACTION,
            uv_scale(texture_spread(half)),
        )
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

    /// The bevels have to survive the block breaking. A wedge is drawn from
    /// its hull, and a hull cut by a plane has square edges — so a block that
    /// cracked went from twenty-six facets to a handful of flat ones at the
    /// one moment the player was watching it.
    #[test]
    fn a_cleaved_wedge_is_still_drawn_with_bevels() {
        let wedge = crate::collision::convex_hull::cube_hull(Vector3::new(0.2, 0.15, 0.18));
        let (plain, _) = hull_mesh(
            &PieceHull::new(&wedge, Vector3::zeros()),
            SurfaceUvs::Fitted,
        );
        let (bevelled, _) = ice_hull_mesh(
            &PieceHull::new(&wedge, Vector3::zeros()),
            SurfaceUvs::Fitted,
        );

        assert_eq!(distinct_normals(&plain).len(), 6, "a box has six faces");
        assert_eq!(
            distinct_normals(&bevelled).len(),
            6 + 12 + 8,
            "the wedge lost its facets"
        );
    }

    /// A wedge and a whole block are the same substance and must be chamfered
    /// the same way, or a broken block reads as two materials. Both are cut
    /// back by the same fraction, so both carry the same twenty-six facets —
    /// including the eight 45° corner facets, which are the ones a chamfer
    /// built out of edge cuts alone silently leaves out.
    #[test]
    fn a_wedge_wears_the_same_facets_the_whole_block_does() {
        let half = Vector3::new(0.25, 0.18, 0.21);
        let (authored, _) = ice_block_mesh(half, PIECE_SPREAD);
        let (cleaved, _) = ice_hull_mesh(
            &PieceHull::new(
                &crate::collision::convex_hull::cube_hull(half),
                Vector3::zeros(),
            ),
            SurfaceUvs::Fitted,
        );

        let from_block = distinct_normals(&authored);
        let from_wedge = distinct_normals(&cleaved);
        assert_eq!(from_block.len(), 6 + 12 + 8);
        assert_eq!(from_wedge.len(), from_block.len());

        for block in &from_block {
            assert!(
                from_wedge.iter().any(|w| (w - block).norm() < 1e-3),
                "the wedge has no facet facing {block:?}"
            );
        }
    }

    /// Texture coordinates per metre of surface, measured off the mesh, one
    /// reading per triangle.
    ///
    /// The square root of a ratio of areas, so it is the linear magnification
    /// the pattern is drawn at whatever shape the triangle is.
    ///
    /// Only the faces that lie square to an axis are read. The chamfers are
    /// deliberately left out: they are projected down the axis they lean away
    /// from, so the pattern on them is foreshortened by cos 45° — which is
    /// what a chamfer on a real surface looks like, and not a reading of what
    /// scale the texture is addressed at.
    fn uv_density(vertices: &[Vertex], indices: &[u32]) -> Vec<f32> {
        indices
            .chunks_exact(3)
            .filter_map(|tri| {
                let v: Vec<&Vertex> = tri.iter().map(|&i| &vertices[i as usize]).collect();
                if v[0].normal.abs().max() < 0.999 {
                    return None;
                }
                let world = (v[1].pos - v[0].pos).cross(&(v[2].pos - v[0].pos)).norm();
                let a = v[1].tex_coords - v[0].tex_coords;
                let b = v[2].tex_coords - v[0].tex_coords;
                let uv = (a.x * b.y - a.y * b.x).abs();
                (world > 1e-9).then(|| (uv / world).sqrt())
            })
            .collect()
    }

    /// The whole point of [`ice_uvs`]. A wedge inherits the block's baked
    /// texture, so unless it addresses that texture at the block's scale the
    /// frost changes size at the instant the block cracks — and it changes by
    /// a different amount on every facet, because the old mapping fitted the
    /// texture to whatever each face happened to measure.
    ///
    /// Large blocks are where this shows: a block bigger than one tile is
    /// drawn at a smaller density than a wedge of it would fit to.
    #[test]
    fn a_wedge_shows_the_pattern_at_the_size_the_block_did() {
        let half = Vector3::new(0.9, 0.7, 0.8);
        let spread = texture_spread(half);
        assert!(spread.tiles() > 1, "wanted a block larger than one tile");

        let (block, block_indices) = ice_block_mesh(half, spread);
        let wedge = crate::collision::convex_hull::cube_hull(half * 0.4);
        let (cleaved, cleaved_indices) =
            ice_hull_mesh(&PieceHull::new(&wedge, Vector3::zeros()), ice_uvs(spread));

        let want = uv_density(&block, &block_indices)[0];
        for measured in uv_density(&cleaved, &cleaved_indices) {
            assert!(
                (measured - want).abs() < 1e-3,
                "the wedge draws the pattern at {measured} per metre, the block at {want}"
            );
        }
    }

    /// Matching the *scale* leaves the pattern in a different place, which on
    /// a block that has cracked but not yet come apart is still a change: the
    /// frost jumps across a hairline crack. Reading the pattern at the wedge's
    /// place in the block it came out of removes the jump, so a cracked block
    /// is the block it was plus a crack.
    #[test]
    fn a_wedge_keeps_the_markings_it_had_inside_the_block() {
        let half = Vector3::new(0.9, 0.7, 0.8);
        let spread = texture_spread(half);
        let scale = uv_scale(spread);
        let offset = Vector3::new(0.45, 0.0, -0.4);

        let wedge = crate::collision::convex_hull::cube_hull(half * 0.4);
        let (drawn, _) = ice_hull_mesh(&PieceHull::new(&wedge, offset), ice_uvs(spread));

        let top: Vec<&Vertex> = drawn.iter().filter(|v| v.normal.y > 0.999).collect();
        assert!(!top.is_empty(), "the wedge has no upward face");
        for vertex in top {
            let inside = vertex.pos + offset;
            let want = Vector2::new(inside.z * scale + 0.5, inside.x * scale + 0.5);
            assert!(
                (vertex.tex_coords - want).norm() < 1e-5,
                "a corner at {inside:?} of the block reads {:?}, the block draws it at {want:?}",
                vertex.tex_coords
            );
        }
    }

    /// The old mapping, kept for objects whose texture is a decoration of one
    /// face rather than the substance behind it. It is recorded here because
    /// it is what a fracturable material must *not* use: two pieces of
    /// different size come out at different magnifications.
    #[test]
    fn fitting_the_texture_to_a_face_magnifies_it_on_a_small_piece() {
        let big = crate::collision::convex_hull::cube_hull(Vector3::new(0.8, 0.8, 0.8));
        let small = crate::collision::convex_hull::cube_hull(Vector3::new(0.2, 0.2, 0.2));
        let density = |hull| {
            let (vertices, indices) =
                hull_mesh(&PieceHull::new(hull, Vector3::zeros()), SurfaceUvs::Fitted);
            uv_density(&vertices, &indices)[0]
        };
        assert!(
            density(&small) > density(&big) * 3.0,
            "fitted coordinates were expected to scale with the piece"
        );
    }

    /// A brick of a dome that loses a neighbour is rebuilt from its collider,
    /// which knows where the brick sits but nothing about what it looked like.
    /// If the offset does not reach the mesh, every brick of the rebuilt dome
    /// wears identical markings — the wall of forty identical bricks that
    /// [`ice_piece_mesh`] exists to avoid, reappearing at the break.
    #[test]
    fn two_bricks_of_one_object_are_marked_differently() {
        let half = Vector3::new(0.2, 0.1, 0.15);
        let uvs = ice_uvs(PIECE_SPREAD);
        let here = ice_piece_mesh(&PiecePlacement::new(half, Vector3::zeros()), uvs);
        let there = ice_piece_mesh(&PiecePlacement::new(half, Vector3::new(0.4, 0.0, 0.0)), uvs);

        let shift = there.0[0].tex_coords - here.0[0].tex_coords;
        assert!(
            shift.norm() > 0.1,
            "the two bricks start {shift:?} apart in the pattern"
        );
        assert_eq!(here.0.len(), there.0.len(), "only the markings should move");
    }

    /// The load-bearing assumption behind [`was_cut`]: a block is a box, so
    /// its own faces point exactly along an axis in the wedge's frame, while
    /// every fracture surface is tilted off one. If the cleave rule's tilt
    /// ever came down to meet the tolerance, wedges would start chamfering
    /// their fracture surfaces in full again and the cracks would open into
    /// grooves. Checked against the real cleave rule, on real cuts.
    #[test]
    fn a_cut_face_is_never_mistaken_for_one_of_the_blocks_own() {
        let half = Vector3::new(0.3, 0.15, 0.2);
        let shape = crate::physics::ColliderShape::ConvexHull {
            hull: std::sync::Arc::new(crate::collision::convex_hull::cube_hull(half)),
        };
        let rule = ice_cleaving();
        let mut cut_faces = 0;
        let mut block_faces = 0;

        for salt in 0..200u32 {
            let hit = Vector3::new(
                ((salt % 7) as f32 / 7.0 - 0.5) * half.x,
                ((salt % 5) as f32 / 5.0 - 0.5) * half.y,
                ((salt % 3) as f32 / 3.0 - 0.5) * half.z,
            );
            let Some(pieces) = rule.cleave(&shape, hit, salt) else {
                continue;
            };
            for piece in &pieces {
                for face in &piece.hull.faces {
                    // Ground truth, which only a test can see: a face of the
                    // block is one lying in one of the block's own six face
                    // planes. Every corner must be on the *same* side, not
                    // merely at that distance from the middle — a cut running
                    // the full height of the block has all four corners on the
                    // top and bottom faces without being either of them. The
                    // wedge sits on its own centroid, so its corners go back
                    // into the block's frame first.
                    let on_the_block = (0..3).any(|axis| {
                        [1.0_f32, -1.0].iter().any(|side| {
                            face.vertex_indices.iter().all(|&i| {
                                let corner = piece.centre + piece.hull.vertices[i as usize];
                                (corner[axis] - side * half[axis]).abs() < 1e-3
                            })
                        })
                    });
                    assert_eq!(
                        !was_cut(face.normal),
                        on_the_block,
                        "face {:?} of a wedge was read wrongly",
                        face.normal
                    );
                    if on_the_block {
                        block_faces += 1;
                    } else {
                        cut_faces += 1;
                    }
                }
            }
        }
        assert!(cut_faces > 100, "only {cut_faces} fracture surfaces seen");
        assert!(block_faces > 100, "only {block_faces} block faces seen");
    }

    /// And the point of telling them apart: a broken block shows a crack, not
    /// a groove. Chamfering a wedge's fracture surfaces as deeply as its
    /// polished faces took several times as much material out of it.
    #[test]
    fn a_fracture_surface_is_barely_chamfered() {
        let half = Vector3::new(0.3, 0.15, 0.2);
        let shape = crate::physics::ColliderShape::ConvexHull {
            hull: std::sync::Arc::new(crate::collision::convex_hull::cube_hull(half)),
        };
        let pieces = ice_cleaving()
            .cleave(&shape, Vector3::new(0.1, 0.05, 0.0), 7)
            .expect("a cleaved block");

        for piece in &pieces {
            let drawn = chamfered(&piece.hull);
            let uniform = crate::collision::hull_bevel::bevel_hull(
                &piece.hull,
                extent_of(&piece.hull) * 0.5 * BEVEL_FRACTION,
            );

            for face in &piece.hull.faces {
                let before = face_area(&piece.hull, face);
                let kept = |hull: &ConvexHull| {
                    hull.faces
                        .iter()
                        .find(|f| (f.normal - face.normal).norm() < 1e-3)
                        .map_or(0.0, |f| face_area(hull, f) / before)
                };
                // A cut can leave a facet only centimetres across, and a
                // chamfer is a band of fixed width: on a face that small it
                // is most of the face whatever it is set to, so there is
                // nothing to measure there.
                let bevel = extent_of(&piece.hull) * 0.5 * BEVEL_FRACTION;
                if before < (bevel * 10.0).powi(2) {
                    continue;
                }
                if was_cut(face.normal) {
                    // The whole point: a fracture surface all but reaches the
                    // edge of the wedge, so two of them side by side leave a
                    // line rather than a channel.
                    assert!(
                        kept(&drawn) > 0.85,
                        "a fracture surface kept only {:.2} of its area",
                        kept(&drawn)
                    );
                    // And measurably more than the same face would keep if
                    // it were chamfered like a polished one, or none of this
                    // is doing anything.
                    assert!(
                        kept(&drawn) - kept(&uniform) > 0.05,
                        "a fracture surface kept {:.2} where a full chamfer leaves {:.2}",
                        kept(&drawn),
                        kept(&uniform)
                    );
                } else {
                    // While a polished face keeps a chamfer you can see. Not
                    // the full uniform one: an edge is cut back by the
                    // narrower of the two faces it joins, so a block face
                    // narrows where it runs into a fracture surface, which is
                    // the rule doing exactly what it is for.
                    assert!(
                        kept(&drawn) < 0.9,
                        "a block face kept {:.2} of its area — it is barely chamfered",
                        kept(&drawn)
                    );
                }
            }
        }
    }

    /// The area of one face of a hull.
    fn face_area(hull: &ConvexHull, face: &crate::collision::convex_hull::HullFace) -> f32 {
        let corners: Vec<Vector3<f32>> = face
            .vertex_indices
            .iter()
            .map(|&i| hull.vertices[i as usize])
            .collect();
        let mut sum = Vector3::zeros();
        for i in 1..corners.len() - 1 {
            sum += (corners[i] - corners[0]).cross(&(corners[i + 1] - corners[0]));
        }
        sum.magnitude() * 0.5
    }

    /// And the drawing must stay inside the hull the wedge collides as, or it
    /// visibly sinks into whatever it lands on.
    #[test]
    fn the_drawn_wedge_fits_inside_the_collider() {
        let half = Vector3::new(0.2, 0.15, 0.18);
        let wedge = crate::collision::convex_hull::cube_hull(half);
        let (bevelled, _) = ice_hull_mesh(
            &PieceHull::new(&wedge, Vector3::zeros()),
            SurfaceUvs::Fitted,
        );
        for vertex in &bevelled {
            for axis in 0..3 {
                assert!(
                    vertex.pos[axis].abs() <= half[axis] + 1e-4,
                    "{:?}",
                    vertex.pos
                );
            }
        }
    }

    /// The directions the faces of a mesh point in, one entry each.
    fn distinct_normals(vertices: &[Vertex]) -> Vec<Vector3<f32>> {
        let mut seen: Vec<Vector3<f32>> = Vec::new();
        for vertex in vertices {
            if !seen.iter().any(|n| (n - vertex.normal).norm() < 1e-3) {
                seen.push(vertex.normal);
            }
        }
        seen
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
