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

use super::super::shared::bevelled_box::bevelled_box;
use super::super::shared::models::{hull_mesh, PieceHull, PiecePlacement, SurfaceUvs};
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::spawnables::shared::bevelled_box::distinct_normals;

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
}
