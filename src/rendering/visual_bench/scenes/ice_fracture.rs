//! A large block of ice, whole and cracked, drawn side by side.
//!
//! One question: does the substance stay the same size when the block breaks?
//!
//! A fragment inherits the block's baked texture, so it has to address that
//! texture at the block's scale. If it does not, the frost changes
//! magnification at the moment of the break — and by a different amount on
//! every facet, because a mapping fitted to a face is at the mercy of whatever
//! that face happens to measure. Large blocks are where it shows: a block
//! bigger than one tile is drawn at a lower density than any wedge of it would
//! be fitted to, so the pattern gets coarser exactly where there is most of it
//! to see.
//!
//! `whole` and `cracked` are the same slab from the same camera, and the
//! comparison is direct: put them side by side and the blobs must be the same
//! size. `fitted` is the failure, kept deliberately — it is the same cracked
//! slab drawn the way a piece is drawn when nobody told it what scale the
//! material is, and it is what this sheet exists to tell apart from `cracked`.
//!
//! Everything here is the game's own: the game's mesh, the game's chamfer, the
//! game's cleave rule. What the sheet shows is what a block in a level does.

use nalgebra::{Point3, Vector2, Vector3};

use crate::app::spawnables::shared::models::{PieceHull, SurfaceUvs};
use crate::app::spawnables::{
    ice_block_mesh, ice_cleaving, ice_hull_mesh, ice_texture_spread, ice_uvs,
};
use crate::cleave::{CleavePiece, CleaveRule};
use crate::core::error::EngineResult;
use crate::physics::ColliderShape;
use crate::rendering::colour::Colour;
use crate::rendering::pattern;
use crate::rendering::pattern::Spread;
use crate::rendering::substance;
use crate::rendering::vertex::Vertex;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};
use crate::resources::textures::TextureHandle;

/// Half-extents of the slab: the paving-slab proportions the pattern was
/// reported wrong on, and wide enough that the spread gives it more than one
/// tile — which is the case the two mappings disagree about.
const SLAB: Vector3<f32> = Vector3::new(2.5, 0.5, 2.5);

/// Half-width of the ground plane.
const GROUND_HALF_EXTENT: f32 = 12.0;

const TEXTURE_SIZE: u32 = 256;

/// Fixed, so the sheet shows the same wedges every time it is rendered.
const CLEAVE_SALT: u32 = 11;

/// How many wedges to break the slab into. More than the rule's own three,
/// because a slab that has been hit twice is the state the report came from
/// and the small pieces are where a mapping fitted per face goes furthest
/// wrong.
const WEDGES: u32 = 5;

pub struct IceFracture;

impl VisualScene for IceFracture {
    fn name(&self) -> &str {
        "ice_fracture"
    }

    fn description(&self) -> &str {
        "A large ice slab whole and cracked. Does the frost stay the same size?"
    }

    fn shots(&self, ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let spread = slab_spread();
        let size = spread.texture_size(TEXTURE_SIZE);
        let texture = ctx.textures.create_from_rgba(
            size,
            size,
            &pattern::ICE.bake_spread(TEXTURE_SIZE, &substance::ICE.palette, 7, spread),
            true,
        )?;

        let environment = SceneEnvironment::default().with_sun(Vector3::new(-0.55, 0.55, 0.63));
        let camera = || {
            SceneCamera::looking_at(Point3::new(1.3, 2.2, 2.4), Point3::new(0.0, SLAB.y, 0.0))
                .with_fov(46.0)
        };

        let whole = {
            let (vertices, indices) = ice_block_mesh(SLAB, spread);
            vec![ground(), ice(&texture, vertices, indices, Vector3::zeros())]
        };
        let cracked = wedges(&texture, ice_uvs(spread));
        let fitted = wedges(&texture, SurfaceUvs::Fitted);

        Ok(vec![
            SceneShot::new("whole", camera())
                .with_environment(environment.clone())
                .with_meshes(whole),
            SceneShot::new("cracked", camera())
                .with_environment(environment.clone())
                .with_meshes(cracked),
            SceneShot::new("fitted", camera())
                .with_environment(environment.clone())
                .with_meshes(fitted),
        ])
    }
}

/// The slab cleaved by the game's own rule, each wedge drawn at `uvs`.
fn wedges(texture: &TextureHandle, uvs: SurfaceUvs) -> Vec<SceneMesh> {
    let shape = ColliderShape::Box { half_extents: SLAB };
    let hit = Vector3::new(0.0, SLAB.y, 0.0);
    let rule = CleaveRule {
        pieces: WEDGES,
        ..ice_cleaving()
    };
    let pieces = rule.cleave(&shape, hit, CLEAVE_SALT).unwrap_or_default();

    let mut meshes = vec![ground()];
    for CleavePiece { hull, centre } in pieces {
        let (vertices, indices) = ice_hull_mesh(&PieceHull::new(&hull, centre), uvs);
        meshes.push(ice(texture, vertices, indices, centre));
    }
    meshes
}

/// The spread the game would give a slab this size, so the sheet judges the
/// texture a real block wears.
fn slab_spread() -> Spread {
    ice_texture_spread(SLAB)
}

/// One piece of ice, with the game's substance on it.
fn ice(
    texture: &TextureHandle,
    vertices: Vec<Vertex>,
    indices: Vec<u32>,
    position: Vector3<f32>,
) -> SceneMesh {
    SceneMesh::new(vertices, indices)
        .at(position + Vector3::new(0.0, SLAB.y, 0.0))
        .with_texture(texture.clone())
        .with_surface(substance::ICE.material(texture.clone()).surface_params())
}

/// A matte quad at `y = 0` for the slab to sit on and shadow.
fn ground() -> SceneMesh {
    let e = GROUND_HALF_EXTENT;
    let colour = Colour::new(0.34, 0.36, 0.34, 1.0).to_vec4();
    let normal = Vector3::y();

    let corners = [
        (Vector3::new(-e, 0.0, -e), [0.0, 0.0]),
        (Vector3::new(e, 0.0, -e), [1.0, 0.0]),
        (Vector3::new(e, 0.0, e), [1.0, 1.0]),
        (Vector3::new(-e, 0.0, e), [0.0, 1.0]),
    ];

    let vertices: Vec<Vertex> = corners
        .iter()
        .map(|(position, uv)| Vertex {
            pos: *position,
            color: colour,
            tex_coords: Vector2::new(uv[0], uv[1]),
            normal,
            ao: 1.0,
        })
        .collect();

    SceneMesh::new(vertices, vec![0, 1, 2, 0, 2, 3])
}
