//! Blast craters, swept across voxel size and blast arrangement.
//!
//! # Why this scene exists at all
//!
//! Every other terrain sheet shares one plot, and that plot is built at
//! [`voxel_terrain::VOXEL_SIZE`] — 0.25 m, four times finer than the metre
//! voxels a level actually ships with. Meshing artefacts are measured in
//! *voxels*, so a defect two voxels wide is half a metre on that plot and two
//! and a half metres in the game. Twice now a crater has looked clean across
//! every bench sheet and blotchy on screen, and both times the difference was
//! this and nothing else.
//!
//! So voxel size is an axis here rather than a constant. Run it with
//! `--columns 4` and each row is one voxel size, coarsest first:
//!
//! ```bash
//! cargo run --bin visual_bench -- craters --out /tmp/craters.png --columns 4
//! ```
//!
//! Read down a column, not across a row. A column holds one blast arrangement
//! at three resolutions, and an artefact of the mesher grows as the voxels do
//! while a feature of the geometry stays put.
//!
//! # What each column is for
//!
//! | Column | The question it answers |
//! |---|---|
//! | `graze` | a shallow cut, where the carve surface skims the ground |
//! | `pit` | a blast centred below the surface, all wall and no rim |
//! | `overlap` | two cuts through the same voxels — the second carve reads a field the first already wrote |
//! | `cluster` | five of them, which is what a fight actually leaves behind |
//!
//! `overlap` is the one to watch. A carve is a `min` against what is already
//! there, so the second blast is the only case in the sheet where the field
//! being cut is not a plain heightfield, and it is the case a single-blast
//! test can never reach.
//!
//! # The ground is deliberately plain
//!
//! Flat, unroughened, one material. Anything dark in these tiles came from the
//! crater, because there is nothing else in frame that could have put it there
//! — which is the whole difference between this sheet and pointing a camera at
//! `terrain_forms`' crater and squinting.

use nalgebra::{Point3, Vector3};

use crate::core::error::EngineResult;
use crate::level::{Extent, MaterialLayer, Terrain, VoxelMaterialId};
use crate::rendering::colour::Colour;
use crate::rendering::material::SurfaceParams;
use crate::rendering::vertex::Vertex;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};
use crate::rendering::visual_bench::scenes::voxel_terrain;
use crate::terrain::{generate_terrain, ChunkGrid, DurabilityConfig, Segment, SegmentFrame};

/// Matches `terrain_forms` and `terrain_ao`, so all three read together.
const SUN_ELEVATION: f32 = 34.0;
const SUN_AZIMUTH: f32 = -155.0;

/// Side of the square plot each tile builds, in metres.
///
/// Big enough that the camera never sees the plot edge, whose boundary-clamped
/// gradients are their own artefact and not the one this sheet is about.
const PLOT_EXTENT: f32 = 48.0;

/// Height of the flat ground every tile is blasted out of.
const GROUND_HEIGHT: f32 = 8.0;

/// Centre of the plot, and the point every camera aims at.
const ORIGIN: Point3<f32> = Point3::new(PLOT_EXTENT * 0.5, GROUND_HEIGHT, PLOT_EXTENT * 0.5);

/// Voxel sizes to sweep, coarsest first.
///
/// The first is what levels ship with; the last is what the shared plot is
/// built at. A defect visible in row one and absent from row three is a defect
/// the rest of the bench cannot see.
const VOXEL_SIZES: [f32; 3] = [1.0, 0.5, voxel_terrain::VOXEL_SIZE];

/// One blast: where its centre sits relative to [`ORIGIN`], and how big it is.
struct Blast {
    offset: Vector3<f32>,
    radius: f32,
}

impl Blast {
    const fn new(x: f32, y: f32, z: f32, radius: f32) -> Self {
        Self {
            offset: Vector3::new(x, y, z),
            radius,
        }
    }
}

/// A named arrangement of blasts, forming one column of the sheet.
struct Arrangement {
    label: &'static str,
    blasts: &'static [Blast],
}

const ARRANGEMENTS: [Arrangement; 4] = [
    Arrangement {
        label: "graze",
        blasts: &[Blast::new(0.0, 1.5, 0.0, 4.0)],
    },
    Arrangement {
        label: "pit",
        blasts: &[Blast::new(0.0, -2.5, 0.0, 5.0)],
    },
    Arrangement {
        label: "overlap",
        blasts: &[
            Blast::new(-2.0, -0.5, 0.0, 4.0),
            Blast::new(2.0, -0.5, 1.0, 4.0),
        ],
    },
    Arrangement {
        label: "cluster",
        blasts: &[
            Blast::new(0.0, -1.0, 0.0, 3.5),
            Blast::new(-4.5, 0.5, -2.0, 3.0),
            Blast::new(4.0, 0.0, -3.5, 3.0),
            Blast::new(-3.0, -0.5, 4.5, 3.0),
            Blast::new(3.5, 1.0, 4.0, 2.5),
        ],
    },
];

pub struct Craters;

impl VisualScene for Craters {
    fn name(&self) -> &str {
        "craters"
    }

    fn description(&self) -> &str {
        "Blast craters swept across voxel size and arrangement. Use --columns 4; rows are voxel sizes."
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let environment = SceneEnvironment::default()
            .with_sun(sun_direction())
            .with_ambient(Colour::rgb(0.34, 0.38, 0.44));

        let mut shots = Vec::new();
        for voxel_size in VOXEL_SIZES {
            for arrangement in &ARRANGEMENTS {
                let mesh = blasted_plot(voxel_size, arrangement.blasts);
                shots.push(
                    SceneShot::new(
                        format!("{} @ {voxel_size} m", arrangement.label),
                        // High and near, looking down the sun's own azimuth so
                        // the crater is lit from behind the camera. A surface
                        // that comes out dark under this framing is dark
                        // because of its normal or its occlusion, not because
                        // it faces away from the light.
                        SceneCamera::looking_at(
                            ORIGIN + Vector3::new(-6.0, 13.0, -12.0),
                            ORIGIN + Vector3::new(0.0, -1.0, 0.0),
                        ),
                    )
                    .with_environment(environment.clone())
                    .with_mesh(mesh),
                );
            }
        }
        Ok(shots)
    }
}

fn sun_direction() -> Vector3<f32> {
    let elevation = SUN_ELEVATION.to_radians();
    let azimuth = SUN_AZIMUTH.to_radians();
    Vector3::new(
        elevation.cos() * azimuth.sin(),
        -elevation.sin(),
        elevation.cos() * azimuth.cos(),
    )
    .normalize()
}

/// Generate flat ground at `voxel_size`, blast it, and mesh the result.
///
/// The blasts go in through `damage_sphere` rather than through the generator's
/// carve, because that is the path a grenade takes and the two encode a cut
/// through different code.
fn blasted_plot(voxel_size: f32, blasts: &[Blast]) -> SceneMesh {
    let terrain = Terrain {
        voxel_size,
        bounds: Extent {
            min: (0.0, 0.0, 0.0),
            max: (PLOT_EXTENT, GROUND_HEIGHT + 8.0, PLOT_EXTENT),
        },
        base_height: GROUND_HEIGHT,
        material_layers: vec![
            MaterialLayer {
                depth: 1.2,
                material: VoxelMaterialId::Grass,
            },
            MaterialLayer {
                depth: 3.0,
                material: VoxelMaterialId::Dirt,
            },
            MaterialLayer {
                depth: 64.0,
                material: VoxelMaterialId::Rock,
            },
        ],
        features: Vec::new(),
        volumes: Vec::new(),
    };

    let mut grid = ChunkGrid::new(voxel_size);
    generate_terrain(
        &mut grid,
        &terrain,
        &DurabilityConfig::default(),
        &terrain.bounds.to_aabb(),
    );

    let mut segment = Segment::new("craters", SegmentFrame::identity(), grid, Vec::new());
    let mut rebuilt = Vec::new();
    segment.update(&mut rebuilt);

    // One blast at a time, each followed by a remesh, because that is how they
    // arrive in a fight: the second grenade cuts a field the first already
    // wrote, and carving both before meshing once would skip that.
    for blast in blasts {
        segment.damage_sphere(ORIGIN + blast.offset, blast.radius, u8::MAX);
        rebuilt.clear();
        segment.update(&mut rebuilt);
    }

    let mut vertices: Vec<Vertex> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();
    segment.append_render_data(&mut vertices, &mut indices);

    SceneMesh::new(vertices, indices).with_surface(SurfaceParams::MATTE)
}
