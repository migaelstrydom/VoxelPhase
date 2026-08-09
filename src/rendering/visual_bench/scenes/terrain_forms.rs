//! A tour of real marching-cubes terrain, one feature per tile.
//!
//! # Why a tour rather than a sweep or a tableau
//!
//! The other sheets vary a parameter across their shots — sun elevation, PCF
//! radius, roughness — because being in twelve places in parameter space at
//! once is the thing a windowed viewer cannot do. This one holds every
//! parameter fixed and varies the *camera* instead, over a single fixed piece
//! of terrain.
//!
//! That is deliberate, and it is what the subject demands. The features being
//! judged — a 1 m crevice, a 0.7 m arch, a crater rim — are decimetre-scale
//! detail. One wide tableau would render each of them a few pixels across and
//! report nothing; the sheet would be trusted and it would be useless, which is
//! precisely the failure mode `VISUAL_HANDOFF.md` records twice. So the terrain
//! is one composition and the sheet is a set of close reads of it, each framed
//! so its feature fills the tile.
//!
//! The last tile is the establishing wide, for context and for judging the
//! whole frame's tonality. It is last rather than first because the close reads
//! are what the sheet is for.
//!
//! # What to look at
//!
//! Read the *thin* tile first, on the same logic as the picket row in
//! `shadows`: the pillar is 0.9 m across and the arch 0.7 m, and they are the
//! only subjects here that a filter, a bake or a mesh simplification can erase
//! outright. Everything else is metres thick and will survive looking roughly
//! right no matter what has broken.
//!
//! Then, when ambient occlusion lands (`BAKED_AO_DESIGN.md`), the sheet answers
//! its four questions directly: does the crevice darken, does the overhang's
//! underside darken, does the ridge stay bright, and does the crater — carved
//! by the destruction path rather than the generator — pick occlusion up at
//! all.
//!
//! Tile by tile:
//!
//! | Tile | Subject | The question it answers |
//! |---|---|---|
//! | `thin: pillar + arch` | a 0.9 m pillar and a 0.7 m arch on open ground | has anything erased fine features, in the mesh or in the shadow filter |
//! | `crevice` | a 1 m slot 3.5 m deep, looked into from above | does an acute inside corner darken |
//! | `overhang underside` | the lip from below, sky behind it | does a downward-facing surface darken |
//! | `crater` | the `damage_sphere` bowl, rim and floor | does destruction geometry get the same treatment as generated terrain |
//! | `ridge + boulder` | two convex forms | do convex surfaces stay *bright* — the sign check |
//! | `slope sweep` | the cliff, flat ground to ~74° in one surface | how much of the slope range shading actually resolves |
//! | `wide` | the whole plot | tonality and the frame as a whole |
//!
//! One thing the sheet already says, before any occlusion work: on `slope
//! sweep`, the shading barely separates flat ground from a 70° face. The sun
//! and the sky hemisphere between them light the two almost equally, so a
//! continuous change of slope arrives as a continuous change of almost nothing.
//! That is the same complaint as the flat crevices, one level up, and it is the
//! case `VISUAL_DIRECTION.md` §4's slope-driven terrain material exists for.

use nalgebra::{Point3, Vector3};

use crate::core::error::EngineResult;
use crate::rendering::colour::Colour;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneShot, VisualScene,
};
use crate::rendering::visual_bench::scenes::voxel_terrain;
use crate::terrain::surface;

/// Sun elevation for the whole sheet, in degrees.
///
/// Mid-height rather than low: this sheet is about the *shape* of terrain, and
/// a raking sun buries half of every feature in its own shadow. The shadow
/// sheets are where a low sun belongs. Low enough, though, that a mid slope
/// catches more light than flat ground does — from a high sun the whole slope
/// range, flat through vertical, arrives within a few percent of one tone.
const SUN_ELEVATION: f32 = 34.0;

/// Sun azimuth, in degrees.
///
/// Faces the cliff rather than standing behind it, so the slope sweep is lit
/// across its whole range instead of being one flat silhouette. The crevice
/// runs the other way, so its walls take the light edge-on and the slot still
/// reads as a slot; the overhang's underside faces down and stays dark at any
/// azimuth.
const SUN_AZIMUTH: f32 = -155.0;

pub struct TerrainForms;

impl VisualScene for TerrainForms {
    fn name(&self) -> &str {
        "terrain_forms"
    }

    fn description(&self) -> &str {
        "Real marching-cubes terrain: crevice, overhang, ridge, crater, slope sweep, thin spires."
    }

    fn shots(&self, ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let framings = [
            // Off to the +x side rather than square on. The pillar and the arch
            // share an x, so any camera looking along z stacks one behind the
            // other; separating them means viewing across the line between them.
            (
                "thin: pillar + arch",
                Point3::new(34.0, 5.8, 4.0),
                Point3::new(25.0, 3.2, 10.5),
            ),
            (
                "crevice",
                Point3::new(14.0, 10.5, 3.0),
                Point3::new(9.9, 2.6, 10.5),
            ),
            // Under the lip and close to its own axis, so the ground it shades
            // and the cliff it springs from are both in frame. Viewed from far
            // off to the side the lip reads as a slab floating against sky, and
            // "occluded" becomes indistinguishable from "facing away from the
            // sun" — which is the one question this tile exists to answer.
            (
                "overhang underside",
                Point3::new(16.0, 3.0, 6.5),
                Point3::new(
                    voxel_terrain::OVERHANG.x - 0.5,
                    voxel_terrain::OVERHANG.y + 3.6,
                    voxel_terrain::OVERHANG.z + 0.6,
                ),
            ),
            ("crater", Point3::new(5.0, 7.2, 2.0), voxel_terrain::CRATER),
            (
                "ridge + boulder",
                Point3::new(23.8, 7.4, 1.4),
                Point3::new(17.6, 3.2, 8.6),
            ),
            // Square on and close, from the low ground, in the lane between the
            // ridge and the boulder. The face is 6 m of rise and wants to fill
            // the tile: framed from further back the flat ground in front of it
            // takes most of the frame and the slope range arrives too small to
            // read. The eye stays below the plateau, or the shot becomes a view
            // down onto the top with the face hidden under its own edge.
            (
                "slope sweep",
                Point3::new(19.0, 7.5, 11.5),
                Point3::new(15.0, 4.5, 21.0),
            ),
            (
                "wide",
                Point3::new(-6.0, 15.0, -8.0),
                Point3::new(15.0, 4.0, 13.0),
            ),
        ];

        let tableau = voxel_terrain::build();
        let surface_texture = surface::create_surface_texture(ctx.textures)?;

        Ok(framings
            .into_iter()
            .map(|(label, eye, target)| {
                SceneShot::new(label, SceneCamera::looking_at(eye, target))
                    .with_environment(environment())
                    .with_mesh(tableau.mesh(&surface_texture))
            })
            .collect())
    }
}

/// One fixed lighting environment for every tile, so any difference between
/// tiles is the terrain rather than the light.
fn environment() -> SceneEnvironment {
    let elevation = SUN_ELEVATION.to_radians();
    let azimuth = SUN_AZIMUTH.to_radians();
    SceneEnvironment::default()
        .with_sun(Vector3::new(
            elevation.cos() * azimuth.sin(),
            elevation.sin(),
            elevation.cos() * azimuth.cos(),
        ))
        .with_sun_colour(Colour::new(1.0, 0.96, 0.88, 1.0))
}
