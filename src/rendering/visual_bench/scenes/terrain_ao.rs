//! Baked ambient occlusion, each subject shot twice: with it, and without.
//!
//! # Why pairs rather than a tour
//!
//! `terrain_forms` already tours this plot and is the place to judge terrain in
//! general. This sheet asks one question — *what did the occlusion bake change*
//! — and the only honest way to answer it is the same frame twice. Shot alone,
//! a darkened crevice is indistinguishable from a crevice that was always going
//! to be dark because its walls face away from the sun, which is exactly the
//! confusion that made the first version of `overhang underside` useless.
//!
//! Read it in rows: **left is AO on, right is AO off**, so run it with
//! `--columns 2`.
//!
//! ```bash
//! cargo run --bin visual_bench -- terrain_ao --out /tmp/terrain_ao.png --columns 2
//! ```
//!
//! The control is produced by flattening `ao` to `1.0` in the vertex data, not
//! by swapping shaders or disabling a pass. Both tiles therefore go through the
//! same pipeline with the same SPIR-V, and the only difference between them is
//! the number the bake wrote.
//!
//! # What each row is for
//!
//! | Row | The question it answers |
//! |---|---|
//! | `crevice` | does an acute inside corner darken |
//! | `overhang underside` | does a downward-facing surface darken |
//! | `crater` | does destruction geometry get the same treatment as generated terrain |
//! | `ridge + boulder` | do convex surfaces stay *bright* — the sign check |
//!
//! The last row is the one to distrust the sheet on. Inverting the effect would
//! darken open ground and brighten creases, and a sheet made only of concave
//! subjects would look plausible either way. If `ridge + boulder` differs
//! noticeably between its two tiles, something is wrong even if the other three
//! rows look right.
//!
//! Lighting is fixed at `terrain_forms`' sun so the two sheets can be read
//! against each other.

use nalgebra::{Point3, Vector3};

use crate::core::error::EngineResult;
use crate::rendering::colour::Colour;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneShot, VisualScene,
};
use crate::rendering::visual_bench::scenes::voxel_terrain;

/// Matches `terrain_forms`, so the two sheets are directly comparable.
const SUN_ELEVATION: f32 = 34.0;
const SUN_AZIMUTH: f32 = -155.0;

pub struct TerrainAo;

impl VisualScene for TerrainAo {
    fn name(&self) -> &str {
        "terrain_ao"
    }

    fn description(&self) -> &str {
        "Baked ambient occlusion on real terrain, each subject with it and without. Use --columns 2."
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let framings = [
            (
                "crevice",
                Point3::new(14.0, 10.5, 3.0),
                Point3::new(9.9, 2.6, 10.5),
            ),
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
        ];

        let tableau = voxel_terrain::build();

        Ok(framings
            .into_iter()
            .flat_map(|(label, eye, target)| {
                let camera = SceneCamera::looking_at(eye, target);
                [
                    SceneShot::new(label, camera)
                        .with_environment(environment())
                        .with_mesh(tableau.mesh()),
                    SceneShot::new(format!("{label} — AO off"), camera)
                        .with_environment(environment())
                        .with_mesh(tableau.mesh_without_occlusion()),
                ]
            })
            .collect())
    }
}

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
