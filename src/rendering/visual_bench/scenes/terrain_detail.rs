//! Does the surface detail survive being walked towards, and die quietly when
//! walked away from?
//!
//! Detail normals and roughness are one system, not two (see §4.2 of
//! VISUAL_DIRECTION.md). The claim that justifies building them together is
//! that the coupling handles distance by itself: where the relief has shrunk
//! below a pixel, the variation folds into roughness and the surface goes
//! matte, with no authored LOD fade anywhere. This sheet is that claim's test.
//!
//! The same boulder from four distances at a fixed field of view:
//!
//! ```text
//!   3 m ────── 8 m ─────── 20 m ──────────────── 50 m
//!   grain      grain       grain fading          no grain,
//!   sharp      legible     into gloss            and no sparkle
//! ```
//!
//! Fixed field of view is the whole design. It is tempting to zoom in as the
//! camera pulls back so the subject stays the same size in frame — and that
//! would prove nothing, because holding apparent size constant holds the amount
//! of surface packed into each pixel constant too, which is the one variable
//! under test. The subject has to be allowed to shrink.
//!
//! # What failure looks like
//!
//! Two opposite failures, and the sheet catches both.
//!
//! - **Detail that persists too far** reads as a fixed-size pattern crawling
//!   over the surface, and in motion it sparkles: the classic specular
//!   aliasing that says "cheap renderer" louder than a missing effect ever
//!   does. In a still frame it shows as pixel-level speckle in the far tiles.
//! - **Detail that dies too early** leaves a mid-distance cliff looking like
//!   untextured plastic, which is where terrain started.
//!
//! A still image cannot show crawling. The far tiles being *smooth* is
//! necessary evidence, not sufficient — the sufficient test is moving the
//! camera, which this harness cannot do.
//!
//! ```bash
//! cargo run --bin visual_bench -- terrain_detail --out /tmp/terrain_detail.png --columns 4
//! ```

use nalgebra::Vector3;

use crate::core::error::EngineResult;
use crate::rendering::colour::Colour;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneShot, VisualScene,
};
use crate::rendering::visual_bench::scenes::voxel_terrain;
use crate::terrain::surface;

/// Matches the other terrain sheets, so they can be read against each other.
const SUN_ELEVATION: f32 = 34.0;
const SUN_AZIMUTH: f32 = -155.0;

/// Distances to view the subject from, in metres. Spread roughly
/// logarithmically, because what matters is the ratio between them: each step
/// is about one more mip level of the surface texture.
const DISTANCES: [f32; 4] = [3.0, 8.0, 20.0, 50.0];

/// Shared by every tile, so the only thing that changes down the row is how
/// much surface each pixel covers.
const FIELD_OF_VIEW: f32 = 45.0;

/// Where the camera stands relative to the subject, normalised at render time.
/// Off to one side and slightly above, so the boulder is lit across its relief
/// rather than flat-on.
const VIEW_DIRECTION: Vector3<f32> = Vector3::new(-0.75, 0.45, -1.0);

pub struct TerrainDetail;

impl VisualScene for TerrainDetail {
    fn name(&self) -> &str {
        "terrain_detail"
    }

    fn description(&self) -> &str {
        "The same boulder at four distances, to check detail fades into gloss rather than sparkling. Use --columns 4."
    }

    fn shots(&self, ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let tableau = voxel_terrain::build();
        let surface_texture = surface::create_surface_texture(ctx.textures)?;
        // A convex form with bulk, rather than a point on a flat face: at
        // 50 m the subject has to still be findable in the frame.
        let subject = voxel_terrain::BOULDER;
        let offset = VIEW_DIRECTION.normalize();

        Ok(DISTANCES
            .iter()
            .map(|distance| {
                let eye = subject + offset * *distance;

                SceneShot::new(
                    format!("{distance} m"),
                    SceneCamera::looking_at(eye, subject).with_fov(FIELD_OF_VIEW),
                )
                .with_environment(environment())
                .with_mesh(tableau.mesh(&surface_texture))
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

#[cfg(test)]
mod tests {
    use super::*;

    /// The experiment is only valid if each tile really does pack more surface
    /// into a pixel than the one before it — that is the quantity the detail
    /// fade responds to. An earlier version of this scene scaled the field of
    /// view to hold the subject at a constant apparent size, which held surface
    /// per pixel constant too and made all four tiles identical.
    #[test]
    fn each_tile_packs_more_surface_into_a_pixel_than_the_last() {
        for pair in DISTANCES.windows(2) {
            assert!(
                pair[1] > pair[0] * 1.5,
                "{} m to {} m is too small a step to change the mip level",
                pair[0],
                pair[1]
            );
        }
    }
}
