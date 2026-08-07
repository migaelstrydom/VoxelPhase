//! Sun warmth against tonemap hue preservation, on level content.
//!
//! Two dials that between them decide whether a frame reads as sunlit or as
//! clinical, and which cannot sensibly be judged apart.
//!
//! Warming the sun separates key from fill: the sky fill stays cool, so lit
//! faces go warm and shaded faces go blue, and the surface gets colour depth
//! that a neutral key cannot give it. Hue preservation decides what happens to
//! a saturated colour as it brightens — hold chroma and it stays vivid all the
//! way up, which on already-saturated content reads as electric; let it bleach
//! and highlights roll towards white the way film does.
//!
//! Read the sheet down a column to see warmth alone, across a row to see
//! bleaching alone, and diagonally for the interaction: a warm key pushes more
//! of the frame up the curve, so it makes the hue-preservation choice matter
//! more, not less.

use nalgebra::{Point3, Vector3};

use crate::core::error::EngineResult;
use crate::rendering::colour::Colour;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneShot, VisualScene,
};
use crate::rendering::visual_bench::scenes::level_props::arrangement;

/// Sun tints, from the current near-white through to a low afternoon key.
const SUN_COLOURS: [(&str, Colour); 3] = [
    ("neutral", Colour::new(1.0, 0.97, 0.9, 1.0)),
    ("warm", Colour::new(1.0, 0.89, 0.7, 1.0)),
    ("warmer", Colour::new(1.0, 0.82, 0.56, 1.0)),
];

/// Endpoints and midpoint of the tonemap's hue dial.
const HUE_PRESERVATION: [(&str, f32); 3] = [("hue 1.0", 1.0), ("hue 0.5", 0.5), ("hue 0.0", 0.0)];

pub struct GradeSweep;

impl VisualScene for GradeSweep {
    fn name(&self) -> &str {
        "grade_sweep"
    }

    fn description(&self) -> &str {
        "Sun warmth x tonemap hue preservation on level content. The clinical-look check."
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        // One camera for every tile: the whole point is that nothing varies but
        // the two dials, so the sheet can be compared cell against cell.
        let camera =
            SceneCamera::looking_at(Point3::new(0.0, 2.6, 11.0), Point3::new(0.0, 1.2, 0.0));

        let mut shots = Vec::with_capacity(SUN_COLOURS.len() * HUE_PRESERVATION.len());

        for (sun_label, sun_colour) in SUN_COLOURS {
            for (hue_label, hue_preservation) in HUE_PRESERVATION {
                let environment = SceneEnvironment::default()
                    .with_sun(Vector3::new(-0.42, 0.68, 0.6))
                    .with_sun_colour(sun_colour)
                    .with_hue_preservation(hue_preservation);

                shots.push(
                    SceneShot::new(format!("{}  {}", sun_label, hue_label), camera)
                        .with_environment(environment)
                        .with_meshes(arrangement()),
                );
            }
        }

        Ok(shots)
    }
}
