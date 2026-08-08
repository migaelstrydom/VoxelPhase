//! The two dials that decide how a shadow *reads*, laddered against each other.
//!
//! `shadows` answers "is the bias right"; this sheet answers "does the result
//! look like sunlight". Those are separate questions with separate dials:
//!
//! - **Fill** (`SceneLighting::ambient_colour`) sets how dark a shadow goes. It
//!   is a flat addition, so it lifts a shadowed surface hard and a sunlit one
//!   barely — which is exactly why it is the dial for this and the sky's
//!   irradiance is not. Raising the sky fill instead would narrow the
//!   key-to-fill ratio and flatten every surface in the frame, lit ones
//!   included.
//! - **Softness** (`ShadowVolume::pcf_radius`) sets how hard the edge is. Too
//!   hard and the frame reads as airless — no atmosphere, one light, nothing
//!   bouncing.
//!
//! They are laddered together rather than one at a time because they are not
//! independent by eye: a soft edge on a near-black shadow still reads as harsh,
//! and a lifted shadow with a razor edge still reads as cut out. Rows are fill
//! levels, columns are kernel widths, so `--columns 3` gives one row per level.
//!
//! Unlike `shadows`, this sheet is shot at a mid sun. Bias is what fails near
//! the horizon; darkness and softness need shadows in the frame to be judged
//! against, and at 7° the sun barely contributes anything for a shadow to
//! remove. Check the pair together: `shadows` for whether the shadow is in the
//! right place, this for whether it looks like one.

use nalgebra::Vector3;

use crate::core::error::EngineResult;
use crate::rendering::colour::Colour;
use crate::rendering::shadow::ShadowVolume;
use crate::rendering::visual_bench::scene::{
    SceneContext, SceneEnvironment, SceneShot, VisualScene,
};
use crate::rendering::visual_bench::scenes::shadows;

/// Sun elevation for the whole sheet.
///
/// Deliberately *not* the low sun `shadows` uses. Bias is judged near the
/// horizon because that is where it fails; darkness and softness are judged
/// here, at an elevation with real shadows in the frame to look at. Near the
/// horizon the sun contributes so little that everything is fill already and
/// the fill ladder has nothing to act on.
const ELEVATION_DEGREES: f32 = 35.0;

/// Mean ambient levels to ladder, from the near-black the feature shipped with
/// up past the shipping default.
///
/// These are *mean* channel values; [`ambient_at`] gives each its tint back.
const FILL_LEVELS: [f32; 4] = [0.037, 0.060, 0.080, 0.105];

/// PCF kernel half-widths: 3x3, 5x5, 7x7 taps.
const PCF_RADII: [u32; 3] = [1, 2, 3];

/// The hue the fill is tinted with, as a ratio between channels.
///
/// Shadowed surfaces are lit by the sky, so the fill leans blue. Held constant
/// across the ladder so that only the *level* varies down a column.
const FILL_TINT: [f32; 3] = [0.82, 0.95, 1.23];

pub struct ShadowTuning;

impl VisualScene for ShadowTuning {
    fn name(&self) -> &str {
        "shadow_tuning"
    }

    fn description(&self) -> &str {
        "Ambient fill against PCF softness at a low sun. The 'how dark, how hard' sheet."
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let camera = shadows::camera();
        let sun = shadows::sun_at(ELEVATION_DEGREES);

        let mut shots = Vec::with_capacity(FILL_LEVELS.len() * PCF_RADII.len());
        for &level in &FILL_LEVELS {
            for &radius in &PCF_RADII {
                let volume = ShadowVolume {
                    pcf_radius: radius,
                    ..ShadowVolume::default()
                };

                shots.push(
                    SceneShot::new(format!("fill {level:.3} / pcf {radius}"), camera)
                        .with_environment(
                            SceneEnvironment::default()
                                .with_sun(sun)
                                .with_ambient(ambient_at(level))
                                .with_shadow_volume(volume),
                        )
                        .with_meshes(shadows::arrangement()),
                );
            }
        }

        Ok(shots)
    }
}

/// A fill colour of the given mean level, carrying the sheet's fixed tint.
fn ambient_at(level: f32) -> Colour {
    let tint = Vector3::from(FILL_TINT);
    let scale = level / (tint.sum() / 3.0);
    Colour::new(tint.x * scale, tint.y * scale, tint.z * scale, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_ladder_hits_the_level_it_is_labelled_with() {
        // The label is the only thing tying a tile back to a number the owner
        // can paste into `SceneLighting::default`, so it has to be the truth.
        for level in FILL_LEVELS {
            let c = ambient_at(level);
            let mean = (c.r + c.g + c.b) / 3.0;
            assert!((mean - level).abs() < 1e-5, "{mean} != {level}");
        }
    }
}
