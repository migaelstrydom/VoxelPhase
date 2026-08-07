//! The game's own palette, for judging exposure and saturation.
//!
//! The other scenes are built from muted, mid-value albedos, which is a
//! comfortable place for a renderer to sit and a misleading one to tune in: a
//! lighting change can multiply scene radiance several times over without any
//! of them looking wrong. The level content is not like that. It is saturated
//! primaries at high value — grass green, primary blue, sign yellow — and those
//! are what clip, and what go electric when a bright saturated colour is
//! tonemapped with hue preservation.
//!
//! So this scene stands the worst-behaved colours in the game on the brightest
//! surface in the game. If exposure and saturation read correctly here they
//! will read correctly on the muted scenes; the reverse does not hold.

use nalgebra::{Point3, Vector3};

use crate::core::error::EngineResult;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneShot, VisualScene,
};
use crate::rendering::visual_bench::scenes::level_props::arrangement;

pub struct Palette;

impl VisualScene for Palette {
    fn name(&self) -> &str {
        "palette"
    }

    fn description(&self) -> &str {
        "Saturated level colours on bright terrain. The exposure and clipping check."
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let environment = SceneEnvironment::default().with_sun(Vector3::new(-0.42, 0.68, 0.6));

        // An eye-level view matching how the game is actually seen, and a
        // close one where a single saturated surface fills most of the frame —
        // the case where oversaturation is most obvious and least escapable.
        let cameras = [
            (
                "eye level",
                SceneCamera::looking_at(Point3::new(0.0, 2.6, 11.0), Point3::new(0.0, 1.2, 0.0)),
            ),
            (
                "close",
                SceneCamera::looking_at(Point3::new(-2.2, 1.4, 4.4), Point3::new(0.0, 1.0, 0.0))
                    .with_fov(40.0),
            ),
        ];

        Ok(cameras
            .into_iter()
            .map(|(label, camera)| {
                SceneShot::new(label, camera)
                    .with_environment(environment.clone())
                    .with_meshes(arrangement())
            })
            .collect())
    }
}
