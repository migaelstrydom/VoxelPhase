//! A level's daylight: where its sun stands, and the colour of its light and
//! air.
//!
//! ```text
//!   level file ──▶ Daylight ──lighting()──▶ SceneLighting ──▶ Renderer::set_lighting
//!                                                              (sky, sun, fill, haze)
//! ```
//!
//! A level that writes none is lit by [`SceneLighting::default`].

use nalgebra::Vector3;
use serde::Deserialize;

use crate::rendering::colour::Colour;
use crate::rendering::frame::SceneLighting;

/// The sun and the air of a level, as its file writes them.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize)]
#[serde(default)]
pub struct Daylight {
    /// Compass bearing the sun stands on, degrees clockwise from north seen
    /// from above. North is −z and east is +x, so 90 puts the sun in the east.
    pub sun_bearing: f32,

    /// The sun's height above the horizon, degrees.
    pub sun_elevation: f32,

    /// Linear colour of the sun's light.
    pub sun_colour: (f32, f32, f32),

    /// Sun brightness multiplier.
    pub sun_intensity: f32,

    /// Linear flat fill on every surface: how dark, and what colour, a shadow
    /// goes.
    pub fill_colour: (f32, f32, f32),

    /// Distance at which the air has faded a surface 63% of the way to the sky
    /// behind it, m; `None` for no haze.
    pub visibility: Option<f32>,
}

impl Daylight {
    /// The lighting the renderer draws this daylight with.
    pub fn lighting(&self) -> SceneLighting {
        SceneLighting {
            sun_direction: self.sun_direction(),
            sun_colour: opaque(self.sun_colour),
            sun_intensity: self.sun_intensity,
            ambient_colour: opaque(self.fill_colour),
            aerial_visibility: self.visibility,
        }
    }

    /// Unit direction from a surface towards the sun.
    fn sun_direction(&self) -> Vector3<f32> {
        let bearing = self.sun_bearing.to_radians();
        let elevation = self.sun_elevation.to_radians();
        Vector3::new(
            bearing.sin() * elevation.cos(),
            elevation.sin(),
            -bearing.cos() * elevation.cos(),
        )
    }
}

impl Default for Daylight {
    /// The renderer's own default lighting, so the two cannot drift apart.
    fn default() -> Self {
        let lighting = SceneLighting::default();
        let sun = lighting.sun_direction;
        Self {
            sun_bearing: sun.x.atan2(-sun.z).to_degrees(),
            sun_elevation: sun.y.asin().to_degrees(),
            sun_colour: rgb(lighting.sun_colour),
            sun_intensity: lighting.sun_intensity,
            fill_colour: rgb(lighting.ambient_colour),
            visibility: lighting.aerial_visibility,
        }
    }
}

fn opaque((r, g, b): (f32, f32, f32)) -> Colour {
    Colour::new(r, g, b, 1.0)
}

fn rgb(colour: Colour) -> (f32, f32, f32) {
    (colour.r, colour.g, colour.b)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_daylight_is_the_renderers_default_lighting() {
        let lighting = Daylight::default().lighting();
        let expected = SceneLighting::default();

        assert!((lighting.sun_direction - expected.sun_direction).norm() < 1e-5);
        assert_eq!(lighting.sun_colour, expected.sun_colour);
        assert_eq!(lighting.sun_intensity, expected.sun_intensity);
        assert_eq!(lighting.ambient_colour, expected.ambient_colour);
        assert_eq!(lighting.aerial_visibility, expected.aerial_visibility);
    }

    #[test]
    fn bearing_runs_clockwise_from_north_and_elevation_lifts_the_sun() {
        let daylight = |sun_bearing, sun_elevation| Daylight {
            sun_bearing,
            sun_elevation,
            ..Daylight::default()
        };

        let north = daylight(0.0, 0.0).sun_direction();
        let east = daylight(90.0, 0.0).sun_direction();
        let overhead = daylight(40.0, 90.0).sun_direction();

        assert!((north - Vector3::new(0.0, 0.0, -1.0)).norm() < 1e-5);
        assert!((east - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-5);
        assert!((overhead - Vector3::y()).norm() < 1e-5);
    }
}
