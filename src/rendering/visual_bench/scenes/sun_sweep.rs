//! Sun elevation, as a sweep.
//!
//! Sky and lit geometry share the sun direction, so this sheet judges both at
//! once: whether the sky reads at each time of day, and whether surfaces stay
//! plausible as the light rakes across them.

use nalgebra::{Point3, Vector3};

use crate::core::error::EngineResult;
use crate::geometry::{generate_sphere_indices, generate_sphere_vertices};
use crate::rendering::colour::Colour;
use crate::rendering::material::SurfaceParams;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};

/// Sun elevations above the horizon, in degrees. Weighted towards the low end,
/// where both the sky and the shading change fastest.
const ELEVATIONS: [f32; 6] = [2.0, 8.0, 16.0, 30.0, 50.0, 75.0];

/// Azimuth held fixed so elevation is the only variable.
///
/// Chosen to put the sun in front of the camera and off to one side: this sheet
/// is half about the sun disc, so a direction behind the viewer would make it
/// judge only the shading.
const AZIMUTH_DEGREES: f32 = 160.0;

const SEGMENTS: u32 = 40;
const RINGS: u32 = 28;

pub struct SunSweep;

impl VisualScene for SunSweep {
    fn name(&self) -> &str {
        "sun_sweep"
    }

    fn description(&self) -> &str {
        "Sky and a test sphere across six sun elevations."
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let vertices =
            generate_sphere_vertices(1.0, SEGMENTS, RINGS, Colour::new(0.8, 0.78, 0.74, 1.0));
        let indices = generate_sphere_indices(SEGMENTS, RINGS);

        // Framed so the sphere sits low and most of the tile is sky — the sky
        // is half of what this sheet is for.
        let camera =
            SceneCamera::looking_at(Point3::new(0.0, 1.2, 4.2), Point3::new(0.0, 0.6, 0.0));

        let surface = SurfaceParams {
            emissive: [0.0, 0.0, 0.0, 0.0],
            surface: [0.35, 0.0, 0.0, 3.0],
            ..SurfaceParams::MATTE
        };

        Ok(ELEVATIONS
            .iter()
            .map(|&elevation| {
                let direction = sun_direction(elevation, AZIMUTH_DEGREES);
                let environment = SceneEnvironment::default().with_sun(direction);

                SceneShot::new(format!("sun {:.0} deg", elevation), camera)
                    .with_environment(environment)
                    .with_mesh(
                        SceneMesh::new(vertices.clone(), indices.clone()).with_surface(surface),
                    )
            })
            .collect())
    }
}

/// Direction from a surface towards the sun, from elevation and azimuth.
fn sun_direction(elevation_degrees: f32, azimuth_degrees: f32) -> Vector3<f32> {
    let elevation = elevation_degrees.to_radians();
    let azimuth = azimuth_degrees.to_radians();

    Vector3::new(
        elevation.cos() * azimuth.sin(),
        elevation.sin(),
        elevation.cos() * azimuth.cos(),
    )
}
