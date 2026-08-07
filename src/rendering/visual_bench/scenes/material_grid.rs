//! Roughness against metallic, as a sweep.
//!
//! The scene the BRDF is actually tuned against: one sphere per cell, every
//! other variable held fixed, so the only thing that differs between two tiles
//! is the parameter being swept.

use nalgebra::{Point3, Vector3};

use crate::core::error::EngineResult;
use crate::geometry::{generate_sphere_indices, generate_sphere_vertices};
use crate::rendering::colour::Colour;
use crate::rendering::material::SurfaceParams;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};

/// Roughness values swept across the sheet, from mirror to fully diffuse.
const ROUGHNESS_STEPS: [f32; 6] = [0.05, 0.15, 0.3, 0.5, 0.75, 1.0];

/// Metallic values swept down the sheet. Only the endpoints are physically
/// meaningful; the middle value is here because it is what a badly authored
/// material tends to land on, and it should look obviously wrong.
const METALLIC_STEPS: [f32; 2] = [0.0, 1.0];

/// Sphere tessellation. High enough that the silhouette is not the thing being
/// judged.
const SEGMENTS: u32 = 48;
const RINGS: u32 = 32;

/// Base colour of every sphere. A mid-saturation warm hue shows both the
/// diffuse response and the tinting a metal applies to its highlight.
const ALBEDO: Colour = Colour {
    r: 0.72,
    g: 0.32,
    b: 0.22,
    a: 1.0,
};

pub struct MaterialGrid;

impl VisualScene for MaterialGrid {
    fn name(&self) -> &str {
        "material_grid"
    }

    fn description(&self) -> &str {
        "Roughness sweep at two metallic values. The BRDF tuning sheet."
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let vertices = generate_sphere_vertices(1.0, SEGMENTS, RINGS, ALBEDO);
        let indices = generate_sphere_indices(SEGMENTS, RINGS);

        let camera =
            SceneCamera::looking_at(Point3::new(0.0, 0.6, 3.4), Point3::new(0.0, 0.0, 0.0));

        // A sun off to one side and above, so the highlight lands where a
        // roughness change is easiest to read rather than dead centre.
        let environment = SceneEnvironment::default().with_sun(Vector3::new(-0.5, 0.6, 0.62));

        let mut shots = Vec::new();

        for metallic in METALLIC_STEPS {
            for roughness in ROUGHNESS_STEPS {
                let surface = SurfaceParams {
                    emissive: [0.0, 0.0, 0.0, 0.0],
                    surface: [roughness, metallic, 0.0, 3.0],
                };

                let label = format!("rough {:.2}  metal {:.0}", roughness, metallic);

                shots.push(
                    SceneShot::new(label, camera)
                        .with_environment(environment.clone())
                        .with_mesh(
                            SceneMesh::new(vertices.clone(), indices.clone()).with_surface(surface),
                        ),
                );
            }
        }

        Ok(shots)
    }
}
