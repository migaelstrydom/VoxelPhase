//! The turtle, out of the water and swimming in it.
//!
//! The meshes are the game's own (`TurtleDef::model`), drawn in its own
//! materials, so a shot here is what island_sea draws. Out of the water the
//! sheet shows its form from the front, the side and above; in a pool it
//! sits as deep as its swimmer keeps it, which is how it is seen from the
//! shore.

use nalgebra::{Isometry3, Point3, Vector3};

use crate::app::creatures::TurtleDef;
use crate::core::error::EngineResult;
use crate::rendering::colour::Colour;
use crate::rendering::material::{MaterialId, SurfaceParams};
use crate::rendering::visual_bench::pool::ScenePool;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};
use crate::rendering::visual_bench::scenes::flat_ground::ground;

/// Height of its centre over the ground when shown out of the water, m:
/// enough to clear its plastron.
const STANDING_HEIGHT: f32 = 0.12;

/// Its centre's depth under the surface while it swims, m: the swimmer's
/// clearance.
const SWIMMING_DEPTH: f32 = 0.3;

/// The pool's surface and floor, m.
const WATER_LEVEL: f32 = 1.0;
const WATER_FLOOR: f32 = 0.0;

pub struct Turtle;

impl VisualScene for Turtle {
    fn name(&self) -> &str {
        "turtle"
    }

    fn description(&self) -> &str {
        "The turtle out of the water from three sides, and swimming as seen from the shore."
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let surfaces: Vec<SurfaceParams> = TurtleDef::materials()
            .iter()
            .map(|material| material.surface_params())
            .collect();
        let ids: Vec<MaterialId> = (0..surfaces.len() as u32).map(MaterialId).collect();
        let model = TurtleDef::model(&ids);
        let meshes = |pose: Isometry3<f32>| -> Vec<SceneMesh> {
            model
                .parts
                .iter()
                .flat_map(|part| part.primitives.iter())
                .map(|primitive| {
                    SceneMesh::new(primitive.vertices.clone(), primitive.indices.clone())
                        .with_transform(pose.to_homogeneous())
                        .with_surface(surfaces[primitive.material.0 as usize])
                })
                .collect()
        };

        // A low golden sun from ahead and to one side.
        let environment = SceneEnvironment::default()
            .with_sun(Vector3::new(0.5, 0.55, 0.65))
            .with_sun_colour(Colour::new(1.0, 0.86, 0.7, 1.0));
        let standing = Isometry3::translation(0.0, STANDING_HEIGHT, 0.0);
        let centre = Point3::new(0.0, STANDING_HEIGHT + 0.1, 0.0);
        let on_ground = |label: &str, eye: Point3<f32>| {
            let mut scene = meshes(standing);
            scene.push(ground());
            SceneShot::new(label, SceneCamera::looking_at(eye, centre).with_fov(40.0))
                .with_environment(environment.clone())
                .with_meshes(scene)
        };
        let swimming_at = Point3::new(0.0, WATER_LEVEL - SWIMMING_DEPTH, 0.0);
        let swimming = |label: &str, eye: Point3<f32>| {
            SceneShot::new(
                label,
                SceneCamera::looking_at(eye, swimming_at + Vector3::y() * 0.2).with_fov(40.0),
            )
            .with_environment(environment.clone())
            .with_meshes(meshes(Isometry3::translation(0.0, swimming_at.y, 0.0)))
            .with_water(ScenePool::new(WATER_LEVEL, WATER_FLOOR, 7.0))
        };
        Ok(vec![
            on_ground("front_quarter", Point3::new(0.9, 0.6, 1.3)),
            on_ground("side", Point3::new(1.6, 0.35, 0.0)),
            on_ground("above", Point3::new(-0.6, 1.5, -0.7)),
            swimming("swimming_near", Point3::new(1.2, WATER_LEVEL + 0.8, 1.6)),
            swimming("from_the_shore", Point3::new(3.5, WATER_LEVEL + 1.7, 4.5)),
        ])
    }
}
