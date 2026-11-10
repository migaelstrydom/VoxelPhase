//! The fountain tower, whole and shattered.
//!
//! The meshes and materials are the game's own (`FountainDef::model`,
//! `FountainDef::shattered_model`). The shattered tower is the whole one
//! with every block that can break cleaved as a blast beside it would
//! cleave it, its wedges drawn a little apart, which is how to judge the
//! broken stone a level viewer cannot show: nothing there is blasted.

use nalgebra::{Isometry3, Point3, Vector3};

use crate::app::spawnables::{rose_stone_materials, FountainDef};
use crate::core::error::EngineResult;
use crate::model::Model;
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId};
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};
use crate::rendering::visual_bench::scenes::flat_ground::ground;

/// How far apart the two towers stand, m.
const SPACING: f32 = 5.0;
/// How far a wedge is drawn from its block's middle, against where it lies.
const SPREAD: f32 = 1.35;
/// Where the blow that shatters the tower comes from, in its frame.
const BLOW: Point3<f32> = Point3::new(2.0, 3.5, 1.0);

pub struct Fountain;

/// `model`'s meshes with its foot at `foot`, drawn in `materials`.
fn meshes(model: &Model, materials: &[Material], foot: Point3<f32>) -> Vec<SceneMesh> {
    let pose = Isometry3::translation(foot.x, foot.y, foot.z).to_homogeneous();
    model
        .parts
        .iter()
        .flat_map(|part| part.primitives.iter())
        .map(|primitive| {
            SceneMesh::new(primitive.vertices.clone(), primitive.indices.clone())
                .with_transform(pose)
                .with_surface(materials[primitive.material.0 as usize].surface_params())
        })
        .collect()
}

impl VisualScene for Fountain {
    fn name(&self) -> &str {
        "fountain"
    }

    fn description(&self) -> &str {
        "The fountain tower whole, and with its blocks cleaved and drawn apart."
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let materials = rose_stone_materials();
        let ids: Vec<MaterialId> = (0..materials.len() as u32).map(MaterialId).collect();
        let tower = FountainDef::at((0.0, 0.0));
        let whole = tower.model(&ids);
        let shattered = tower.shattered_model(&ids, BLOW, SPREAD);

        let left = Point3::new(-0.5 * SPACING, 0.0, 0.0);
        let right = Point3::new(0.5 * SPACING, 0.0, 0.0);
        // A low golden sun from ahead and to one side, as on the island.
        let environment = SceneEnvironment::default()
            .with_sun(Vector3::new(0.5, 0.5, 0.7))
            .with_sun_colour(Colour::new(1.0, 0.82, 0.6, 1.0));
        let shot = |label: &str, eye: Point3<f32>, target: Point3<f32>| {
            let mut scene = meshes(&whole, &materials, left);
            scene.extend(meshes(&shattered, &materials, right));
            scene.push(ground());
            SceneShot::new(label, SceneCamera::looking_at(eye, target).with_fov(40.0))
                .with_environment(environment.clone())
                .with_meshes(scene)
        };
        Ok(vec![
            shot(
                "both",
                Point3::new(2.0, 4.5, 15.0),
                Point3::new(0.0, 3.2, 0.0),
            ),
            shot(
                "whole",
                Point3::new(left.x + 2.5, 4.5, 8.0),
                Point3::new(left.x, 3.3, 0.0),
            ),
            shot(
                "shattered",
                Point3::new(right.x + 2.5, 4.5, 8.0),
                Point3::new(right.x, 3.3, 0.0),
            ),
            shot(
                "bulb",
                Point3::new(right.x + 1.8, 4.8, 3.2),
                Point3::new(right.x, 4.0, 0.0),
            ),
        ])
    }
}
