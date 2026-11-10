//! The garden's giant fruit and its glazed orb, on a lawn and afloat.
//!
//! The meshes and materials are the game's own (`FruitDef::model`,
//! `GlazedOrbDef::model`), so a shot here is what a level draws. On the
//! lawn the sheet shows each one's form and skin; in a pool they sit as deep
//! as they float, which is how a lake shows them.

use nalgebra::{Isometry3, Point3, Vector3};

use crate::app::spawnables::{FruitDef, FruitKind, GlazedOrbDef};
use crate::core::error::EngineResult;
use crate::model::Model;
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId};
use crate::rendering::visual_bench::pool::ScenePool;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};
use crate::rendering::visual_bench::scenes::flat_ground::ground;

/// Radius every piece is shown at, m.
const RADIUS: f32 = 0.5;

/// Distance between neighbours in the row, m.
const SPACING: f32 = 1.5;

/// The pool's surface and floor, m.
const WATER_LEVEL: f32 = 1.5;
const WATER_FLOOR: f32 = 0.0;

/// How far under the surface a floating piece's centre sits, as a share of
/// its radius: about a third of it under water.
const FLOATING_DEPTH: f32 = -0.25;

pub struct Fruit;

/// One piece to show: its model and the materials it is drawn in.
struct Piece {
    model: Model,
    materials: Vec<Material>,
}

impl Piece {
    fn new(materials: Vec<Material>, model: impl FnOnce(&[MaterialId]) -> Model) -> Self {
        let ids: Vec<MaterialId> = (0..materials.len() as u32).map(MaterialId).collect();
        Self {
            model: model(&ids),
            materials,
        }
    }

    /// Its meshes with its centre at `centre`.
    fn meshes(&self, centre: Point3<f32>) -> Vec<SceneMesh> {
        let pose = Isometry3::translation(centre.x, centre.y, centre.z).to_homogeneous();
        self.model
            .parts
            .iter()
            .flat_map(|part| part.primitives.iter())
            .map(|primitive| {
                let material = &self.materials[primitive.material.0 as usize];
                let mesh = SceneMesh::new(primitive.vertices.clone(), primitive.indices.clone())
                    .with_transform(pose)
                    .with_surface(material.surface_params());
                match &material.diffuse_texture {
                    Some(texture) => mesh.with_texture(texture.clone()),
                    None => mesh,
                }
            })
            .collect()
    }
}

impl VisualScene for Fruit {
    fn name(&self) -> &str {
        "fruit"
    }

    fn description(&self) -> &str {
        "The giant cherry, peach and blackberry and the glazed orb, on a lawn and afloat."
    }

    fn shots(&self, ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let mut pieces = Vec::new();
        for kind in FruitKind::ALL {
            pieces.push(Piece::new(
                FruitDef::materials(kind, ctx.textures)?,
                |ids| FruitDef::model(kind, RADIUS, ids),
            ));
        }
        pieces.push(Piece::new(GlazedOrbDef::materials(), |ids| {
            GlazedOrbDef::model(RADIUS, ids)
        }));

        let place = |index: usize| (index as f32 - (pieces.len() - 1) as f32 * 0.5) * SPACING;
        let row = |height: f32| -> Vec<SceneMesh> {
            pieces
                .iter()
                .enumerate()
                .flat_map(|(index, piece)| piece.meshes(Point3::new(place(index), height, 0.0)))
                .collect()
        };

        // A low golden sun from ahead and to one side, as on the island.
        let environment = SceneEnvironment::default()
            .with_sun(Vector3::new(0.5, 0.5, 0.7))
            .with_sun_colour(Colour::new(1.0, 0.82, 0.6, 1.0));
        let on_lawn = |label: &str, eye: Point3<f32>, target: Point3<f32>| {
            let mut scene = row(RADIUS);
            scene.push(ground());
            SceneShot::new(label, SceneCamera::looking_at(eye, target).with_fov(40.0))
                .with_environment(environment.clone())
                .with_meshes(scene)
        };
        let close = |index: usize| {
            let x = place(index);
            on_lawn(
                &format!("close_{index}"),
                Point3::new(x + 0.9, RADIUS + 0.75, 1.6),
                Point3::new(x, RADIUS + 0.15, 0.0),
            )
        };
        let floating = Point3::new(0.0, WATER_LEVEL + FLOATING_DEPTH * RADIUS, 0.0);
        let mut shots = vec![on_lawn(
            "row",
            Point3::new(1.2, 1.6, 5.2),
            Point3::new(0.0, RADIUS + 0.1, 0.0),
        )];
        shots.extend((0..pieces.len()).map(close));
        shots.push(
            SceneShot::new(
                "afloat",
                SceneCamera::looking_at(
                    Point3::new(1.5, WATER_LEVEL + 1.4, 5.5),
                    floating + Vector3::y() * 0.3,
                )
                .with_fov(40.0),
            )
            .with_environment(environment.clone())
            .with_meshes(row(floating.y))
            .with_water(ScenePool::new(WATER_LEVEL, WATER_FLOOR, 9.0)),
        );
        Ok(shots)
    }
}
