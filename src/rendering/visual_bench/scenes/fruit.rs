//! The garden's giant fruit and its glazed orb, on a lawn and afloat.
//!
//! The meshes and materials are the game's own (`FruitDef::model`,
//! `GlazedOrbDef::model`, `BananaDef::model`), so a shot here is what a
//! level draws. On the lawn the sheet shows each one's form and skin; in a
//! pool the ones that float sit as deep as they float, which is how a lake
//! shows them.

use nalgebra::{Isometry3, Point3, Vector3};

use crate::app::spawnables::{BananaDef, FruitDef, FruitKind, GlazedOrbDef};
use crate::core::error::EngineResult;
use crate::model::Model;
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId};
use crate::rendering::visual_bench::pool::ScenePool;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};
use crate::rendering::visual_bench::scenes::flat_ground::ground;

/// Radius every round piece is shown at, m.
const RADIUS: f32 = 0.5;

/// Clear lawn between neighbours in the row, m.
const GAP: f32 = 0.5;

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
    /// Whether it floats, and so belongs in the pool.
    floats: bool,
}

impl Piece {
    fn new(
        materials: Vec<Material>,
        floats: bool,
        model: impl FnOnce(&[MaterialId]) -> Model,
    ) -> Self {
        let ids: Vec<MaterialId> = (0..materials.len() as u32).map(MaterialId).collect();
        Self {
            model: model(&ids),
            materials,
            floats,
        }
    }

    fn positions(&self) -> impl Iterator<Item = Vector3<f32>> + '_ {
        self.model
            .parts
            .iter()
            .flat_map(|part| part.primitives.iter())
            .flat_map(|primitive| primitive.vertices.iter().map(|vertex| vertex.pos))
    }

    /// How far it reaches to either side of its centre along the row, m.
    fn half_width(&self) -> f32 {
        self.positions().map(|p| p.x.abs()).fold(0.0, f32::max)
    }

    /// How high its centre sits when it lies on the ground, m.
    fn resting_height(&self) -> f32 {
        -self.positions().map(|p| p.y).fold(0.0, f32::min)
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
        "The giant cherry, peach, blackberry and banana and the glazed orb, on a lawn and afloat."
    }

    fn shots(&self, ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let mut pieces = Vec::new();
        for kind in FruitKind::ALL {
            pieces.push(Piece::new(
                FruitDef::materials(kind, ctx.textures)?,
                true,
                |ids| FruitDef::model(kind, RADIUS, ids),
            ));
        }
        pieces.push(Piece::new(GlazedOrbDef::materials(), true, |ids| {
            GlazedOrbDef::model(RADIUS, ids)
        }));
        let banana = BananaDef::at((0.0, 0.0, 0.0));
        pieces.push(Piece::new(
            vec![banana.material(ctx.textures)?],
            false,
            |ids| banana.model(ids[0]),
        ));

        // Each piece's place along the row, the row centred on the origin.
        let mut edge = 0.0;
        let mut places: Vec<f32> = pieces
            .iter()
            .map(|piece| {
                let place = edge + piece.half_width();
                edge = place + piece.half_width() + GAP;
                place
            })
            .collect();
        let middle = (edge - GAP) * 0.5;
        places.iter_mut().for_each(|place| *place -= middle);

        // A low golden sun from ahead and to one side, as on the island.
        let environment = SceneEnvironment::default()
            .with_sun(Vector3::new(0.5, 0.5, 0.7))
            .with_sun_colour(Colour::new(1.0, 0.82, 0.6, 1.0));
        let on_lawn = |label: &str, eye: Point3<f32>, target: Point3<f32>| {
            let mut scene: Vec<SceneMesh> = pieces
                .iter()
                .zip(&places)
                .flat_map(|(piece, &x)| piece.meshes(Point3::new(x, piece.resting_height(), 0.0)))
                .collect();
            scene.push(ground());
            SceneShot::new(label, SceneCamera::looking_at(eye, target).with_fov(40.0))
                .with_environment(environment.clone())
                .with_meshes(scene)
        };
        // A close look at one piece, from far enough to hold all of it.
        let close = |index: usize| {
            let x = places[index];
            let reach = 0.5 + 0.5 * pieces[index].half_width().max(RADIUS) / RADIUS;
            on_lawn(
                &format!("close_{index}"),
                Point3::new(x + 0.9 * reach, RADIUS + 0.75 * reach, 1.6 * reach),
                Point3::new(x, RADIUS + 0.15, 0.0),
            )
        };
        let floating = Point3::new(0.0, WATER_LEVEL + FLOATING_DEPTH * RADIUS, 0.0);
        let afloat: Vec<SceneMesh> = pieces
            .iter()
            .zip(&places)
            .filter(|(piece, _)| piece.floats)
            .flat_map(|(piece, &x)| piece.meshes(Point3::new(x, floating.y, 0.0)))
            .collect();
        let mut shots = vec![on_lawn(
            "row",
            Point3::new(1.2, 2.0, 7.5),
            Point3::new(0.0, RADIUS + 0.1, 0.0),
        )];
        shots.extend((0..pieces.len()).map(close));
        shots.push(
            SceneShot::new(
                "afloat",
                SceneCamera::looking_at(
                    Point3::new(0.0, WATER_LEVEL + 1.6, 6.5),
                    Point3::new(places[1], floating.y + 0.3, 0.0),
                )
                .with_fov(40.0),
            )
            .with_environment(environment.clone())
            .with_meshes(afloat)
            .with_water(ScenePool::new(WATER_LEVEL, WATER_FLOOR, 9.0)),
        );
        Ok(shots)
    }
}
