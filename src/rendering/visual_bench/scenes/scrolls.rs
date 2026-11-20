//! The scrolls: a page of Euclid's *Elements* for each proposition, on a
//! lawn, beside a solid to be judged against.
//!
//! The meshes and materials are the game's own (`ScrollDef::model`), so a
//! shot here is what a level draws. The scrolls are shown face on, turned
//! through their spin (so the verso and the rolls are seen), close to on
//! each figure and on a roll's end, and from across a level.

use std::f32::consts::PI;

use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

use super::objective::Drawn;
use crate::app::spawnables::{PlatonicSolidDef, Proposition, ScrollDef};
use crate::core::error::EngineResult;
use crate::geometry::PlatonicSolid;
use crate::rendering::colour::Colour;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};
use crate::rendering::visual_bench::scenes::flat_ground::ground;

/// How high the scrolls hang, m, and how far apart.
const HANG: f32 = 1.1;
const SPACING: f32 = 0.85;

/// Where the solid hangs for comparison: at the row's end.
const SOLID_AT: Vector3<f32> = Vector3::new(2.4, HANG, -0.3);

pub struct Scrolls;

impl VisualScene for Scrolls {
    fn name(&self) -> &str {
        "scrolls"
    }

    fn description(&self) -> &str {
        "The four Euclid scrolls hovering on a lawn, beside a solid, face on, turned and close to."
    }

    fn shots(&self, ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let materials = ScrollDef::materials(ctx.textures)?;
        let scrolls: Vec<(Proposition, Drawn)> = Proposition::ALL
            .iter()
            .map(|&proposition| {
                (
                    proposition,
                    Drawn::new(materials.clone(), |ids| ScrollDef::model(proposition, ids)),
                )
            })
            .collect();
        let solid = Drawn::new(
            PlatonicSolidDef::materials(PlatonicSolid::Dodecahedron),
            |ids| PlatonicSolidDef::model(PlatonicSolid::Dodecahedron, ids),
        );

        let scroll_at =
            |index: usize| Vector3::new((index as f32 - 1.5) * SPACING - 0.4, HANG, 0.0);
        // Each scroll turned `turns[i]` about the vertical.
        let meshes = |turns: [f32; 4]| -> Vec<SceneMesh> {
            let mut meshes: Vec<SceneMesh> = scrolls
                .iter()
                .enumerate()
                .flat_map(|(index, (_, drawn))| {
                    let turn = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), turns[index]);
                    let pose = Isometry3::from_parts(Translation3::from(scroll_at(index)), turn);
                    drawn.meshes(pose.to_homogeneous(), 1.0)
                })
                .collect();
            let held = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.5)
                * PlatonicSolidDef::rest(PlatonicSolid::Dodecahedron);
            let pose = Isometry3::from_parts(Translation3::from(SOLID_AT), held);
            meshes.extend(solid.meshes(pose.to_homogeneous(), 1.0));
            meshes.push(ground());
            meshes
        };

        // A low golden sun from ahead and to one side, as on the island.
        let day = SceneEnvironment::default()
            .with_sun(Vector3::new(0.5, 0.5, 0.7))
            .with_sun_colour(Colour::new(1.0, 0.82, 0.6, 1.0));
        let shot = |label: &str, eye: Point3<f32>, target: Point3<f32>, fov: f32, turns| {
            SceneShot::new(label, SceneCamera::looking_at(eye, target).with_fov(fov))
                .with_environment(day.clone())
                .with_meshes(meshes(turns))
        };

        let face_on = [0.15, -0.1, 0.1, -0.15];
        let mut shots = vec![
            shot(
                "together",
                Point3::new(0.4, 1.7, 4.6),
                Point3::new(0.3, HANG, 0.0),
                40.0,
                face_on,
            ),
            shot(
                "turned",
                Point3::new(0.4, 1.7, 4.6),
                Point3::new(0.3, HANG, 0.0),
                40.0,
                [0.7, 1.4, 2.4, PI],
            ),
            shot(
                "from_above",
                Point3::new(-0.5, 3.4, 3.4),
                Point3::new(0.0, HANG, 0.0),
                40.0,
                [0.4, -0.3, 0.9, 0.2],
            ),
            shot(
                "afar",
                Point3::new(9.0, 3.0, 12.0),
                Point3::new(0.4, HANG, 0.0),
                40.0,
                [0.6, 0.2, -0.3, 0.8],
            ),
        ];
        for (index, (proposition, _)) in scrolls.iter().enumerate() {
            let at = Point3::from(scroll_at(index));
            shots.push(shot(
                &format!("close_{proposition:?}").to_lowercase(),
                at + Vector3::new(0.12, 0.08, 0.75),
                at,
                35.0,
                face_on,
            ));
        }
        let first = scroll_at(0);
        shots.push(shot(
            "roll_end",
            Point3::from(first + Vector3::new(-0.35, 0.45, 0.3)),
            Point3::from(first + Vector3::new(-0.2, 0.13, 0.0)),
            30.0,
            [0.0, 0.0, 0.0, 0.0],
        ));
        Ok(shots)
    }
}
