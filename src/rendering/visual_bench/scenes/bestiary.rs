//! The bestiary: the creatures that walk on stilts, and the ones that roll,
//! squat, fly and lie in wait, side by side and one at a time.
//!
//! Each is the game's own rig and costume or its own model
//! (`app::creatures`), the walkers settled on a ground plane in the mood
//! they hunt in, so a shot here is what the game draws. The line-ups are
//! for judging them against one another: their sizes, and whether each
//! reads as its own thing at a glance.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};

use crate::animation::stilts::{Costume, Mood, StiltRigConfig};
use crate::app::creatures::{
    sculpt_material, BellCarrierDef, EggWalkerDef, FishWalkerDef, FlyingFishDef, MusselDef,
    PiperDef, RollerDef, StalkerDef,
};
use crate::core::error::EngineResult;
use crate::model::Model;
use crate::rendering::colour::Colour;
use crate::rendering::material::MaterialId;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};

use super::flat_ground::ground;
use super::peeper::Walker;

/// Seconds a walker stands before it is drawn, so its gait has settled.
const SETTLE_SECONDS: f32 = 1.2;
/// How far apart the walkers stand in the line-up, m.
const SPACING: f32 = 1.5;

pub struct Bestiary;

/// A walker: its name, its proportions, and what it wears.
type Cast = (&'static str, StiltRigConfig, Option<Arc<Costume>>);

fn cast() -> Vec<Cast> {
    vec![
        ("peeper", StiltRigConfig::default(), None),
        (
            "stalker",
            StalkerDef::rig(),
            Some(Arc::new(StalkerDef::costume())),
        ),
        (
            "egg_walker",
            EggWalkerDef::rig(),
            Some(Arc::new(EggWalkerDef::costume())),
        ),
        (
            "fish_walker",
            FishWalkerDef::rig(),
            Some(Arc::new(FishWalkerDef::costume())),
        ),
        (
            "bell_carrier",
            BellCarrierDef::rig(),
            Some(Arc::new(BellCarrierDef::costume())),
        ),
    ]
}

/// One of the cast, settled and alert, moved `aside` along x.
fn standing(member: &Cast, aside: f32) -> SceneMesh {
    let mut walker = Walker::dressed(member.1, member.2.clone());
    let mood = Mood {
        alertness: 1.0,
        thrust: 0.0,
    };
    walker.run(SETTLE_SECONDS, 0.0, mood);
    let (vertices, indices) = walker.mesh();
    SceneMesh::new(vertices, indices).at(Vector3::new(aside, 0.0, 0.0))
}

/// A creature that is one model: its name, the model, how high its middle
/// is held off the ground, and how big it is, m.
type Sculpt = (&'static str, Model, f32, f32);

fn sculpts() -> Vec<Sculpt> {
    let paint = MaterialId(0);
    vec![
        ("roller", RollerDef::model(0.6, paint), 0.6, 0.6),
        ("piper", PiperDef::model(paint), 0.42, 0.9),
        ("flying_fish", FlyingFishDef::model(paint), 1.2, 0.9),
        ("mussel", MusselDef::model(paint, 1.0), 0.1, 0.9),
        ("mussel_shut", MusselDef::model(paint, 0.0), 0.1, 0.9),
    ]
}

/// One of the sculpts, moved `aside` along x.
fn placed(sculpt: &Sculpt, aside: f32) -> Vec<SceneMesh> {
    let surface = sculpt_material().surface_params();
    let meshes = sculpt.1.parts.iter().flat_map(|part| &part.primitives);
    meshes
        .map(|mesh| {
            SceneMesh::new(mesh.vertices.clone(), mesh.indices.clone())
                .at(Vector3::new(aside, sculpt.2, 0.0))
                .with_surface(surface)
        })
        .collect()
}

impl VisualScene for Bestiary {
    fn name(&self) -> &str {
        "bestiary"
    }

    fn description(&self) -> &str {
        "Every creature: the stilt walkers and the rest in two lines, then each alone."
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let environment = SceneEnvironment::default()
            .with_sun(Vector3::new(0.5, 0.6, 0.65))
            .with_sun_colour(Colour::new(1.0, 0.86, 0.68, 1.0))
            .with_ambient(Colour::new(0.3, 0.32, 0.38, 1.0));
        let cast = cast();
        let shot = |label: &str, eye: Point3<f32>, target: Point3<f32>, meshes: Vec<SceneMesh>| {
            SceneShot::new(label, SceneCamera::looking_at(eye, target).with_fov(40.0))
                .with_environment(environment.clone())
                .with_meshes(meshes)
        };

        let middle = 0.5 * SPACING * (cast.len() - 1) as f32;
        let mut line: Vec<SceneMesh> = cast
            .iter()
            .enumerate()
            .map(|(index, member)| standing(member, SPACING * index as f32 - middle))
            .collect();
        line.push(ground());
        let mut shots = vec![shot(
            "line_up",
            Point3::new(2.2, 1.9, 8.2),
            Point3::new(0.0, 1.1, 0.0),
            line,
        )];
        for member in &cast {
            let height = member.1.body_height();
            let back = 1.6 + 1.3 * height;
            shots.push(shot(
                member.0,
                Point3::new(0.75 * back, 0.75 * height, 0.8 * back),
                Point3::new(0.0, 0.55 * height, 0.0),
                vec![standing(member, 0.0), ground()],
            ));
        }

        let sculpts = sculpts();
        let spacing = 2.2;
        let middle = 0.5 * spacing * (sculpts.len() - 1) as f32;
        let mut line: Vec<SceneMesh> = sculpts
            .iter()
            .enumerate()
            .flat_map(|(index, sculpt)| placed(sculpt, spacing * index as f32 - middle))
            .collect();
        line.push(ground());
        shots.push(shot(
            "others",
            Point3::new(2.5, 2.4, 9.5),
            Point3::new(0.0, 0.7, 0.0),
            line,
        ));
        for sculpt in &sculpts {
            let back = 1.2 + 2.2 * sculpt.3;
            let mut meshes = placed(sculpt, 0.0);
            meshes.push(ground());
            shots.push(shot(
                sculpt.0,
                Point3::new(0.7 * back, sculpt.2 + 0.5 * back, 0.85 * back),
                Point3::new(0.0, sculpt.2, 0.0),
                meshes,
            ));
        }
        Ok(shots)
    }
}
