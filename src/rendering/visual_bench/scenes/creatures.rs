//! The bestiary: a scene for each creature, and one of them all in a row.
//!
//! Each creature is the game's own sculpture, in the game's own materials
//! (`app::creatures`). A walker is stood on a ground plane and driven by
//! its own rig, watching the camera, so its eyes, lids and stride are what
//! the game draws; a creature that is one model is shown in each of the
//! states its maker sets out ([`Exhibit`]).
//!
//! ```text
//!   portrait   the whole creature, three-quarters on, watching the camera
//!   face       its head, close
//!   back       from behind
//!   walkers:   doze (asleep on its feet), strike (the blow landing), walk
//!   others:    one portrait more for each further state
//! ```

use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::animation::stilts::{Dress, Mood, StiltAnimator, StiltRigConfig};
use crate::app::creatures::{
    creature_materials, BellCarrierDef, EggWalkerDef, Exhibit, FerryFishDef, FishWalkerDef,
    FlyingFishDef, MusselDef, PeeperDef, PiperDef, RollerDef, StalkerDef,
};
use crate::character::{CharacterIntent, Grounding};
use crate::core::error::EngineResult;
use crate::model::{MeshPrimitive, Model, Transform};
use crate::rendering::colour::Colour;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};

use super::flat_ground::{ground, probe_flat_ground};

/// Frames a second a walker is driven at, and seconds it stands before a
/// shot so its gait and its lids have settled.
const RATE: f32 = 60.0;
const SETTLE_SECONDS: f32 = 1.2;

/// A creature the bench can show.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Creature {
    Peeper,
    Stalker,
    EggWalker,
    FishWalker,
    BellCarrier,
    Roller,
    Piper,
    FlyingFish,
    Mussel,
    /// The fish that ferries the player: a platform, so not in the row.
    FerryFish,
    /// All of them, in a row.
    Bestiary,
}

impl Creature {
    /// Every creature, in the order the row stands in.
    pub const ALL: [Creature; 9] = [
        Creature::Peeper,
        Creature::Stalker,
        Creature::EggWalker,
        Creature::FishWalker,
        Creature::BellCarrier,
        Creature::Roller,
        Creature::Piper,
        Creature::FlyingFish,
        Creature::Mussel,
    ];

    /// The rig and dress of one that walks.
    fn walker(self) -> Option<(StiltRigConfig, Arc<Dress>)> {
        Some(match self {
            Creature::Peeper => (PeeperDef::rig(), PeeperDef::dress()),
            Creature::Stalker => (StalkerDef::rig(), StalkerDef::dress()),
            Creature::EggWalker => (EggWalkerDef::rig(), EggWalkerDef::dress()),
            Creature::FishWalker => (FishWalkerDef::rig(), FishWalkerDef::dress()),
            Creature::BellCarrier => (BellCarrierDef::rig(), BellCarrierDef::dress()),
            _ => return None,
        })
    }

    /// The states one that does not walk is shown in.
    fn exhibits(self) -> Vec<Exhibit> {
        match self {
            Creature::Roller => RollerDef::exhibits(),
            Creature::Piper => PiperDef::exhibits(),
            Creature::FlyingFish => FlyingFishDef::exhibits(),
            Creature::Mussel => MusselDef::exhibits(),
            Creature::FerryFish => FerryFishDef::exhibits(),
            _ => Vec::new(),
        }
    }
}

/// A model's parts, each at its pose and moved `aside`, in the bestiary's
/// materials.
fn posed_meshes(model: &Model, poses: &[Transform], aside: Vector3<f32>) -> Vec<SceneMesh> {
    let materials = creature_materials();
    let mut meshes = Vec::new();
    for (index, part) in model.parts.iter().enumerate() {
        let pose = poses.get(index).cloned().unwrap_or_default();
        let matrix = pose.to_matrix();
        for mesh in &part.primitives {
            let mut vertices = mesh.vertices.clone();
            for vertex in &mut vertices {
                vertex.pos = matrix.transform_point(&Point3::from(vertex.pos)).coords + aside;
                vertex.normal = pose.rotation * vertex.normal;
            }
            meshes.push(
                SceneMesh::new(vertices, mesh.indices.clone())
                    .with_surface(materials[mesh.material.0 as usize].surface_params()),
            );
        }
    }
    meshes
}

/// Meshes shaped in a creature's own frame, set `aside`.
fn shaped_meshes(shaped: &[MeshPrimitive], aside: Vector3<f32>) -> Vec<SceneMesh> {
    let materials = creature_materials();
    shaped
        .iter()
        .map(|mesh| {
            let mut vertices = mesh.vertices.clone();
            for vertex in &mut vertices {
                vertex.pos += aside;
            }
            SceneMesh::new(vertices, mesh.indices.clone())
                .with_surface(materials[mesh.material.0 as usize].surface_params())
        })
        .collect()
}

/// A walker driven across flat ground, the way the game drives it.
///
/// The probes are answered by intersecting them with the ground plane,
/// which is the one thing the bench has to stand in for. Everything else
/// (the gait, the legs, the neck, the eyes) is the shipping code.
struct Walker {
    animator: StiltAnimator,
    /// How far it has travelled, along +Z.
    travelled: f32,
    clearance: f32,
}

impl Walker {
    fn new(rig: StiltRigConfig, dress: Arc<Dress>) -> Self {
        // The rig is drawn around a body whose origin rides one standing
        // height above the floor.
        let clearance = rig.standing_height();
        let animator = StiltAnimator::new(rig, Point3::new(0.0, clearance, 0.0), clearance, 0.0)
            .wearing(dress);
        Self {
            animator,
            travelled: 0.0,
            clearance,
        }
    }

    /// Walk forward at `speed` for `seconds`, in `mood`.
    fn run(&mut self, seconds: f32, speed: f32, mood: Mood) {
        let dt = 1.0 / RATE;
        let velocity = Vector3::new(0.0, 0.0, speed);
        for _ in 0..(seconds * RATE).round() as usize {
            self.travelled += speed * dt;
            let body = Point3::new(0.0, self.clearance, self.travelled);
            let pelvis = self.animator.pelvis_for(body);
            let contacts = probe_flat_ground(&self.animator.configure_probes(pelvis, 0.0));
            self.animator.update(
                dt,
                pelvis,
                0.0,
                velocity,
                &Grounding::on(Vector3::y()),
                &CharacterIntent {
                    direction: velocity.try_normalize(1e-4).unwrap_or_else(Vector3::zeros),
                    ..CharacterIntent::default()
                },
                &contacts,
                mood,
            );
        }
    }

    fn meshes(&self, aside: Vector3<f32>) -> Vec<SceneMesh> {
        let dress = self.animator.dress().expect("a bench walker is dressed");
        let mut meshes = posed_meshes(&dress.model, &self.animator.poses(), aside);
        let mut draped = Vec::new();
        self.animator
            .drape_into(&-aside, &UnitQuaternion::identity(), &mut draped);
        let materials = creature_materials();
        meshes.extend(draped.into_iter().map(|mesh| {
            SceneMesh::new(mesh.vertices, mesh.indices)
                .with_surface(materials[mesh.material.0 as usize].surface_params())
        }));
        meshes
    }
}

/// The light every creature is shown in: the island's low gold sun from
/// ahead and to one side, and a dim cool sky.
fn daylight() -> SceneEnvironment {
    SceneEnvironment::default()
        .with_sun(Vector3::new(0.55, 0.42, 0.72))
        .with_sun_colour(Colour::new(1.0, 0.84, 0.62, 1.0))
        .with_ambient(Colour::new(0.22, 0.24, 0.3, 1.0))
}

fn shot(
    label: &str,
    eye: Point3<f32>,
    target: Point3<f32>,
    mut meshes: Vec<SceneMesh>,
) -> SceneShot {
    meshes.push(ground());
    SceneShot::new(label, SceneCamera::looking_at(eye, target).with_fov(36.0))
        .with_environment(daylight())
        .with_meshes(meshes)
}

/// Where the camera stands for a view of something `height` tall whose
/// middle is at `middle`: `round` radians round from straight ahead of
/// it, `back` heights away.
fn stand(middle: Point3<f32>, height: f32, round: f32, back: f32) -> Point3<f32> {
    middle + Vector3::new(round.sin(), 0.22, round.cos()) * back * height
}

/// The shots of a walker.
fn walker_shots(rig: StiltRigConfig, dress: &Arc<Dress>) -> Vec<SceneShot> {
    let height = rig.body_height();
    let middle = Point3::new(0.0, 0.52 * height, 0.0);
    let head = Point3::new(0.0, height - rig.head_radius, 0.0);
    let take = |label: &str, eye: Point3<f32>, target: Point3<f32>, mood: Mood, walk: f32| {
        let mut walker = Walker::new(rig, dress.clone());
        let mood = Mood {
            watching: mood.watching.map(|_| eye),
            ..mood
        };
        walker.run(SETTLE_SECONDS, 0.0, mood);
        walker.run(walk, 1.2, mood);
        let along = Vector3::z() * walker.travelled;
        shot(
            label,
            eye + along,
            target + along,
            walker.meshes(Vector3::zeros()),
        )
    };
    // Awake, and looking at whoever is looking at it.
    let watching = Mood {
        alertness: 1.0,
        thrust: 0.0,
        watching: Some(Point3::origin()),
    };
    vec![
        take(
            "portrait",
            stand(middle, height, 0.6, 1.9),
            middle,
            watching,
            0.0,
        ),
        take(
            "face",
            stand(head, rig.head_radius * 2.0, 0.45, 2.3),
            head,
            watching,
            0.0,
        ),
        take(
            "back",
            stand(middle, height, 2.6, 1.9),
            middle,
            watching,
            0.0,
        ),
        take(
            "doze",
            stand(head, rig.head_radius * 2.0, -0.5, 2.6),
            head,
            Mood::default(),
            0.0,
        ),
        take(
            "strike",
            stand(middle, height, 1.3, 1.9),
            middle,
            Mood {
                alertness: 0.7,
                thrust: 1.0,
                watching: None,
            },
            0.0,
        ),
        take(
            "walk",
            stand(middle, height, 1.57, 2.0),
            middle,
            watching,
            2.6,
        ),
    ]
}

/// The shots of a creature that is shown as exhibits.
fn exhibit_shots(exhibits: &[Exhibit]) -> Vec<SceneShot> {
    let mut shots = Vec::new();
    for (index, exhibit) in exhibits.iter().enumerate() {
        let middle = Point3::new(0.0, exhibit.lift, 0.0);
        let meshes = || {
            let mut meshes = posed_meshes(&exhibit.model, &exhibit.poses, middle.coords);
            meshes.extend(shaped_meshes(&exhibit.shaped, middle.coords));
            meshes
        };
        let view = |label: String, round: f32, back: f32| {
            shot(
                &label,
                stand(middle, exhibit.reach, round, back),
                middle,
                meshes(),
            )
        };
        shots.push(view(exhibit.name.to_string(), 0.6, 3.4));
        if index == 0 {
            shots.push(view("face".to_string(), 0.3, 1.9));
            shots.push(view("back".to_string(), 2.6, 3.4));
        }
    }
    shots
}

/// The shots of the ferry fish: its exhibits as any creature's, then as
/// the player meets it: from a ledge above as it comes to be boarded, from
/// under it against the sky, and side on; and its mouth close to, shut and
/// in a gulp.
fn ferry_shots(exhibits: &[Exhibit]) -> Vec<SceneShot> {
    let mut shots = exhibit_shots(exhibits);
    let meshes = |exhibit: &Exhibit| {
        let middle = Vector3::new(0.0, exhibit.lift, 0.0);
        let mut meshes = posed_meshes(&exhibit.model, &exhibit.poses, middle);
        meshes.extend(shaped_meshes(&exhibit.shaped, middle));
        meshes
    };
    let Some(gliding) = exhibits.first() else {
        return shots;
    };
    let middle = Point3::new(0.0, gliding.lift, 0.0);
    for (label, eye) in [
        ("boarding", Point3::new(3.5, 5.2, -7.5)),
        ("under", Point3::new(2.5, 0.25, 4.0)),
        ("side", Point3::new(9.5, 2.2, 0.5)),
    ] {
        shots.push(shot(label, eye, middle, meshes(gliding)));
    }
    let mouth = middle + Vector3::new(0.03, -0.05, 2.15);
    let close = middle + Vector3::new(0.75, 0.15, 3.3);
    shots.push(shot("mouth_shut", close, mouth, meshes(gliding)));
    if let Some(gulping) = exhibits.iter().find(|exhibit| exhibit.name == "gulping") {
        shots.push(shot("mouth_open", close, mouth, meshes(gulping)));
    }
    shots
}

impl VisualScene for Creature {
    fn name(&self) -> &str {
        match self {
            Creature::Peeper => "peeper",
            Creature::Stalker => "stalker",
            Creature::EggWalker => "egg_walker",
            Creature::FishWalker => "fish_walker",
            Creature::BellCarrier => "bell_carrier",
            Creature::Roller => "roller",
            Creature::Piper => "piper",
            Creature::FlyingFish => "flying_fish",
            Creature::Mussel => "mussel",
            Creature::FerryFish => "ferry_fish",
            Creature::Bestiary => "bestiary",
        }
    }

    fn description(&self) -> &str {
        match self {
            Creature::Bestiary => "Every creature in a row, for their sizes against one another.",
            _ => "One creature: whole, close, from behind, and in each of its states.",
        }
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        if *self == Creature::Bestiary {
            return Ok(vec![row()]);
        }
        if *self == Creature::FerryFish {
            return Ok(ferry_shots(&self.exhibits()));
        }
        Ok(match self.walker() {
            Some((rig, dress)) => walker_shots(rig, &dress),
            None => exhibit_shots(&self.exhibits()),
        })
    }
}

/// Every creature standing in a row, a pace and a half apart.
fn row() -> SceneShot {
    const SPACING: f32 = 1.7;
    let mut meshes = Vec::new();
    let first = -0.5 * SPACING * (Creature::ALL.len() - 1) as f32;
    for (place, creature) in Creature::ALL.into_iter().enumerate() {
        let aside = Vector3::x() * (first + SPACING * place as f32);
        match creature.walker() {
            Some((rig, dress)) => {
                let mut walker = Walker::new(rig, dress);
                let awake = Mood {
                    alertness: 1.0,
                    thrust: 0.0,
                    watching: Some(Point3::new(0.0, 1.6, 12.0)),
                };
                walker.run(SETTLE_SECONDS, 0.0, awake);
                meshes.extend(walker.meshes(aside));
            }
            None => {
                if let Some(exhibit) = creature.exhibits().first() {
                    let at = aside + Vector3::y() * exhibit.lift;
                    meshes.extend(posed_meshes(&exhibit.model, &exhibit.poses, at));
                }
            }
        }
    }
    shot(
        "row",
        Point3::new(1.5, 2.0, 13.0),
        Point3::new(0.0, 1.0, 0.0),
        meshes,
    )
}
