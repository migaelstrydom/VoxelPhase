//! The peeper, and the four faces it pulls.
//!
//! A rig is a thing you look at, and no numeric check catches a lid that
//! slides off the side of the head or a pupil buried in its own eye. This
//! stands one peeper on a ground plane and works through the states the
//! creature actually spends its life in: dozing, wide awake, reared back
//! mid-wind-up, and speared forward on a landed peck.
//!
//! The expressions are driven by the same [`Mood`] the ECS system builds,
//! so a shot here is what the game draws — not an approximation of it.

use nalgebra::{Point3, Vector3};

use crate::animation::peeper::{Mood, PeeperAnimator, PeeperRigConfig};
use crate::character::{CharacterIntent, Grounding};
use crate::core::error::EngineResult;
use crate::rendering::colour::Colour;
use crate::rendering::vertex::Vertex;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};

use super::flat_ground::{ground, probe_flat_ground};

/// Seconds of settling before a shot, so the feelers hang where they would
/// hang and the lid has finished its slide.
const SETTLE_SECONDS: f32 = 1.2;
const SETTLE_RATE: f32 = 60.0;

pub struct Peeper;

impl VisualScene for Peeper {
    fn name(&self) -> &str {
        "peeper"
    }

    fn description(&self) -> &str {
        "The peeper dozing, alert, winding up and striking. Checks the rig and the eye."
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let environment = SceneEnvironment::default()
            .with_sun(Vector3::new(-0.4, 0.72, 0.57))
            .with_ambient(Colour::new(0.34, 0.36, 0.42, 1.0));

        let config = PeeperRigConfig::default();

        // Framed on the head, since that is where every one of these shots
        // differs, but far enough out that the legs stay in frame — a peck
        // that reads well on a head floating in space is no use if the
        // stance under it has collapsed.
        let height = config.body_height();
        let target = Point3::new(0.0, height * 0.62, 0.0);
        let distance = 3.1;

        let takes = [
            ("dozing", 0.7_f32, 0.0, 0.0, Mood::default()),
            (
                "alert",
                0.7,
                0.0,
                0.0,
                Mood {
                    alertness: 1.0,
                    thrust: 0.0,
                },
            ),
            (
                "winding_up",
                1.15,
                0.0,
                0.0,
                Mood {
                    alertness: 0.65,
                    thrust: -1.0,
                },
            ),
            (
                "striking",
                1.15,
                0.0,
                0.0,
                Mood {
                    alertness: 0.65,
                    thrust: 1.0,
                },
            ),
            // Mid-stride from the side, which is where a hock that breaks
            // the wrong way shows up.
            (
                "walking",
                1.57,
                2.6,
                1.1,
                Mood {
                    alertness: 0.9,
                    thrust: 0.0,
                },
            ),
        ];

        Ok(takes
            .into_iter()
            .map(|(label, angle, walk_seconds, speed, mood)| {
                let eye = Point3::new(
                    distance * angle.sin(),
                    height * 0.78,
                    distance * angle.cos(),
                );
                let (meshes, travelled) = rig_meshes(config, walk_seconds, speed, mood);
                // Follow whatever ground the peeper covered, or a walking
                // shot frames the patch of dirt it set off from.
                let along = Vector3::z() * travelled;
                let shot = SceneCamera::looking_at(eye + along, target + along).with_fov(40.0);
                SceneShot::new(label, shot)
                    .with_environment(environment.clone())
                    .with_meshes(meshes)
            })
            .collect())
    }
}

/// One settled peeper and the ground under it, plus how far it walked to
/// get there.
fn rig_meshes(
    config: PeeperRigConfig,
    walk_seconds: f32,
    speed: f32,
    mood: Mood,
) -> (Vec<SceneMesh>, f32) {
    let mut walker = Walker::new(config);
    walker.run(SETTLE_SECONDS, 0.0, mood);
    walker.run(walk_seconds, speed, mood);
    let (vertices, indices) = walker.mesh();
    (
        vec![ground(), SceneMesh::new(vertices, indices)],
        walker.travelled,
    )
}

/// Drives a peeper across flat ground, the way the ECS systems do.
///
/// The probes are answered by intersecting them with the ground plane,
/// which is the one thing the bench has to stand in for. Everything else —
/// the gait, the legs, the neck, the eye — is the shipping code.
pub struct Walker {
    animator: PeeperAnimator,
    /// How far the peeper has travelled, along +Z. Held so the body keeps
    /// moving across successive `run` calls rather than snapping back.
    pub travelled: f32,
    clearance: f32,
}

impl Walker {
    pub fn new(config: PeeperRigConfig) -> Self {
        // The rig is drawn around a body whose origin rides one standing
        // height above the floor, which is where a capsule for it would
        // sit.
        let clearance = config.standing_height();
        let animator =
            PeeperAnimator::new(config, Point3::new(0.0, clearance, 0.0), clearance, 0.0);
        Self {
            animator,
            travelled: 0.0,
            clearance,
        }
    }

    /// Walk forward at `speed` for `seconds`, in the given mood.
    pub fn run(&mut self, seconds: f32, speed: f32, mood: Mood) {
        let dt = 1.0 / SETTLE_RATE;
        let velocity = Vector3::new(0.0, 0.0, speed);
        for _ in 0..(seconds * SETTLE_RATE).round() as usize {
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

    pub fn mesh(&mut self) -> (Vec<Vertex>, Vec<u32>) {
        let (vertices, indices) = self.animator.mesh();
        (vertices.to_vec(), indices.to_vec())
    }
}
