//! The heart critter walking, as a strip of frames.
//!
//! The gait itself is measured offline by `anim_viewer` on the humanoid,
//! and the critter walks on the same `LeggedLocomotion`. What this checks
//! is the part that does *not* carry over: whether a gait tuned for a
//! 1.7 m person still looks like walking on a 45 cm creature, and whether
//! the ears do anything worth watching while it happens.

use nalgebra::{Point3, Vector3};

use crate::animation::critter::CritterRigConfig;
use crate::core::error::EngineResult;
use crate::rendering::colour::Colour;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};

use super::critter::Walker;
use super::flat_ground::ground;

/// Seconds of standing before the walk, so the ears start settled.
const SETTLE_SECONDS: f32 = 0.6;

/// Walking pace. A critter this size at 1.2 m/s is trotting: leg length is
/// 0.2 m, which puts the Froude number near 0.7 and the gait well into the
/// part of the duty-factor curve where the feet leave the ground.
const SPEED: f32 = 1.2;

/// Frames in the strip, and how far apart they are.
const FRAMES: usize = 8;
const FRAME_INTERVAL: f32 = 0.07;

pub struct CritterWalk;

impl VisualScene for CritterWalk {
    fn name(&self) -> &str {
        "critter_walk"
    }

    fn description(&self) -> &str {
        "The heart critter walking, one tile per beat. Checks the gait at critter scale."
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let environment = SceneEnvironment::default()
            .with_sun(Vector3::new(-0.45, 0.7, 0.55))
            .with_ambient(Colour::new(0.34, 0.36, 0.42, 1.0));

        let config = CritterRigConfig::default();
        let height = config.body_height();

        let mut walker = Walker::new(config);
        walker.run(SETTLE_SECONDS, 0.0);

        let mut shots = Vec::with_capacity(FRAMES);
        for frame in 0..FRAMES {
            walker.run(FRAME_INTERVAL, SPEED);

            // The camera travels with the critter, three-quarter on, so a
            // foot that skates shows as a foot moving against the frame.
            let centre = Point3::new(0.0, height * 0.55, walker.travelled);
            let eye = centre + Vector3::new(0.85, height * 0.5, -0.75);
            let (vertices, indices) = walker.mesh();

            shots.push(
                SceneShot::new(
                    format!("t{:.2}", (frame + 1) as f32 * FRAME_INTERVAL),
                    SceneCamera::looking_at(eye, centre).with_fov(40.0),
                )
                .with_environment(environment.clone())
                .with_meshes(vec![ground(), SceneMesh::new(vertices, indices)]),
            );
        }

        Ok(shots)
    }
}
