//! The heart critter, from every side.
//!
//! A rig is a thing you look at, and no numeric check catches an ear
//! planted through a cheek or a foot facing backwards. This stands one
//! critter on a ground plane and walks the camera round it.

use nalgebra::{Point3, Vector3};

use crate::animation::critter::{CritterAnimator, CritterRigConfig};
use crate::character::{CharacterIntent, Grounding};
use crate::core::error::EngineResult;
use crate::rendering::colour::Colour;
use crate::rendering::vertex::Vertex;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};
use crate::sensing::{ContactCandidate, Probe};

/// Half-width of the ground plane.
const GROUND_HALF_EXTENT: f32 = 4.0;

/// Seconds of settling to run before the shot, so the ears hang where
/// they would hang rather than where they were built.
const SETTLE_SECONDS: f32 = 1.0;
const SETTLE_RATE: f32 = 60.0;

pub struct Critter;

impl VisualScene for Critter {
    fn name(&self) -> &str {
        "critter"
    }

    fn description(&self) -> &str {
        "The heart critter at rest, from four sides. Checks the rig, not the gait."
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let environment = SceneEnvironment::default()
            .with_sun(Vector3::new(-0.4, 0.72, 0.57))
            .with_ambient(Colour::new(0.34, 0.36, 0.42, 1.0));

        let config = CritterRigConfig::default();

        // Four angles at the critter's own height, close enough that it
        // fills the frame: it is 45 cm tall and the failures worth seeing
        // are centimetres across.
        let height = config.body_height();
        let target = Point3::new(0.0, height * 0.6, 0.0);
        let distance = 1.15;

        let angles = [
            ("front", 0.0_f32),
            ("three_quarter", 0.7),
            ("side", std::f32::consts::FRAC_PI_2),
            ("back", std::f32::consts::PI),
        ];

        Ok(angles
            .into_iter()
            .map(|(label, angle)| {
                let eye = Point3::new(
                    distance * angle.sin(),
                    height * 0.85,
                    distance * angle.cos(),
                );
                SceneShot::new(label, SceneCamera::looking_at(eye, target).with_fov(40.0))
                    .with_environment(environment.clone())
                    .with_meshes(rig_meshes(config))
            })
            .collect())
    }
}

/// The critter standing on the ground, settled.
fn rig_meshes(config: CritterRigConfig) -> Vec<SceneMesh> {
    let mut walker = Walker::new(config);
    walker.run(SETTLE_SECONDS, 0.0);
    let (vertices, indices) = walker.mesh();
    vec![ground(), SceneMesh::new(vertices, indices)]
}

/// Drives a critter across flat ground, the way the ECS systems do.
///
/// The probes are answered by intersecting them with the ground plane,
/// which is the one thing the bench has to stand in for. Everything else
/// — the gait, the legs, the ears — is the shipping code.
pub struct Walker {
    animator: CritterAnimator,
    /// How far the critter has travelled, along +Z.
    pub travelled: f32,
    clearance: f32,
}

impl Walker {
    pub fn new(config: CritterRigConfig) -> Self {
        // The rig is drawn around a body whose origin rides one standing
        // height above the floor, which is where a capsule for it would sit.
        let clearance = config.standing_height();
        let animator =
            CritterAnimator::new(config, Point3::new(0.0, clearance, 0.0), clearance, 0.0);
        Self {
            animator,
            travelled: 0.0,
            clearance,
        }
    }

    /// Walk forward at `speed` for `seconds`.
    pub fn run(&mut self, seconds: f32, speed: f32) {
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
            );
        }
    }

    pub fn mesh(&mut self) -> (Vec<Vertex>, Vec<u32>) {
        let (vertices, indices) = self.animator.mesh();
        (vertices.to_vec(), indices.to_vec())
    }
}

/// Answer each probe against the plane `y = 0`.
fn probe_flat_ground(probes: &[Probe]) -> Vec<ContactCandidate> {
    probes
        .iter()
        .filter_map(|probe| {
            if probe.direction.y >= -1e-4 {
                return None;
            }
            let distance = -probe.origin.y / probe.direction.y;
            (distance <= probe.length).then(|| ContactCandidate {
                tag: probe.tag,
                point: probe.origin + probe.direction * distance,
                normal: Vector3::y(),
                distance,
            })
        })
        .collect()
}

/// A matte quad for the critter to stand on.
pub fn ground() -> SceneMesh {
    let e = GROUND_HALF_EXTENT;
    let colour = Colour::new(0.3, 0.33, 0.31, 1.0).to_vec4();
    let normal = Vector3::new(0.0, 1.0, 0.0);

    let corners = [
        (Vector3::new(-e, 0.0, -e), [0.0, 0.0]),
        (Vector3::new(e, 0.0, -e), [1.0, 0.0]),
        (Vector3::new(e, 0.0, e), [1.0, 1.0]),
        (Vector3::new(-e, 0.0, e), [0.0, 1.0]),
    ];

    let vertices: Vec<Vertex> = corners
        .iter()
        .map(|(position, uv)| Vertex {
            pos: *position,
            color: colour,
            tex_coords: nalgebra::Vector2::new(uv[0], uv[1]),
            normal,
            ao: 1.0,
        })
        .collect();

    SceneMesh::new(vertices, vec![0, 1, 2, 0, 2, 3])
}
