//! The hover sled and its force field, posed over still ground.
//!
//! Each shot plays a few seconds of pod readouts into the real
//! `ForceFieldVisual` pieces, so the field is in the state those loads would
//! leave it in: settled at rest, straining under a landing, reaching over a
//! gap, flashing as a pod bottoms out. The readouts are written here rather
//! than stepped, so the sheet is the same every run; `character_viewer`
//! shows the sled under real physics.

use nalgebra::{Matrix4, Point3, UnitQuaternion, Vector2, Vector3, Vector4};

use crate::app::spawnables::{hover_sled_parts, hover_sled_pod_anchors, HoverSledDef};
use crate::core::error::EngineResult;
use crate::force_field::kick_up::KickUp;
use crate::force_field::{
    ring_draws, ring_model, ColumnMesh, FieldStyle, PodField, PodLook, PodSample, Underglow,
};
use crate::geometry::{generate_cube_indices, generate_cube_vertices};
use crate::lighting::{ActiveLight, LightId};
use crate::particles::ParticlePool;
use crate::physics::{SuspensionHit, SuspensionReadout};
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, SurfaceParams};
use crate::rendering::vertex::Vertex;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};
use crate::terrain::VoxelMaterial;

/// Frames a second the readouts are played at.
const RATE: f32 = 60.0;
/// Gravity, for the kick-up's particles.
const GRAVITY: f32 = 9.81;
/// Half-width of the ground plane.
const GROUND_HALF_EXTENT: f32 = 30.0;
/// The plank deck of the gap shot: its height, and the planks' extents along
/// the sled, with a 0.6 m gap under its front pods.
const DECK: f32 = 0.56;
const PLANKS: [(f32, f32); 5] = [
    (-1.9, -1.14),
    (-1.06, -0.3),
    (-0.22, 0.25),
    (0.85, 1.61),
    (1.69, 2.45),
];

pub struct HoverSled;

/// What is under one pod.
#[derive(Clone, Copy)]
enum Under {
    /// Ground at this height, the pod carrying this fraction of its cap.
    Ground { height: f32, load: f32 },
    /// Nothing within reach.
    Gap,
}

/// The hull's pose and what each pod has under it, held for a while.
#[derive(Clone, Copy)]
struct Phase {
    seconds: f32,
    /// How far the hull sits below its resting ride height, m.
    sink: f32,
    /// Roll about its length and pitch about its width, degrees.
    roll: f32,
    pitch: f32,
    pods: [Under; 4],
}

impl Phase {
    /// The sled at rest on flat ground at `height`, each pod carrying a
    /// quarter of the hull.
    fn rest(seconds: f32, height: f32) -> Self {
        Self {
            seconds,
            sink: 0.0,
            roll: 0.0,
            pitch: 0.0,
            pods: [Under::Ground {
                height,
                load: REST_LOAD,
            }; 4],
        }
    }
}

/// A pod's share of the hull's weight, as a fraction of its cap: a quarter
/// of the weight against a cap of six times it.
const REST_LOAD: f32 = 1.0 / 24.0;

/// Where the sled stands in every shot, and its yaw.
const SLED_AT: Point3<f32> = Point3::new(0.0, 0.0, 0.0);

impl VisualScene for HoverSled {
    fn name(&self) -> &str {
        "hover_sled"
    }

    fn description(&self) -> &str {
        "The hover sled's force field: at rest, under a landing, over a gap, tipped, bottomed out, at dusk."
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let three_quarter =
            SceneCamera::looking_at(Point3::new(2.3, 1.05, 2.1), Point3::new(0.0, 0.32, 0.0));
        let low = SceneCamera::looking_at(Point3::new(0.4, 0.45, 2.6), Point3::new(0.0, 0.3, 0.0));
        let side_on =
            SceneCamera::looking_at(Point3::new(0.2, 0.9, 3.0), Point3::new(0.0, 0.45, 0.0));

        let landing = Under::Ground {
            height: 0.0,
            load: 0.55,
        };
        let heavy = Under::Ground {
            height: 0.0,
            load: 0.3,
        };
        let light = Under::Ground {
            height: 0.0,
            load: 0.08,
        };
        let floor = Under::Ground {
            height: 0.0,
            load: 1.0,
        };
        let plank = Under::Ground {
            height: DECK,
            load: 0.35,
        };

        let shots = vec![
            (
                "rest",
                three_quarter,
                daylight(),
                vec![Phase::rest(3.0, 0.0)],
                false,
            ),
            (
                "rest_low",
                low,
                daylight(),
                vec![Phase::rest(3.0, 0.0)],
                false,
            ),
            (
                "landing",
                three_quarter,
                daylight(),
                vec![
                    Phase::rest(2.0, 0.0),
                    Phase {
                        seconds: 0.12,
                        sink: 0.04,
                        pods: [landing; 4],
                        ..Phase::rest(0.0, 0.0)
                    },
                ],
                false,
            ),
            (
                "tipped",
                three_quarter,
                daylight(),
                vec![
                    Phase::rest(1.0, 0.0),
                    Phase {
                        seconds: 1.0,
                        sink: 0.03,
                        roll: -5.0,
                        pods: [heavy, light, heavy, light],
                        ..Phase::rest(0.0, 0.0)
                    },
                ],
                false,
            ),
            (
                "over_gap",
                side_on,
                daylight(),
                vec![
                    Phase::rest(2.0, DECK),
                    Phase {
                        seconds: 0.25,
                        sink: 0.04,
                        pitch: -4.0,
                        pods: [Under::Gap, Under::Gap, plank, plank],
                        ..Phase::rest(0.0, DECK)
                    },
                ],
                true,
            ),
            (
                "bottomed",
                three_quarter,
                daylight(),
                vec![
                    Phase::rest(2.0, 0.0),
                    Phase {
                        seconds: 0.04,
                        sink: 0.24,
                        pods: [floor; 4],
                        ..Phase::rest(0.0, 0.0)
                    },
                ],
                false,
            ),
            (
                "dusk",
                three_quarter,
                dusk(),
                vec![Phase::rest(3.0, 0.0)],
                false,
            ),
            (
                "dusk_landing",
                three_quarter,
                dusk(),
                vec![
                    Phase::rest(2.0, 0.0),
                    Phase {
                        seconds: 0.12,
                        sink: 0.04,
                        pods: [landing; 4],
                        ..Phase::rest(0.0, 0.0)
                    },
                ],
                false,
            ),
        ];

        Ok(shots
            .into_iter()
            .map(|(label, camera, environment, phases, planks)| {
                shot(label, camera, environment, &phases, planks)
            })
            .collect())
    }
}

fn daylight() -> SceneEnvironment {
    SceneEnvironment::default().with_sun(Vector3::new(-0.5, 0.75, 0.45))
}

fn dusk() -> SceneEnvironment {
    let mut environment = SceneEnvironment::default()
        .with_sun(Vector3::new(-0.8, 0.12, 0.3))
        .with_sun_colour(Colour::new(1.0, 0.55, 0.35, 1.0))
        .with_ambient(Colour::new(0.05, 0.06, 0.1, 1.0));
    environment.lighting.sun_intensity *= 0.25;
    environment
}

/// One shot: the sled in the last phase's pose, its field after playing
/// every phase, the ground, and the planks if asked for.
fn shot(
    label: &str,
    camera: SceneCamera,
    environment: SceneEnvironment,
    phases: &[Phase],
    planks: bool,
) -> SceneShot {
    let style = FieldStyle::HOVER;
    let mut fields: Vec<PodField> = (1..=4).map(PodField::new).collect();
    let mut kick_up = KickUp::default();
    let mut particles = ParticlePool::new(4096);
    let mut looks: Vec<PodLook> = Vec::new();
    let dt = 1.0 / RATE;

    let last = phases.last().copied().unwrap_or(Phase::rest(0.0, 0.0));
    for phase in phases {
        for _ in 0..(phase.seconds * RATE).ceil() as usize {
            let samples = samples(phase);
            looks = fields
                .iter_mut()
                .zip(&samples)
                .map(|(field, sample)| {
                    field.update(sample, &style, dt);
                    field.look(&style)
                })
                .collect();
            kick_up.emit(&looks, dt, &mut particles);
            particles.update(dt, GRAVITY);
        }
    }

    let mut environment = environment;
    let underglow = underglow(&looks, &style);
    if let Some(light) = underglow {
        environment = environment.with_point_light(light);
    }

    let (centre, rotation) = pose(&last);
    let hull = Matrix4::new_translation(&centre.coords) * rotation.to_homogeneous();
    let mut shot = SceneShot::new(label, camera)
        .with_environment(environment)
        .with_mesh(ground())
        .with_meshes(hover_sled_parts().into_iter().map(|part| {
            let surface = part
                .substance
                .shade(Material::coloured(Colour::WHITE))
                .surface_params();
            SceneMesh::new(part.vertices, part.indices)
                .with_transform(hull)
                .with_surface(surface)
        }))
        .with_meshes(rings(&looks, &style))
        .with_force_field(ColumnMesh::of(&looks))
        .with_particles(particles);
    if planks {
        shot = shot.with_meshes(plank_deck());
    }
    shot
}

/// The hull's centre and rotation in `phase`.
fn pose(phase: &Phase) -> (Point3<f32>, UnitQuaternion<f32>) {
    let height = phase
        .pods
        .iter()
        .find_map(|under| match under {
            Under::Ground { height, .. } => Some(*height),
            Under::Gap => None,
        })
        .unwrap_or(0.0);
    let rotation =
        UnitQuaternion::from_euler_angles(phase.roll.to_radians(), 0.0, phase.pitch.to_radians());
    let centre = SLED_AT + Vector3::y() * (height + HoverSledDef::ride_height() - phase.sink);
    (centre, rotation)
}

/// Each pod's readout for the hull posed as `phase` says, over what
/// `phase` says is under it.
fn samples(phase: &Phase) -> Vec<PodSample> {
    let (centre, rotation) = pose(phase);
    let axis = rotation * Vector3::y();
    let radius = HoverSledDef::POD_CAST_RADIUS;
    let rest = HoverSledDef::POD_REST_LENGTH;
    hover_sled_pod_anchors()
        .iter()
        .zip(phase.pods)
        .map(|(local, under)| {
            let anchor = centre + rotation * local;
            let readout = match under {
                Under::Ground { height, load } => {
                    let length = (anchor.y - height - radius) / axis.y;
                    let sphere = anchor - axis * length;
                    let point = Point3::new(sphere.x, height, sphere.z);
                    SuspensionReadout {
                        hit: Some(SuspensionHit {
                            point,
                            normal: Vector3::y(),
                            partner: None,
                            friction: 0.0,
                            axis,
                            anchor,
                            length,
                        }),
                        grounded: true,
                        length,
                        compression: rest - length,
                        load: 0.0,
                        load_fraction: load,
                        stored_energy: 0.0,
                    }
                }
                Under::Gap => SuspensionReadout {
                    hit: None,
                    grounded: false,
                    length: rest,
                    compression: 0.0,
                    load: 0.0,
                    load_fraction: 0.0,
                    stored_energy: 0.0,
                },
            };
            PodSample {
                anchor,
                axis,
                rest_length: rest,
                readout,
                ground_speed: 0.0,
            }
        })
        .collect()
}

/// The ring meshes for `looks`.
fn rings(looks: &[PodLook], style: &FieldStyle) -> Vec<SceneMesh> {
    let torus = &ring_model().parts[0].primitives[0];
    ring_draws(looks, style)
        .into_iter()
        .map(|ring| {
            SceneMesh::new(torus.vertices.clone(), torus.indices.clone())
                .with_transform(ring.transform)
                .with_surface(ring.surface)
        })
        .collect()
}

/// The underglow the game would light for `looks`.
fn underglow(looks: &[PodLook], style: &FieldStyle) -> Option<ActiveLight> {
    Underglow::of(looks, style, looks.len()).map(|glow| ActiveLight {
        id: LightId(0),
        position: glow.position.coords,
        range: style.light_range,
        colour: glow.colour,
        intensity: glow.intensity,
    })
}

/// A grass plane at the ground's height.
fn ground() -> SceneMesh {
    let [r, g, b, a] = VoxelMaterial::Grass.color();
    let colour = Vector4::new(r, g, b, a);
    let e = GROUND_HALF_EXTENT;
    let corner = |x: f32, z: f32| Vertex {
        pos: Vector3::new(x, 0.0, z),
        color: colour,
        tex_coords: Vector2::zeros(),
        normal: Vector3::y(),
        ao: 1.0,
    };
    SceneMesh::new(
        vec![corner(-e, -e), corner(e, -e), corner(e, e), corner(-e, e)],
        vec![0, 2, 1, 0, 3, 2],
    )
    .with_surface(SurfaceParams::MATTE)
}

/// Planks across the sled's path at deck height, with a gap under its nose.
fn plank_deck() -> Vec<SceneMesh> {
    let wood = Colour::new(0.55, 0.38, 0.22, 1.0);
    PLANKS
        .into_iter()
        .map(|(from, to)| {
            let half = Vector3::new(0.5 * (to - from), 0.06, 1.2);
            SceneMesh::new(generate_cube_vertices(half, wood), generate_cube_indices())
                .at(Vector3::new(0.5 * (from + to), DECK - half.y, 0.0))
                .with_surface(SurfaceParams {
                    surface: [0.7, 0.0, 0.0, 3.0],
                    ..SurfaceParams::MATTE
                })
        })
        .collect()
}
