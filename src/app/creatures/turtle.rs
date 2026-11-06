//! Turtle — a sea turtle that cruises the shallows.
//!
//! It looks like a creature out of Bosch's garden rather than the sea (see
//! [`TurtleDef::model`]); `visual_bench turtle` shows it. A swimmer ([`Swimmer`]) like the fish, slower and calmer: it keeps just
//! under the surface, where it can be seen from the shore, and only edges
//! away when someone comes right up to it. It cannot be hurt.

use std::f32::consts::PI;
use std::sync::Arc;

use nalgebra::Vector3;
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::character::CharacterIntent;
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::creature::{Brain, Perception, Swimmer};
use crate::model::{Model, ModelPart};
use crate::physics::{ColliderDesc, ConstraintKind, RigidBodyDesc};
use crate::rendering::colour::Colour;
use crate::rendering::grain::GrainSpec;
use crate::rendering::material::{Material, MaterialId, SurfaceFinish};
use crate::systems::PhysicsResource;

use super::body_paint::{scatter, ContactShade, Mottle};
use super::body_parts::{BodyPart, Ellipsoid};
use super::body_warp::Warp;
use super::in_water::spawn_point_in_water;

/// As dense as the water: buoyancy carries it and its flippers only move it.
const DENSITY: f32 = 1000.0;

/// Radius of the sphere it collides as, m.
const COLLIDER_RADIUS: f32 = 0.3;

/// The shell's and face's finish: the soft sheen of eggshell or oil glaze,
/// not the tight highlight of plastic.
const GLAZE: SurfaceFinish = SurfaceFinish {
    roughness: 0.55,
    metallic: 0.0,
};

/// The glaze's microstructure: fine enough to read as a surface, not as
/// bumps (about 24 features a metre), and faint, so that it breaks up the
/// highlight without roughening the shell.
const GLAZE_GRAIN: GrainSpec = GrainSpec::STONE.with_scale(1.0).with_strength(0.14);

/// Skin: matte, with only a breath of sheen.
const SKIN: SurfaceFinish = SurfaceFinish {
    roughness: 0.8,
    metallic: 0.0,
};

/// Skin's microstructure: a pebbly hide, coarser and stronger than the
/// glaze's.
const SKIN_GRAIN: GrainSpec = GrainSpec::STONE.with_scale(1.2).with_strength(0.28);

/// Eyes: glossy, but softer than a polished bead.
const BEAD: SurfaceFinish = SurfaceFinish {
    roughness: 0.3,
    metallic: 0.0,
};

/// The finial's gilding.
const GILT: SurfaceFinish = SurfaceFinish {
    roughness: 0.35,
    metallic: 0.5,
};

/// Rings of studs on the dome: each ring's elevation above the dome's
/// equator, radians, and how many studs it holds.
const STUD_RINGS: [(f32, usize); 3] = [(0.3, 16), (0.65, 12), (1.0, 7)];

/// Radius of a stud, m, and how far a stud's radius strays from it, as a
/// share of it.
const STUD_RADIUS: f32 = 0.02;
const STUD_SIZE_SPREAD: f32 = 0.3;

/// How far a stud strays from its place in the ring, as a share of the
/// spacing round the ring, and up or down it, radians.
const STUD_AZIMUTH_SPREAD: f32 = 0.25;
const STUD_ELEVATION_SPREAD: f32 = 0.07;

/// Patches over the whole body.
const MOTTLE: Mottle = Mottle {
    frequency: 7.0,
    value: 0.24,
    warmth: 0.15,
    seed: 11,
};

/// The shade where one part meets another.
const CONTACT_SHADE: ContactShade = ContactShade {
    reach: 0.05,
    depth: 0.6,
    floor: 0.3,
};

/// A turtle. Position is `(x, z)`; it starts in the water over that point.
#[derive(Deserialize)]
pub struct TurtleDef {
    pub pos: (f32, f32),
    /// How close the player may come before it edges away, m.
    #[serde(default = "TurtleDef::default_flee_range")]
    pub flee_range: f32,
}

impl TurtleDef {
    pub fn default_flee_range() -> f32 {
        2.0
    }

    fn mass() -> f32 {
        (4.0 / 3.0) * PI * COLLIDER_RADIUS.powi(3) * DENSITY
    }
}

impl TurtleDef {
    /// Its materials, one per finish, in the order [`Self::model`] indexes
    /// them: a painted glaze for the shell, a softer one for skin, glossy
    /// eyes, and gilt for the finial.
    pub fn materials() -> Vec<Material> {
        let with = |finish| Material::coloured(Colour::WHITE).with_finish(finish);
        vec![
            with(GLAZE).with_grain(GLAZE_GRAIN),
            with(SKIN).with_grain(SKIN_GRAIN),
            with(BEAD),
            with(GILT),
        ]
    }

    /// How it looks, drawn in `materials` (registered from
    /// [`Self::materials`], in order), in its body's frame (+Z forward,
    /// +Y up, the collider's centre at the origin).
    ///
    /// A creature out of a painted garden rather than a sea: a high pearl
    /// dome studded with lapis beads and crowned by a gilt finial that
    /// breaks the surface as it swims, a frill of rose lobes round its rim,
    /// a cream hood over a slate-blue face that ends in a hooked beak, and
    /// long rose-cream flippers that thin to their tips and sweep back.
    pub fn model(materials: &[MaterialId]) -> Model {
        let (glaze, skin, bead, gilt) = (materials[0], materials[1], materials[2], materials[3]);
        let pearl = (Colour::new(0.9, 0.8, 0.64, 1.0), glaze);
        let rose = (Colour::new(0.86, 0.36, 0.38, 1.0), glaze);
        let cream = (Colour::new(0.92, 0.7, 0.6, 1.0), skin);
        let hood = (Colour::new(0.94, 0.86, 0.74, 1.0), skin);
        let slate = (Colour::new(0.2, 0.3, 0.5, 1.0), glaze);
        let beak = (Colour::new(0.1, 0.14, 0.26, 1.0), glaze);
        let lapis = (Colour::new(0.05, 0.08, 0.24, 1.0), glaze);
        let ink = (Colour::new(0.02, 0.02, 0.03, 1.0), bead);
        let gold = (Colour::new(1.0, 0.62, 0.18, 1.0), gilt);

        let dome = BodyPart::new(Ellipsoid::new((0.28, 0.24, 0.35), (0.0, 0.02, 0.0)), pearl)
            .warped(Warp::Lumps {
                frequency: 5.0,
                amplitude: 0.012,
                seed: 3,
            });
        let forward = Vector3::z();
        let back = -Vector3::z();
        let down = -Vector3::y();
        let mut parts = vec![
            // A frill round the rim, its edge in lobes.
            BodyPart::new(Ellipsoid::new((0.31, 0.06, 0.38), (0.0, 0.0, 0.0)), rose).warped(
                Warp::Scallop {
                    lobes: 14,
                    bulge: 0.07,
                    lift: 0.015,
                },
            ),
            BodyPart::new(Ellipsoid::new((0.28, 0.07, 0.34), (0.0, -0.03, 0.0)), cream),
            // Neck, hood, the face set in its front, and a beak that
            // narrows to a hooked point.
            BodyPart::new(
                Ellipsoid::new((0.07, 0.065, 0.13), (0.0, 0.03, 0.36)).pitched(-0.25),
                cream,
            )
            .warped(Warp::Taper {
                along: forward,
                tip: 0.75,
            }),
            // A cowl drawn back to a point behind the head.
            BodyPart::new(
                Ellipsoid::new((0.09, 0.085, 0.13), (0.0, 0.085, 0.46)),
                hood,
            )
            .warped(Warp::Taper {
                along: back,
                tip: 0.3,
            })
            .warped(Warp::Bend {
                along: back,
                toward: Vector3::y(),
                amount: 0.05,
            }),
            BodyPart::new(
                Ellipsoid::new((0.07, 0.065, 0.05), (0.0, 0.07, 0.54)),
                slate,
            ),
            BodyPart::new(
                Ellipsoid::new((0.036, 0.034, 0.095), (0.0, 0.055, 0.6)).pitched(0.1),
                beak,
            )
            .warped(Warp::Taper {
                along: forward,
                tip: 0.4,
            })
            .warped(Warp::Bend {
                along: forward,
                toward: down,
                amount: 0.055,
            }),
            BodyPart::new(Ellipsoid::ball(0.013, Vector3::new(0.05, 0.095, 0.56)), ink),
            BodyPart::new(
                Ellipsoid::ball(0.013, Vector3::new(-0.05, 0.095, 0.56)),
                ink,
            ),
            // The finial: a lapis collar on the crown, a gilt stem, a ball
            // and a point.
            BodyPart::new(
                Ellipsoid::new((0.075, 0.03, 0.075), (0.0, 0.25, 0.0)),
                lapis,
            ),
            BodyPart::new(Ellipsoid::new((0.016, 0.07, 0.016), (0.0, 0.32, 0.0)), gold),
            BodyPart::new(Ellipsoid::ball(0.04, Vector3::new(0.0, 0.39, 0.0)), gold),
            BodyPart::new(Ellipsoid::new((0.009, 0.04, 0.009), (0.0, 0.46, 0.0)), gold),
            // A short tail, tapering and drooping.
            BodyPart::new(
                Ellipsoid::new((0.04, 0.025, 0.08), (0.0, -0.01, -0.4)).pitched(0.2),
                cream,
            )
            .warped(Warp::Taper {
                along: back,
                tip: 0.2,
            })
            .warped(Warp::Bend {
                along: back,
                toward: down,
                amount: 0.02,
            }),
        ];
        // Flippers that thin to their tips and sweep back.
        for side in [1.0_f32, -1.0] {
            let out = Vector3::x() * side;
            parts.push(
                BodyPart::new(
                    Ellipsoid::new((0.26, 0.026, 0.08), (0.36 * side, -0.01, 0.2))
                        .yawed(0.5 * side)
                        .rolled(-0.15 * side),
                    cream,
                )
                .warped(Warp::Taper {
                    along: out,
                    tip: 0.55,
                })
                .warped(Warp::Bend {
                    along: out,
                    toward: back,
                    amount: 0.07,
                }),
            );
            parts.push(
                BodyPart::new(
                    Ellipsoid::new((0.13, 0.022, 0.065), (0.22 * side, -0.02, -0.33))
                        .yawed(-0.7 * side),
                    cream,
                )
                .warped(Warp::Taper {
                    along: out,
                    tip: 0.45,
                })
                .warped(Warp::Bend {
                    along: out,
                    toward: back,
                    amount: 0.03,
                }),
            );
        }
        let studs = STUD_RINGS
            .iter()
            .enumerate()
            .flat_map(|(ring, &(elevation, count))| {
                let stagger = if ring % 2 == 0 { 0.0 } else { 0.5 };
                (0..count).map(move |i| (ring, elevation, count, stagger, i))
            })
            .map(|(ring, elevation, count, stagger, i)| {
                let key = (ring * 64 + i) as u32;
                let azimuth = (i as f32 + stagger + STUD_AZIMUTH_SPREAD * scatter(key, 0))
                    / count as f32
                    * 2.0
                    * PI;
                let elevation = elevation + STUD_ELEVATION_SPREAD * scatter(key, 1);
                let radius = STUD_RADIUS * (1.0 + STUD_SIZE_SPREAD * scatter(key, 2));
                let (point, normal) = dome.surface_at(azimuth, elevation);
                BodyPart::new(
                    Ellipsoid::ball(radius, point - normal * radius * 0.5),
                    lapis,
                )
            })
            .collect::<Vec<_>>();
        parts.extend(studs);
        parts.push(dome);

        let mut meshes: Vec<_> = parts
            .iter()
            .map(|part| {
                let mut mesh = part.mesh();
                MOTTLE.paint(&mut mesh);
                mesh
            })
            .collect();
        let shapes: Vec<Ellipsoid> = parts.iter().map(|part| part.shape).collect();
        CONTACT_SHADE.shade(&mut meshes, &shapes);
        Model::flat(vec![ModelPart::new(meshes)])
    }
}

impl Spawnable for TurtleDef {
    fn material_count(&self) -> usize {
        Self::materials().len()
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(Self::materials()
            .into_iter()
            .map(|material| ctx.materials.register(material))
            .collect())
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let swimmer = Swimmer {
            mass: Self::mass(),
            yaw_inertia: 0.4 * Self::mass() * COLLIDER_RADIUS * COLLIDER_RADIUS,
            sprint_speed: 1.4,
            cruise_speed: 0.5,
            accel_time: 1.2,
            depth_share: 0.0,
            clearance: 0.3,
            beat_rate: 0.7,
            beat_swing: 0.06,
            beat_phase: 0.0,
        };
        let Some(start) =
            spawn_point_in_water(world, self.pos, swimmer.depth_share, swimmer.clearance)
        else {
            return Vec::new();
        };
        let model = Arc::new(Self::model(materials));

        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();
            let handle = physics
                .world
                .create_body(RigidBodyDesc::dynamic().position(start));
            physics.world.attach_collider(
                handle,
                ColliderDesc::sphere(COLLIDER_RADIUS)
                    .density(DENSITY)
                    .friction(0.5),
            );
            physics
                .world
                .create_constraint(ConstraintKind::KeepAttitude {
                    body: handle,
                    pitch: 0.0,
                    compliance: 0.0,
                    max_impulse: f32::INFINITY,
                });
            handle
        };

        vec![world
            .create_entity()
            .with(Position(start.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(body_handle))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .with(CharacterIntent::default())
            .with(Perception::ground_creature(8.0))
            .with(Brain::skittish(self.flee_range))
            .with(swimmer)
            .build()]
    }
}
