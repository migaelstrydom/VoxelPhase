//! Piper — a bagpipe that plays itself: a swollen pink bag squatting on two
//! small feet, drones standing out of it, and a little head blowing.
//!
//! The ranged threat. It never moves. Whoever comes into its sight has a
//! bomb lobbed at them every few seconds, in a high slow arc, so a route it
//! overlooks is crossed in dashes; and the bombs are real ones, so the route
//! does not stay as it was. It bursts at a blow. `visual_bench bestiary`
//! shows it.

use std::sync::Arc;

use nalgebra::{Point3, UnitVector3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::palette::Paint;
use super::sculpted::{all_round, ground_under, sculpt_material, Exhibit};
use crate::app::sculpt::{painted_model, ContactShade, Ellipsoid, Mottle, SculptedPart, Warp};
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::creature::Lobber;
use crate::damage::{Health, Ragdoll};
use crate::model::Model;
use crate::physics::{ColliderDesc, ConstraintKind, RigidBodyDesc};
use crate::rendering::colour::Colour;
use crate::rendering::material::MaterialId;
use crate::systems::PhysicsResource;

/// Kilograms to the cubic metre: a bag of air and skin.
const DENSITY: f32 = 500.0;
/// Radius of the ball it collides as, m.
const RADIUS: f32 = 0.42;
/// Nearest and furthest it throws, m, seconds between throws, and seconds a
/// bomb is in the air.
const RANGE: (f32, f32) = (4.0, 24.0);
const INTERVAL: f32 = 3.5;
const FLIGHT_TIME: f32 = 1.9;
/// How high its tallest drone's mouth stands above its middle, m: where a
/// bomb leaves it.
const MUZZLE: f32 = 1.25;
/// Hit points it can take.
const HEALTH: f32 = 25.0;

const MOTTLE: Mottle = Mottle {
    frequency: 7.0,
    value: 0.2,
    warmth: 0.2,
    seed: 61,
};
const SHADE: ContactShade = ContactShade {
    reach: 0.05,
    depth: 0.6,
    floor: 0.3,
};

/// A piper.
#[derive(Deserialize)]
pub struct PiperDef {
    /// Position (x, z). It squats on the ground there.
    pub pos: (f32, f32),
}

impl PiperDef {
    /// How it looks, drawn in `material`, about the middle of its bag.
    pub fn model(material: MaterialId) -> Model {
        let look = |r: f32, g: f32, b: f32| (Colour::new(r, g, b, 1.0), material);
        let bag = look(0.8, 0.45, 0.45);
        let wood = look(0.2, 0.12, 0.08);
        let gilt = look(0.8, 0.6, 0.2);
        let flesh = look(0.82, 0.62, 0.52);
        let dark = look(0.04, 0.03, 0.03);

        let mut parts = vec![
            // The bag, lumpy and tight.
            SculptedPart::new(Ellipsoid::new((0.42, 0.38, 0.5), (0.0, 0.0, 0.0)), bag).warped(
                Warp::Lumps {
                    frequency: 4.0,
                    amplitude: 0.035,
                    seed: 5,
                },
            ),
            // A tied-off neck at the back.
            SculptedPart::new(Ellipsoid::new((0.1, 0.1, 0.16), (0.0, 0.06, -0.52)), bag),
            SculptedPart::new(Ellipsoid::new((0.12, 0.12, 0.03), (0.0, 0.06, -0.44)), gilt),
            // The head, cheeks blown out, its mouth on the blowpipe.
            SculptedPart::new(Ellipsoid::new((0.13, 0.12, 0.12), (0.0, 0.36, 0.38)), flesh),
            SculptedPart::new(
                Ellipsoid::new((0.07, 0.06, 0.06), (-0.1, 0.33, 0.43)),
                flesh,
            ),
            SculptedPart::new(Ellipsoid::new((0.07, 0.06, 0.06), (0.1, 0.33, 0.43)), flesh),
            SculptedPart::new(
                Ellipsoid::new((0.025, 0.025, 0.16), (0.0, 0.26, 0.4)).pitched(0.9),
                wood,
            ),
            // The chanter, hanging down in front.
            SculptedPart::new(
                Ellipsoid::new((0.03, 0.03, 0.26), (0.0, -0.18, 0.52)).pitched(1.1),
                wood,
            ),
        ];
        for side in [-1.0, 1.0] {
            parts.push(SculptedPart::new(
                Ellipsoid::ball(0.02, Vector3::new(0.05 * side, 0.41, 0.48)),
                dark,
            ));
            // A bare foot each side, toes out.
            parts.push(SculptedPart::new(
                Ellipsoid::new((0.07, 0.04, 0.13), (0.2 * side, -0.38, 0.22)).yawed(0.4 * side),
                flesh,
            ));
        }
        // Three drones standing out of the top: the middle one tallest, and
        // each ending in a gilt mouth.
        for (lean, length) in [(-0.35f32, 0.75f32), (0.0, 1.0), (0.35, 0.75)] {
            let foot = Vector3::new(0.3 * lean, 0.3, -0.08);
            let up = Vector3::new(lean.sin(), lean.cos(), -0.1).normalize();
            let turn = nalgebra::UnitQuaternion::rotation_between(&Vector3::z(), &up)
                .unwrap_or_else(nalgebra::UnitQuaternion::identity);
            parts.push(SculptedPart::new(
                Ellipsoid {
                    radii: Vector3::new(0.035, 0.035, 0.5 * length),
                    centre: foot + up * 0.5 * length,
                    turn,
                },
                wood,
            ));
            parts.push(
                SculptedPart::new(
                    Ellipsoid {
                        radii: Vector3::new(0.075, 0.075, 0.09),
                        centre: foot + up * length,
                        turn,
                    },
                    gilt,
                )
                .warped(Warp::Taper {
                    along: -Vector3::z(),
                    tip: 0.4,
                }),
            );
        }
        painted_model(&parts, &MOTTLE, &SHADE)
    }
}

impl PiperDef {
    /// The states it is shown in by tools.
    pub fn exhibits() -> Vec<Exhibit> {
        vec![Exhibit::at_rest(
            "portrait",
            Arc::new(Self::model(Paint::Skin.id())),
            RADIUS,
            1.6,
        )]
    }
}

impl Spawnable for PiperDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(vec![ctx.materials.register(sculpt_material())])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let Some(ground) = ground_under(world, self.pos) else {
            return Vec::new();
        };
        let at = Point3::new(self.pos.0, ground + RADIUS, self.pos.1);
        let (body, keep_upright) = {
            let mut physics = world.write_resource::<PhysicsResource>();
            // It sits where it is put: heavy damping, so a shove rocks it
            // and it settles.
            let body = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(at)
                    .linear_damping(0.4)
                    .angular_damping(0.98),
            );
            physics.world.attach_collider(
                body,
                ColliderDesc::sphere(RADIUS)
                    .density(DENSITY)
                    .friction(1.2)
                    .restitution(0.0),
            );
            // A ball rolls off down any slope, drones and all. Kept
            // upright while it lives; let go when it bursts, so it slumps.
            let keep_upright = physics
                .world
                .create_constraint(ConstraintKind::KeepUpright {
                    body,
                    target_up: UnitVector3::new_normalize(Vector3::y()),
                    compliance: 0.0,
                    max_impulse: f32::INFINITY,
                });
            (body, keep_upright)
        };
        vec![world
            .create_entity()
            .with(Position(at.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(body))
            .with(ModelInstance::new(Arc::new(Self::model(materials[0]))))
            .with(Renderable)
            .with(all_round(RANGE.1))
            .with(Lobber {
                range: RANGE,
                interval: INTERVAL,
                remaining: INTERVAL,
                flight_time: FLIGHT_TIME,
                muzzle: Vector3::y() * MUZZLE,
            })
            .with(Health::new(HEALTH, 4.0))
            .with(Ragdoll::new(vec![keep_upright]))
            .build()]
    }
}
