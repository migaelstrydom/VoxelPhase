//! Flying fish — a fish on a pair of bird's wings, circling over open
//! ground.
//!
//! The air threat. It holds a height nothing on foot can reach, comes over
//! whoever it has seen, and stoops to bite when it is over them; standing
//! in the open is what it punishes. It is frail: one good blow, or a blast
//! anywhere near it, ends it. `visual_bench bestiary` shows it.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::palette::Paint;
use super::sculpted::{all_round, ground_under, sculpt_material, Exhibit};
use crate::app::sculpt::{painted_model, ContactShade, Ellipsoid, Mottle, SculptedPart, Warp};
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::character::CharacterIntent;
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::creature::{Brain, Flyer, MeleeAttack};
use crate::damage::Health;
use crate::model::Model;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::colour::Colour;
use crate::rendering::material::MaterialId;
use crate::systems::PhysicsResource;

/// Radius of the ball it collides as, m.
const RADIUS: f32 = 0.25;
/// Kilograms to the cubic metre.
const DENSITY: f32 = 250.0;
/// Level speed, m/s, and seconds to reach it.
const SPEED: f32 = 6.5;
const ACCEL_TIME: f32 = 0.8;
/// Height it holds over the ground, and the distance from its target
/// within which it comes down, m.
const CRUISE_HEIGHT: f32 = 6.0;
const STOOP_RANGE: f32 = 5.0;
/// How far it sees, m.
const SIGHT: f32 = 30.0;
/// Its bite: hit points, reach, and the seconds before and after.
const BITE: f32 = 8.0;
const REACH: f32 = 1.1;
const BITE_WINDUP: f32 = 0.25;
const BITE_RECOVERY: f32 = 1.6;
/// Hit points it can take.
const HEALTH: f32 = 14.0;

const MOTTLE: Mottle = Mottle {
    frequency: 12.0,
    value: 0.2,
    warmth: 0.15,
    seed: 67,
};
const SHADE: ContactShade = ContactShade {
    reach: 0.04,
    depth: 0.55,
    floor: 0.35,
};

/// A flying fish.
#[derive(Deserialize)]
pub struct FlyingFishDef {
    /// Position (x, z). It starts perched on the ground there, and takes
    /// off as soon as the game runs.
    pub pos: (f32, f32),
}

impl FlyingFishDef {
    /// How it looks, drawn in `material`, nose along +Z.
    pub fn model(material: MaterialId) -> Model {
        let look = |r: f32, g: f32, b: f32| (Colour::new(r, g, b, 1.0), material);
        let scales = look(0.62, 0.3, 0.26);
        let belly = look(0.9, 0.8, 0.62);
        let feather = look(0.2, 0.26, 0.4);
        let pale = look(0.75, 0.78, 0.8);
        let bead = look(0.03, 0.03, 0.03);
        let white = look(0.92, 0.9, 0.78);

        let mut parts =
            vec![
                SculptedPart::new(Ellipsoid::new((0.13, 0.17, 0.42), (0.0, 0.0, 0.0)), scales)
                    .warped(Warp::Taper {
                        along: -Vector3::z(),
                        tip: 0.2,
                    }),
                SculptedPart::new(Ellipsoid::new((0.11, 0.1, 0.32), (0.0, -0.08, 0.04)), belly)
                    .warped(Warp::Taper {
                        along: -Vector3::z(),
                        tip: 0.3,
                    }),
                // Tail, forked.
                SculptedPart::new(
                    Ellipsoid::new((0.01, 0.12, 0.14), (0.0, 0.08, -0.5)).pitched(-0.5),
                    scales,
                ),
                SculptedPart::new(
                    Ellipsoid::new((0.01, 0.12, 0.14), (0.0, -0.08, -0.5)).pitched(0.5),
                    scales,
                ),
            ];
        for side in [-1.0, 1.0] {
            // A wing each side, raised and swept back, in two rows of
            // feathers.
            parts.push(
                SculptedPart::new(
                    Ellipsoid::new((0.5, 0.015, 0.2), (0.48 * side, 0.16, -0.02))
                        .rolled(0.35 * side)
                        .yawed(-0.3 * side),
                    feather,
                )
                .warped(Warp::Taper {
                    along: Vector3::x() * side,
                    tip: 0.25,
                }),
            );
            parts.push(
                SculptedPart::new(
                    Ellipsoid::new((0.36, 0.012, 0.13), (0.4 * side, 0.15, -0.16))
                        .rolled(0.35 * side)
                        .yawed(-0.45 * side),
                    pale,
                )
                .warped(Warp::Taper {
                    along: Vector3::x() * side,
                    tip: 0.2,
                }),
            );
            parts.push(SculptedPart::new(
                Ellipsoid::new((0.02, 0.05, 0.05), (0.105 * side, 0.05, 0.24)),
                white,
            ));
            parts.push(SculptedPart::new(
                Ellipsoid::new((0.02, 0.025, 0.025), (0.118 * side, 0.05, 0.245)),
                bead,
            ));
        }
        painted_model(&parts, &MOTTLE, &SHADE)
    }
}

impl FlyingFishDef {
    /// The states it is shown in by tools.
    pub fn exhibits() -> Vec<Exhibit> {
        vec![Exhibit::at_rest(
            "portrait",
            Arc::new(Self::model(Paint::Skin.id())),
            1.2,
            1.2,
        )]
    }
}

impl Spawnable for FlyingFishDef {
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
        // Perched: it takes off when it is first flown.
        let at = Point3::new(self.pos.0, ground + RADIUS, self.pos.1);
        let (body, mass) = {
            let mut physics = world.write_resource::<PhysicsResource>();
            let body = physics
                .world
                .create_body(RigidBodyDesc::dynamic().position(at).angular_damping(0.9));
            physics.world.attach_collider(
                body,
                ColliderDesc::sphere(RADIUS)
                    .density(DENSITY)
                    .restitution(0.0),
            );
            let mass = physics.world.body(body).map_or(1.0, |body| body.mass());
            (body, mass)
        };
        vec![world
            .create_entity()
            .with(Position(at.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(body))
            .with(ModelInstance::new(Arc::new(Self::model(materials[0]))))
            .with(Renderable)
            .with(CharacterIntent::default())
            .with(all_round(SIGHT))
            .with(Brain::hunter(0.5 * REACH))
            .with(Flyer {
                mass,
                yaw_inertia: 0.4 * mass * RADIUS * RADIUS,
                speed: SPEED,
                accel_time: ACCEL_TIME,
                cruise_height: CRUISE_HEIGHT,
                stoop_range: STOOP_RANGE,
                stoop_height: 0.4,
            })
            .with(MeleeAttack::new(BITE, REACH).with_timing(BITE_WINDUP, BITE_RECOVERY))
            .with(Health::new(HEALTH, 5.0))
            .build()]
    }
}
