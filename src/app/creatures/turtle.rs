//! Turtle — a sea turtle that cruises the shallows.
//!
//! A swimmer ([`Swimmer`]) like the fish, slower and calmer: it keeps just
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
use crate::rendering::material::{Material, MaterialId, SurfaceFinish};
use crate::systems::PhysicsResource;

use super::body_parts::Ellipsoid;
use super::in_water::spawn_point_in_water;

/// As dense as the water: buoyancy carries it and its flippers only move it.
const DENSITY: f32 = 1000.0;

/// Radius of the sphere it collides as, m.
const COLLIDER_RADIUS: f32 = 0.3;

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

impl Spawnable for TurtleDef {
    fn material_count(&self) -> usize {
        2
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        // Shell and eyes are glossy; skin is not.
        let with = |finish| Material::coloured(Colour::WHITE).with_finish(finish);
        Ok(vec![
            ctx.materials.register(with(SurfaceFinish::POLISHED)),
            ctx.materials.register(with(SurfaceFinish::MATTE)),
        ])
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

        let (glossy, matte) = (materials[0], materials[1]);
        // Shell a warm olive-brown, its rim and ridge amber, skin sage,
        // plastron cream.
        let shell = (Colour::new(0.36, 0.3, 0.12, 1.0), glossy);
        let rim = (Colour::new(0.62, 0.48, 0.2, 1.0), glossy);
        let skin = (Colour::new(0.42, 0.55, 0.36, 1.0), matte);
        let plastron = (Colour::new(0.88, 0.82, 0.62, 1.0), matte);
        let eye = (Colour::new(0.02, 0.02, 0.03, 1.0), glossy);
        let parts = [
            (Ellipsoid::new((0.3, 0.13, 0.38), (0.0, 0.05, 0.0)), shell),
            (Ellipsoid::new((0.33, 0.05, 0.41), (0.0, 0.0, 0.0)), rim),
            (
                Ellipsoid::new((0.29, 0.06, 0.36), (0.0, -0.03, 0.0)),
                plastron,
            ),
            // A ridge of scutes down the back.
            (Ellipsoid::new((0.1, 0.06, 0.12), (0.0, 0.16, 0.14)), rim),
            (Ellipsoid::new((0.11, 0.065, 0.13), (0.0, 0.17, -0.06)), rim),
            (Ellipsoid::new((0.09, 0.08, 0.12), (0.0, 0.03, 0.47)), skin),
            (
                Ellipsoid::new((0.24, 0.025, 0.08), (0.36, -0.01, 0.2))
                    .yawed(0.5)
                    .rolled(-0.15),
                skin,
            ),
            (
                Ellipsoid::new((0.24, 0.025, 0.08), (-0.36, -0.01, 0.2))
                    .yawed(-0.5)
                    .rolled(0.15),
                skin,
            ),
            (
                Ellipsoid::new((0.11, 0.02, 0.07), (0.24, -0.02, -0.35)).yawed(-0.6),
                skin,
            ),
            (
                Ellipsoid::new((0.11, 0.02, 0.07), (-0.24, -0.02, -0.35)).yawed(0.6),
                skin,
            ),
            (
                Ellipsoid::new((0.016, 0.016, 0.016), (0.06, 0.06, 0.55)),
                eye,
            ),
            (
                Ellipsoid::new((0.016, 0.016, 0.016), (-0.06, 0.06, 0.55)),
                eye,
            ),
        ];
        let model = Arc::new(Model::flat(vec![ModelPart::new(
            parts
                .iter()
                .map(|(part, (colour, material))| part.mesh(*colour, *material))
                .collect(),
        )]));

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
