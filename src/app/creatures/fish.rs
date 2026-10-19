//! Fish — a small reef fish that darts away from the player.
//!
//! A swimmer ([`Swimmer`]) with a skittish brain: it noses about the water
//! until something comes close, then bolts. It cannot be hurt and is not a
//! threat; it is there to make the water look lived in. Its colours are a
//! reef's, drawn from its position, so a shoal authored in one spot is not a
//! row of copies.

use std::f32::consts::PI;
use std::sync::Arc;

use nalgebra::Vector3;
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use crate::app::spawnables::shared::textures::seed_from_ground;
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

/// As dense as the water, so it neither sinks nor floats: buoyancy holds it
/// where it swims and its fins only have to move it.
const DENSITY: f32 = 1000.0;

/// Radius of the sphere it collides as, m: its body's depth, not its length,
/// so it can slip close past a rock.
const COLLIDER_RADIUS: f32 = 0.1;

/// A reef's colours: body, band and fin. Picked by the fish's position.
const LIVERIES: [(Colour, Colour, Colour); 3] = [
    // Clownfish: orange, white bands, black-edged fins.
    (
        Colour::new(1.0, 0.42, 0.05, 1.0),
        Colour::new(0.97, 0.97, 0.95, 1.0),
        Colour::new(0.95, 0.35, 0.04, 1.0),
    ),
    // Yellow tang: yellow all over, a pale band.
    (
        Colour::new(1.0, 0.86, 0.08, 1.0),
        Colour::new(1.0, 0.95, 0.6, 1.0),
        Colour::new(1.0, 0.78, 0.05, 1.0),
    ),
    // Blue tang: royal blue, a yellow tail.
    (
        Colour::new(0.08, 0.3, 0.95, 1.0),
        Colour::new(0.05, 0.08, 0.25, 1.0),
        Colour::new(1.0, 0.82, 0.1, 1.0),
    ),
];

/// A fish. Position is `(x, z)`; it starts in the water over that point.
#[derive(Deserialize)]
pub struct FishDef {
    pub pos: (f32, f32),
    /// How close the player may come before it bolts, m.
    #[serde(default = "FishDef::default_flee_range")]
    pub flee_range: f32,
}

impl FishDef {
    pub fn default_flee_range() -> f32 {
        4.0
    }

    fn mass() -> f32 {
        (4.0 / 3.0) * PI * COLLIDER_RADIUS.powi(3) * DENSITY
    }

    fn livery(&self) -> (Colour, Colour, Colour) {
        LIVERIES[seed_from_ground(self.pos, 7) as usize % LIVERIES.len()]
    }
}

impl Spawnable for FishDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        // Wet and glossy: a fish's scales are the shiniest thing in the water.
        let glossy = Material::coloured(Colour::WHITE).with_finish(SurfaceFinish::POLISHED);
        Ok(vec![ctx.materials.register(glossy)])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let swimmer = Swimmer {
            mass: Self::mass(),
            yaw_inertia: 0.4 * Self::mass() * COLLIDER_RADIUS * COLLIDER_RADIUS,
            sprint_speed: 3.0,
            cruise_speed: 0.9,
            accel_time: 0.25,
            depth_share: 0.25,
            clearance: 0.2,
            beat_rate: 4.0,
            beat_swing: 0.25,
            beat_phase: self.pos.0 * 3.1 + self.pos.1 * 1.7,
        };
        let Some(start) =
            spawn_point_in_water(world, self.pos, swimmer.depth_share, swimmer.clearance)
        else {
            return Vec::new();
        };

        let (body, band, fin) = self.livery();
        let white = Colour::new(0.95, 0.95, 0.95, 1.0);
        let black = Colour::new(0.02, 0.02, 0.03, 1.0);
        let parts = [
            (Ellipsoid::new((0.07, 0.11, 0.19), (0.0, 0.0, 0.0)), body),
            // Bands stand a hair proud of the body so they read as markings.
            (
                Ellipsoid::new((0.073, 0.114, 0.035), (0.0, 0.0, 0.08)),
                band,
            ),
            (Ellipsoid::new((0.064, 0.1, 0.03), (0.0, 0.0, -0.07)), band),
            (Ellipsoid::new((0.012, 0.1, 0.07), (0.0, 0.0, -0.24)), fin),
            (Ellipsoid::new((0.01, 0.06, 0.09), (0.0, 0.1, -0.03)), fin),
            (
                Ellipsoid::new((0.05, 0.012, 0.04), (0.06, -0.04, 0.03))
                    .yawed(-0.6)
                    .rolled(-0.5),
                fin,
            ),
            (
                Ellipsoid::new((0.05, 0.012, 0.04), (-0.06, -0.04, 0.03))
                    .yawed(0.6)
                    .rolled(0.5),
                fin,
            ),
            (
                Ellipsoid::new((0.024, 0.024, 0.024), (0.05, 0.03, 0.12)),
                white,
            ),
            (
                Ellipsoid::new((0.024, 0.024, 0.024), (-0.05, 0.03, 0.12)),
                white,
            ),
            (
                Ellipsoid::new((0.013, 0.013, 0.013), (0.069, 0.032, 0.128)),
                black,
            ),
            (
                Ellipsoid::new((0.013, 0.013, 0.013), (-0.069, 0.032, 0.128)),
                black,
            ),
        ];
        let model = Arc::new(Model::flat(vec![ModelPart::new(
            parts
                .iter()
                .map(|(part, colour)| part.mesh(*colour, materials[0]))
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
                    .friction(0.3),
            );
            // Level in the water; its fins turn it about the vertical only.
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
            .with(Perception::ground_creature(10.0))
            .with(Brain::skittish(self.flee_range))
            .with(swimmer)
            .build()]
    }
}
