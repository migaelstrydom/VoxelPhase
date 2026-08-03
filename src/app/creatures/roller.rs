//! Roller — a living boulder that hunts by rolling.
//!
//! The first creature, chosen because it needs no skeleton. It reuses the
//! brain, perception, steering and damage layers wholesale and adds only a
//! rolling body, so the question it answers is whether the combat loop is fun —
//! not whether the gait tuning is right.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use crate::app::spawnables::shared::textures::{hash_pair, rand_u32, Rgb};
use crate::app::spawnables::MaterialCtx;
use crate::app::spawnables::Spawnable;
use crate::character::CharacterIntent;
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::creature::{Brain, Perception, Roller};
use crate::damage::Health;
use crate::fire::components::Flammable;
use crate::geometry::{generate_sphere_indices, generate_sphere_vertices};
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId};
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;
use crate::utils::noise::fbm_2d_periodic;

const SEGMENTS: u32 = 24;
const RINGS: u32 = 18;
const TEXTURE_SIZE: u32 = 256;

/// Stone, so a roller is heavy enough to be a threat and to shrug off a
/// grazing blast.
const DENSITY: f32 = 2400.0;

/// A rolling boulder creature.
///
/// Position is `(x, z)` — the spawn height is taken from the terrain, since a
/// creature authored at a hand-picked Y would either hang in the air or start
/// embedded once the terrain around it is reshaped.
#[derive(Deserialize)]
pub struct RollerDef {
    pub pos: (f32, f32),

    #[serde(default = "RollerDef::default_radius")]
    pub radius: f32,
    /// Flat-ground top speed in m/s. The player walks at 5.0, so the default
    /// is a shade slower — a roller should be escapable on the flat and
    /// terrifying downhill.
    #[serde(default = "RollerDef::default_speed")]
    pub speed: f32,
    #[serde(default = "RollerDef::default_health")]
    pub health: f32,
    /// How far it can see. Hearing range is close and fixed.
    #[serde(default = "RollerDef::default_sight_range")]
    pub sight_range: f32,
    /// When true the roller never breaks off, however badly hurt.
    #[serde(default)]
    pub relentless: bool,
}

impl RollerDef {
    pub fn default_radius() -> f32 {
        0.6
    }
    pub fn default_speed() -> f32 {
        4.5
    }
    pub fn default_health() -> f32 {
        80.0
    }
    pub fn default_sight_range() -> f32 {
        25.0
    }

    /// Mass of the sphere, used to size the drive torque.
    fn mass(&self) -> f32 {
        (4.0 / 3.0) * std::f32::consts::PI * self.radius.powi(3) * DENSITY
    }
}

impl Spawnable for RollerDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let pixels = generate_hide_texture(rand_u32());
        let texture = ctx
            .textures
            .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
        Ok(vec![ctx.materials.register(Material::textured(texture))])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let surface_y = {
            let terrain = world.read_resource::<TerrainWorld>();
            terrain.mesh_surface_height_at(self.pos.0, self.pos.1)
        };
        // No terrain beneath the authored position means the level moved and
        // this creature's spot went with it. Dropping it silently beats
        // spawning something that falls forever.
        let Some(surface_y) = surface_y else {
            return Vec::new();
        };

        let initial_pos = Point3::new(self.pos.0, surface_y + self.radius, self.pos.1);

        let parts = vec![ModelPart::new(vec![MeshPrimitive {
            vertices: generate_sphere_vertices(self.radius, SEGMENTS, RINGS, Colour::WHITE),
            indices: generate_sphere_indices(SEGMENTS, RINGS),
            material: materials[0],
        }])];
        let model = Arc::new(Model::flat(parts));

        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();
            let body_handle = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(initial_pos)
                    .gravity_scale(1.0)
                    // Low linear damping so momentum carries downhill, but
                    // enough angular damping that a roller which loses its
                    // target coasts to a stop instead of spinning forever.
                    .linear_damping(0.05)
                    .angular_damping(0.4),
            );
            physics.world.attach_collider(
                body_handle,
                ColliderDesc::sphere(self.radius)
                    .density(DENSITY)
                    // High friction is what makes torque become travel. With a
                    // slick sphere the creature would spin in place.
                    .friction(1.2)
                    .restitution(0.1),
            );
            body_handle
        };

        let brain = {
            // Attack at the point of contact: a roller's attack is running
            // into you, so its range is the sum of the two bodies' radii plus
            // a little slack.
            let brain = Brain::hunter(self.radius + 0.9);
            if self.relentless {
                brain.relentless()
            } else {
                brain
            }
        };

        vec![world
            .create_entity()
            .with(Position(initial_pos.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(body_handle))
            .with(ModelInstance::new(model))
            .with(Renderable)
            // Intent is the seam: `BrainSystem` writes it, `RollerLocomotionSystem`
            // reads it. Neither knows about the other.
            .with(CharacterIntent::default())
            .with(Perception::ground_creature(self.sight_range))
            .with(brain)
            .with(Roller::new(self.radius, self.mass(), self.speed))
            // Stone takes blast and impact badly but barely notices fire.
            .with(
                Health::new(self.health, 8.0)
                    .with_burn_resistance(0.25)
                    .with_impact_tolerance(18.0),
            )
            // Flammable anyway: a burning boulder chasing you is worth the
            // one line, and the fuel is low so it goes out.
            .with(Flammable {
                fuel: 6.0,
                ignition_threshold: 0.4,
            })
            .build()]
    }
}

/// Mottled stone hide with darker veins, so the creature reads as rock but not
/// as terrain — a little warmer than the ground it rolls over.
fn generate_hide_texture(seed: u32) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let stone = Rgb::new(0.36, 0.32, 0.30);
    let vein = Rgb::new(0.14, 0.12, 0.13);

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let mottle = fbm_2d_periodic(u * 8.0, v * 8.0, 4, 0.5, 2.0, seed, Some(8));
            let veining = fbm_2d_periodic(
                u * 3.0,
                v * 3.0,
                2,
                0.6,
                2.0,
                seed.wrapping_add(101),
                Some(3),
            );

            // Sharp threshold on the low-frequency layer gives cracks rather
            // than a smooth gradient.
            let vein_strength = ((0.45 - veining) * 6.0).clamp(0.0, 1.0);

            let speck = hash_pair(x as i32, y as i32) as f32 / u32::MAX as f32;
            let base = stone.scale(0.8 + mottle * 0.4 + speck * 0.06);

            base.lerp(vein, vein_strength * 0.8).write_rgba(&mut pixels);
        }
    }
    pixels
}
