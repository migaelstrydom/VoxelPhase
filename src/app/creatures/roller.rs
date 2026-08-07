//! Roller — a living boulder that hunts by rolling.
//!
//! The first creature, chosen because it needs no skeleton. It reuses the
//! brain, perception and steering layers wholesale and adds only a rolling
//! body.
//!
//! **A roller cannot be killed.** It has no `Health` and no `Flammable`, so
//! nothing in `crate::damage` touches it. You deal with one by outrunning it or
//! by blowing a hole in the ground and stranding it — and since explosion
//! knockback is applied to `Velocity` independently of damage, a grenade is
//! still the tool for the job, just as a way to *move* the creature rather than
//! to destroy it.
//!
//! That is a deliberate design choice, not an unfinished one. It play-tests
//! better than the alternative: trapping a roller uses the terrain destruction
//! the game is built around, where shooting it until it pops uses none of it.
//! It also avoids committing to a death model — corpses, ragdolls, respawns —
//! before there is a reason to prefer one.
//!
//! The consequence to keep in mind is that rollers are permanent. Nothing
//! despawns them, so a level's population only ever grows.

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
use crate::creature::{AlertTelegraph, Brain, Perception, Roller};
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

/// Contact friction. High, because friction is what turns drive torque into
/// travel — a slick roller would spin in place. Shared with `Roller::new`,
/// which derives the traction limit on its drive torque from it.
const FRICTION: f32 = 1.2;

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
    /// Seconds to reach `speed` from rest on the flat. Lower is more
    /// aggressive; below roughly `speed / (friction * g)` the ground cannot
    /// transmit the extra torque and the value stops having an effect.
    #[serde(default = "RollerDef::default_spin_up_time")]
    pub spin_up_time: f32,
    /// How far it can see. Hearing range is close and fixed.
    #[serde(default = "RollerDef::default_sight_range")]
    pub sight_range: f32,
}

impl RollerDef {
    pub fn default_radius() -> f32 {
        0.6
    }
    pub fn default_speed() -> f32 {
        4.5
    }
    pub fn default_spin_up_time() -> f32 {
        0.7
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
                    .friction(FRICTION)
                    .restitution(0.1),
            );
            body_handle
        };

        // Attack at the point of contact: a roller's attack is running into
        // you, so its range is the sum of the two bodies' radii plus a little
        // slack. Relentless because a roller has no health to break off over —
        // `next_behaviour` would reach the same conclusion from a missing
        // `Health`, but saying it here means the intent survives someone later
        // giving rollers health back.
        let brain = Brain::hunter(self.radius + 0.9).relentless();

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
            // A sphere has no face to pull, so the alert beat is expressed
            // as a hop and a shiver instead.
            .with(AlertTelegraph::ground_creature())
            .with(brain)
            .with(Roller::new(
                self.radius,
                self.mass(),
                self.speed,
                self.spin_up_time,
                FRICTION,
            ))
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
