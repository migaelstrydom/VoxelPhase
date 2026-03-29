//! Trampoline spawnable — bouncy pad on four short legs.
//!
//! A compound body with a high-restitution top surface that launches
//! anything that lands on it. The legs are low-restitution so only the
//! pad bounces.

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::multi_material_compound_cuboid_model;
use super::shared::textures::*;
use super::spawnable::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::{Material, MaterialId};
use crate::systems::PhysicsResource;

const TEXTURE_SIZE: u32 = 128;

#[derive(Deserialize)]
pub struct TrampolineDef {
    pub pos: (f32, f32, f32),
    /// Half-extents of the bouncy pad (x, y_thickness, z).
    #[serde(default = "TrampolineDef::default_pad_half_extents")]
    pub pad_half_extents: (f32, f32, f32),
    /// Half-extents of each leg.
    #[serde(default = "TrampolineDef::default_leg_half_extents")]
    pub leg_half_extents: (f32, f32, f32),
    /// Density of the entire trampoline.
    #[serde(default = "TrampolineDef::default_density")]
    pub density: f32,
    /// Restitution of the pad surface. Values above 1.0 add energy (super-bounce).
    #[serde(default = "TrampolineDef::default_restitution")]
    pub restitution: f32,
}

impl TrampolineDef {
    pub fn default_pad_half_extents() -> (f32, f32, f32) {
        (1.8, 0.06, 1.8)
    }
    pub fn default_leg_half_extents() -> (f32, f32, f32) {
        (0.12, 0.36, 0.12)
    }
    pub fn default_density() -> f32 {
        300.0
    }
    pub fn default_restitution() -> f32 {
        1.5
    }
}

impl TrampolineDef {
    fn pad_he(&self) -> Vector3<f32> {
        Vector3::new(
            self.pad_half_extents.0,
            self.pad_half_extents.1,
            self.pad_half_extents.2,
        )
    }

    fn leg_he(&self) -> Vector3<f32> {
        Vector3::new(
            self.leg_half_extents.0,
            self.leg_half_extents.1,
            self.leg_half_extents.2,
        )
    }

    fn total_height(&self) -> f32 {
        self.leg_he().y * 2.0 + self.pad_he().y * 2.0
    }
}

impl Spawnable for TrampolineDef {
    fn material_count(&self) -> usize {
        2
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        // [0] = fabric pad, [1] = metal legs
        let fabric_pixels = generate_trampoline_fabric();
        let fabric_tex =
            ctx.textures
                .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &fabric_pixels, true)?;
        let fabric_mat = ctx.materials.register(Material::textured(fabric_tex));

        let metal_pixels = generate_leg_metal();
        let metal_tex =
            ctx.textures
                .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &metal_pixels, true)?;
        let metal_mat = ctx.materials.register(Material::textured(metal_tex));

        Ok(vec![fabric_mat, metal_mat])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let initial_pos = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let pad_he = self.pad_he();
        let leg_he = self.leg_he();
        let total_height = self.total_height();

        let pad_y = total_height * 0.5 - pad_he.y;
        let leg_y = -pad_he.y;
        let leg_x = pad_he.x - leg_he.x;
        let leg_z = pad_he.z - leg_he.z;

        let leg_positions = [
            Vector3::new(-leg_x, leg_y, -leg_z),
            Vector3::new(leg_x, leg_y, -leg_z),
            Vector3::new(-leg_x, leg_y, leg_z),
            Vector3::new(leg_x, leg_y, leg_z),
        ];

        // Build compound model — pad gets fabric, legs get metal
        let fabric_mat = materials[0];
        let metal_mat = materials[1];
        let mut boxes = vec![(pad_he, Vector3::new(0.0, pad_y, 0.0), fabric_mat)];
        for &pos in &leg_positions {
            boxes.push((leg_he, pos, metal_mat));
        }
        let model = multi_material_compound_cuboid_model(&boxes);

        // Create compound physics body
        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body_desc = RigidBodyDesc::dynamic()
                .position(initial_pos)
                .gravity_scale(1.0)
                .linear_damping(0.01)
                .angular_damping(0.005);

            let body_handle = physics.world.create_body(body_desc);

            // Bouncy pad — high restitution
            physics.world.attach_collider(
                body_handle,
                ColliderDesc::box_shape(pad_he)
                    .offset_translation(Vector3::new(0.0, pad_y, 0.0))
                    .density(self.density)
                    .restitution(self.restitution)
                    .friction(0.8),
            );

            // Four legs — low restitution, just structural
            for &pos in &leg_positions {
                physics.world.attach_collider(
                    body_handle,
                    ColliderDesc::box_shape(leg_he)
                        .offset_translation(pos)
                        .density(self.density)
                        .restitution(0.1)
                        .friction(0.6),
                );
            }

            body_handle
        };

        vec![world
            .create_entity()
            .with(Position(Vector3::new(
                initial_pos.x,
                initial_pos.y,
                initial_pos.z,
            )))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(body_handle))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .build()]
    }
}

// ---------------------------------------------------------------------------
// Procedural trampoline fabric texture
// ---------------------------------------------------------------------------

/// Generates a brushed steel texture for the trampoline legs.
fn generate_leg_metal() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let base = Rgb::new(0.55, 0.57, 0.62);
    let seed = rand_u32();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let brush = crate::utils::noise::fbm_2d_periodic(
                u * 6.0, v * 30.0, 3, 0.5, 2.0, seed, Some(6),
            );
            let scratches = crate::utils::noise::fbm_2d_periodic(
                u * 20.0, v * 20.0, 2, 0.3, 2.0, seed + 7, Some(20),
            );
            let c_factor = 0.82 + brush * 0.12 + scratches * 0.06;
            let c = base.scale(c_factor).scale(edge_vignette(u, v));

            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

/// Generates a woven fabric texture with bold concentric target rings
/// and a cross-hatch weave pattern.
fn generate_trampoline_fabric() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    // Pick a vibrant base colour for the fabric
    let hue = rand_range(0.0, 6.0);
    let base = hue_to_rgb(hue, 0.70, 0.85);
    let accent = hue_to_rgb((hue + 3.0) % 6.0, 0.65, 0.90);
    let seed = rand_u32();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Concentric rings from center — alternating base/accent
            let cx = (u - 0.5) * 2.0;
            let cy = (v - 0.5) * 2.0;
            let dist = (cx * cx + cy * cy).sqrt();
            let ring = ((dist * 6.0).sin() * 0.5 + 0.5).powf(0.4);
            let mut c = base.lerp(accent, ring * 0.6);

            // Cross-hatch weave pattern — subtle grid of light/dark threads
            let thread_u = ((u * size as f32 * 2.0).sin() * 0.5 + 0.5).powf(4.0);
            let thread_v = ((v * size as f32 * 2.0).sin() * 0.5 + 0.5).powf(4.0);
            let weave = (thread_u + thread_v) * 0.5;
            c = c.scale(0.92 + weave * 0.08);

            // Subtle fabric noise
            let noise = crate::utils::noise::fbm_2d_periodic(
                u * 16.0,
                v * 16.0,
                2,
                0.3,
                2.0,
                seed,
                Some(16),
            );
            c = c.scale(0.94 + noise * 0.06);

            // Edge border — darker frame ring
            let edge = border_band(u, v, 0.08);
            c = c.scale(1.0 - edge * 0.4);

            c.write_rgba(&mut pixels);
        }
    }
    pixels
}
