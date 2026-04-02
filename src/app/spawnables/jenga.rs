//! Jenga tower spawnable — alternating layers of three planks rotated 90°.
//!
//! Uses real Jenga proportions (each block is 3× as long as the tower is wide,
//! 1/3 tower-width across, and 1/5 of the length tall). Each layer rotates 90°
//! so the tower is self-supporting and extremely sensitive to solver precision.

use nalgebra::{Point3, UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::cuboid_model;
use super::shared::textures::*;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::{Material, MaterialId, MaterialManagerBuilder};
use crate::resources::textures::TextureManager;
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 128;
const BLOCKS_PER_LAYER: u32 = 3;

#[derive(Deserialize)]
pub struct JengaDef {
    /// World position of the tower base center (ground level).
    pub base: (f32, f32, f32),
    /// Number of layers (standard Jenga has 18).
    #[serde(default = "JengaDef::default_layers")]
    pub layers: u32,
    /// Half-length of each block along its long axis.
    #[serde(default = "JengaDef::default_block_half_length")]
    pub block_half_length: f32,
    #[serde(default = "JengaDef::default_density")]
    pub density: f32,
    #[serde(default = "JengaDef::default_friction")]
    pub friction: f32,
}

impl JengaDef {
    pub fn default_layers() -> u32 {
        18
    }
    pub fn default_block_half_length() -> f32 {
        0.75
    }
    pub fn default_density() -> f32 {
        500.0
    }
    pub fn default_friction() -> f32 {
        0.6
    }

    fn total_blocks(&self) -> usize {
        (self.layers * BLOCKS_PER_LAYER) as usize
    }

    /// Block dimensions derived from the half-length, maintaining real Jenga
    /// proportions: width = length/3, height = length/5.
    fn block_half_extents(&self) -> Vector3<f32> {
        let half_len = self.block_half_length;
        let half_width = half_len / 3.0;
        let half_height = half_len / 5.0;
        Vector3::new(half_len, half_height, half_width)
    }
}

impl Spawnable for JengaDef {
    fn material_count(&self) -> usize {
        self.total_blocks()
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let count = self.total_blocks();
        let mut mats = Vec::with_capacity(count);
        for _ in 0..count {
            mats.push(create_jenga_wood_material(ctx.textures, ctx.materials)?);
        }
        Ok(mats)
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let he = self.block_half_extents();
        let block_height = he.y * 2.0;
        let block_width = he.z * 2.0;

        let mut entities = Vec::with_capacity(self.total_blocks());
        let mut mat_idx = 0;

        let yaw_90 =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_2);

        for layer in 0..self.layers {
            let y = self.base.1 + he.y + layer as f32 * block_height;
            let rotated = layer % 2 == 1;

            for slot in 0..BLOCKS_PER_LAYER {
                // Offset each block within the layer: centered at 0, spaced by block_width.
                let lateral_offset =
                    (slot as f32 - (BLOCKS_PER_LAYER - 1) as f32 / 2.0) * block_width;

                let (x, z) = if rotated {
                    (self.base.0 + lateral_offset, self.base.2)
                } else {
                    (self.base.0, self.base.2 + lateral_offset)
                };

                let pos = Point3::new(x, y, z);
                let model = cuboid_model(he, materials[mat_idx]);

                let orientation = if rotated {
                    Orientation(yaw_90)
                } else {
                    Orientation::default()
                };

                let body_handle = {
                    let mut physics = world.write_resource::<PhysicsResource>();
                    let body_desc = RigidBodyDesc::dynamic()
                        .position(pos)
                        .rotation(orientation.0)
                        .gravity_scale(1.0)
                        .linear_damping(0.01)
                        .angular_damping(0.05);
                    let body_handle = physics.world.create_body(body_desc);
                    physics.world.attach_collider(
                        body_handle,
                        ColliderDesc::box_shape(he)
                            .density(self.density)
                            .restitution(0.05)
                            .friction(self.friction),
                    );
                    body_handle
                };

                entities.push(
                    world
                        .create_entity()
                        .with(Position(Vector3::new(pos.x, pos.y, pos.z)))
                        .with(Velocity(Vector3::zeros()))
                        .with(orientation)
                        .with(RigidBodyComponent(body_handle))
                        .with(ModelInstance::new(model))
                        .with(Renderable)
                        .build(),
                );

                mat_idx += 1;
            }
        }

        entities
    }
}

// ---------------------------------------------------------------------------
// Jenga wood texture generation
// ---------------------------------------------------------------------------

fn create_jenga_wood_material(
    texture_manager: &TextureManager,
    material_builder: &mut MaterialManagerBuilder,
) -> EngineResult<MaterialId> {
    let pixels = generate_jenga_wood_texture();
    let texture = texture_manager.create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
    let material = Material::textured(texture);
    Ok(material_builder.register(material))
}

/// Procedural pale wood: light birch/maple base with horizontal grain lines
/// and subtle knot features — the classic Jenga block look.
fn generate_jenga_wood_texture() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let warmth = rand_range(-0.02, 0.02);
    let base = Rgb::new(0.88 + warmth, 0.78 + warmth * 0.8, 0.62 + warmth * 0.6);
    let grain_dark = Rgb::new(0.72, 0.60, 0.44);

    let seed_grain = rand_u32();
    let seed_fine = rand_u32();
    let seed_knot = rand_u32();

    let knot_u = rand_range(0.2, 0.8);
    let knot_v = rand_range(0.3, 0.7);
    let has_knot = rand_range(0.0, 1.0) > 0.7;

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Horizontal wood grain — wavy lines running along u.
            let grain_warp = fbm_2d_periodic(u * 3.0, v * 1.0, 2, 0.5, 2.0, seed_grain, Some(3));
            let grain_v = v * 18.0 + grain_warp * 2.0;
            let grain_line = ((grain_v * std::f32::consts::PI).sin() * 0.5 + 0.5).powi(6);
            let grain_strength = grain_line * 0.25;

            // Fine noise for surface roughness.
            let fine = fbm_2d_periodic(u * 24.0, v * 24.0, 2, 0.4, 2.0, seed_fine, Some(24));
            let fine_factor = 0.94 + fine * 0.06;

            let mut c = base.lerp(grain_dark, grain_strength);
            c = c.scale(fine_factor);

            // Occasional wood knot — small dark oval.
            if has_knot {
                let du = (u - knot_u) * 1.5;
                let dv = v - knot_v;
                let knot_dist = (du * du + dv * dv).sqrt();
                if knot_dist < 0.08 {
                    let knot_ring =
                        fbm_2d_periodic(u * 40.0, v * 40.0, 2, 0.5, 2.0, seed_knot, Some(40));
                    let t = 1.0 - (knot_dist / 0.08);
                    let knot_dark = Rgb::new(0.50, 0.38, 0.26);
                    c = c.lerp(knot_dark, t * 0.6 + knot_ring * 0.15);
                }
            }

            // Subtle edge bevel — slightly lighter at borders.
            let edge = border_band(u, v, 0.04);
            c = c.scale(1.0 - edge * 0.12);

            c = c.scale(edge_vignette(u, v));
            c.write_rgba(&mut pixels);
        }
    }
    pixels
}
