//! Pyramid spawnable — square-based pyramid of sandstone blocks.
//!
//! Builds a pyramid from the base up, with each layer a square grid one block
//! smaller on each side. Each block gets its own procedurally generated
//! sandstone texture.

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::Entity;

use super::shared::finish::{ColliderSurface, MaterialSurface};
use super::shared::models::cuboid_model;
use super::shared::textures::*;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::{Material, MaterialId, MaterialManagerBuilder};
use crate::rendering::physical_finish::PhysicalSurface;
use crate::resources::textures::TextureManager;
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

use super::shared::textures::seed_from_position;
use super::shared::textures::TextureRng;
use specs::{Builder, WorldExt};

const TEXTURE_SIZE: u32 = 128;

#[derive(Deserialize)]
pub struct PyramidDef {
    pub base: (f32, f32, f32),
    pub block_half_extents: (f32, f32, f32),
    /// Number of blocks along each side of the bottom layer.
    pub base_width: u32,
    #[serde(default = "PyramidDef::default_density")]
    pub density: f32,
}

impl PyramidDef {
    pub fn default_density() -> f32 {
        1800.0
    }

    /// The one declaration of this object's physics. The collider takes the
    /// coefficients and the material takes the finish they imply, so the two
    /// cannot drift apart.
    fn surface(&self) -> PhysicalSurface {
        PhysicalSurface {
            restitution: 0.15,
            friction: 0.7,
            density: self.density,
        }
    }
}

impl PyramidDef {
    fn total_blocks(&self) -> usize {
        (1..=self.base_width).map(|n| n * n).sum::<u32>() as usize
    }
}

impl Spawnable for PyramidDef {
    fn material_count(&self) -> usize {
        self.total_blocks()
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let count = self.total_blocks();
        let mut mats = Vec::with_capacity(count);
        for block in 0..count {
            mats.push(create_sandstone_material(
                self.surface(),
                seed_from_position(self.base, block as u32),
                ctx.textures,
                ctx.materials,
            )?);
        }
        Ok(mats)
    }

    fn spawn(&self, world: &mut specs::World, materials: &[MaterialId]) -> Vec<Entity> {
        let he = Vector3::new(
            self.block_half_extents.0,
            self.block_half_extents.1,
            self.block_half_extents.2,
        );
        let block_w = he.x * 2.0;
        let block_d = he.z * 2.0;
        let block_h = he.y * 2.0;

        let mut entities = Vec::with_capacity(self.total_blocks());
        let mut mat_idx = 0;

        for layer in 0..self.base_width {
            let side = self.base_width - layer;
            let y = self.base.1 + he.y + layer as f32 * block_h;
            let extent_x = side as f32 * block_w;
            let extent_z = side as f32 * block_d;
            let start_x = self.base.0 - extent_x * 0.5 + he.x;
            let start_z = self.base.2 - extent_z * 0.5 + he.z;

            for row in 0..side {
                for col in 0..side {
                    let x = start_x + col as f32 * block_w;
                    let z = start_z + row as f32 * block_d;
                    let pos = Point3::new(x, y, z);
                    let model = cuboid_model(he, materials[mat_idx]);

                    let body_handle = {
                        let mut physics = world.write_resource::<PhysicsResource>();
                        let body_desc = RigidBodyDesc::dynamic()
                            .position(pos)
                            .gravity_scale(1.0)
                            .linear_damping(0.01)
                            .angular_damping(0.005);
                        let body_handle = physics.world.create_body(body_desc);
                        physics.world.attach_collider(
                            body_handle,
                            ColliderDesc::box_shape(he).with_physical_surface(self.surface()),
                        );
                        body_handle
                    };

                    entities.push(
                        world
                            .create_entity()
                            .with(Position(Vector3::new(pos.x, pos.y, pos.z)))
                            .with(Velocity(Vector3::zeros()))
                            .with(Orientation::default())
                            .with(RigidBodyComponent(body_handle))
                            .with(ModelInstance::new(model))
                            .with(Renderable)
                            .build(),
                    );

                    mat_idx += 1;
                }
            }
        }

        entities
    }
}

// ---------------------------------------------------------------------------
// Sandstone texture generation
// ---------------------------------------------------------------------------

fn create_sandstone_material(
    surface: PhysicalSurface,
    seed: u32,
    texture_manager: &TextureManager,
    material_builder: &mut MaterialManagerBuilder,
) -> EngineResult<MaterialId> {
    let pixels = generate_sandstone(seed);
    let texture = texture_manager.create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
    let material = Material::textured(texture).with_derived_finish(surface);
    Ok(material_builder.register(material))
}

/// Procedural sandstone: warm sandy base with horizontal sediment bands,
/// fine grain noise, and subtle erosion pitting.
fn generate_sandstone(seed: u32) -> Vec<u8> {
    let rng = &mut TextureRng::new(seed);
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let warmth = rng.range(-0.03, 0.03);
    let base = Rgb::new(0.82 + warmth, 0.72 + warmth * 0.8, 0.55 + warmth * 0.5);
    let dark_band = Rgb::new(0.68, 0.58, 0.42);

    let seed_grain = rng.u32();
    let seed_sediment = rng.u32();
    let seed_erosion = rng.u32();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Fine grain noise — gives the sandy, granular look
            let grain = fbm_2d_periodic(u * 16.0, v * 16.0, 3, 0.5, 2.0, seed_grain, Some(16));
            let grain_factor = 0.88 + grain * 0.12;

            // Horizontal sediment layering — bands of slightly darker stone
            let sediment_raw =
                fbm_2d_periodic(u * 2.0, v * 12.0, 3, 0.6, 2.0, seed_sediment, Some(12));
            let band = ((sediment_raw * 6.0).sin() * 0.5 + 0.5).powi(2);
            let band_strength = band * 0.3;

            let mut c = base.lerp(dark_band, band_strength);
            c = c.scale(grain_factor);

            // Erosion pitting — small dark spots
            let erosion = fbm_2d_periodic(u * 20.0, v * 20.0, 2, 0.4, 2.0, seed_erosion, Some(20));
            if erosion > 0.65 {
                let pit = (erosion - 0.65) / 0.35;
                c = c.scale(1.0 - pit * 0.25);
            }

            // Subtle chisel edge around the block
            let edge = border_band(u, v, 0.05);
            c = c.scale(1.0 - edge * 0.2);

            c = c.scale(edge_vignette(u, v));
            c.write_rgba(&mut pixels);
        }
    }
    pixels
}
