//! Dolos spawnable — concrete breakwater armour unit.
//!
//! A dolos has a central shank with a fluke at each end, the two flukes
//! perpendicular to each other. The resulting shape interlocks with its
//! neighbours on a breakwater and is notoriously awkward to stack flat —
//! which makes it a fun obstacle.

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::compound_cuboid_model;
use super::shared::textures::Rgb;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::{Material, MaterialId};
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 128;

#[derive(Deserialize)]
pub struct DolosDef {
    pub pos: (f32, f32, f32),
    /// End-to-end length of the central shank.
    #[serde(default = "DolosDef::default_shank_length")]
    pub shank_length: f32,
    /// End-to-end length of each perpendicular fluke.
    #[serde(default = "DolosDef::default_fluke_length")]
    pub fluke_length: f32,
    /// Side length of the square cross-section of all three bars.
    #[serde(default = "DolosDef::default_thickness")]
    pub thickness: f32,
    /// Concrete density by default.
    #[serde(default = "DolosDef::default_density")]
    pub density: f32,
    #[serde(default = "DolosDef::default_restitution")]
    pub restitution: f32,
    #[serde(default = "DolosDef::default_friction")]
    pub friction: f32,
}

impl DolosDef {
    pub fn default_shank_length() -> f32 {
        1.4
    }
    pub fn default_fluke_length() -> f32 {
        1.0
    }
    pub fn default_thickness() -> f32 {
        0.42
    }
    /// Concrete.
    pub fn default_density() -> f32 {
        2400.0
    }
    pub fn default_restitution() -> f32 {
        0.1
    }
    pub fn default_friction() -> f32 {
        0.8
    }
}

impl Spawnable for DolosDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let pixels = generate_concrete_texture(rand::random::<u32>());
        let texture = ctx
            .textures
            .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
        Ok(vec![ctx.materials.register(Material::textured(texture))])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let initial_pos = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let material = materials[0];

        let half_shank = self.shank_length * 0.5;
        let half_fluke = self.fluke_length * 0.5;
        let half_thk = self.thickness * 0.5;

        // Shank runs along Z. Flukes sit flush at each end, offset inward by
        // half the thickness so the bar ends meet cleanly.
        let shank_he = Vector3::new(half_thk, half_thk, half_shank);
        let shank_offset = Vector3::zeros();

        let fluke_a_he = Vector3::new(half_fluke, half_thk, half_thk);
        let fluke_a_offset = Vector3::new(0.0, 0.0, half_shank - half_thk);

        // Second fluke along Y — perpendicular to the first.
        let fluke_b_he = Vector3::new(half_thk, half_fluke, half_thk);
        let fluke_b_offset = Vector3::new(0.0, 0.0, -(half_shank - half_thk));

        let boxes = [
            (shank_he, shank_offset),
            (fluke_a_he, fluke_a_offset),
            (fluke_b_he, fluke_b_offset),
        ];
        let model = compound_cuboid_model(&boxes, material);

        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body_desc = RigidBodyDesc::dynamic()
                .position(initial_pos)
                .gravity_scale(1.0)
                .linear_damping(0.01)
                .angular_damping(0.02);

            let body_handle = physics.world.create_body(body_desc);

            for &(half_extents, offset) in &boxes {
                physics.world.attach_collider(
                    body_handle,
                    ColliderDesc::box_shape(half_extents)
                        .offset_translation(offset)
                        .density(self.density)
                        .restitution(self.restitution)
                        .friction(self.friction),
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

/// Weathered grey concrete with aggregate speckle and subtle blotching.
fn generate_concrete_texture(seed: u32) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let base = Rgb::new(0.62, 0.62, 0.60);
    let dark = Rgb::new(0.42, 0.42, 0.40);
    let light = Rgb::new(0.78, 0.78, 0.76);

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let blotch = fbm_2d_periodic(u * 4.0, v * 4.0, 3, 0.5, 2.0, seed, Some(4));
            let t = (blotch * 0.5 + 0.5).clamp(0.0, 1.0);
            let mut colour = base.lerp(light, t * 0.4);

            let stain = fbm_2d_periodic(
                u * 2.5,
                v * 2.5,
                2,
                0.5,
                2.0,
                seed.wrapping_add(17),
                Some(3),
            );
            if stain > 0.2 {
                let st = ((stain - 0.2) / 0.5).clamp(0.0, 0.6);
                colour = colour.lerp(dark, st);
            }

            let aggregate = fbm_2d_periodic(
                u * 32.0,
                v * 32.0,
                2,
                0.5,
                2.0,
                seed.wrapping_add(29),
                Some(32),
            );
            colour = colour.scale(0.90 + aggregate * 0.10);

            colour.write_rgba(&mut pixels);
        }
    }
    pixels
}
