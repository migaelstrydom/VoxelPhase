//! Capsule spawnable — cylinder with hemisphere caps and two-tone pill texture.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::textures::hue_to_rgb;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::geometry::generate_capsule;
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId};
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 128;
const MESH_SEGMENTS: u32 = 24;
const CAP_RINGS: u32 = 12;

#[derive(Deserialize)]
pub struct CapsuleDef {
    pub pos: (f32, f32, f32),
    pub half_height: f32,
    pub radius: f32,
    #[serde(default = "CapsuleDef::default_density")]
    pub density: f32,
    #[serde(default = "CapsuleDef::default_restitution")]
    pub restitution: f32,
    #[serde(default = "CapsuleDef::default_friction")]
    pub friction: f32,
}

impl CapsuleDef {
    pub fn default_density() -> f32 {
        50.0
    }
    pub fn default_restitution() -> f32 {
        0.2
    }
    pub fn default_friction() -> f32 {
        0.6
    }
}

impl Spawnable for CapsuleDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let pixels = generate_pill_texture();
        let texture = ctx
            .textures
            .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
        let material = Material::textured(texture);
        Ok(vec![ctx.materials.register(material)])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let initial_pos = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let material = materials[0];

        let (vertices, indices) = generate_capsule(
            self.half_height,
            self.radius,
            MESH_SEGMENTS,
            CAP_RINGS,
            Colour::WHITE,
        );
        let parts = vec![ModelPart::new(vec![MeshPrimitive {
            vertices,
            indices,
            material,
        }])];
        let model = Arc::new(Model::flat(parts));

        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body_desc = RigidBodyDesc::dynamic()
                .position(initial_pos)
                .gravity_scale(1.0)
                .linear_damping(0.01)
                .angular_damping(0.005);

            let body_handle = physics.world.create_body(body_desc);

            let collider_desc = ColliderDesc::capsule(self.half_height, self.radius)
                .density(self.density)
                .restitution(self.restitution)
                .friction(self.friction);

            physics.world.attach_collider(body_handle, collider_desc);

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

/// Generate a two-tone pill/medicine capsule texture.
fn generate_pill_texture() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let hue_a = rand::random::<f32>() * 6.0;
    let hue_b = (hue_a + 3.0) % 6.0;
    let colour_a = hue_to_rgb(hue_a, 0.55, 0.85);
    let colour_b = hue_to_rgb(hue_b, 0.55, 0.85);
    let seed = rand::random::<u32>();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let base = if v < 0.48 {
                colour_a
            } else if v > 0.52 {
                colour_b
            } else {
                let t = (v - 0.48) / 0.04;
                colour_a.lerp(colour_b, t).scale(0.7)
            };

            let noise = fbm_2d_periodic(u * 8.0, v * 8.0, 3, 0.4, 2.0, seed, Some(8));
            let factor = 0.90 + noise * 0.10;
            let c = base.scale(factor);

            let highlight = ((u - 0.3).abs() / 0.15).min(1.0);
            let c = c.scale(0.95 + (1.0 - highlight) * 0.08);

            c.write_rgba(&mut pixels);
        }
    }
    pixels
}
