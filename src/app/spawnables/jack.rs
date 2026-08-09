//! Jack spawnable — the classic metal six-pointed toy from the game of jacks.
//!
//! Three perpendicular bars cross at the origin, giving the jack six spikes
//! along ±x, ±y, ±z. The shape has no stable resting face, so it settles on
//! three adjacent tips and rolls unpredictably when nudged — a fun hazard in a
//! platformer.

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::finish::{ColliderSurface, MaterialSurface};
use super::shared::models::compound_cuboid_model;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::{Material, MaterialId};
use crate::rendering::physical_finish::PhysicalSurface;
use crate::systems::PhysicsResource;

use super::shared::textures::hue_to_rgb;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 128;

#[derive(Deserialize)]
pub struct JackDef {
    pub pos: (f32, f32, f32),
    /// Tip-to-tip length of each bar.
    #[serde(default = "JackDef::default_length")]
    pub length: f32,
    /// Side length of the square cross-section of each bar.
    #[serde(default = "JackDef::default_thickness")]
    pub thickness: f32,
    #[serde(default = "JackDef::default_density")]
    pub density: f32,
    #[serde(default = "JackDef::default_restitution")]
    pub restitution: f32,
    #[serde(default = "JackDef::default_friction")]
    pub friction: f32,
}

impl JackDef {
    pub fn default_length() -> f32 {
        0.6
    }
    pub fn default_thickness() -> f32 {
        0.12
    }
    /// Steel — jacks are traditionally cast metal.
    pub fn default_density() -> f32 {
        7800.0
    }
    pub fn default_restitution() -> f32 {
        0.25
    }
    pub fn default_friction() -> f32 {
        0.5
    }

    /// The one declaration of this object's physics. The collider takes the
    /// coefficients and the material takes the finish they imply, so the two
    /// cannot drift apart.
    fn surface(&self) -> PhysicalSurface {
        PhysicalSurface {
            restitution: self.restitution,
            friction: self.friction,
            density: self.density,
        }
    }
}

impl Spawnable for JackDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let pixels = generate_jack_metal(rand::random::<u32>());
        let texture = ctx
            .textures
            .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
        Ok(vec![ctx.materials.register(
            Material::textured(texture).with_derived_finish(self.surface()),
        )])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let initial_pos = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let material = materials[0];

        let half_len = self.length * 0.5;
        let half_thk = self.thickness * 0.5;

        // Three perpendicular bars through origin: one along each axis.
        let bar_x = Vector3::new(half_len, half_thk, half_thk);
        let bar_y = Vector3::new(half_thk, half_len, half_thk);
        let bar_z = Vector3::new(half_thk, half_thk, half_len);
        let origin = Vector3::zeros();

        let boxes = [(bar_x, origin), (bar_y, origin), (bar_z, origin)];
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
                        .with_physical_surface(self.surface()),
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

/// Polished metal with a subtle coloured tint and light brushing.
fn generate_jack_metal(seed: u32) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    // Random tint — brassy, steely, or copperish.
    let hue = rand::random::<f32>() * 6.0;
    let base = hue_to_rgb(hue, 0.25, 0.78);

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let brush = fbm_2d_periodic(u * 4.0, v * 32.0, 3, 0.5, 2.0, seed, Some(4));
            let speck = fbm_2d_periodic(
                u * 18.0,
                v * 18.0,
                2,
                0.4,
                2.0,
                seed.wrapping_add(11),
                Some(18),
            );

            let factor = 0.82 + brush * 0.14 + speck * 0.06;
            let c = base.scale(factor);
            c.write_rgba(&mut pixels);
        }
    }
    pixels
}
