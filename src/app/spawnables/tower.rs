//! Tower spawnable — N identical boxes stacked vertically.

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::Entity;

use super::box_object::create_box_material_for_style;
use super::shared::models::cuboid_model;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::fire::components::Flammable;
use crate::level::BoxStyle;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::MaterialId;
use crate::systems::PhysicsResource;

use specs::{Builder, WorldExt};

fn default_tower_density() -> f32 {
    50.0
}

#[derive(Deserialize)]
pub struct TowerDef {
    pub base: (f32, f32, f32),
    pub box_half_extents: (f32, f32, f32),
    pub count: u32,
    #[serde(default = "default_tower_density")]
    pub density: f32,
}

impl Spawnable for TowerDef {
    fn material_count(&self) -> usize {
        self.count as usize
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let mut mats = Vec::with_capacity(self.count as usize);
        for _ in 0..self.count {
            mats.push(create_box_material_for_style(
                BoxStyle::Random,
                ctx.textures,
                ctx.materials,
            )?);
        }
        Ok(mats)
    }

    fn spawn(&self, world: &mut specs::World, materials: &[MaterialId]) -> Vec<Entity> {
        let he = Vector3::new(
            self.box_half_extents.0,
            self.box_half_extents.1,
            self.box_half_extents.2,
        );
        let mut entities = Vec::with_capacity(self.count as usize);
        let mut y = self.base.1;

        for i in 0..self.count as usize {
            y += he.y;
            let pos = Point3::new(self.base.0, y, self.base.2);
            let model = cuboid_model(he, materials[i]);

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
                    ColliderDesc::box_shape(he)
                        .density(self.density)
                        .restitution(0.2)
                        .friction(0.6),
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
                    .with(Flammable::wood())
                    .build(),
            );

            y += he.y;
        }

        entities
    }
}
