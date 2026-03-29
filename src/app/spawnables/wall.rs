//! BoxWall spawnable — grid of boxes, optionally staggered.

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

#[derive(Deserialize)]
pub struct BoxWallDef {
    pub base: (f32, f32, f32),
    pub box_half_extents: (f32, f32, f32),
    pub columns: u32,
    pub rows: u32,
    #[serde(default = "BoxWallDef::default_density")]
    pub density: f32,
    #[serde(default)]
    pub stagger: bool,
}

impl BoxWallDef {
    pub fn default_density() -> f32 {
        50.0
    }
}

impl Spawnable for BoxWallDef {
    fn material_count(&self) -> usize {
        (self.columns * self.rows) as usize
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let count = self.material_count();
        let mut mats = Vec::with_capacity(count);
        for _ in 0..count {
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
        let box_w = he.x * 2.0;
        let box_h = he.y * 2.0;

        let mut entities = Vec::with_capacity(self.material_count());
        let mut mat_idx = 0;

        for row in 0..self.rows {
            let y = self.base.1 + he.y + row as f32 * box_h;
            let x_offset = if self.stagger && row % 2 == 1 {
                he.x
            } else {
                0.0
            };
            let start_x = self.base.0 - (self.columns as f32 - 1.0) * he.x + x_offset;

            for col in 0..self.columns {
                let x = start_x + col as f32 * box_w;
                let pos = Point3::new(x, y, self.base.2);
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

                mat_idx += 1;
            }
        }

        entities
    }
}
