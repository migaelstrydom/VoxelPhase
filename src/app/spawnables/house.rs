//! House spawnable — prefab structure built from boxes.

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Entity, World};

use super::shared::models::cuboid_model;
use super::box_object::create_box_materials;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::fire::components::Flammable;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::MaterialId;
use crate::systems::PhysicsResource;

use specs::{Builder, WorldExt};

/// Number of distinct materials used for house pieces.
const HOUSE_MATERIAL_COUNT: usize = 10;

#[derive(Deserialize)]
pub struct HouseDef {
    pub pos: (f32, f32, f32),
    pub half_extents: (f32, f32, f32),
}

impl Spawnable for HouseDef {
    fn material_count(&self) -> usize {
        HOUSE_MATERIAL_COUNT
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        create_box_materials(HOUSE_MATERIAL_COUNT, ctx.textures, ctx.materials)
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let base_pos = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let wx = self.half_extents.0;
        let hy = self.half_extents.1;
        let wz = self.half_extents.2;

        let wt = hy * 0.25;
        let fh = hy * 0.3;
        let wh = hy;
        let rt = hy * 0.3;
        let ro = hy * 0.5;

        let door_half_w = wx * 0.4;
        let win_half_z = wz * 0.35;
        let sill_h = wh * 0.3;

        let mut entities = Vec::new();
        let mat = |i: usize| materials[i % materials.len()];

        let y0 = base_pos.y;
        let foundation_cy = y0 + fh;
        let y_wall_base = y0 + 2.0 * fh;
        let lower_cy = y_wall_base + wh;
        let upper_cy = y_wall_base + 3.0 * wh;
        let walls_top = y_wall_base + 4.0 * wh;
        let roof_cy = walls_top + rt;

        let spawn = |world: &mut World,
                      entities: &mut Vec<Entity>,
                      pos: Point3<f32>,
                      he: Vector3<f32>,
                      material: MaterialId| {
            let model = cuboid_model(he, material);

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
                        .density(50.0)
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
        };

        // Foundation slab
        spawn(
            world,
            &mut entities,
            Point3::new(base_pos.x, foundation_cy, base_pos.z),
            Vector3::new(wx + wt, fh, wz + wt),
            mat(0),
        );

        // Back wall
        spawn(
            world,
            &mut entities,
            Point3::new(base_pos.x, y_wall_base + 2.0 * wh, base_pos.z - wz),
            Vector3::new(wx + wt, 2.0 * wh, wt),
            mat(1),
        );

        // Front wall — door opening in lower layer
        let door_side_half = (wx + wt - door_half_w) / 2.0;
        let door_left_cx = base_pos.x - door_half_w - door_side_half;
        let door_right_cx = base_pos.x + door_half_w + door_side_half;

        spawn(
            world,
            &mut entities,
            Point3::new(door_left_cx, lower_cy, base_pos.z + wz),
            Vector3::new(door_side_half, wh, wt),
            mat(2),
        );
        spawn(
            world,
            &mut entities,
            Point3::new(door_right_cx, lower_cy, base_pos.z + wz),
            Vector3::new(door_side_half, wh, wt),
            mat(2),
        );
        spawn(
            world,
            &mut entities,
            Point3::new(base_pos.x, upper_cy, base_pos.z + wz),
            Vector3::new(wx + wt, wh, wt),
            mat(3),
        );

        // Side walls with window openings
        let win_col_half = (wz - wt - win_half_z) / 2.0;
        let win_col_neg_cz = base_pos.z - win_half_z - win_col_half;
        let win_col_pos_cz = base_pos.z + win_half_z + win_col_half;
        let sill_cy = y_wall_base + sill_h;
        let lintel_cy = y_wall_base + 2.0 * wh - sill_h;

        for (sign, mat_base) in [(-1.0_f32, 4_usize), (1.0_f32, 7_usize)] {
            let cx = base_pos.x + sign * wx;

            spawn(
                world,
                &mut entities,
                Point3::new(cx, lower_cy, win_col_neg_cz),
                Vector3::new(wt, wh, win_col_half),
                mat(mat_base),
            );
            spawn(
                world,
                &mut entities,
                Point3::new(cx, lower_cy, win_col_pos_cz),
                Vector3::new(wt, wh, win_col_half),
                mat(mat_base),
            );
            spawn(
                world,
                &mut entities,
                Point3::new(cx, sill_cy, base_pos.z),
                Vector3::new(wt, sill_h, win_half_z),
                mat(mat_base + 1),
            );
            spawn(
                world,
                &mut entities,
                Point3::new(cx, lintel_cy, base_pos.z),
                Vector3::new(wt, sill_h, win_half_z),
                mat(mat_base + 1),
            );
            spawn(
                world,
                &mut entities,
                Point3::new(cx, upper_cy, base_pos.z),
                Vector3::new(wt, wh, wz - wt),
                mat(mat_base + 2),
            );
        }

        // Roof slab
        spawn(
            world,
            &mut entities,
            Point3::new(base_pos.x, roof_cy, base_pos.z),
            Vector3::new(wx + ro, rt, wz + ro),
            mat(0),
        );

        entities
    }
}
