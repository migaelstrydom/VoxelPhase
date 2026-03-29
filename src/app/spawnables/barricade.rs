//! Barricade spawnable — two posts with planks welded across them.

use nalgebra::{Point3, UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::cuboid_model;
use super::{MaterialCtx, Spawnable};
use super::box_object::create_box_material_for_style;
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::fire::components::Flammable;
use crate::level::BoxStyle;
use crate::physics::constraint::ConstraintKind;
use crate::physics::{ColliderDesc, RigidBodyDesc, RigidBodyHandle};
use crate::rendering::material::MaterialId;
use crate::systems::PhysicsResource;

/// Dimensions for a barricade.
pub struct BarricadeDimensions {
    /// Half-extents of each vertical post.
    pub post_half_extents: Vector3<f32>,
    /// Half-extents of each horizontal plank.
    pub plank_half_extents: Vector3<f32>,
    /// Number of planks.
    pub plank_count: u32,
}

impl Default for BarricadeDimensions {
    fn default() -> Self {
        Self {
            post_half_extents: Vector3::new(0.08, 0.5, 0.08),
            plank_half_extents: Vector3::new(0.45, 0.08, 0.06),
            plank_count: 3,
        }
    }
}

impl BarricadeDimensions {
    /// Number of box materials needed to render this barricade.
    pub fn piece_count(&self) -> usize {
        2 + self.plank_count as usize
    }
}

#[derive(Deserialize)]
pub struct BarricadeDef {
    pub pos: (f32, f32, f32),
    #[serde(default = "BarricadeDef::default_plank_count")]
    pub plank_count: u32,
    #[serde(default = "BarricadeDef::default_density")]
    pub density: f32,
}

impl BarricadeDef {
    pub fn default_plank_count() -> u32 {
        3
    }
    pub fn default_density() -> f32 {
        400.0
    }
}

impl Spawnable for BarricadeDef {
    fn material_count(&self) -> usize {
        let dims = BarricadeDimensions {
            plank_count: self.plank_count,
            ..Default::default()
        };
        dims.piece_count()
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let count = self.material_count();
        let mut mats = Vec::with_capacity(count);
        for _ in 0..count {
            mats.push(create_box_material_for_style(
                BoxStyle::WoodenCrate,
                ctx.textures,
                ctx.materials,
            )?);
        }
        Ok(mats)
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let base_pos = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let dims = BarricadeDimensions {
            plank_count: self.plank_count,
            ..Default::default()
        };

        let post_he = dims.post_half_extents;
        let plank_he = dims.plank_half_extents;
        let plank_count = dims.plank_count;

        let post_x = plank_he.x - post_he.x;
        let post_cy = base_pos.y + post_he.y;

        let (body_handles, positions, half_extents_list) = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let mut handles = Vec::new();
            let mut positions = Vec::new();
            let mut he_list = Vec::new();

            for &sign in &[-1.0f32, 1.0] {
                let pos = Point3::new(base_pos.x + sign * post_x, post_cy, base_pos.z);
                let h = spawn_piece(&mut physics.world, pos, post_he, self.density);
                handles.push(h);
                positions.push(pos);
                he_list.push(post_he);
            }

            let weld_gap = 0.03;
            let inner_gap = post_x - post_he.x - weld_gap;
            let actual_plank_he = Vector3::new(inner_gap, plank_he.y, plank_he.z);

            let usable_height = post_he.y * 2.0 - plank_he.y * 2.0;
            for i in 0..plank_count {
                let t = if plank_count <= 1 {
                    0.5
                } else {
                    i as f32 / (plank_count - 1) as f32
                };
                let plank_cy = base_pos.y + plank_he.y + t * usable_height;
                let pos = Point3::new(base_pos.x, plank_cy, base_pos.z);
                let h = spawn_piece(&mut physics.world, pos, actual_plank_he, self.density);
                handles.push(h);
                positions.push(pos);
                he_list.push(actual_plank_he);
            }

            let left_post = handles[0];
            let right_post = handles[1];

            for i in 0..plank_count as usize {
                let plank_handle = handles[2 + i];
                let plank_pos = positions[2 + i];

                let left_post_pos = positions[0];
                weld_pieces(
                    &mut physics.world,
                    left_post,
                    plank_handle,
                    Vector3::new(post_he.x, plank_pos.y - left_post_pos.y, 0.0),
                    Vector3::new(-actual_plank_he.x, 0.0, 0.0),
                );

                let right_post_pos = positions[1];
                weld_pieces(
                    &mut physics.world,
                    right_post,
                    plank_handle,
                    Vector3::new(-post_he.x, plank_pos.y - right_post_pos.y, 0.0),
                    Vector3::new(actual_plank_he.x, 0.0, 0.0),
                );
            }

            (handles, positions, he_list)
        };

        let mut entities = Vec::with_capacity(body_handles.len());
        for (idx, (&handle, &pos)) in body_handles.iter().zip(positions.iter()).enumerate() {
            let mat = materials[idx % materials.len()];
            let he = half_extents_list[idx];
            let model = cuboid_model(he, mat);

            let entity = world
                .create_entity()
                .with(Position(Vector3::new(pos.x, pos.y, pos.z)))
                .with(Velocity(Vector3::zeros()))
                .with(Orientation::default())
                .with(RigidBodyComponent(handle))
                .with(ModelInstance::new(model))
                .with(Renderable)
                .with(Flammable::wood())
                .build();
            entities.push(entity);
        }

        entities
    }
}

fn spawn_piece(
    physics: &mut crate::physics::PhysicsWorld,
    pos: Point3<f32>,
    half_extents: Vector3<f32>,
    density: f32,
) -> RigidBodyHandle {
    let body = physics.create_body(
        RigidBodyDesc::dynamic()
            .position(pos)
            .gravity_scale(1.0)
            .linear_damping(0.01)
            .angular_damping(0.005),
    );
    physics.attach_collider(
        body,
        ColliderDesc::box_shape(half_extents)
            .density(density)
            .restitution(0.1)
            .friction(0.6),
    );
    body
}

fn weld_pieces(
    physics: &mut crate::physics::PhysicsWorld,
    body_a: RigidBodyHandle,
    body_b: RigidBodyHandle,
    local_anchor_a: Vector3<f32>,
    local_anchor_b: Vector3<f32>,
) {
    let rot_a = physics
        .body(body_a)
        .map(|b| b.rotation())
        .unwrap_or(UnitQuaternion::identity());
    let rot_b = physics
        .body(body_b)
        .map(|b| b.rotation())
        .unwrap_or(UnitQuaternion::identity());
    let relative_orientation = rot_a.inverse() * rot_b;

    let _ = physics.create_constraint(ConstraintKind::Weld {
        body_a,
        body_b,
        local_anchor_a,
        local_anchor_b,
        relative_orientation,
        compliance: 0.0,
        angular_compliance: 0.0,
    });
}
