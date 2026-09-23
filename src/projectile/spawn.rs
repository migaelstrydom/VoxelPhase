use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use specs::{Builder, Entity};

use super::components::{Grenade, Lifetime, Projectile};
use super::config::GrenadeConfig;
use crate::aim::launch::gravity_scale;
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::model::Model;
use crate::physics::{ColliderDesc, PhysicsWorld, RigidBodyDesc};

/// Create a live grenade: its body in `physics` and its entity through
/// `builder`, fuse lit.
///
/// Takes any `Builder` so a system can spawn through `LazyUpdate` and an
/// offline harness straight into the world, with nothing else differing.
pub fn spawn_grenade<B: Builder>(
    builder: B,
    physics: &mut PhysicsWorld,
    config: &GrenadeConfig,
    model: Arc<Model>,
    origin: Point3<f32>,
    velocity: Vector3<f32>,
) -> Entity {
    let world_gravity = physics.config().gravity;
    let body_desc = RigidBodyDesc::dynamic()
        .position(origin)
        .linear_velocity(velocity)
        .gravity_scale(gravity_scale(world_gravity, config.gravity));
    let body_handle = physics.create_body(body_desc);

    let collider_desc = ColliderDesc::sphere(config.radius)
        .density(2000.0)
        .restitution(0.0)
        .friction(0.3);
    physics.attach_collider(body_handle, collider_desc);

    builder
        .with(Position(origin.coords))
        .with(Velocity(velocity))
        .with(Orientation::default())
        .with(RigidBodyComponent(body_handle))
        .with(Projectile)
        .with(Grenade::new(config.fuse_time, config.arm_delay))
        .with(Lifetime::new(config.max_lifetime))
        .with(ModelInstance::new(model))
        .with(Renderable)
        .build()
}
