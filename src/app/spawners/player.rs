use nalgebra::Vector3;
use specs::{Builder, Entity, World, WorldExt};

use crate::biped::{BipedConfig, BipedController};
use crate::components::{
    Orientation, Position, Renderable, RigidBodyComponent, Rotation, Velocity, VelocityDriven,
};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::player::{Player, PlayerConfig, PlayerTargetState};
use crate::sensing::{ContactCandidates, SensorSet};
use crate::systems::PhysicsResource;

/// Spawns the player entity with all required components.
pub fn spawn_player(world: &mut World, initial_pos: nalgebra::Point3<f32>) -> Entity {
    let player_config = world.read_resource::<PlayerConfig>();
    drop(player_config);

    let biped_config = BipedConfig::default();
    let body_radius = biped_config.body_radius;
    let biped_controller = BipedController::new(biped_config, initial_pos);

    let body_handle = {
        let mut physics = world.write_resource::<PhysicsResource>();
        let body_desc = RigidBodyDesc::dynamic()
            .position(initial_pos)
            .gravity_scale(1.0)
            .linear_damping(0.0)
            .angular_damping(1.0);
        let body_handle = physics.0.create_body(body_desc);
        let collider_desc = ColliderDesc::sphere(body_radius)
            .density(30.0)
            .restitution(0.0)
            .friction(0.3);
        physics.0.attach_collider(body_handle, collider_desc);
        body_handle
    };

    world
        .create_entity()
        .with(Player)
        .with(PlayerTargetState::default())
        .with(biped_controller)
        .with(Position(Vector3::new(
            initial_pos.x,
            initial_pos.y,
            initial_pos.z,
        )))
        .with(Velocity(Vector3::zeros()))
        .with(Rotation(0.0))
        .with(Orientation::default())
        .with(Renderable)
        .with(SensorSet::default())
        .with(ContactCandidates::default())
        .with(RigidBodyComponent(body_handle))
        .with(VelocityDriven::default())
        .build()
}
