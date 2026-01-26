use nalgebra::Vector3;
use specs::{Builder, Entity, World, WorldExt};

use crate::biped::{BipedConfig, BipedController};
use crate::camera::{CameraConfig, FollowTarget};
use crate::components::{
    Acceleration, CameraComponent, Collider, Gravity, ModelInstance, MotionState, PhysicsBody,
    Position, Renderable, Rotation, Velocity,
};
use crate::player::{Player, PlayerConfig, PlayerTargetState};
use crate::rendering::camera::Camera;
use crate::sensing::{ContactCandidates, SensorSet};

/// Spawns the player entity with all required components
pub fn spawn_player(world: &mut World, initial_pos: nalgebra::Point3<f32>) -> Entity {
    let player_config = world.read_resource::<PlayerConfig>();
    let gravity = player_config.gravity;
    drop(player_config);

    let biped_config = BipedConfig::default();
    let body_radius = biped_config.body_radius;
    let biped_controller = BipedController::new(biped_config, initial_pos);

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
        .with(Acceleration(Vector3::zeros()))
        .with(Gravity(gravity))
        .with(Rotation(0.0))
        .with(Renderable)
        .with(MotionState::new(initial_pos))
        .with(SensorSet::default())
        .with(ContactCandidates::default())
        .with(PhysicsBody {
            restitution: 0.1,
            friction: 0.8,
            mass: 70.0,
        })
        .with(Collider::sphere(body_radius))
        .build()
}

/// Spawns the camera entity that follows the player
pub fn spawn_camera(
    world: &mut World,
    follow_target: Entity,
    window_width: u32,
    window_height: u32,
) -> Entity {
    let camera_config = world.read_resource::<CameraConfig>();
    let default_distance = camera_config.default_distance;
    drop(camera_config);

    let initial_aspect_ratio = window_width as f32 / window_height as f32;

    let camera = Camera::new(
        nalgebra::Point3::new(0.0, 10.0, default_distance),
        nalgebra::Point3::new(0.0, 0.0, 0.0),
        nalgebra::Vector3::y(),
        std::f32::consts::FRAC_PI_4,
        initial_aspect_ratio,
        0.1,
        500.0,
    );

    world
        .create_entity()
        .with(CameraComponent(camera))
        .with(FollowTarget::new(follow_target, default_distance))
        .build()
}

/// Spawns a beach ball entity with bouncy physics
pub fn spawn_beach_ball(
    world: &mut World,
    initial_pos: nalgebra::Point3<f32>,
    model: std::sync::Arc<crate::model::Model>,
) -> Entity {
    world
        .create_entity()
        .with(Position(Vector3::new(
            initial_pos.x,
            initial_pos.y,
            initial_pos.z,
        )))
        .with(Velocity(Vector3::zeros()))
        .with(Acceleration(Vector3::zeros()))
        .with(Gravity(20.0))
        .with(Rotation(0.0))
        .with(Collider::sphere(0.5))
        .with(MotionState::new(initial_pos))
        .with(PhysicsBody {
            restitution: 0.8,
            friction: 0.25,
            mass: 0.5,
        })
        .with(ModelInstance::new(model))
        .with(Renderable)
        .build()
}
