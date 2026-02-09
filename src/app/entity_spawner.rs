use nalgebra::Vector3;
use specs::{Builder, Entity, World, WorldExt};

use crate::biped::{BipedConfig, BipedController};
use crate::camera::{CameraConfig, FollowTarget};
use crate::components::{
    Acceleration, CameraComponent, Gravity, ModelInstance, MotionState, Orientation, PhysicsBody,
    Position, Renderable, RigidBodyComponent, Rotation, Velocity,
};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::player::{Player, PlayerConfig, PlayerTargetState};
use crate::rendering::camera::Camera;
use crate::sensing::{ContactCandidates, SensorSet};
use crate::systems::PhysicsResource;

/// Spawns the player entity with all required components
pub fn spawn_player(world: &mut World, initial_pos: nalgebra::Point3<f32>) -> Entity {
    let player_config = world.read_resource::<PlayerConfig>();
    let gravity = player_config.gravity;
    drop(player_config);

    let biped_config = BipedConfig::default();
    let body_radius = biped_config.body_radius;
    let biped_controller = BipedController::new(biped_config, initial_pos);
    let physics_body = PhysicsBody {
        restitution: 0.1,
        friction: 0.8,
        mass: 70.0,
    };

    let body_handle = {
        let mut physics = world.write_resource::<PhysicsResource>();
        let body_desc = RigidBodyDesc::kinematic().position(initial_pos);
        let body_handle = physics.0.create_body(body_desc);
        let collider_desc = ColliderDesc::sphere(body_radius)
            .restitution(physics_body.restitution)
            .friction(physics_body.friction);
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
        .with(Acceleration(Vector3::zeros()))
        .with(Gravity(gravity))
        .with(Rotation(0.0))
        .with(Orientation::default())
        .with(Renderable)
        .with(MotionState::new(initial_pos))
        .with(SensorSet::default())
        .with(ContactCandidates::default())
        .with(physics_body)
        .with(RigidBodyComponent(body_handle))
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

/// Spawns a box entity with physics.
pub fn spawn_box(
    world: &mut World,
    initial_pos: nalgebra::Point3<f32>,
    half_extents: Vector3<f32>,
    model: std::sync::Arc<crate::model::Model>,
) -> Entity {
    let body_handle = {
        let mut physics = world.write_resource::<PhysicsResource>();

        let body_desc = RigidBodyDesc::dynamic()
            .position(initial_pos)
            .gravity_scale(1.0)
            .linear_damping(0.01)
            .angular_damping(0.05);

        let body_handle = physics.0.create_body(body_desc);

        let collider_desc = ColliderDesc::box_shape(half_extents)
            .density(5.0)
            .restitution(0.2)
            .friction(0.6);

        physics.0.attach_collider(body_handle, collider_desc);

        body_handle
    };

    world
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
        .build()
}

/// Spawns a beach ball entity with bouncy physics using the new physics engine.
pub fn spawn_beach_ball(
    world: &mut World,
    initial_pos: nalgebra::Point3<f32>,
    model: std::sync::Arc<crate::model::Model>,
) -> Entity {
    let radius = 0.5;

    // Create rigid body in physics world
    let body_handle = {
        let mut physics = world.write_resource::<PhysicsResource>();

        let body_desc = RigidBodyDesc::dynamic()
            .position(initial_pos)
            .gravity_scale(1.0)
            .linear_damping(0.01)
            .angular_damping(0.02);

        let body_handle = physics.0.create_body(body_desc);

        // Attach sphere collider with bouncy material
        // Density ~1 kg/m³ gives mass ~0.52kg for a 0.5m radius sphere.
        let collider_desc = ColliderDesc::sphere(radius)
            .density(1.0)
            .restitution(0.5)
            .friction(0.5);

        physics.0.attach_collider(body_handle, collider_desc);

        body_handle
    };

    world
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
        .build()
}
