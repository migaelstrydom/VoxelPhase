use specs::{Builder, Entity, World, WorldExt};

use crate::camera::{CameraConfig, FollowTarget};
use crate::components::CameraComponent;
use crate::rendering::camera::Camera;

/// Spawns the camera entity that follows the player.
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
