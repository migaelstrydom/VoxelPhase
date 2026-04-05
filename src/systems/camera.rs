use crate::camera::{CameraConfig, FollowTarget};
use crate::components::{CameraComponent, Position};
use crate::input::{GameplayActions, InputState};
use crate::time::Time;
use nalgebra::{Point3, Vector3};
use specs::{Entities, Join, ReadExpect, ReadStorage, System, WriteStorage};

/// Updates cameras that have a FollowTarget component.
/// Handles smooth following, mouse orbit, and zoom controls.
pub struct CameraControlSystem;

impl<'a> System<'a> for CameraControlSystem {
    type SystemData = (
        Entities<'a>,
        ReadExpect<'a, Time>,
        ReadExpect<'a, InputState>,
        ReadExpect<'a, GameplayActions>,
        ReadExpect<'a, CameraConfig>,
        ReadStorage<'a, Position>,
        WriteStorage<'a, FollowTarget>,
        WriteStorage<'a, CameraComponent>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, time, input, actions, config, positions, mut follow_targets, mut cameras) =
            data;

        let dt = time.delta_seconds();

        for (_entity, follow, camera) in (&entities, &mut follow_targets, &mut cameras).join() {
            // Get target position
            let target_pos = match positions.get(follow.target) {
                Some(pos) => pos.0 + Vector3::new(0.0, 1.0, 0.0),
                None => {
                    log::warn!("CameraControlSystem: Follow target entity has no Position");
                    continue;
                }
            };

            // Handle mouse input for orbit and pitch
            let (mouse_dx, mouse_dy) = actions.camera_delta;

            // Only process mouse input if mouse is captured
            if input.is_mouse_captured() {
                follow.orbit_angle -= mouse_dx * config.orbit_sensitivity;
                follow.pitch += mouse_dy * config.height_sensitivity;

                // Clamp pitch to valid range
                follow.pitch = follow.pitch.clamp(config.min_pitch, config.max_pitch);
            }

            // Handle keyboard zoom
            if actions.zoom_in {
                follow.distance += config.zoom_speed * dt;
            }
            if actions.zoom_out {
                follow.distance -= config.zoom_speed * dt;
            }

            // Clamp distance
            follow.distance = follow
                .distance
                .clamp(config.min_distance, config.max_distance);

            // Calculate desired camera position based on orbit angle, pitch, and distance
            // Using spherical coordinates: orbit_angle is azimuth, pitch is elevation
            let horizontal_dist = follow.distance * follow.pitch.cos();
            let vertical_dist = follow.distance * follow.pitch.sin();

            let desired_pos = Point3::new(
                target_pos.x + horizontal_dist * follow.orbit_angle.sin(),
                target_pos.y + vertical_dist + config.height_gradient * follow.distance,
                target_pos.z + horizontal_dist * follow.orbit_angle.cos(),
            );

            // Smooth interpolation toward desired position
            // Using exponential decay: pos = lerp(pos, desired, 1 - e^(-speed * dt))
            let t = 1.0 - (-config.follow_speed * dt).exp();

            let cam = &mut camera.0;
            cam.position.x += (desired_pos.x - cam.position.x) * t;
            cam.position.y += (desired_pos.y - cam.position.y) * t;
            cam.position.z += (desired_pos.z - cam.position.z) * t;

            // Target smoothly follows the actual target position
            cam.target.x += (target_pos.x - cam.target.x) * t;
            cam.target.y += (target_pos.y - cam.target.y) * t;
            cam.target.z += (target_pos.z - cam.target.z) * t;

            // // Debug logging (every ~60 frames to avoid spam)
            // if time.frame_count() % 60 == 0 {
            //     log::debug!(
            //         "Camera: pos=({:.1}, {:.1}, {:.1}) target=({:.1}, {:.1}, {:.1}) pitch={:.2} orbit={:.2}",
            //         cam.position.x, cam.position.y, cam.position.z,
            //         cam.target.x, cam.target.y, cam.target.z,
            //         follow.pitch, follow.orbit_angle
            //     );
            // }
        }
    }
}
