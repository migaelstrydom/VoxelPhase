//! How a grenade leaves the hand.
//!
//! One function, called twice: once by the spawner that actually throws the
//! grenade, and once by the aiming cursor that draws where it would land. The
//! cursor is only honest for as long as there is exactly one copy of this.

use nalgebra::{Point3, Vector3};

use crate::aim::launch::{gravity_along, Launch};

use super::config::GrenadeConfig;

/// The arc a grenade thrown right now would fly.
///
/// `look_angle` is the direction the camera looks (not where it sits) and
/// `camera_pitch` is positive looking down, which is why it is subtracted:
/// looking down throws flatter, looking up throws higher, and
/// [`GrenadeConfig::pitch_influence`] decides how much of that carries over.
pub fn grenade_launch(
    config: &GrenadeConfig,
    thrower_position: Point3<f32>,
    look_angle: f32,
    camera_pitch: f32,
    world_gravity: Vector3<f32>,
) -> Launch {
    let pitch = (config.upward_offset - camera_pitch * config.pitch_influence).clamp(-0.4, 0.8);

    let direction = Vector3::new(
        look_angle.sin() * pitch.cos(),
        pitch.sin(),
        look_angle.cos() * pitch.cos(),
    );

    // Out of the hand rather than out of the pelvis: forward a little, and up
    // to roughly chest height.
    let horizontal = Vector3::new(look_angle.sin(), 0.0, look_angle.cos());
    let origin = thrower_position + horizontal * 0.1 + Vector3::new(0.0, 0.5, 0.0);

    let velocity = direction * config.throw_speed + Vector3::new(0.0, config.arc_factor, 0.0);

    Launch {
        origin,
        velocity,
        gravity: gravity_along(world_gravity, config.gravity),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn level_launch(camera_pitch: f32) -> Launch {
        grenade_launch(
            &GrenadeConfig::default(),
            Point3::origin(),
            0.0,
            camera_pitch,
            Vector3::new(0.0, -9.81, 0.0),
        )
    }

    #[test]
    fn looking_down_throws_flatter_than_looking_up() {
        let down = level_launch(0.6);
        let up = level_launch(-0.6);

        assert!(
            down.velocity.y < up.velocity.y,
            "down {} should be flatter than up {}",
            down.velocity.y,
            up.velocity.y
        );
    }

    #[test]
    fn the_grenade_leaves_above_the_thrower() {
        assert!(level_launch(0.0).origin.y > 0.0);
    }
}
