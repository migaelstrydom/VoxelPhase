use nalgebra::Vector3;
use specs::{Component, DenseVecStorage, Join, Read, ReadStorage, System, Write};

use crate::character::CharacterIntent;
use crate::components::RigidBodyComponent;
use crate::damage::Dead;
use crate::systems::PhysicsResource;
use crate::time::Time;

/// A creature that moves by rolling rather than walking.
///
/// Deliberately *not* driven by `CharacterControlSystem`. That system is built
/// around a walk FSM — grounded, coyote time, jump buffering — and it steers by
/// overwriting linear velocity, which would make a sphere slide across the
/// ground without turning. A boulder forced through a walk cycle is the wrong
/// abstraction, and it would look it.
///
/// What *is* reused is the seam that matters: this reads the same
/// [`CharacterIntent`] the keyboard and `BrainSystem` write. The intent layer is
/// agnostic at both ends — any producer, any consumer — so the brain, the
/// perception, the steering and the damage all work here unchanged. Only the
/// last step, turning a direction into motion, is rolling-specific.
///
/// Motion comes from angular impulse and nothing else. Friction against the
/// terrain converts the spin into travel, which means the creature is subject
/// to the real surface: it accelerates down into a grenade crater, struggles
/// out of it, and skids on the way. That behaviour is free, and it is the
/// reason this creature is the first one.
#[derive(Component, Debug, Clone)]
#[storage(DenseVecStorage)]
pub struct Roller {
    /// Angular impulse applied per second while the creature wants to move.
    /// Scales with mass, so heavier rollers need proportionally more.
    pub torque: f32,
    /// Spin rate (rad/s) past which no further torque is applied. This is the
    /// creature's top speed, expressed the only way a rolling body can express
    /// one — `LocomotionConfig::walk_speed` has no meaning without a gait.
    pub max_spin: f32,
}

impl Roller {
    /// Torque and spin cap for a roller of the given radius and mass, tuned so
    /// `surface_speed` is roughly its flat-ground top speed in m/s.
    pub fn new(radius: f32, mass: f32, surface_speed: f32) -> Self {
        let max_spin = surface_speed / radius.max(1e-3);
        Self {
            // Reach the spin cap in about half a second from rest. The moment
            // of inertia of a solid sphere is 2/5 m r².
            torque: 0.4 * mass * radius * radius * max_spin * 2.0,
            max_spin,
        }
    }
}

/// Applies rolling motion from intent.
///
/// The impulse axis is the intent direction rotated a quarter turn about world
/// up: to roll *toward* +Z a sphere spins about +X. Steering is a consequence
/// of where the torque points, so there is no separate turn control — a roller
/// arcs into its turns, which is exactly what a boulder should do.
pub struct RollerLocomotionSystem;

impl<'a> System<'a> for RollerLocomotionSystem {
    type SystemData = (
        Read<'a, Time>,
        Write<'a, PhysicsResource>,
        ReadStorage<'a, Roller>,
        ReadStorage<'a, CharacterIntent>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, Dead>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (time, mut physics_res, rollers, intents, rigid_bodies, deads) = data;
        let dt = time.delta_seconds();

        for (roller, intent, rb, _) in (&rollers, &intents, &rigid_bodies, !&deads).join() {
            let Some(direction) = intent.direction.try_normalize(1e-4) else {
                continue;
            };

            let Some(body) = physics_res.world.body_mut(rb.0) else {
                continue;
            };

            // Rolling toward `direction` means spinning about the axis to its
            // left: up × direction.
            let axis = Vector3::y().cross(&direction);

            // Only the component of spin that drives this direction counts
            // toward the cap. A roller already spinning the other way is
            // under its cap on this axis and gets full torque to reverse,
            // which is what stops it drifting past a target it overshot.
            let spin_along_axis = body.angular_velocity().dot(&axis);
            if spin_along_axis >= roller.max_spin {
                continue;
            }

            body.apply_angular_impulse(axis * roller.torque * dt);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_bigger_roller_needs_more_torque_for_the_same_speed() {
        let small = Roller::new(0.5, 100.0, 6.0);
        let large = Roller::new(1.5, 900.0, 6.0);
        assert!(
            large.torque > small.torque,
            "torque must scale with mass and radius or heavy rollers won't move"
        );
    }

    #[test]
    fn spin_cap_falls_as_radius_grows_for_the_same_surface_speed() {
        let small = Roller::new(0.5, 100.0, 6.0);
        let large = Roller::new(1.5, 100.0, 6.0);
        assert!(
            large.max_spin < small.max_spin,
            "a larger wheel covers the same ground with less spin"
        );
        assert!((small.max_spin - 12.0).abs() < 1e-4, "6 m/s at r=0.5");
    }

    #[test]
    fn roll_axis_is_perpendicular_to_travel_and_to_up() {
        let direction: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);
        let axis = Vector3::y().cross(&direction);
        assert!((axis - Vector3::new(1.0, 0.0, 0.0)).magnitude() < 1e-6);
        assert!(axis.dot(&direction).abs() < 1e-6);
        assert!(axis.dot(&Vector3::y()).abs() < 1e-6);
    }
}
