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

/// Gravity magnitude used to derive the traction limit. Matches
/// `PhysicsConfig::default().gravity`.
const GRAVITY: f32 = 9.81;

/// Moment of inertia of a solid sphere, as a fraction of `m r²`.
const SOLID_SPHERE_INERTIA: f32 = 0.4;

impl Roller {
    /// Torque and spin cap for a roller of the given radius and mass, tuned so
    /// `surface_speed` is roughly its flat-ground top speed and `spin_up_time`
    /// roughly how long it takes to get there from rest.
    ///
    /// The torque follows from rolling without slipping, which couples the
    /// creature's spin to its travel. Driving the body with torque `τ`:
    ///
    /// ```text
    ///   linear:   f = m a                 (f = friction at the contact patch)
    ///   angular:  τ − f r = I α,  α = a/r,  I = 0.4 m r²
    ///   ⇒         τ = 1.4 m r a
    /// ```
    ///
    /// The `1.4` is the part worth keeping in mind: torque has to accelerate
    /// the creature's linear mass as well as spin it up. Sizing torque from the
    /// moment of inertia alone — as if the sphere were spinning in free space —
    /// under-drives it by that factor and produces a creature that visibly
    /// labours to get going.
    ///
    /// `friction` must match the collider's, since it sets the traction limit
    /// below.
    pub fn new(
        radius: f32,
        mass: f32,
        surface_speed: f32,
        spin_up_time: f32,
        friction: f32,
    ) -> Self {
        let radius = radius.max(1e-3);
        let max_spin = surface_speed / radius;

        // Torque beyond what friction can transmit spins the creature in place
        // instead of moving it faster, so an over-eager `spin_up_time` would
        // trade acceleration for a comic wheelspin. Clamping means the number
        // in the level file means what it says right up to the point where the
        // ground stops cooperating.
        let requested_accel = surface_speed / spin_up_time.max(1e-3);
        let traction_limit = friction * GRAVITY;
        let accel = requested_accel.min(traction_limit);

        Self {
            torque: (1.0 + SOLID_SPHERE_INERTIA) * mass * radius * accel,
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

            // Rolling toward `direction` means spinning about the axis to its
            // left: up × direction.
            let axis = Vector3::y().cross(&direction);

            let Some(body) = physics_res.world.body(rb.0) else {
                continue;
            };

            // Only the component of spin that drives this direction counts
            // toward the cap. A roller already spinning the other way is
            // under its cap on this axis and gets full torque to reverse,
            // which is what stops it drifting past a target it overshot.
            let spin_along_axis = body.angular_velocity().dot(&axis);
            if spin_along_axis >= roller.max_spin {
                continue;
            }

            // Goes through the world rather than the body so the creature wakes
            // itself. A roller spawns asleep — it is placed at rest — and would
            // otherwise stay that way, spinning up a velocity the integrator
            // ignores, until something bumped it awake.
            physics_res
                .world
                .apply_angular_impulse(rb.0, axis * roller.torque * dt);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_bigger_roller_needs_more_torque_for_the_same_speed() {
        let small = Roller::new(0.5, 100.0, 6.0, 0.7, 1.2);
        let large = Roller::new(1.5, 900.0, 6.0, 0.7, 1.2);
        assert!(
            large.torque > small.torque,
            "torque must scale with mass and radius or heavy rollers won't move"
        );
    }

    #[test]
    fn spin_cap_falls_as_radius_grows_for_the_same_surface_speed() {
        let small = Roller::new(0.5, 100.0, 6.0, 0.7, 1.2);
        let large = Roller::new(1.5, 100.0, 6.0, 0.7, 1.2);
        assert!(
            large.max_spin < small.max_spin,
            "a larger wheel covers the same ground with less spin"
        );
        assert!((small.max_spin - 12.0).abs() < 1e-4, "6 m/s at r=0.5");
    }

    /// Flat-ground linear acceleration implied by a roller's drive torque,
    /// inverting the rolling-without-slipping relation in `Roller::new`.
    fn implied_accel(roller: &Roller, radius: f32, mass: f32) -> f32 {
        roller.torque / ((1.0 + SOLID_SPHERE_INERTIA) * mass * radius)
    }

    #[test]
    fn torque_delivers_the_requested_spin_up_time() {
        // The bug this pins: sizing torque from the moment of inertia alone
        // ignores the linear mass the contact patch must also accelerate, and
        // under-drives the creature by 3.5x.
        let (radius, mass, speed, spin_up) = (0.6, 2171.0, 4.5, 0.7);
        let roller = Roller::new(radius, mass, speed, spin_up, 1.2);

        let accel = implied_accel(&roller, radius, mass);
        let time_to_speed = speed / accel;

        assert!(
            (time_to_speed - spin_up).abs() < 0.01,
            "asked for {spin_up}s to reach {speed} m/s, got {time_to_speed}s"
        );
    }

    #[test]
    fn a_shorter_spin_up_time_accelerates_harder() {
        let (radius, mass) = (0.6, 2171.0);
        let brisk = Roller::new(radius, mass, 4.5, 0.4, 1.2);
        let lazy = Roller::new(radius, mass, 4.5, 1.4, 1.2);
        assert!(brisk.torque > lazy.torque);
        assert!(
            (brisk.max_spin - lazy.max_spin).abs() < 1e-4,
            "spin_up_time must not change top speed — the two dials are separate"
        );
    }

    #[test]
    fn acceleration_is_capped_by_what_friction_can_transmit() {
        let (radius, mass, friction) = (0.6, 2171.0, 1.2);
        // An absurdly short spin-up: the ground cannot deliver it.
        let roller = Roller::new(radius, mass, 4.5, 0.001, friction);

        let accel = implied_accel(&roller, radius, mass);
        assert!(
            accel <= friction * GRAVITY * 1.001,
            "torque beyond the traction limit spins the creature in place"
        );
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
