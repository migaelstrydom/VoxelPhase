//! Flying: a creature that holds a height over the ground and stoops on
//! what it hunts.
//!
//! ```text
//!   CharacterIntent ──▶ level flight toward where the brain steers
//!   TerrainWorld ─────▶ cruise height over the ground beneath
//!   Perception ───────▶ within stoop range of its target: down to it
//! ```
//!
//! The same seam as walking and swimming: the brain says which way, and
//! this turns that into impulses on the body. Gravity pulls a flyer like
//! anything else; what holds it up is the lift given it here every frame,
//! so one that dies, and is no longer flown, falls.

use std::f32::consts::{PI, TAU};

use nalgebra::Vector3;
use specs::{Component, DenseVecStorage, Join, Read, ReadStorage, System, Write};

use super::perception::Perception;
use crate::character::CharacterIntent;
use crate::components::RigidBodyComponent;
use crate::damage::Dead;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;
use crate::time::Time;

/// Climb or dive speed wanted per metre off the height it is making for,
/// 1/s, and the fastest it climbs or dives, m/s.
const HEIGHT_GAIN: f32 = 2.5;
const CLIMB_SPEED: f32 = 6.0;

/// Turn rate wanted per radian off its heading, 1/s, and the fastest it
/// turns, rad/s.
const TURN_GAIN: f32 = 5.0;
const TURN_RATE: f32 = 4.0;

/// How a creature flies.
#[derive(Component, Debug, Clone, Copy)]
#[storage(DenseVecStorage)]
pub struct Flyer {
    /// Its body's mass, kg.
    pub mass: f32,
    /// Its body's inertia about the vertical, kg·m².
    pub yaw_inertia: f32,
    /// Level speed, m/s.
    pub speed: f32,
    /// Seconds it takes to reach that speed from rest.
    pub accel_time: f32,
    /// Height it holds over the ground while it has nothing to stoop on, m.
    pub cruise_height: f32,
    /// Distance from its target, level, within which it comes down to it, m.
    pub stoop_range: f32,
    /// How far above its target's position it aims its stoop, m.
    pub stoop_height: f32,
}

/// Applies flying from intent.
pub struct FlyerLocomotionSystem;

impl<'a> System<'a> for FlyerLocomotionSystem {
    type SystemData = (
        Read<'a, Time>,
        Write<'a, PhysicsResource>,
        Option<Read<'a, TerrainWorld>>,
        ReadStorage<'a, Flyer>,
        ReadStorage<'a, CharacterIntent>,
        ReadStorage<'a, Perception>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, Dead>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (time, mut physics, terrain, flyers, intents, perceptions, bodies, deads) = data;
        let dt = time.delta_seconds();
        if dt <= 0.0 {
            return;
        }
        let gravity = physics.world.config().gravity;
        for (flyer, intent, perception, rb, _) in
            (&flyers, &intents, &perceptions, &bodies, !&deads).join()
        {
            let Some(body) = physics.world.body(rb.0) else {
                continue;
            };
            let (position, velocity) = (body.position(), body.linear_velocity());
            let (rotation, spin) = (body.rotation(), body.angular_velocity());

            let ground = terrain
                .as_ref()
                .and_then(|terrain| terrain.mesh_surface_height_at(position.x, position.z))
                .unwrap_or(position.y - flyer.cruise_height);
            let stoop = perception
                .target
                .filter(|target| target.visible && target.distance <= flyer.stoop_range)
                .map(|target| target.position.y + flyer.stoop_height);
            let change = steer(
                flyer,
                Vector3::new(intent.direction.x, 0.0, intent.direction.z),
                velocity,
                stoop.unwrap_or(ground + flyer.cruise_height) - position.y,
                dt,
            );
            // Lift: what gravity will take from it this frame, given back.
            let lift = -gravity * body.gravity_scale() * dt;
            physics
                .world
                .apply_impulse(rb.0, (change + lift) * flyer.mass);

            // It faces the way it flies.
            let flat = Vector3::new(velocity.x, 0.0, velocity.z);
            let Some(travel) = flat.try_normalize(0.1) else {
                continue;
            };
            let forward = rotation * Vector3::z();
            let off = wrap_angle(travel.z.atan2(travel.x) - forward.z.atan2(forward.x));
            // atan2(z, x) grows clockwise seen from above; a yaw about +Y
            // turns +Z toward +X, so the rate that closes `off` is its
            // negative.
            let wanted_spin = (-off * TURN_GAIN).clamp(-TURN_RATE, TURN_RATE);
            physics.world.apply_angular_impulse(
                rb.0,
                Vector3::y() * (wanted_spin - spin.y) * flyer.yaw_inertia,
            );
        }
    }
}

/// The change of velocity a flyer makes this step: toward level flight
/// along `heading` (zero to hover) and toward the height `rise` above it.
fn steer(
    flyer: &Flyer,
    heading: Vector3<f32>,
    velocity: Vector3<f32>,
    rise: f32,
    dt: f32,
) -> Vector3<f32> {
    let accel = flyer.speed / flyer.accel_time.max(1e-3);
    let flat = Vector3::new(velocity.x, 0.0, velocity.z);
    let wanted_flat = heading
        .try_normalize(1e-4)
        .map_or_else(Vector3::zeros, |direction| direction * flyer.speed);
    let flat_change = (wanted_flat - flat).cap_magnitude(accel * dt);
    let wanted_climb = (rise * HEIGHT_GAIN).clamp(-CLIMB_SPEED, CLIMB_SPEED);
    let climb_change = (wanted_climb - velocity.y).clamp(-accel * dt, accel * dt);
    flat_change + Vector3::y() * climb_change
}

fn wrap_angle(angle: f32) -> f32 {
    (angle + PI).rem_euclid(TAU) - PI
}

#[cfg(test)]
mod tests {
    use super::*;

    const FLYER: Flyer = Flyer {
        mass: 10.0,
        yaw_inertia: 1.0,
        speed: 6.0,
        accel_time: 1.0,
        cruise_height: 5.0,
        stoop_range: 4.0,
        stoop_height: 0.5,
    };

    /// Fly from rest for `seconds`, always `rise_to` above where it started.
    fn fly(heading: Vector3<f32>, rise_to: f32, seconds: f32) -> (Vector3<f32>, f32) {
        let dt = 1.0 / 60.0;
        let (mut velocity, mut height) = (Vector3::zeros(), 0.0);
        for _ in 0..(seconds / dt) as usize {
            velocity += steer(&FLYER, heading, velocity, rise_to - height, dt);
            height += velocity.y * dt;
        }
        (velocity, height)
    }

    #[test]
    fn it_comes_up_to_speed_along_its_heading_and_no_faster() {
        let (velocity, _) = fly(Vector3::z(), 0.0, 3.0);
        assert!((velocity.z - FLYER.speed).abs() < 0.05, "{velocity:?}");
        assert!(velocity.x.abs() < 1e-3);
    }

    #[test]
    fn it_climbs_to_the_height_it_is_given_and_holds_it() {
        let (velocity, height) = fly(Vector3::zeros(), 5.0, 6.0);
        assert!((height - 5.0).abs() < 0.1, "at {height} m");
        assert!(velocity.norm() < 0.2, "still moving at {velocity:?}");
    }

    #[test]
    fn it_stoops_as_readily_as_it_climbs() {
        let (_, height) = fly(Vector3::zeros(), -4.0, 6.0);
        assert!((height + 4.0).abs() < 0.1, "at {height} m");
    }
}
