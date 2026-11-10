//! Lobbing: a creature that throws bombs at what it sees.
//!
//! ```text
//!   Perception ──target in range──▶ a grenade, launched on the arc that
//!                                   lands on the target after `flight_time`
//! ```
//!
//! The bombs are the game's own grenades, so they fall, bounce, go off and
//! carve the ground exactly as the player's do: a lobber pins a route down,
//! and breaks it up while it does.

use nalgebra::{Point3, Vector3};
use specs::{
    Component, DenseVecStorage, Entities, Join, LazyUpdate, Read, ReadStorage, System, Write,
    WriteStorage,
};

use super::perception::Perception;
use crate::aim::launch::gravity_along;
use crate::components::Position;
use crate::damage::Dead;
use crate::projectile::{spawn_grenade, GrenadeConfig, GrenadeModelResource};
use crate::systems::PhysicsResource;
use crate::time::Time;

/// How a creature lobs.
#[derive(Component, Debug, Clone, Copy)]
#[storage(DenseVecStorage)]
pub struct Lobber {
    /// Nearest and furthest it will throw, m: nearer, the bomb would land on
    /// itself.
    pub range: (f32, f32),
    /// Seconds between throws.
    pub interval: f32,
    /// Seconds until it may throw again.
    pub remaining: f32,
    /// Seconds a bomb is in the air: long is a high slow arc, with time to
    /// get out from under it.
    pub flight_time: f32,
    /// Where a bomb leaves it, from its position.
    pub muzzle: Vector3<f32>,
}

/// The velocity that carries a bomb from `from` to `to` in `flight_time`
/// under `gravity`.
pub fn lob(
    from: Point3<f32>,
    to: Point3<f32>,
    flight_time: f32,
    gravity: Vector3<f32>,
) -> Vector3<f32> {
    (to - from) / flight_time - gravity * (0.5 * flight_time)
}

/// Throws every lobber's bombs.
pub struct LobberSystem;

impl<'a> System<'a> for LobberSystem {
    type SystemData = (
        Entities<'a>,
        Read<'a, Time>,
        Option<Read<'a, GrenadeConfig>>,
        Read<'a, GrenadeModelResource>,
        Write<'a, PhysicsResource>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Perception>,
        WriteStorage<'a, Lobber>,
        ReadStorage<'a, Dead>,
        Read<'a, LazyUpdate>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            entities,
            time,
            config,
            model,
            mut physics,
            positions,
            perceptions,
            mut lobbers,
            deads,
            lazy,
        ) = data;
        // A world with no grenades to throw (a tool that only looks) has no
        // lobbing either.
        let (Some(config), Some(model)) = (config, model.model.clone()) else {
            return;
        };
        let gravity = gravity_along(physics.world.config().gravity, config.gravity);
        for (position, perception, lobber, _) in
            (&positions, &perceptions, &mut lobbers, !&deads).join()
        {
            lobber.remaining = (lobber.remaining - time.delta_seconds()).max(0.0);
            let Some(target) = perception.target.filter(|target| target.visible) else {
                continue;
            };
            let in_range = target.distance >= lobber.range.0 && target.distance <= lobber.range.1;
            if lobber.remaining > 0.0 || !in_range {
                continue;
            }
            lobber.remaining = lobber.interval;
            let origin = Point3::from(position.0 + lobber.muzzle);
            spawn_grenade(
                lazy.create_entity(&entities),
                &mut physics.world,
                &config,
                model.clone(),
                origin,
                lob(origin, target.position, lobber.flight_time, gravity),
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_lobbed_bomb_lands_on_what_it_was_thrown_at() {
        let gravity = Vector3::new(0.0, -12.0, 0.0);
        let (from, to) = (Point3::new(1.0, 3.0, -2.0), Point3::new(9.0, 0.5, 4.0));
        let flight = 1.8;
        let velocity = lob(from, to, flight, gravity);
        // Flown in small steps, as the physics flies it.
        let (mut at, mut speed) = (from, velocity);
        let dt = 1.0e-3;
        for _ in 0..(flight / dt).round() as usize {
            speed += gravity * dt;
            at += speed * dt;
        }
        assert!((at - to).norm() < 0.05, "landed at {at:?}");
        assert!(velocity.y > 0.0, "thrown downward");
    }
}
