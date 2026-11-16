//! The [`Lobber`] component and [`LobberSystem`], which throws every
//! lobber's bombs and poses the bodies that show them coming.

use nalgebra::{Point3, UnitQuaternion, Vector3};
use specs::{
    Component, DenseVecStorage, Entities, Join, LazyUpdate, Read, ReadStorage, System, Write,
    WriteStorage,
};

use super::body::{LobberBody, Showing, Throw};
use super::puffing::Puffing;
use crate::aim::launch::gravity_along;
use crate::components::{Orientation, PosedModel, Position, ProceduralMesh};
use crate::creature::perception::Perception;
use crate::damage::Dead;
use crate::projectile::{spawn_grenade, GrenadeConfig, GrenadeModelResource};
use crate::systems::PhysicsResource;
use crate::time::Time;

/// How a creature lobs.
#[derive(Component, Debug, Clone)]
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
    /// The parts of its [`PosedModel`] that show a throw coming, if it has
    /// any.
    pub body: Option<LobberBody>,
    /// Seconds since it last threw, if it has.
    pub since_throw: Option<f32>,
    /// Where its cheeks are in their puffing.
    pub puffing: Puffing,
}

impl Lobber {
    /// How far it is from its last throw to its next, 0 to 1.
    pub fn fullness(&self) -> f32 {
        if self.interval <= 0.0 {
            return 1.0;
        }
        (1.0 - self.remaining / self.interval).clamp(0.0, 1.0)
    }
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

/// Throws every lobber's bombs, and poses the bodies that show them coming.
pub struct LobberSystem;

impl<'a> System<'a> for LobberSystem {
    type SystemData = (
        Entities<'a>,
        Read<'a, Time>,
        Option<Read<'a, GrenadeConfig>>,
        Read<'a, GrenadeModelResource>,
        Write<'a, PhysicsResource>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Orientation>,
        ReadStorage<'a, Perception>,
        WriteStorage<'a, Lobber>,
        WriteStorage<'a, PosedModel>,
        WriteStorage<'a, ProceduralMesh>,
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
            orientations,
            perceptions,
            mut lobbers,
            mut posed,
            mut procedural,
            deads,
            lazy,
        ) = data;
        // A world with no grenades to throw (a tool that only looks) has no
        // lobbing either.
        let grenades = config.zip(model.model.clone());
        let gravity = grenades
            .as_ref()
            .map(|(config, _)| gravity_along(physics.world.config().gravity, config.gravity));

        for (entity, position, perception, lobber, _) in
            (&entities, &positions, &perceptions, &mut lobbers, !&deads).join()
        {
            lobber.remaining = (lobber.remaining - time.delta_seconds()).max(0.0);
            if let Some(since) = &mut lobber.since_throw {
                *since += time.delta_seconds();
            }
            let fullness = lobber.fullness();
            lobber.puffing.advance(fullness, time.delta_seconds());
            let target = perception.target.filter(|target| target.visible);
            let in_range = target.is_some_and(|target| {
                target.distance >= lobber.range.0 && target.distance <= lobber.range.1
            });

            if let (Some(body), Some(posed)) = (&lobber.body, posed.get_mut(entity)) {
                let facing = orientations
                    .get(entity)
                    .map_or_else(UnitQuaternion::identity, |orientation| orientation.0);
                let watched =
                    target.map(|target| facing.inverse() * (target.position.coords - position.0));
                // No two breathe together.
                let clock = time.total_seconds() + 1.7 * entity.id() as f32;
                let now = Showing {
                    throw: Throw {
                        fullness,
                        since: lobber.since_throw,
                        until: in_range.then_some(lobber.remaining),
                    },
                    clock,
                    blown: lobber.puffing.blown(),
                    target: watched,
                };
                // A body without a mesh of its own to shape shapes its
                // cheeks into nothing kept.
                let mut unkept = Vec::new();
                let shaped = procedural
                    .get_mut(entity)
                    .map_or(&mut unkept, |mesh| &mut mesh.primitives);
                body.pose(&mut posed.poses, shaped, &now);
            }

            let (Some(target), Some((config, model)), Some(gravity)) = (target, &grenades, gravity)
            else {
                continue;
            };
            if lobber.remaining > 0.0 || !in_range {
                continue;
            }
            lobber.remaining = lobber.interval;
            lobber.since_throw = Some(0.0);
            let origin = Point3::from(position.0 + lobber.muzzle);
            spawn_grenade(
                lazy.create_entity(&entities),
                &mut physics.world,
                config,
                model.clone(),
                origin,
                lob(origin, target.position, lobber.flight_time, gravity),
            );
        }

        for (entity, lobber, posed, _) in (&entities, &lobbers, &mut posed, &deads).join() {
            if let Some(body) = &lobber.body {
                let mut unkept = Vec::new();
                let shaped = procedural
                    .get_mut(entity)
                    .map_or(&mut unkept, |mesh| &mut mesh.primitives);
                body.slump(&mut posed.poses, shaped);
            }
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
