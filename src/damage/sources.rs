//! The three ways something gets hurt. Each is a pure producer: it reads the
//! world and pushes [`DamageEvent`]s, never touching `Health` itself.

use nalgebra::Vector3;
use specs::{
    Component, DenseVecStorage, Entities, Join, Read, ReadStorage, System, Write, WriteStorage,
};

use super::event::{DamageKind, DamageQueue};
use super::health::{Dead, Health};
use crate::components::{Position, Velocity};
use crate::explosion::Explosion;
use crate::fire::components::OnFire;
use crate::time::Time;

/// Damage at the centre of a blast, before falloff.
const BLAST_PEAK_DAMAGE: f32 = 120.0;

/// Hit points per second of burning.
const BURN_DAMAGE_PER_SECOND: f32 = 6.0;

/// Hit points per m/s of speed change beyond a body's impact tolerance.
const IMPACT_DAMAGE_PER_EXCESS_SPEED: f32 = 7.0;

/// Converts explosions into damage on anything with `Health` in the blast.
///
/// Falloff is quadratic rather than the linear curve `ExplosionSystem` uses for
/// knockback: knockback wants to fling things at the rim for spectacle, but
/// damage should reward landing a grenade *on* a target rather than near one.
pub struct BlastDamageSystem;

impl<'a> System<'a> for BlastDamageSystem {
    type SystemData = (
        Entities<'a>,
        ReadStorage<'a, Explosion>,
        ReadStorage<'a, Health>,
        ReadStorage<'a, Position>,
        Write<'a, DamageQueue>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, explosions, healths, positions, mut queue) = data;

        // Explosions are marked processed by `ExplosionSystem` and deleted at
        // the next `world.maintain()`, so they are still readable here.
        let blasts: Vec<_> = (&explosions)
            .join()
            .map(|e| (e.center, e.blast_radius))
            .collect();
        if blasts.is_empty() {
            return;
        }

        for (entity, _health, pos) in (&entities, &healths, &positions).join() {
            for &(center, blast_radius) in &blasts {
                if blast_radius <= 0.0 {
                    continue;
                }
                let distance = (pos.0 - center.coords).magnitude();
                if distance >= blast_radius {
                    continue;
                }
                let falloff = 1.0 - (distance / blast_radius);
                queue.push(
                    entity,
                    BLAST_PEAK_DAMAGE * falloff * falloff,
                    DamageKind::Blast,
                );
            }
        }
    }
}

/// Burns anything that is both on fire and alive, scaled by its resistance.
pub struct BurnDamageSystem;

impl<'a> System<'a> for BurnDamageSystem {
    type SystemData = (
        Entities<'a>,
        Read<'a, Time>,
        ReadStorage<'a, OnFire>,
        ReadStorage<'a, Health>,
        Write<'a, DamageQueue>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, time, on_fires, healths, mut queue) = data;
        let dt = time.delta_seconds();

        for (entity, _fire, health) in (&entities, &on_fires, &healths).join() {
            queue.push(
                entity,
                BURN_DAMAGE_PER_SECOND * health.burn_resistance * dt,
                DamageKind::Burn,
            );
        }
    }
}

/// Damage from being brought to a sudden stop.
///
/// One mechanism covers falling, being crushed under a dropped menhir, and
/// being slammed into a wall by a blast: all three show up as the body *losing*
/// a large amount of speed in a single frame. Comparing frame to frame avoids
/// needing per-contact solver impulses, which the physics engine does not
/// expose per body.
///
/// Only speed lost counts, never speed gained. Being flung does not hurt —
/// landing does. Without that asymmetry a grenade would damage its target
/// twice: once as blast, then again for the knockback it just imparted, and
/// `ExplosionSystem` adds knockback straight to `Velocity` in amounts that
/// would be instantly lethal.
///
/// The previous velocity lives in a `LastVelocity` component rather than in the
/// system so it survives the system being reordered or run in parallel.
#[derive(Default)]
pub struct ImpactDamageSystem;

impl<'a> System<'a> for ImpactDamageSystem {
    type SystemData = (
        Entities<'a>,
        ReadStorage<'a, Health>,
        ReadStorage<'a, Velocity>,
        ReadStorage<'a, Dead>,
        WriteStorage<'a, LastVelocity>,
        Write<'a, DamageQueue>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, healths, velocities, deads, mut last_velocities, mut queue) = data;

        for (entity, health, vel, _) in (&entities, &healths, &velocities, !&deads).join() {
            let previous = last_velocities
                .get(entity)
                .map(|l| l.0)
                .unwrap_or_else(|| vel.0);
            let speed_lost = previous.magnitude() - vel.0.magnitude();

            let excess = speed_lost - health.impact_tolerance;
            if excess > 0.0 {
                queue.push(
                    entity,
                    excess * IMPACT_DAMAGE_PER_EXCESS_SPEED,
                    DamageKind::Impact,
                );
            }

            let _ = last_velocities.insert(entity, LastVelocity(vel.0));
        }
    }
}

/// Previous frame's velocity, kept for [`ImpactDamageSystem`]'s delta.
///
/// Inserted automatically for any entity with `Health`; nothing needs to spawn
/// with it. The first frame reads the current velocity as its own previous, so
/// a body spawned already moving is not damaged for existing.
#[derive(Component, Debug, Default)]
#[storage(DenseVecStorage)]
pub struct LastVelocity(pub Vector3<f32>);
