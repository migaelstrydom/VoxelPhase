//! Spray and mist where falling water lands, on a fixed budget.
//!
//! ```text
//!   FallingWater::landings ──▶ those landing in water that stays: not a
//!                              step down a cliff, whose water falls on
//!                          ──▶ weight each by its power, Q·u², none past
//!                              SPRAY_RANGE from the eye
//!                          ──▶ spend this frame's share of the budget on
//!                              landings drawn by weight: droplets thrown up
//!                              and out of the foam, and now and then a puff
//!                              of mist rising off it; where water falling
//!                              into the void has broken up in the air, only
//!                              haze going on down
//! ```
//!
//! The budget is a rate, not a count per fall: however many falls a level
//! has, they share `DROPLETS_PER_SECOND` and `MIST_PER_SECOND`, the bigger
//! and nearer taking more of it.

use nalgebra::{Point3, Vector3, Vector4};
use rand::Rng;

use super::spray_budget::{share, SprayBudget};
use crate::particles::{ColourRamp, Particle, ParticlePool};
use crate::water::falling::{FallingWater, Landing};

/// Droplets and puffs of mist spawned a second, over every fall at once.
const DROPLETS_PER_SECOND: f32 = 360.0;
const MIST_PER_SECOND: f32 = 30.0;

/// The least power a landing must have to throw spray, m³/s · (m/s)²: a
/// trickle lands without any.
const LEAST_POWER: f32 = 2.0;

/// Power at which a single landing takes the whole budget, m³/s · (m/s)².
/// Below it the budget is spent in proportion, so a level with only small
/// falls throws less spray than one with a big one.
const FULL_POWER: f32 = 2000.0;

/// Farthest from the eye a landing throws spray, m.
const SPRAY_RANGE: f32 = 80.0;

/// How far back up its fall, as seconds of its speed where it is gone,
/// water broken up in the void hangs as haze: about the time it takes to
/// break up (`SHEET_BREAKUP` in `shader/water.frag`).
const HAZE_SPREAD: f32 = 0.6;

/// Spray and mist where falls land, on budgets carried between frames.
#[derive(Debug)]
pub struct FallSpray {
    droplets: SprayBudget,
    mist: SprayBudget,
}

impl Default for FallSpray {
    fn default() -> Self {
        Self {
            droplets: SprayBudget::new(DROPLETS_PER_SECOND),
            mist: SprayBudget::new(MIST_PER_SECOND),
        }
    }
}

impl FallSpray {
    /// Spend `dt`'s budget on the landings of `falling` near `eye`.
    pub fn emit(
        &mut self,
        falling: &FallingWater,
        eye: Option<Point3<f32>>,
        dt: f32,
        pool: &mut ParticlePool,
    ) {
        let landings: Vec<(Landing, f32)> = falling
            .landings()
            .filter(|l| !l.onward)
            .filter(|l| eye.is_none_or(|e| (l.at - e).norm() < SPRAY_RANGE))
            .map(|l| (l, l.discharge * l.velocity.norm_squared()))
            .filter(|&(_, power)| power > LEAST_POWER)
            .collect();
        let total: f32 = landings.iter().map(|&(_, power)| power).sum();
        let share = share(total, FULL_POWER);
        let mut rng = rand::thread_rng();
        // Water broken up in the void throws no droplets out of foam: it is
        // spray already, and goes on down as a haze.
        let splashing: Vec<(Landing, f32)> =
            landings.iter().copied().filter(|(l, _)| !l.void).collect();
        self.droplets
            .spend(&splashing, share, dt, &mut rng, |l, rng| {
                pool.spawn(droplet(l, rng))
            });
        self.mist.spend(&landings, share, dt, &mut rng, |l, rng| {
            pool.spawn(if l.void { haze(l, rng) } else { mist(l, rng) })
        });
    }
}

/// A puff of the spray water falling into the void has broken up into:
/// somewhere along the last of its fall, carried on down the way the water
/// was going, slowing and spreading as it fades.
fn haze(landing: &Landing, rng: &mut impl Rng) -> Particle {
    let angle = rng.gen_range(0.0..std::f32::consts::TAU);
    let drift = Vector3::new(angle.cos(), 0.0, angle.sin()) * rng.gen_range(0.3..1.0);
    let ramp = ColourRamp::fade(
        Vector4::new(0.88, 0.94, 1.0, 0.1),
        Vector4::new(0.9, 0.95, 1.0, 0.0),
    );
    let back_up = landing.velocity * rng.gen_range(-HAZE_SPREAD..0.0);
    let mut puff = Particle::new(
        along(landing, rng) + back_up,
        landing.velocity * rng.gen_range(0.2..0.35) + drift,
        rng.gen_range(1.0..1.8),
        rng.gen_range(2.0..3.0),
        ramp,
    )
    .with_dynamics(-0.02, 1.2);
    puff.growth = 3.0;
    puff.billow = 1.0;
    puff.angle = rng.gen_range(0.0..std::f32::consts::TAU);
    puff.seed = rng.gen_range(0.0..1.0);
    puff
}

/// Somewhere along where `landing` meets the water.
fn along(landing: &Landing, rng: &mut impl Rng) -> Vector3<f32> {
    landing.at.coords + landing.across * rng.gen_range(-1.0..1.0) * landing.half_length
}

/// A droplet thrown up and out of the foam: back up at a share of the
/// speed the water lands with, the way it was going and every way round.
fn droplet(landing: &Landing, rng: &mut impl Rng) -> Particle {
    let level = Vector3::new(landing.velocity.x, 0.0, landing.velocity.z);
    let angle = rng.gen_range(0.0..std::f32::consts::TAU);
    let scatter = Vector3::new(angle.cos(), 0.0, angle.sin());
    let speed = landing.velocity.norm() * rng.gen_range(0.15..0.35);
    let direction = (scatter + level * 0.15 + Vector3::y() * rng.gen_range(0.8..2.0)).normalize();
    let ramp = ColourRamp::fade(
        Vector4::new(0.88, 0.94, 1.0, 0.9),
        Vector4::new(0.8, 0.9, 1.0, 0.0),
    );
    Particle::new(
        along(landing, rng),
        direction * speed,
        rng.gen_range(0.04..0.1),
        rng.gen_range(0.4..0.8),
        ramp,
    )
    .with_dynamics(1.0, 0.6)
}

/// A puff of mist rising slowly off the foam and spreading as it fades:
/// thin, so that a big fall's many puffs make a haze over its foot and not a
/// white ball in front of it.
fn mist(landing: &Landing, rng: &mut impl Rng) -> Particle {
    let angle = rng.gen_range(0.0..std::f32::consts::TAU);
    let drift = Vector3::new(angle.cos(), 0.0, angle.sin()) * rng.gen_range(0.2..0.8);
    let ramp = ColourRamp::fade(
        Vector4::new(0.85, 0.92, 1.0, 0.14),
        Vector4::new(0.9, 0.95, 1.0, 0.0),
    );
    let mut puff = Particle::new(
        along(landing, rng) + Vector3::y() * 0.2,
        drift + Vector3::y() * rng.gen_range(0.3..0.9),
        rng.gen_range(0.3..0.6),
        rng.gen_range(1.5..2.5),
        ramp,
    )
    .with_dynamics(-0.02, 1.2);
    puff.growth = 2.0;
    puff.billow = 1.0;
    puff.angle = rng.gen_range(0.0..std::f32::consts::TAU);
    puff.seed = rng.gen_range(0.0..1.0);
    puff
}
