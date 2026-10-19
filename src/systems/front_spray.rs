//! Whitewater where a surge runs out over dry ground, on a fixed budget.
//!
//! ```text
//!   WaterField::fronts ──▶ weight each by how far its water stands over
//!                          the floor it runs on to, none past SPRAY_RANGE
//!                          from the eye
//!                      ──▶ spend this frame's share of the budget on fronts
//!                          drawn by weight: spume riding the front over its
//!                          leading edge, and droplets thrown on ahead
//! ```
//!
//! A front is where the drawn surface ends against nothing but its cells'
//! edges, so the spume is what the eye reads there, as it would on a real
//! surge breaking over a floor. It is weighed by its height, not its power:
//! the field's leading edge is a film running out slowly however fast the
//! surge behind it, so its discharge says little of what the eye sees, which
//! is a wall of water standing over dry ground.

use nalgebra::{Point3, Vector3, Vector4};
use rand::Rng;

use super::spray_budget::{share, SprayBudget};
use crate::particles::{ColourRamp, Particle, ParticlePool};
use crate::water::field::{Front, WaterField};

/// Puffs of spume and droplets spawned a second, over every front at once.
const SPUME_PER_SECOND: f32 = 1600.0;
const DROPLETS_PER_SECOND: f32 = 800.0;

/// Heights a front's water stands over the floor ahead that it is weighed
/// over, m: lower, it creeps on rather than breaks; higher counts no more.
const LEAST_HEIGHT: f32 = 0.1;
const FULL_HEIGHT: f32 = 0.6;

/// Summed heights at which the fronts take the whole budget, m: a tunnel's
/// width of half-metre surge, both edges and up its walls.
const FULL_WEIGHT: f32 = 12.0;

/// Share of the way from the water running out up to the surge's crest at
/// which the surface is drawn over a front's face.
const TILT_SHARE: f32 = 0.35;

/// How far past the ends of its face a front's spray reaches, as a share of
/// the face's half width: neighbouring fronts' spray overlaps, so a stepped
/// edge is covered across its steps.
const SIDEWAYS: f32 = 2.0;

/// Furthest out over the floor ahead spray starts, m: past the half column
/// the drawn surface carries on beyond the front's face.
const AHEAD: f32 = 0.9;

/// Farthest from the eye a front throws spray, m.
const SPRAY_RANGE: f32 = 60.0;

/// Spume and droplets where the water's fronts run out, on budgets carried
/// between frames.
#[derive(Debug)]
pub struct FrontSpray {
    spume: SprayBudget,
    droplets: SprayBudget,
}

impl Default for FrontSpray {
    fn default() -> Self {
        Self {
            spume: SprayBudget::new(SPUME_PER_SECOND),
            droplets: SprayBudget::new(DROPLETS_PER_SECOND),
        }
    }
}

impl FrontSpray {
    /// Spend `dt`'s budget on the fronts of `field` near `eye`.
    pub fn emit(
        &mut self,
        field: &WaterField,
        eye: Option<Point3<f32>>,
        dt: f32,
        pool: &mut ParticlePool,
    ) {
        let fronts: Vec<(Front, f32)> = field
            .fronts(eye.map(|e| (e, SPRAY_RANGE)))
            .filter(|f| eye.is_none_or(|e| (f.top - e).norm() < SPRAY_RANGE))
            .map(|f| (f, (f.crest - f.foot).min(FULL_HEIGHT)))
            .filter(|&(_, height)| height > LEAST_HEIGHT)
            .collect();
        let total: f32 = fronts.iter().map(|&(_, height)| height).sum();
        let share = share(total, FULL_WEIGHT);
        let mut rng = rand::thread_rng();
        self.spume.spend(&fronts, share, dt, &mut rng, |f, rng| {
            pool.spawn(spume(f, rng))
        });
        self.droplets.spend(&fronts, share, dt, &mut rng, |f, rng| {
            pool.spawn(droplet(f, rng))
        });
    }
}

/// Somewhere over the front's edge: along its face, out over the floor
/// ahead, a little over where the surface is drawn there. That is above the
/// water running out, since the film's corners are drawn tilting up towards
/// the surge (under the surface, the water would be drawn over the spray),
/// but well below the crest: higher, seen from above, the spray would stand
/// over the water behind the edge.
fn on_face(front: &Front, rng: &mut impl Rng) -> Vector3<f32> {
    let along = front.across * rng.gen_range(-1.0..1.0) * Front::HALF_WIDTH * SIDEWAYS;
    let out = front
        .velocity
        .try_normalize(1.0e-6)
        .unwrap_or_else(Vector3::zeros)
        * rng.gen_range(0.0..AHEAD);
    let drawn = front.top.y + (front.crest - front.top.y) * TILT_SHARE;
    let height = drawn + rng.gen_range(0.05..0.25);
    Vector3::new(front.top.x, height, front.top.z) + along + out
}

/// A puff of spume riding the front over its edge: white, torn, swelling
/// and gone within a second.
fn spume(front: &Front, rng: &mut impl Rng) -> Particle {
    let ramp = ColourRamp::fade(
        Vector4::new(0.92, 0.96, 1.0, 0.55),
        Vector4::new(0.9, 0.95, 1.0, 0.0),
    );
    let velocity =
        front.velocity * rng.gen_range(0.6..1.0) + Vector3::y() * rng.gen_range(0.3..1.0);
    let mut puff = Particle::new(
        on_face(front, rng),
        velocity,
        rng.gen_range(0.12..0.26),
        rng.gen_range(0.5..1.0),
        ramp,
    )
    .with_dynamics(0.1, 1.5);
    puff.growth = 2.5;
    puff.billow = 1.0;
    puff.angle = rng.gen_range(0.0..std::f32::consts::TAU);
    puff.seed = rng.gen_range(0.0..1.0);
    puff
}

/// A droplet thrown on ahead of the front and up off it.
fn droplet(front: &Front, rng: &mut impl Rng) -> Particle {
    let angle = rng.gen_range(0.0..std::f32::consts::TAU);
    let scatter = Vector3::new(angle.cos(), 0.0, angle.sin()) * rng.gen_range(0.0..0.8);
    let velocity =
        front.velocity * rng.gen_range(0.8..1.3) + scatter + Vector3::y() * rng.gen_range(1.0..2.5);
    let ramp = ColourRamp::fade(
        Vector4::new(0.88, 0.94, 1.0, 0.9),
        Vector4::new(0.8, 0.9, 1.0, 0.0),
    );
    Particle::new(
        on_face(front, rng),
        velocity,
        rng.gen_range(0.04..0.09),
        rng.gen_range(0.5..0.9),
        ramp,
    )
    .with_dynamics(1.0, 0.4)
    .with_stretch(0.03)
}
