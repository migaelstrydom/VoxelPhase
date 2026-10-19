//! What a field throws up where it meets the ground: sparks off the ring,
//! more of them under load and in a burst under strain, and dust when the
//! pod sweeps across the ground.
//!
//! ```text
//!   PodLook (ground ring, heat, strain, speed) ──▶ rates, per second
//!        ──▶ owed this frame ──▶ sparks and dust spawned on the ring
//! ```

use nalgebra::{Point3, Vector3, Vector4};
use rand::Rng;

use crate::particles::{ColourRamp, Particle, ParticlePool};

use super::column::basis;
use super::pod::PodLook;

/// Sparks a second from a ring at rest, at full heat, and at full strain.
const SPARKS_AT_REST: f32 = 3.0;
const SPARKS_AT_HEAT: f32 = 30.0;
const SPARKS_AT_STRAIN: f32 = 120.0;
/// Dust a second per m/s of sweep across the ground.
const DUST_PER_SPEED: f32 = 7.0;
/// Least sweep that raises dust, m/s.
const DUST_FROM_SPEED: f32 = 0.4;
/// Colour of the dust: a pale earth, whatever the ground.
const DUST: Vector3<f32> = Vector3::new(0.58, 0.53, 0.45);
/// Most particles one field may spawn in a frame, whatever it is owed.
const MOST_PER_FRAME: usize = 40;

/// The kick-up of one field: carries what it owes between frames.
#[derive(Debug, Default, Clone)]
pub struct KickUp {
    sparks_owed: f32,
    dust_owed: f32,
}

impl KickUp {
    /// Spend `dt`'s worth of kick-up on the ground rings among `looks`.
    pub fn emit(&mut self, looks: &[PodLook], dt: f32, pool: &mut ParticlePool) {
        let grounded: Vec<&PodLook> = looks.iter().filter(|l| l.ground_ring.is_some()).collect();
        if grounded.is_empty() {
            self.sparks_owed = 0.0;
            self.dust_owed = 0.0;
            return;
        }

        let mut rng = rand::thread_rng();
        let mut spawned = 0;
        for look in grounded {
            let strain = look.column.map_or(0.0, |c| c.strain);
            self.sparks_owed += (SPARKS_AT_REST
                + SPARKS_AT_HEAT * look.heat * look.heat
                + SPARKS_AT_STRAIN * strain)
                * dt;
            let sweep = (look.ground_speed - DUST_FROM_SPEED).max(0.0);
            self.dust_owed += DUST_PER_SPEED * sweep * (0.5 + look.heat) * dt;

            while self.sparks_owed >= 1.0 && spawned < MOST_PER_FRAME {
                self.sparks_owed -= 1.0;
                spawned += 1;
                pool.spawn(spark(look, &mut rng));
            }
            while self.dust_owed >= 1.0 && spawned < MOST_PER_FRAME {
                self.dust_owed -= 1.0;
                spawned += 1;
                pool.spawn(dust(look, &mut rng));
            }
        }
        self.sparks_owed = self.sparks_owed.min(1.0);
        self.dust_owed = self.dust_owed.min(1.0);
    }
}

/// A point on `look`'s ground ring, and the outward direction there.
fn on_ring(look: &PodLook, rng: &mut impl Rng) -> (Point3<f32>, Vector3<f32>, Vector3<f32>) {
    let ring = look.ground_ring.expect("only grounded pods kick up");
    let (across, along) = basis(&ring.normal);
    let angle = rng.gen_range(0.0..std::f32::consts::TAU);
    let outward = across * angle.cos() + along * angle.sin();
    (ring.centre + outward * ring.radius, outward, ring.normal)
}

/// A spark flicked outward off the ring in the field's colour, glowing.
fn spark(look: &PodLook, rng: &mut impl Rng) -> Particle {
    let (at, outward, up) = on_ring(look, rng);
    let speed = rng.gen_range(0.8..2.2) * (1.0 + look.heat);
    let velocity = outward * speed + up * rng.gen_range(0.4..1.4);
    let c = look.colour;
    let ramp = ColourRamp::fade(
        Vector4::new(c.x, c.y, c.z, 1.0),
        Vector4::new(c.x, c.y, c.z, 0.0),
    );
    let mut spark = Particle::new(
        at.coords + up * 0.02,
        velocity,
        rng.gen_range(0.007..0.015),
        rng.gen_range(0.2..0.45),
        ramp,
    )
    .with_dynamics(0.8, 1.5)
    .with_stretch(0.025);
    spark.additive = 1.0;
    spark
}

/// A puff of dust raised off the ground at the ring's edge.
fn dust(look: &PodLook, rng: &mut impl Rng) -> Particle {
    let (at, outward, up) = on_ring(look, rng);
    let velocity = outward * rng.gen_range(0.4..1.1) + up * rng.gen_range(0.15..0.5);
    let ramp = ColourRamp::fade(
        Vector4::new(DUST.x, DUST.y, DUST.z, 0.35),
        Vector4::new(DUST.x, DUST.y, DUST.z, 0.0),
    );
    let mut puff = Particle::new(
        at.coords + up * 0.05,
        velocity,
        rng.gen_range(0.06..0.12),
        rng.gen_range(0.5..0.9),
        ramp,
    )
    .with_dynamics(-0.05, 2.0);
    puff.growth = 3.0;
    puff.billow = 1.0;
    puff.angle = rng.gen_range(0.0..std::f32::consts::TAU);
    puff.seed = rng.gen_range(0.0..1.0);
    puff
}
