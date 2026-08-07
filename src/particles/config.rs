//! The library of authored particle effects.
//!
//! Every effect is a [`ParticleSpec`] — data, not code. The numbers here are
//! the art: what follows is where an explosion's look is tuned, and nothing in
//! this file does anything but describe particles.
//!
//! Colours are linear and unbounded above 1.0, because the scene target is
//! floating point: a white-hot core authored at 4.0 stays 4.0 through the frame
//! and blows out on tonemap the way a real one does, rather than clipping to a
//! flat white disc.

use nalgebra::Vector4;

use super::emitter::ParticleEffectType;
use super::ramp::{ColourRamp, ColourStop};
use super::spec::{LaunchPattern, ParticleSpec, Spread};

/// Shorthand for a linear RGBA colour.
const fn rgba(r: f32, g: f32, b: f32, a: f32) -> Vector4<f32> {
    Vector4::new(r, g, b, a)
}

/// Every particle effect the engine can spawn, plus the shared gravity all of
/// them are pulled by.
#[derive(Debug, Clone)]
pub struct ParticleConfig {
    /// The white-hot instant of detonation.
    pub blast_core: ParticleSpec,
    /// The rolling fireball that follows it.
    pub fireball: ParticleSpec,
    /// The low sheet of dust thrown outward along the ground.
    pub blast_dust: ParticleSpec,
    /// Burning fragments thrown clear of the blast.
    pub embers: ParticleSpec,
    /// Solid chunks torn out of whatever was hit.
    pub debris: ParticleSpec,
    /// The column left behind once the fire is out.
    pub smoke: ParticleSpec,
    /// Droplets thrown up by a body entering water.
    pub splash: ParticleSpec,
    /// Cooling embers shed by a hot object in flight.
    pub ember_trail: ParticleSpec,

    /// Gravity in world units per second squared, before each particle's own
    /// `gravity_scale`.
    pub gravity: f32,
}

impl ParticleConfig {
    pub fn new() -> Self {
        Self::default()
    }

    /// The spec for an effect type.
    pub fn spec(&self, effect_type: ParticleEffectType) -> &ParticleSpec {
        match effect_type {
            ParticleEffectType::BlastCore => &self.blast_core,
            ParticleEffectType::Fireball => &self.fireball,
            ParticleEffectType::BlastDust => &self.blast_dust,
            ParticleEffectType::Embers => &self.embers,
            ParticleEffectType::Debris => &self.debris,
            ParticleEffectType::Smoke => &self.smoke,
            ParticleEffectType::WaterSplash => &self.splash,
            ParticleEffectType::EmberTrail => &self.ember_trail,
        }
    }
}

impl Default for ParticleConfig {
    fn default() -> Self {
        Self {
            blast_core: blast_core(),
            fireball: fireball(),
            blast_dust: blast_dust(),
            embers: embers(),
            debris: debris(),
            smoke: smoke(),
            splash: splash(),
            ember_trail: ember_trail(),
            gravity: 9.81,
        }
    }
}

/// The detonation itself: a handful of enormous, blinding puffs that expand
/// through their whole life and are gone in a fifth of a second.
///
/// Short enough that it is never really *seen*, only registered — which is the
/// point. It exists so the first frame of an explosion is overwhelming and the
/// fireball has something to emerge from.
fn blast_core() -> ParticleSpec {
    ParticleSpec {
        lifetime: Spread::new(0.10, 0.22),
        size: Spread::new(1.0, 1.9),
        speed: Spread::new(2.0, 7.0),
        launch: LaunchPattern::Sphere,
        spawn_radius: 0.5,
        ramp: ColourRamp::new(&[
            ColourStop::new(0.0, rgba(4.0, 3.6, 3.0, 1.0)),
            ColourStop::new(0.35, rgba(3.4, 2.1, 0.9, 0.9)),
            ColourStop::new(1.0, rgba(1.2, 0.4, 0.1, 0.0)),
        ]),
        growth: 3.5,
        spin: Spread::new(0.5, 2.0),
        additive: 1.0,
        billow: 1.0,
        gravity_scale: 0.0,
        drag: 1.2,
        turbulence: 0.0,
        stretch: 0.0,
    }
}

/// The fireball: the part of an explosion the eye actually reads.
///
/// Three properties carry it, and all three are easy to get wrong:
///
/// - **It stops, but not before it has got somewhere.** Drag has to bleed off
///   most of the launch speed within a few tenths of a second, having first let
///   the puffs cover a couple of metres. Damped too hard and the whole blast
///   collapses into a ball the size of the thing that made it; damped too
///   little and fire keeps sailing outward like a firework.
/// - **It grows.** Each puff more than triples in size as it goes, so the ball
///   expands even after the particles have stopped moving apart.
/// - **It cools.** One ramp takes each puff from white-hot through yellow and
///   orange to dark soot, so the fire becomes the smoke instead of handing over
///   to a second effect at a visible seam.
fn fireball() -> ParticleSpec {
    ParticleSpec {
        lifetime: Spread::new(0.55, 1.3),
        size: Spread::new(0.4, 0.9),
        speed: Spread::new(6.0, 17.0),
        launch: LaunchPattern::Sphere,
        spawn_radius: 0.6,
        ramp: ColourRamp::new(&[
            ColourStop::new(0.0, rgba(3.0, 2.4, 1.5, 1.0)),
            ColourStop::new(0.10, rgba(2.2, 1.2, 0.35, 1.0)),
            ColourStop::new(0.35, rgba(1.0, 0.35, 0.07, 0.95)),
            ColourStop::new(0.65, rgba(0.30, 0.08, 0.02, 0.8)),
            ColourStop::new(1.0, rgba(0.07, 0.06, 0.055, 0.0)),
        ]),
        growth: 3.6,
        spin: Spread::new(0.4, 2.4),
        additive: 0.9,
        billow: 1.0,
        // Buoyant, but only just: the ball should lift as it burns out, not
        // launch. The rise the eye reads comes mostly from the smoke column.
        gravity_scale: -0.12,
        drag: 0.55,
        turbulence: 2.5,
        stretch: 0.0,
    }
}

/// The dust skirt: a fast, low ring running outward along the ground.
///
/// Nothing else communicates that a blast had a *ground* to happen on. It never
/// glows — it is lit debris, not fire — so it stays fully blended, and it is
/// what gives the explosion a visible radius after the fire has gone.
fn blast_dust() -> ParticleSpec {
    ParticleSpec {
        lifetime: Spread::new(1.0, 2.0),
        size: Spread::new(0.4, 0.9),
        speed: Spread::new(12.0, 22.0),
        launch: LaunchPattern::Ring {
            elevation: Spread::new(0.0, 18.0),
        },
        spawn_radius: 0.4,
        ramp: ColourRamp::new(&[
            ColourStop::new(0.0, rgba(0.055, 0.045, 0.035, 0.0)),
            ColourStop::new(0.08, rgba(0.065, 0.053, 0.042, 0.6)),
            ColourStop::new(0.5, rgba(0.095, 0.082, 0.070, 0.45)),
            ColourStop::new(1.0, rgba(0.14, 0.13, 0.12, 0.0)),
        ]),
        growth: 5.0,
        spin: Spread::new(0.3, 1.5),
        additive: 0.0,
        billow: 1.0,
        gravity_scale: 0.1,
        drag: 0.35,
        turbulence: 1.2,
        stretch: 0.0,
    }
}

/// Burning fragments thrown clear.
///
/// Long-lived and barely damped, so they are still arcing through the air after
/// the fireball is out — the detail that keeps an explosion alive in its second
/// half. Streaked along their travel, which at these speeds is what separates
/// an ember from a dot.
fn embers() -> ParticleSpec {
    ParticleSpec {
        lifetime: Spread::new(0.8, 2.2),
        size: Spread::new(0.04, 0.11),
        speed: Spread::new(8.0, 28.0),
        launch: LaunchPattern::Sphere,
        spawn_radius: 0.3,
        ramp: ColourRamp::new(&[
            ColourStop::new(0.0, rgba(3.0, 2.0, 0.8, 1.0)),
            ColourStop::new(0.15, rgba(1.6, 0.5, 0.08, 1.0)),
            ColourStop::new(0.55, rgba(0.8, 0.12, 0.02, 0.9)),
            ColourStop::new(1.0, rgba(0.4, 0.03, 0.0, 0.0)),
        ]),
        growth: 1.0,
        spin: Spread::fixed(0.0),
        additive: 1.0,
        billow: 0.0,
        gravity_scale: 0.55,
        drag: 0.12,
        turbulence: 0.6,
        stretch: 0.035,
    }
}

/// Solid chunks torn out of whatever the blast hit. Unlit, heavy, tumbling.
fn debris() -> ParticleSpec {
    ParticleSpec {
        lifetime: Spread::new(1.0, 2.6),
        size: Spread::new(0.08, 0.26),
        speed: Spread::new(7.0, 19.0),
        launch: LaunchPattern::Upward { up_bias: 0.3 },
        spawn_radius: 0.4,
        ramp: ColourRamp::new(&[
            ColourStop::new(0.0, rgba(0.30, 0.22, 0.15, 1.0)),
            ColourStop::new(0.8, rgba(0.26, 0.19, 0.13, 1.0)),
            ColourStop::new(1.0, rgba(0.26, 0.19, 0.13, 0.0)),
        ]),
        growth: 1.0,
        // Fast tumble: a chunk of rock has no reason to hold an orientation,
        // and the spin is most of what stops it reading as a sprite.
        spin: Spread::new(3.0, 10.0),
        additive: 0.0,
        billow: 0.35,
        gravity_scale: 1.0,
        drag: 0.1,
        turbulence: 0.0,
        stretch: 0.0,
    }
}

/// The column left behind: slow, dark, swelling, and by far the longest-lived
/// thing an explosion produces.
///
/// Its alpha starts at zero and rises over the first tenth of its life. A
/// cloud this large appearing at full opacity in one frame is the single most
/// obvious particle artefact there is, and the fade-in costs nothing.
fn smoke() -> ParticleSpec {
    ParticleSpec {
        lifetime: Spread::new(2.5, 5.0),
        size: Spread::new(0.8, 1.7),
        speed: Spread::new(2.0, 6.5),
        launch: LaunchPattern::Upward { up_bias: 0.35 },
        spawn_radius: 0.9,
        ramp: ColourRamp::new(&[
            ColourStop::new(0.0, rgba(0.018, 0.016, 0.015, 0.0)),
            ColourStop::new(0.10, rgba(0.024, 0.022, 0.020, 0.9)),
            ColourStop::new(0.5, rgba(0.055, 0.052, 0.050, 0.55)),
            ColourStop::new(1.0, rgba(0.115, 0.110, 0.105, 0.0)),
        ]),
        growth: 4.2,
        spin: Spread::new(0.15, 0.9),
        additive: 0.0,
        billow: 1.0,
        gravity_scale: -0.18,
        drag: 0.35,
        turbulence: 2.0,
        stretch: 0.0,
    }
}

/// Droplets thrown up by a body entering water.
fn splash() -> ParticleSpec {
    ParticleSpec {
        lifetime: Spread::new(0.4, 0.9),
        size: Spread::new(0.08, 0.20),
        speed: Spread::new(5.0, 10.0),
        launch: LaunchPattern::Upward { up_bias: 0.6 },
        spawn_radius: 0.0,
        ramp: ColourRamp::fade(rgba(0.7, 0.85, 1.0, 0.9), rgba(0.8, 0.9, 1.0, 0.0)),
        growth: 1.0,
        spin: Spread::fixed(0.0),
        additive: 0.0,
        billow: 0.0,
        gravity_scale: 1.0,
        drag: 0.3,
        turbulence: 0.0,
        stretch: 0.0,
    }
}

/// Cooling embers shed by a hot object in flight.
///
/// Tuned to hang in the air rather than fly: an ember shed by a moving object
/// already carries a share of that object's velocity, so its own launch speed
/// only needs to scatter it off the path. Heavy drag then parks it, which is
/// what turns a stream of embers into a tail that stays where it was laid.
fn ember_trail() -> ParticleSpec {
    ParticleSpec {
        lifetime: Spread::new(0.25, 0.8),
        size: Spread::new(0.025, 0.075),
        speed: Spread::new(0.3, 1.6),
        launch: LaunchPattern::Sphere,
        spawn_radius: 0.0,
        ramp: ColourRamp::fade(rgba(1.6, 1.0, 0.35, 1.0), rgba(0.75, 0.08, 0.0, 0.0)),
        growth: 1.0,
        spin: Spread::fixed(0.0),
        additive: 1.0,
        billow: 0.0,
        // Hot embers rise as they cool.
        gravity_scale: -0.25,
        drag: 1.4,
        turbulence: 0.0,
        stretch: 0.05,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_effect_type_has_a_spec() {
        let config = ParticleConfig::default();
        for effect_type in [
            ParticleEffectType::BlastCore,
            ParticleEffectType::Fireball,
            ParticleEffectType::BlastDust,
            ParticleEffectType::Embers,
            ParticleEffectType::Debris,
            ParticleEffectType::Smoke,
            ParticleEffectType::WaterSplash,
            ParticleEffectType::EmberTrail,
        ] {
            let spec = config.spec(effect_type);
            assert!(spec.lifetime.min > 0.0, "{effect_type:?} lives no time");
            assert!(spec.size.min > 0.0, "{effect_type:?} has no size");
        }
    }

    #[test]
    fn the_fireball_cools_from_white_hot_to_dark() {
        let ramp = fireball().ramp;

        let born = ramp.sample(0.0);
        let dying = ramp.sample(1.0);

        // Born hot enough to blow out the tonemap.
        assert!(born.x > 2.0 && born.y > 1.0);
        // Ends sooty and gone, not merely dimmer fire.
        assert!(dying.x < 0.15 && dying.w == 0.0);
        // And reddens on the way, rather than desaturating straight to grey.
        let midway = ramp.sample(0.35);
        assert!(midway.x > midway.y * 2.0, "not red enough at the midpoint");
    }

    #[test]
    fn effects_that_glow_are_additive_and_effects_that_do_not_are_blended() {
        assert_eq!(blast_core().additive, 1.0);
        assert_eq!(embers().additive, 1.0);
        assert!(fireball().additive > 0.5);

        assert_eq!(smoke().additive, 0.0);
        assert_eq!(blast_dust().additive, 0.0);
        assert_eq!(debris().additive, 0.0);
    }

    /// How far a particle of `spec` launched flat out at `speed` gets in
    /// `seconds`, and how much of its launch speed it has left.
    fn coast(spec: &ParticleSpec, speed: f32, seconds: f32) -> (f32, f32) {
        let mut particle = spec.sample(nalgebra::Vector3::zeros(), 1.0, &mut rand::thread_rng());
        particle.velocity = nalgebra::Vector3::new(speed, 0.0, 0.0);
        particle.gravity_scale = 0.0;
        particle.turbulence = 0.0;

        let dt = 1.0 / 120.0;
        for _ in 0..(seconds / dt) as u32 {
            particle.update(dt, 0.0);
        }
        (
            particle.position.magnitude(),
            particle.velocity.magnitude() / speed,
        )
    }

    /// Drag is the single number that decides how big an explosion looks, and
    /// it is not readable as a size — it has to be checked as one. These bounds
    /// are what "a blast about five metres across" means in the tuning.
    #[test]
    fn the_fireball_reaches_a_few_metres_and_then_stops() {
        let spec = fireball();
        let (reach, remaining) = coast(&spec, spec.speed.max, 0.4);

        assert!(
            (1.5..4.0).contains(&reach),
            "fastest puff covers {reach}m in 0.4s"
        );
        assert!(
            remaining < 0.35,
            "still at {remaining} of launch speed; fire is sailing away"
        );
    }

    #[test]
    fn the_dust_skirt_runs_out_past_the_fireball() {
        let dust = blast_dust();
        let fire = fireball();

        let (dust_reach, remaining) = coast(&dust, dust.speed.max, 0.6);
        let (fire_reach, _) = coast(&fire, fire.speed.max, 0.6);

        assert!(
            dust_reach > fire_reach,
            "dust ({dust_reach}m) must outrun the fireball ({fire_reach}m) — it is what
             shows the blast radius"
        );
        assert!(
            (4.0..9.0).contains(&dust_reach),
            "dust reaches {dust_reach}m"
        );
        assert!(remaining < 0.4, "dust still at {remaining} of launch speed");
    }

    #[test]
    fn smoke_fades_in_rather_than_appearing_at_full_opacity() {
        let ramp = smoke().ramp;
        assert_eq!(ramp.sample(0.0).w, 0.0);
        assert!(ramp.sample(0.10).w > 0.5);
    }
}
