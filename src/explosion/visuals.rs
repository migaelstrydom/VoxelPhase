//! What an explosion looks like.
//!
//! An explosion is not one effect but a sequence, and the sequence is the whole
//! trick. Fired all at once, the same particles read as a puff; staged, they
//! read as an event with a beginning and an end:
//!
//! ```text
//!   t=0.00  ██ blast core     blinding, gone in a fifth of a second
//!   t=0.00  ████ fireball     expands, cools, becomes its own smoke
//!   t=0.00  ████ dust skirt   low ring running outward along the ground
//!   t=0.00  ██████ embers     still arcing when the fire is out
//!   t=0.00  ██████ debris     thrown up and falling back
//!   t=0.22       ████████ smoke column   what is left behind
//!
//!   t=0.00  ████ blast light  the only part that lights the world
//! ```
//!
//! The light matters more than any single particle: particles are drawn in the
//! transparent pass and so cannot bloom or illuminate anything (see the bloom
//! notes on `PostProcessConfig`). A blast that does not briefly light the walls
//! around it will never look bright however hot its colours are.

use nalgebra::{Point3, Vector3};
use specs::{
    Builder, Component, Entities, Join, LazyUpdate, ReadExpect, System, VecStorage, WriteStorage,
};

use crate::components::Position;
use crate::lighting::PointLight;
use crate::particles::{ParticleEffectType, ParticleEmitter};
use crate::rendering::colour::Colour;
use crate::time::Time;

/// One stage of the explosion: an effect, how much of it, and when.
#[derive(Debug, Clone, Copy)]
pub struct Stage {
    /// Which effect this stage spawns.
    pub effect: ParticleEffectType,
    /// Particles thrown out the moment the stage starts.
    pub burst: u32,
    /// Particles per second for as long as the stage runs. Zero for stages
    /// that are purely a burst.
    pub rate: f32,
    /// Seconds after detonation this stage starts.
    pub delay: f32,
    /// Seconds the stage keeps emitting after it starts. A burst-only stage
    /// still needs a non-zero span to survive its first frame.
    pub duration: f32,
}

impl Stage {
    /// A stage that fires everything at once and is done.
    const fn burst(effect: ParticleEffectType, count: u32) -> Self {
        Self {
            effect,
            burst: count,
            rate: 0.0,
            delay: 0.0,
            duration: 0.05,
        }
    }

    /// A stage that bursts and then keeps emitting.
    const fn sustained(effect: ParticleEffectType, count: u32, rate: f32, duration: f32) -> Self {
        Self {
            effect,
            burst: count,
            rate,
            delay: 0.0,
            duration,
        }
    }

    /// Hold this stage back until `seconds` after detonation.
    const fn after(mut self, seconds: f32) -> Self {
        self.delay = seconds;
        self
    }

    /// The emitter that realises this stage at the given scale.
    fn emitter(&self, scale: f32) -> ParticleEmitter {
        ParticleEmitter::new(self.effect)
            .with_burst(scale_count(self.burst, scale))
            .with_spawn_rate(self.rate * scale)
            .with_lifetime(self.duration)
            .with_delay(self.delay)
            .with_scale(scale)
    }
}

/// Particle counts grow with the linear scale of the blast, not its volume.
///
/// Particles are scaled up in size as well, so their coverage already grows
/// quadratically; making the count follow suit would quadruple the fill cost of
/// a blast twice the size for very little that the eye can pick out.
fn scale_count(count: u32, scale: f32) -> u32 {
    (count as f32 * scale).round().max(1.0) as u32
}

/// Tuning for the whole explosion look.
///
/// Authored for one blast radius; everything scales off the ratio between that
/// and the actual explosion, so a satchel charge and a grenade are the same
/// event at different sizes rather than two separately tuned effects.
#[derive(Debug, Clone)]
pub struct ExplosionVisuals {
    /// Blast radius these numbers were authored against, in world units.
    pub reference_radius: f32,
    /// The particle stages, in the order they are spawned.
    pub stages: Vec<Stage>,
    /// The light the blast casts.
    pub light: BlastLightSpec,
}

impl Default for ExplosionVisuals {
    fn default() -> Self {
        use ParticleEffectType::*;

        Self {
            reference_radius: 5.0,
            stages: vec![
                Stage::burst(BlastCore, 7),
                // A short tail on the fireball rather than a single burst: the
                // ball keeps feeding outward for a moment, which is what gives
                // it a rolling edge instead of a fixed shell of puffs.
                Stage::sustained(Fireball, 28, 90.0, 0.25),
                Stage::burst(BlastDust, 28),
                Stage::burst(Embers, 36),
                Stage::burst(Debris, 16),
                // Late enough that the fire is already cooling when the column
                // starts, so the smoke looks like the fire's residue rather
                // than something that was there all along.
                Stage::sustained(Smoke, 6, 18.0, 1.1).after(0.22),
            ],
            light: BlastLightSpec::default(),
        }
    }
}

impl ExplosionVisuals {
    /// How much larger than the authored explosion this one is.
    fn scale(&self, blast_radius: f32) -> f32 {
        if self.reference_radius <= 0.0 {
            1.0
        } else {
            (blast_radius / self.reference_radius).max(0.05)
        }
    }

    /// Create every entity that makes up one explosion at `centre`.
    ///
    /// Each stage is its own emitter entity, deleting itself when it has run;
    /// nothing has to be cleaned up by the caller.
    pub fn spawn(
        &self,
        entities: &Entities,
        lazy: &LazyUpdate,
        centre: Point3<f32>,
        blast_radius: f32,
    ) {
        let scale = self.scale(blast_radius);
        let position = Position(centre.coords);

        for stage in &self.stages {
            lazy.create_entity(entities)
                .with(position)
                .with(stage.emitter(scale))
                .build();
        }

        lazy.create_entity(entities)
            .with(position)
            .with(self.light.at_scale(scale))
            .build();
    }
}

/// Tuning for the flash of light a blast casts on its surroundings.
#[derive(Debug, Clone, Copy)]
pub struct BlastLightSpec {
    /// Peak luminance, at the authored blast radius.
    pub peak_intensity: f32,
    /// Reach of the light, at the authored blast radius.
    pub range: f32,
    /// Seconds from detonation to darkness.
    pub duration: f32,
    /// Colour of the initial flash: near-white, barely warm.
    pub flash_colour: Colour,
    /// Colour the light has cooled to by the end: deep ember.
    pub ember_colour: Colour,
    /// Height above the blast centre the light sits at. Lifted clear of the
    /// ground so a blast at a surface lights the surface instead of burying
    /// half its falloff in it.
    pub height: f32,
}

impl Default for BlastLightSpec {
    fn default() -> Self {
        Self {
            peak_intensity: 46.0,
            range: 22.0,
            duration: 0.9,
            flash_colour: Colour::rgb(1.0, 0.93, 0.80),
            ember_colour: Colour::rgb(1.0, 0.34, 0.09),
            height: 0.6,
        }
    }
}

impl BlastLightSpec {
    /// A light of this spec, sized for an explosion `scale` times the authored
    /// one.
    ///
    /// Intensity grows with the square of scale — a blast twice as wide has
    /// roughly four times the burning surface — while the duration does not,
    /// for the same reason particle lifetimes do not scale.
    pub fn at_scale(&self, scale: f32) -> BlastLight {
        BlastLight {
            spec: *self,
            peak_intensity: self.peak_intensity * scale * scale,
            range: self.range * scale,
            elapsed: 0.0,
        }
    }
}

/// The transient light of a single explosion.
///
/// Owns its own decay rather than leaning on the particle system, because the
/// light is what the player actually perceives as brightness and it has to
/// outlive the fireball slightly — the moment after a blast is lit by embers,
/// not by nothing.
#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct BlastLight {
    spec: BlastLightSpec,
    /// Peak luminance for this particular blast.
    peak_intensity: f32,
    /// Reach for this particular blast.
    range: f32,
    /// Seconds since detonation.
    pub elapsed: f32,
}

/// Share of the peak carried by the fast-decaying flash, the rest being the
/// slower ember glow underneath it. The two-term curve is what gives the light
/// a hard snap followed by a lingering warmth; a single exponential reads as
/// either too soft or too abrupt, never both.
const FLASH_SHARE: f32 = 0.75;

/// How much of the light's duration the colour takes to reach full ember. The
/// hue settles well before the brightness does, so the tail is unambiguously an
/// afterglow rather than a dimmed flash.
const COLOUR_SETTLE: f32 = 0.35;

impl BlastLight {
    /// Fraction of the light's life elapsed, 0 to 1.
    fn progress(&self) -> f32 {
        if self.spec.duration <= 0.0 {
            1.0
        } else {
            (self.elapsed / self.spec.duration).clamp(0.0, 1.0)
        }
    }

    /// Whether the light has burned out and its entity can go.
    pub fn is_spent(&self) -> bool {
        self.elapsed >= self.spec.duration
    }

    /// The light as it stands right now.
    pub fn current(&self) -> PointLight {
        let remaining = 1.0 - self.progress();
        let intensity = self.peak_intensity
            * (FLASH_SHARE * remaining.powi(6) + (1.0 - FLASH_SHARE) * remaining.powi(2));

        let cooling = (self.progress() / COLOUR_SETTLE).clamp(0.0, 1.0);

        PointLight {
            colour: self.spec.flash_colour.lerp(self.spec.ember_colour, cooling),
            intensity,
            // The lit area shrinks as the fire dies back to the crater.
            range: self.range * (0.35 + 0.65 * remaining),
            offset: Vector3::new(0.0, self.spec.height, 0.0),
        }
    }
}

/// Advances every blast light and removes it once it has burned out.
pub struct BlastLightSystem;

impl<'a> System<'a> for BlastLightSystem {
    type SystemData = (
        Entities<'a>,
        ReadExpect<'a, Time>,
        WriteStorage<'a, BlastLight>,
        WriteStorage<'a, PointLight>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, time, mut blasts, mut lights) = data;
        let dt = time.delta_seconds();

        let mut spent = Vec::new();

        for (entity, blast) in (&entities, &mut blasts).join() {
            blast.elapsed += dt;

            if blast.is_spent() {
                spent.push(entity);
                continue;
            }

            let light = blast.current();
            if let Ok(entry) = lights.entry(entity) {
                *entry.or_insert(light) = light;
            }
        }

        for entity in spent {
            let _ = entities.delete(entity);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn blast(scale: f32) -> BlastLight {
        BlastLightSpec::default().at_scale(scale)
    }

    #[test]
    fn the_light_is_at_its_brightest_the_instant_it_is_created() {
        let light = blast(1.0);
        assert_eq!(light.current().intensity, light.peak_intensity);
    }

    #[test]
    fn the_flash_collapses_fast_and_leaves_a_glow_behind() {
        let mut light = blast(1.0);
        let peak = light.current().intensity;

        // A tenth of the way in, the flash proper is already mostly gone.
        light.elapsed = light.spec.duration * 0.25;
        let after_flash = light.current().intensity;
        assert!(after_flash < peak * 0.3, "flash lingers: {after_flash}");

        // But something is still burning most of the way through.
        light.elapsed = light.spec.duration * 0.6;
        assert!(light.current().intensity > 0.0);

        // And it reaches zero exactly at the end rather than snapping out.
        light.elapsed = light.spec.duration;
        assert_eq!(light.current().intensity, 0.0);
    }

    #[test]
    fn the_light_cools_from_flash_white_to_ember() {
        let mut light = blast(1.0);
        let spec = light.spec;

        assert_eq!(light.current().colour, spec.flash_colour);

        light.elapsed = spec.duration * COLOUR_SETTLE;
        let settled = light.current().colour;
        assert!(
            (settled.r - spec.ember_colour.r).abs() < 1e-5
                && (settled.g - spec.ember_colour.g).abs() < 1e-5
                && (settled.b - spec.ember_colour.b).abs() < 1e-5,
            "cooled to {settled:?}"
        );
    }

    #[test]
    fn a_bigger_blast_lights_more_of_the_world_for_the_same_time() {
        let small = blast(1.0);
        let large = blast(2.0);

        assert!(large.current().intensity > small.current().intensity);
        assert!(large.current().range > small.current().range);
        assert_eq!(large.spec.duration, small.spec.duration);
    }

    #[test]
    fn stages_scale_their_counts_and_their_particles_but_not_their_timing() {
        let visuals = ExplosionVisuals::default();
        let stage = visuals.stages[0];

        let authored = stage.emitter(1.0);
        let doubled = stage.emitter(2.0);

        assert_eq!(doubled.initial_burst, authored.initial_burst * 2);
        assert_eq!(doubled.scale, 2.0);
        assert_eq!(doubled.lifetime, authored.lifetime);
        assert_eq!(doubled.delay, authored.delay);
    }

    #[test]
    fn the_choreography_starts_with_fire_and_ends_with_smoke() {
        let visuals = ExplosionVisuals::default();

        let core = visuals
            .stages
            .iter()
            .find(|stage| stage.effect == ParticleEffectType::BlastCore)
            .expect("no blast core");
        let smoke = visuals
            .stages
            .iter()
            .find(|stage| stage.effect == ParticleEffectType::Smoke)
            .expect("no smoke column");

        assert_eq!(core.delay, 0.0, "the core must be instantaneous");
        assert!(
            smoke.delay > core.delay + core.duration,
            "smoke must start after the core is gone"
        );
    }

    #[test]
    fn scale_is_relative_to_the_authored_blast_radius() {
        let visuals = ExplosionVisuals::default();

        assert_eq!(visuals.scale(visuals.reference_radius), 1.0);
        assert_eq!(visuals.scale(visuals.reference_radius * 3.0), 3.0);
        // A degenerate explosion must not produce a zero or negative scale,
        // which would spawn particles of no size that never expire.
        assert!(visuals.scale(0.0) > 0.0);
    }
}
