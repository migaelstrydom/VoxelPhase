//! How a grenade looks while it flies.
//!
//! The model itself is static geometry (see `super::model`); everything that
//! makes a grenade feel alive is driven here, from one number — how hard it is
//! moving:
//!
//! ```text
//!                        ┌──▶ MaterialModulation ──▶ emissive scale (blooms)
//!   speed ──▶ heat ──┬───┼──▶ PointLight ─────────▶ light it casts on the world
//!                    │   └──▶ ParticleEmitter ────▶ ember trail rate
//!                    │
//!   time ──▶ pulse ──┘   (a beat laid over the first two, so it breathes)
//! ```
//!
//! Speed rather than a fuse timer, because a grenade explodes on contact: the
//! fuse rarely runs out, so a countdown would never be seen. Speed is always
//! visible — a thrown grenade tears through the air white-hot and a spent one
//! sits in the grass smouldering.
//!
//! This system also *attaches* those three components the first time it sees a
//! grenade, so the entire look lives in one place and throwing a grenade
//! (`super::systems`) stays about physics.

use nalgebra::Vector3;
use specs::{Entities, Join, ReadExpect, ReadStorage, System, WriteStorage};

use super::components::Grenade;
use crate::components::{MaterialModulation, Velocity};
use crate::lighting::PointLight;
use crate::particles::ParticleEmitter;
use crate::rendering::colour::Colour;
use crate::rendering::material::SurfaceModulation;
use crate::time::Time;

/// Tuning for the grenade's flight look.
///
/// Emissive scales are multipliers on the authored material emission, so 1.0 is
/// "exactly as the material was written" — the resting, smouldering state — and
/// the flight value is how much hotter a grenade at full speed looks.
#[derive(Debug, Clone)]
pub struct GrenadeVisuals {
    /// Speed at which the grenade reaches full heat, in units/second.
    pub full_heat_speed: f32,

    /// Emissive multiplier for a grenade at rest.
    pub rest_emissive_scale: f32,

    /// Emissive multiplier for a grenade at full heat.
    pub flight_emissive_scale: f32,

    /// Colour the light casts at rest: deep ember.
    pub rest_light_colour: Colour,

    /// Colour the light casts at full heat: forge-white.
    pub flight_light_colour: Colour,

    /// Cast luminance at rest.
    pub rest_light_intensity: f32,

    /// Cast luminance at full heat.
    pub flight_light_intensity: f32,

    /// Reach of the light at rest, in world units.
    pub rest_light_range: f32,

    /// Reach of the light at full heat.
    pub flight_light_range: f32,

    /// Embers shed per second at full heat. Scales down with heat, so a grenade
    /// rolling to a stop stops trailing.
    pub trail_rate: f32,

    /// Height above the grenade centre where the light sits. Zero: the glow
    /// comes from inside the shell, not from above it.
    pub light_offset: Vector3<f32>,

    /// Fraction of the mid-point brightness the pulse swings either way.
    pub pulse_depth: f32,
}

impl Default for GrenadeVisuals {
    fn default() -> Self {
        Self {
            full_heat_speed: 16.0,
            rest_emissive_scale: 1.0,
            flight_emissive_scale: 2.8,
            rest_light_colour: Colour::rgb(1.0, 0.35, 0.10),
            flight_light_colour: Colour::rgb(1.0, 0.78, 0.45),
            rest_light_intensity: 0.9,
            flight_light_intensity: 4.5,
            rest_light_range: 4.0,
            flight_light_range: 8.0,
            trail_rate: 110.0,
            light_offset: Vector3::new(0.0, 0.0, 0.0),
            pulse_depth: 0.10,
        }
    }
}

/// Two detuned sines: their beat gives an irregular throb without per-entity
/// state or a random source. Matches the approach used for firelight.
const PULSE_HZ_A: f32 = 5.7;
const PULSE_HZ_B: f32 = 9.1;

/// Phase offset per entity id, in seconds. Grenades thrown in a burst share a
/// clock, so without this they would all pulse in lockstep.
const PHASE_STAGGER: f32 = 0.618;

impl GrenadeVisuals {
    /// How hot the grenade reads, from 0 (at rest) to 1 (at or above
    /// `full_heat_speed`).
    ///
    /// The square root front-loads the ramp: most of the visible change happens
    /// over the first few units/second, so a grenade nudged by an explosion
    /// flares up noticeably instead of needing to be thrown to look alive.
    pub fn heat(&self, speed: f32) -> f32 {
        if self.full_heat_speed <= 0.0 {
            return 1.0;
        }
        (speed / self.full_heat_speed).clamp(0.0, 1.0).sqrt()
    }

    /// Brightness multiplier of the throb at a given phase, centred on 1.
    pub fn pulse(&self, phase: f32) -> f32 {
        let wobble = (phase * PULSE_HZ_A).sin() * 0.6 + (phase * PULSE_HZ_B).sin() * 0.4;
        1.0 + wobble * self.pulse_depth
    }

    /// Emissive multiplier for the model's materials.
    pub fn emissive_scale(&self, heat: f32, pulse: f32) -> f32 {
        lerp(self.rest_emissive_scale, self.flight_emissive_scale, heat) * pulse
    }

    /// Embers to shed per second.
    fn trail_rate(&self, heat: f32) -> f32 {
        self.trail_rate * heat
    }

    /// The light a grenade at this heat casts.
    fn light(&self, heat: f32, pulse: f32) -> PointLight {
        PointLight {
            colour: self.rest_light_colour.lerp(self.flight_light_colour, heat),
            intensity: lerp(self.rest_light_intensity, self.flight_light_intensity, heat) * pulse,
            range: lerp(self.rest_light_range, self.flight_light_range, heat),
            offset: self.light_offset,
        }
    }
}

fn lerp(from: f32, to: f32, t: f32) -> f32 {
    from + (to - from) * t
}

/// Drives every grenade's glow, light and ember trail from its speed.
#[derive(Default)]
pub struct GrenadeVisualSystem {
    visuals: GrenadeVisuals,
}

impl GrenadeVisualSystem {
    /// Use a non-default look.
    #[allow(dead_code)]
    pub fn new(visuals: GrenadeVisuals) -> Self {
        Self { visuals }
    }
}

impl<'a> System<'a> for GrenadeVisualSystem {
    type SystemData = (
        Entities<'a>,
        ReadExpect<'a, Time>,
        ReadStorage<'a, Grenade>,
        ReadStorage<'a, Velocity>,
        WriteStorage<'a, PointLight>,
        WriteStorage<'a, MaterialModulation>,
        WriteStorage<'a, ParticleEmitter>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, time, grenades, velocities, mut lights, mut modulations, mut emitters) =
            data;

        for (entity, _grenade) in (&entities, &grenades).join() {
            let speed = velocities
                .get(entity)
                .map(|velocity| velocity.0.magnitude())
                .unwrap_or(0.0);

            let heat = self.visuals.heat(speed);
            let phase = time.total_seconds() + entity.id() as f32 * PHASE_STAGGER;
            let pulse = self.visuals.pulse(phase);

            let light = self.visuals.light(heat, pulse);
            if let Ok(entry) = lights.entry(entity) {
                *entry.or_insert(light) = light;
            }

            let modulation = MaterialModulation(SurfaceModulation::emissive(
                self.visuals.emissive_scale(heat, pulse),
            ));
            if let Ok(entry) = modulations.entry(entity) {
                *entry.or_insert(modulation) = modulation;
            }

            if let Ok(entry) = emitters.entry(entity) {
                entry
                    .or_insert_with(ParticleEmitter::ember_trail)
                    .spawn_rate = self.visuals.trail_rate(heat);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use specs::{Builder, RunNow, World, WorldExt};

    use super::*;

    #[test]
    fn heat_runs_from_rest_to_full_and_stops_there() {
        let visuals = GrenadeVisuals::default();

        assert_eq!(visuals.heat(0.0), 0.0);
        assert_eq!(visuals.heat(visuals.full_heat_speed), 1.0);
        // Beyond full speed the look must not keep escalating, or a grenade
        // caught in an explosion would white out the screen.
        assert_eq!(visuals.heat(visuals.full_heat_speed * 10.0), 1.0);

        let slow = visuals.heat(1.0);
        let medium = visuals.heat(8.0);
        assert!(slow > 0.0 && slow < medium && medium < 1.0);
    }

    #[test]
    fn a_grenade_at_rest_still_smoulders_but_sheds_nothing() {
        let visuals = GrenadeVisuals::default();
        let heat = visuals.heat(0.0);

        // The materials are authored for the resting look, so this must be the
        // identity scale — otherwise "at rest" would not match what was drawn.
        assert_eq!(visuals.emissive_scale(heat, 1.0), 1.0);
        assert_eq!(visuals.trail_rate(heat), 0.0);
        assert!(visuals.light(heat, 1.0).intensity > 0.0);
    }

    #[test]
    fn a_thrown_grenade_outglows_a_resting_one() {
        let visuals = GrenadeVisuals::default();
        let resting = visuals.heat(0.0);
        let thrown = visuals.heat(20.0);

        assert!(visuals.emissive_scale(thrown, 1.0) > visuals.emissive_scale(resting, 1.0));
        assert!(visuals.light(thrown, 1.0).intensity > visuals.light(resting, 1.0).intensity);
        assert!(visuals.light(thrown, 1.0).range > visuals.light(resting, 1.0).range);
        assert!(visuals.trail_rate(thrown) > 0.0);
    }

    #[test]
    fn the_pulse_stays_inside_its_band_and_actually_moves() {
        let visuals = GrenadeVisuals::default();

        let mut min = f32::MAX;
        let mut max = f32::MIN;
        for step in 0..20_000 {
            let value = visuals.pulse(step as f32 * 0.001);
            min = min.min(value);
            max = max.max(value);
        }

        assert!(min >= 1.0 - visuals.pulse_depth - 1e-3, "dipped to {min}");
        assert!(max <= 1.0 + visuals.pulse_depth + 1e-3, "peaked at {max}");
        assert!(max - min > visuals.pulse_depth, "pulse barely moves");
    }

    #[test]
    fn the_system_dresses_a_bare_grenade_and_then_keeps_it_updated() {
        let mut world = World::new();
        world.register::<Grenade>();
        world.register::<Velocity>();
        world.register::<PointLight>();
        world.register::<MaterialModulation>();
        world.register::<ParticleEmitter>();
        world.insert(Time::default());

        let entity = world
            .create_entity()
            .with(Grenade::new())
            .with(Velocity(Vector3::new(0.0, 0.0, 20.0)))
            .build();

        let mut system = GrenadeVisualSystem::default();
        system.run_now(&world);
        world.maintain();

        let visuals = GrenadeVisuals::default();
        let hot = visuals.heat(20.0);

        assert!(world.read_storage::<PointLight>().get(entity).is_some());
        assert!(
            world
                .read_storage::<ParticleEmitter>()
                .get(entity)
                .expect("no trail emitter")
                .spawn_rate
                > 0.0
        );
        let scale = world
            .read_storage::<MaterialModulation>()
            .get(entity)
            .expect("no modulation")
            .0
            .emissive_scale;
        assert!(scale > visuals.rest_emissive_scale);

        // Coming to a stop must cool it back down rather than latch it hot.
        world
            .write_storage::<Velocity>()
            .insert(entity, Velocity(Vector3::zeros()))
            .expect("failed to stop the grenade");
        system.run_now(&world);

        let cooled = world
            .read_storage::<MaterialModulation>()
            .get(entity)
            .expect("no modulation")
            .0
            .emissive_scale;
        assert!(cooled < visuals.emissive_scale(hot, 1.0 - visuals.pulse_depth));
        assert_eq!(
            world
                .read_storage::<ParticleEmitter>()
                .get(entity)
                .expect("no trail emitter")
                .spawn_rate,
            0.0
        );
    }
}
