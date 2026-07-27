//! Fire as a light source.
//!
//! Burning entities cast light. The `PointLight` is attached when `OnFire`
//! appears and removed when it goes out, so nothing else has to know that fire
//! and lighting are related.

use nalgebra::Vector3;
use specs::{Component, DenseVecStorage, Entities, Join, Read, ReadStorage, System, WriteStorage};

use crate::components::RigidBodyComponent;
use crate::fire::components::OnFire;
use crate::lighting::PointLight;
use crate::rendering::colour::Colour;
use crate::systems::PhysicsResource;

/// Colour of firelight: warm, well short of the orange of the flame itself, so
/// lit surfaces read as firelit rather than tinted orange.
const FIRE_COLOUR: Colour = Colour::rgb(1.0, 0.65, 0.3);

/// Light intensity at the middle of the flicker, as luminance.
///
/// Re-derived for the luminance convention: the old value (2.0) was a
/// multiplier on `FIRE_COLOUR` (luminance 0.699), so `2.0 * 0.699 = 1.398`
/// keeps the same cast light.
const BASE_INTENSITY: f32 = 1.398;

/// Fraction of `BASE_INTENSITY` the flicker swings either side of the middle.
const FLICKER_DEPTH: f32 = 0.25;

/// Reach of firelight for a body of unit bounding radius. Scaled by the actual
/// radius, so a burning crate lights more of the room than a burning pebble.
const RANGE_PER_RADIUS: f32 = 12.0;

/// Range used when the entity has no collider to measure.
const DEFAULT_RANGE: f32 = 6.0;

/// Height above the entity origin where the flame is brightest. The fire volume
/// rises from the entity's base, so its light does not come from the centre.
const FLAME_CENTRE_HEIGHT: f32 = 0.6;

/// Two detuned sine waves; their beat gives an irregular flicker without
/// needing a random source or any per-entity state.
const FLICKER_HZ_A: f32 = 7.3;
const FLICKER_HZ_B: f32 = 11.9;

/// Phase offset per entity id, in seconds. Fires lit by the same explosion
/// share a burn time, so without this they would flicker in lockstep. An
/// irrational-ish stride keeps neighbouring ids far apart in phase.
const PHASE_STAGGER: f32 = 0.618;

/// Marks a `PointLight` as owned by `FireLightSystem`.
///
/// Without this, the system cannot tell its own lights apart from lights
/// authored elsewhere (e.g. a glowing orb's permanent glow) — both are just a
/// `PointLight` on an entity. The marker lets attach/update/removal all key
/// on "does fire own this light" rather than "does this entity have a light",
/// so extinguishing a fire never touches a light it didn't create.
#[derive(Component, Debug, Default)]
#[storage(DenseVecStorage)]
pub struct FireLight;

/// Attaches a `PointLight` to burning entities, animates its flicker, and
/// removes it when the fire goes out.
///
/// Each burning entity occupies one of the collector's active-light slots while
/// it burns.
pub struct FireLightSystem;

impl<'a> System<'a> for FireLightSystem {
    type SystemData = (
        Entities<'a>,
        ReadStorage<'a, OnFire>,
        ReadStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, PointLight>,
        WriteStorage<'a, FireLight>,
        Option<Read<'a, PhysicsResource>>,
    );

    fn run(
        &mut self,
        (entities, on_fires, bodies, mut lights, mut fire_lights, physics): Self::SystemData,
    ) {
        for (entity, on_fire) in (&entities, &on_fires).join() {
            let range = bodies
                .get(entity)
                .and_then(|rb| body_light_range(physics.as_deref(), rb))
                .unwrap_or(DEFAULT_RANGE);

            let light =
                lights.entry(entity).ok().map(|entry| {
                    entry.or_insert_with(|| {
                        PointLight::new(FIRE_COLOUR, BASE_INTENSITY, range)
                            .with_offset(Vector3::new(0.0, FLAME_CENTRE_HEIGHT, 0.0))
                    })
                });

            if light.is_some() {
                fire_lights.insert(entity, FireLight).ok();
            }

            if let Some(light) = light {
                let phase = on_fire.burn_time + entity.id() as f32 * PHASE_STAGGER;
                light.range = range;
                light.intensity = flicker_intensity(phase, on_fire.burn_time);
            }
        }

        // A light whose fire has gone out is no longer a light — but only if
        // this system is the one that put it there.
        let extinguished: Vec<_> = (&entities, &fire_lights, !&on_fires)
            .join()
            .map(|(entity, _, _)| entity)
            .collect();

        for entity in extinguished {
            lights.remove(entity);
            fire_lights.remove(entity);
        }
    }
}

/// Firelight reach for a body, from the bounding radius of its first collider.
fn body_light_range(physics: Option<&PhysicsResource>, rb: &RigidBodyComponent) -> Option<f32> {
    let physics = physics?;
    let body = physics.world.body(rb.0)?;
    let handle = *body.colliders().first()?;
    let collider = physics.world.collider(handle)?;
    Some(collider.shape().bounding_radius() * RANGE_PER_RADIUS)
}

/// Flicker as the beat of two detuned sines, faded in over the first second so
/// a fire does not pop into existence at full brightness.
///
/// `phase` is the fire's own burn time plus a per-entity stagger, so fires do
/// not pulse in unison.
fn flicker_intensity(phase: f32, burn_time: f32) -> f32 {
    let wobble = (phase * FLICKER_HZ_A).sin() * 0.6 + (phase * FLICKER_HZ_B).sin() * 0.4;
    let ignition_fade = burn_time.clamp(0.0, 1.0);
    BASE_INTENSITY * (1.0 + wobble * FLICKER_DEPTH) * ignition_fade
}

#[cfg(test)]
mod tests {
    use specs::{Builder, RunNow, World, WorldExt};

    use super::*;

    #[test]
    fn a_point_light_without_fire_light_is_left_alone() {
        // Regression: the removal join used to key on `!OnFire` alone, which
        // matched *any* entity with a `PointLight` and no `OnFire` — deleting
        // lights this system never created (e.g. a glowing orb's permanent
        // glow). The `FireLight` marker must be required for removal.
        let mut world = World::new();
        world.register::<OnFire>();
        world.register::<PointLight>();
        world.register::<FireLight>();
        world.register::<RigidBodyComponent>();

        let entity = world
            .create_entity()
            .with(PointLight::new(Colour::WHITE, 1.0, 10.0))
            .build();

        let mut system = FireLightSystem;
        system.run_now(&world);
        world.maintain();

        assert!(
            world.read_storage::<PointLight>().get(entity).is_some(),
            "FireLightSystem removed a PointLight it did not own"
        );
    }

    #[test]
    fn flicker_stays_within_the_intended_band() {
        // Sampled densely enough to catch the beat peaks of both sines.
        let mut min = f32::MAX;
        let mut max = f32::MIN;
        for step in 0..20_000 {
            let value = flicker_intensity(step as f32 * 0.001, 5.0);
            min = min.min(value);
            max = max.max(value);
        }
        let bound = BASE_INTENSITY * FLICKER_DEPTH;
        assert!(min >= BASE_INTENSITY - bound - 1e-3, "dipped to {}", min);
        assert!(max <= BASE_INTENSITY + bound + 1e-3, "peaked at {}", max);
        // The flicker must actually move, or it is not a flicker.
        assert!(max - min > bound, "flicker range {} too small", max - min);
    }

    #[test]
    fn a_new_fire_fades_in_rather_than_popping() {
        assert_eq!(flicker_intensity(0.0, 0.0), 0.0);
        let quarter = flicker_intensity(0.0, 0.25);
        let full = flicker_intensity(0.0, 1.0);
        assert!(quarter > 0.0 && quarter < full);
        // Fully faded in by one second, and staying there.
        assert_eq!(flicker_intensity(0.0, 1.0), flicker_intensity(0.0, 30.0));
    }
}
