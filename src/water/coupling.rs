//! Wave-body coupling: injects wave disturbances from rigid body interactions.
//!
//! Three coupling mechanisms:
//! 1. **Impact**: when a body enters the water, inject negative wave velocity
//!    proportional to downward speed, creating a splash depression.
//! 2. **Bobbing**: floating bodies continuously perturb the surface at their
//!    footprint, creating standing wave patterns.
//! 3. **Wake**: velocity-driven bodies (player) moving horizontally through
//!    water inject displacement proportional to speed, creating a wake.
//!
//! No ECS or physics engine dependency — the caller provides body snapshots,
//! the water to test them against, and the ripple field to disturb.

use std::collections::HashMap;

use nalgebra::Point3;

use crate::water::buoyancy::WaterSurface;

/// How a disturbance moves the surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Disturbance {
    /// Kicks the surface's vertical velocity: a splash that then spreads.
    Velocity(f32),
    /// Moves the surface directly: tracks a body's motion however damped.
    Displacement(f32),
}

/// Whatever carries surface ripples: the thing the coupler disturbs.
pub trait RippleField {
    /// Disturb the surface over a disc of `radius` about `at`, full at the
    /// centre and fading to nothing at the rim. A zero radius disturbs the
    /// single point. `at` is on the surface, so its height picks the water
    /// where one column holds two.
    fn disturb(&mut self, at: Point3<f32>, radius: f32, disturbance: Disturbance);
}

/// A field with no ripples: disturbances go nowhere. Splash and wake events
/// are still emitted.
#[derive(Debug, Default, Clone, Copy)]
pub struct StillSurface;

impl RippleField for StillSurface {
    fn disturb(&mut self, _at: Point3<f32>, _radius: f32, _disturbance: Disturbance) {}
}

/// Disturbances collected to apply later, when whatever the coupler read the
/// water through no longer holds it.
#[derive(Debug, Default, Clone)]
pub struct Disturbances {
    pub pending: Vec<(Point3<f32>, f32, Disturbance)>,
}

impl RippleField for Disturbances {
    fn disturb(&mut self, at: Point3<f32>, radius: f32, disturbance: Disturbance) {
        self.pending.push((at, radius, disturbance));
    }
}

/// Emitted when a body impacts the water surface.
///
/// Consumed by the ECS layer to spawn splash particle effects.
#[derive(Debug, Clone)]
pub struct SplashEvent {
    /// World-space position on the water surface where the impact occurred.
    pub position: Point3<f32>,
    /// Downward speed of the body at impact (always positive).
    pub speed: f32,
    /// XZ footprint radius of the impacting body.
    pub radius: f32,
}

/// Emitted each frame for a body moving through water with a wake.
///
/// Consumed by the ECS layer to spawn continuous wake spray particles.
#[derive(Debug, Clone)]
pub struct WakeEvent {
    /// World-space position behind the body where spray originates.
    pub position: Point3<f32>,
    /// Horizontal speed of the body (always positive).
    pub speed: f32,
    /// Normalized horizontal movement direction (XZ plane).
    pub direction: nalgebra::Vector3<f32>,
    /// XZ footprint radius of the body.
    pub radius: f32,
}

/// Snapshot of a rigid body's state relevant to wave coupling.
#[derive(Clone)]
pub struct BodySnapshot {
    /// Unique identifier for tracking per-body state across frames.
    pub id: u64,
    /// World-space center of the collider.
    pub position: nalgebra::Point3<f32>,
    /// Linear velocity.
    pub velocity: nalgebra::Vector3<f32>,
    /// Radius of the body's XZ footprint on the water surface.
    pub footprint_radius: f32,
    /// Mass of the body (kg). Used to scale wave injection — light objects
    /// create smaller disturbances, preventing feedback-driven bouncing.
    pub mass: f32,
    /// Whether this body moves under its own power — a character, a lift —
    /// rather than being carried by the physics it is in.
    ///
    /// Gates the wake in [`WaveBodyCoupler::update`]: a self-propelled
    /// thing pushes water behind it, and a drifting log does not. That is a
    /// gameplay and VFX categorisation, not a physics special case, so its
    /// source is the entity carrying an `Actuator` — see
    /// `docs/TRACTION_DRIVE_DESIGN.md` §7.
    pub is_self_propelled: bool,
}

/// Per-body persistent state for tracking submersion transitions.
#[derive(Clone, Copy)]
struct BodyWaterState {
    /// Whether the body was submerged (any part below surface) last frame.
    was_submerged: bool,
    /// Frames since last seen (for cleanup).
    age: u32,
}

/// Configuration for wave-body coupling strengths.
///
/// Bobbing and wake use displacement injection (directly moves the surface)
/// which is robust against high wave damping. Impact uses velocity injection
/// for a natural splash animation.
pub struct WaveCouplingConfig {
    /// Multiplier for impact wave velocity injection.
    /// Scaled by body downward speed. Higher values create deeper splash
    /// depressions. Must overcome wave damping to produce visible splashes.
    pub impact_strength: f32,
    /// Multiplier for bobbing displacement injection (m per m/s of body vy).
    /// Applied every frame as displacement, so the surface visibly tracks
    /// the body's vertical oscillation.
    pub bobbing_strength: f32,
    /// Multiplier for player wake displacement injection (m per m/s of speed).
    /// Applied every frame behind the moving body.
    pub wake_strength: f32,
    /// Minimum downward speed (m/s) to trigger an impact splash.
    pub impact_speed_threshold: f32,
    /// Minimum horizontal speed (m/s) to generate a wake.
    pub wake_speed_threshold: f32,
    /// Reference mass (kg) for wave injection scaling. Bodies at or above this
    /// mass inject at full strength; lighter bodies inject proportionally less.
    /// Prevents light objects (beach balls, etc.) from creating oversized waves
    /// that feed back into buoyancy and cause perpetual bouncing.
    pub reference_mass: f32,
}

impl Default for WaveCouplingConfig {
    fn default() -> Self {
        Self {
            impact_strength: 8.0,
            bobbing_strength: 0.6,
            wake_strength: 0.04,
            impact_speed_threshold: 0.5,
            wake_speed_threshold: 0.3,
            reference_mass: 100.0,
        }
    }
}

/// Tracks per-body water state and injects wave disturbances.
pub struct WaveBodyCoupler {
    config: WaveCouplingConfig,
    body_states: HashMap<u64, BodyWaterState>,
    /// Splash events emitted this frame. Drained by the ECS layer.
    splash_events: Vec<SplashEvent>,
    /// Wake events emitted this frame. Drained by the ECS layer.
    wake_events: Vec<WakeEvent>,
}

impl WaveBodyCoupler {
    pub fn new(config: WaveCouplingConfig) -> Self {
        Self {
            config,
            body_states: HashMap::new(),
            splash_events: Vec::new(),
            wake_events: Vec::new(),
        }
    }

    /// Drain accumulated splash events from the last `update()` call.
    pub fn drain_splash_events(&mut self) -> Vec<SplashEvent> {
        std::mem::take(&mut self.splash_events)
    }

    /// Drain accumulated wake events from the last `update()` call.
    pub fn drain_wake_events(&mut self) -> Vec<WakeEvent> {
        std::mem::take(&mut self.wake_events)
    }

    /// Process all bodies and inject wave disturbances.
    ///
    /// Call this each frame before stepping the wave grid.
    pub fn update(
        &mut self,
        bodies: &[BodySnapshot],
        ripples: &mut dyn RippleField,
        water: &dyn WaterSurface,
    ) {
        self.splash_events.clear();
        self.wake_events.clear();

        // Mark all existing entries as aged; we'll reset age for bodies we see.
        for state in self.body_states.values_mut() {
            state.age += 1;
        }

        for body in bodies {
            let sample = water.sample(body.position);

            let is_submerged = sample
                .as_ref()
                .is_some_and(|s| body.position.y - body.footprint_radius < s.surface_level);

            let prev_state = self.body_states.get(&body.id).copied();
            let was_submerged = prev_state.map_or(false, |s| s.was_submerged);

            // Update tracking state.
            self.body_states.insert(
                body.id,
                BodyWaterState {
                    was_submerged: is_submerged,
                    age: 0,
                },
            );

            if !is_submerged {
                continue;
            }
            // Bobbing and wakes stir the surface only while the body breaks
            // it; one moving deep under water leaves the surface alone.
            let breaks_surface = sample
                .as_ref()
                .is_some_and(|s| body.position.y + body.footprint_radius >= s.surface_level);

            // Wave injection scales with mass: light objects create smaller
            // disturbances, breaking the feedback loop where self-generated
            // waves inflate buoyancy and cause perpetual bouncing.
            let mass_factor = (body.mass / self.config.reference_mass).min(1.0).max(0.0);

            // 1. Impact: body just entered water with downward velocity.
            if !was_submerged {
                let down_speed = -body.velocity.y;
                if down_speed > self.config.impact_speed_threshold {
                    let strength = -down_speed * self.config.impact_strength * mass_factor;
                    let surface = sample.as_ref().map_or(body.position.y, |s| s.surface_level);
                    ripples.disturb(
                        Point3::new(body.position.x, surface, body.position.z),
                        body.footprint_radius,
                        Disturbance::Velocity(strength),
                    );

                    let surface_y = sample.as_ref().map_or(body.position.y, |s| s.surface_level);
                    self.splash_events.push(SplashEvent {
                        position: Point3::new(body.position.x, surface_y, body.position.z),
                        speed: down_speed,
                        radius: body.footprint_radius,
                    });
                }
            }

            // 2. Bobbing: floating body's vertical motion perturbs surface.
            //    Uses displacement injection so the surface visibly tracks the
            //    body's oscillation regardless of wave damping. Measured
            //    against the surface: a float riding the swell stirs nothing.
            if was_submerged && breaks_surface {
                let vy = body.velocity.y - sample.as_ref().map_or(0.0, |s| s.surface_rise);
                if vy.abs() > 0.01 {
                    let strength = -vy * self.config.bobbing_strength * mass_factor;
                    let surface = sample.as_ref().map_or(body.position.y, |s| s.surface_level);
                    ripples.disturb(
                        Point3::new(body.position.x, surface, body.position.z),
                        body.footprint_radius,
                        Disturbance::Displacement(strength),
                    );
                }
            }

            // 3. Wake: self-propelled body moving horizontally through water.
            //    Uses displacement injection behind the body.
            if body.is_self_propelled && breaks_surface {
                let horizontal_vel = nalgebra::Vector3::new(body.velocity.x, 0.0, body.velocity.z);
                let h_speed = horizontal_vel.magnitude();
                if h_speed > self.config.wake_speed_threshold {
                    let move_dir = horizontal_vel / h_speed;

                    // Inject behind the body (opposite movement direction).
                    let wake_offset = -move_dir * body.footprint_radius * 0.8;
                    let wake_pos = body.position + wake_offset;
                    let strength = -h_speed * self.config.wake_strength;
                    let surface_y = sample.as_ref().map_or(body.position.y, |s| s.surface_level);
                    ripples.disturb(
                        Point3::new(wake_pos.x, surface_y, wake_pos.z),
                        0.0,
                        Disturbance::Displacement(strength),
                    );

                    self.wake_events.push(WakeEvent {
                        position: Point3::new(wake_pos.x, surface_y, wake_pos.z),
                        speed: h_speed,
                        direction: move_dir,
                        radius: body.footprint_radius,
                    });
                }
            }
        }

        // Prune bodies not seen for several frames.
        self.body_states.retain(|_, state| state.age < 60);
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use super::*;
    use crate::water::buoyancy::WaterSample;

    /// Water standing at 5 m over a flat floor, everywhere.
    struct Flat;

    impl WaterSurface for Flat {
        fn sample(&self, _point: Point3<f32>) -> Option<WaterSample> {
            Some(WaterSample {
                surface_level: 5.0,
                floor_level: 0.0,
                velocity: nalgebra::Vector3::zeros(),
                surface_rise: 0.0,
            })
        }
    }

    /// A ripple field that remembers every disturbance.
    #[derive(Default)]
    struct Recorder {
        disturbances: Vec<Disturbance>,
    }

    impl RippleField for Recorder {
        fn disturb(&mut self, _at: Point3<f32>, _radius: f32, disturbance: Disturbance) {
            self.disturbances.push(disturbance);
        }
    }

    fn body(y: f32, velocity: Vector3<f32>, self_propelled: bool) -> BodySnapshot {
        BodySnapshot {
            id: 1,
            position: Point3::new(5.0, y, 5.0),
            velocity,
            footprint_radius: 0.5,
            mass: 20.0,
            is_self_propelled: self_propelled,
        }
    }

    #[test]
    fn impact_splashes_and_disturbs() {
        let mut ripples = Recorder::default();
        let mut coupler = WaveBodyCoupler::new(WaveCouplingConfig::default());
        coupler.update(
            &[body(8.0, Vector3::new(0.0, -5.0, 0.0), false)],
            &mut ripples,
            &Flat,
        );
        assert!(
            ripples.disturbances.is_empty(),
            "no disturbance above water"
        );

        coupler.update(
            &[body(4.8, Vector3::new(0.0, -5.0, 0.0), false)],
            &mut ripples,
            &Flat,
        );
        assert!(matches!(ripples.disturbances[..], [Disturbance::Velocity(v)] if v < 0.0));
        assert_eq!(coupler.drain_splash_events().len(), 1);
    }

    #[test]
    fn bobbing_disturbs_every_frame() {
        let mut ripples = Recorder::default();
        let mut coupler = WaveBodyCoupler::new(WaveCouplingConfig::default());
        coupler.update(
            &[body(4.5, Vector3::new(0.0, 0.5, 0.0), false)],
            &mut ripples,
            &Flat,
        );
        coupler.update(
            &[body(4.7, Vector3::new(0.0, -0.3, 0.0), false)],
            &mut ripples,
            &Flat,
        );
        assert!(ripples
            .disturbances
            .iter()
            .any(|d| matches!(d, Disturbance::Displacement(_))));
    }

    /// Water at 5 m whose surface is rising at 0.4 m/s: the swell's heave.
    struct Heaving;

    impl WaterSurface for Heaving {
        fn sample(&self, _point: Point3<f32>) -> Option<WaterSample> {
            Some(WaterSample {
                surface_rise: 0.4,
                ..Flat
                    .sample(Point3::origin())
                    .expect("flat water everywhere")
            })
        }
    }

    #[test]
    fn a_float_riding_the_swell_stirs_nothing() {
        let mut ripples = Recorder::default();
        let mut coupler = WaveBodyCoupler::new(WaveCouplingConfig::default());
        for _ in 0..2 {
            coupler.update(
                &[body(4.7, Vector3::new(0.0, 0.4, 0.0), false)],
                &mut ripples,
                &Heaving,
            );
        }
        assert!(
            ripples.disturbances.is_empty(),
            "{:?}",
            ripples.disturbances
        );
    }

    #[test]
    fn player_wake_from_horizontal_movement() {
        let mut ripples = Recorder::default();
        let mut coupler = WaveBodyCoupler::new(WaveCouplingConfig::default());
        coupler.update(&[body(4.5, Vector3::zeros(), true)], &mut ripples, &Flat);
        coupler.update(
            &[body(4.5, Vector3::new(3.0, 0.0, 0.0), true)],
            &mut ripples,
            &Flat,
        );
        assert_eq!(coupler.drain_wake_events().len(), 1);
        assert!(!ripples.disturbances.is_empty());
    }

    #[test]
    fn stale_bodies_cleaned_up() {
        let mut ripples = Recorder::default();
        let mut coupler = WaveBodyCoupler::new(WaveCouplingConfig::default());
        coupler.update(&[body(4.5, Vector3::zeros(), false)], &mut ripples, &Flat);
        assert!(coupler.body_states.contains_key(&1));
        for _ in 0..60 {
            coupler.update(&[], &mut ripples, &Flat);
        }
        assert!(!coupler.body_states.contains_key(&1));
    }

    #[test]
    fn a_body_deep_under_water_leaves_the_surface_alone() {
        let mut ripples = Recorder::default();
        let mut coupler = WaveBodyCoupler::new(WaveCouplingConfig::default());
        let gliding = Vector3::new(3.0, 0.0, 0.0);
        let sinking = Vector3::new(3.0, -1.0, 0.0);
        coupler.update(&[body(2.0, gliding, true)], &mut ripples, &Flat);
        coupler.update(&[body(1.9, sinking, true)], &mut ripples, &Flat);
        assert!(ripples.disturbances.is_empty());
        assert!(coupler.wake_events.is_empty());
    }
}
