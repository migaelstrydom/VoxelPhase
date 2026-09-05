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
//! No ECS or physics engine dependency — the caller provides body snapshots.

use std::collections::HashMap;

use nalgebra::Point3;

use super::{WaterGrid, WaveGrid};
use crate::water::buoyancy::sample_water;

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
        wave_grid: &mut WaveGrid,
        flow_grid: &WaterGrid,
    ) {
        self.splash_events.clear();
        self.wake_events.clear();

        // Mark all existing entries as aged; we'll reset age for bodies we see.
        for state in self.body_states.values_mut() {
            state.age += 1;
        }

        for body in bodies {
            let sample = sample_water(flow_grid, None, body.position.x, body.position.z);

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

            // Wave injection scales with mass: light objects create smaller
            // disturbances, breaking the feedback loop where self-generated
            // waves inflate buoyancy and cause perpetual bouncing.
            let mass_factor = (body.mass / self.config.reference_mass).min(1.0).max(0.0);

            // 1. Impact: body just entered water with downward velocity.
            if !was_submerged {
                let down_speed = -body.velocity.y;
                if down_speed > self.config.impact_speed_threshold {
                    let strength = -down_speed * self.config.impact_strength * mass_factor;
                    self.inject_at_footprint(body, wave_grid, InjectionMode::Velocity(strength));

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
            //    body's oscillation regardless of wave damping.
            if was_submerged {
                let vy = body.velocity.y;
                if vy.abs() > 0.01 {
                    let strength = -vy * self.config.bobbing_strength * mass_factor;
                    self.inject_at_footprint(
                        body,
                        wave_grid,
                        InjectionMode::Displacement(strength),
                    );
                }
            }

            // 3. Wake: self-propelled body moving horizontally through water.
            //    Uses displacement injection behind the body.
            if body.is_self_propelled {
                let horizontal_vel = nalgebra::Vector3::new(body.velocity.x, 0.0, body.velocity.z);
                let h_speed = horizontal_vel.magnitude();
                if h_speed > self.config.wake_speed_threshold {
                    let move_dir = horizontal_vel / h_speed;

                    // Inject behind the body (opposite movement direction).
                    let wake_offset = -move_dir * body.footprint_radius * 0.8;
                    let wake_pos = body.position + wake_offset;
                    let strength = -h_speed * self.config.wake_strength;
                    wave_grid.inject_displacement_at(wake_pos.x, wake_pos.z, strength);

                    let surface_y = sample.as_ref().map_or(body.position.y, |s| s.surface_level);
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

    /// Inject a wave disturbance across the body's XZ footprint cells.
    fn inject_at_footprint(
        &self,
        body: &BodySnapshot,
        wave_grid: &mut WaveGrid,
        mode: InjectionMode,
    ) {
        let r = body.footprint_radius;
        let cell_size = wave_grid.cell_size();
        let origin = wave_grid.origin();
        let dims = wave_grid.dims();

        // Compute the bounding box of the footprint in wave grid coords.
        let min_x = body.position.x - r;
        let max_x = body.position.x + r;
        let min_z = body.position.z - r;
        let max_z = body.position.z + r;

        let i_min = ((min_x - origin.x) / cell_size).floor().max(0.0) as usize;
        let i_max = (((max_x - origin.x) / cell_size).ceil() as usize).min(dims.0);
        let j_min = ((min_z - origin.z) / cell_size).floor().max(0.0) as usize;
        let j_max = (((max_z - origin.z) / cell_size).ceil() as usize).min(dims.1);

        let r_sq = r * r;
        let cx = body.position.x;
        let cz = body.position.z;

        for wj in j_min..j_max {
            for wi in i_min..i_max {
                let wx = origin.x + (wi as f32 + 0.5) * cell_size;
                let wz = origin.z + (wj as f32 + 0.5) * cell_size;
                let dx = wx - cx;
                let dz = wz - cz;
                let dist_sq = dx * dx + dz * dz;

                if dist_sq > r_sq {
                    continue;
                }

                // Smooth falloff from center to edge.
                let t = 1.0 - (dist_sq / r_sq).sqrt();

                match mode {
                    InjectionMode::Velocity(amount) => {
                        wave_grid.cell_mut(wi, wj).velocity += amount * t;
                    }
                    InjectionMode::Displacement(amount) => {
                        wave_grid.cell_mut(wi, wj).displacement += amount * t;
                    }
                }
            }
        }
    }
}

#[allow(dead_code)]
enum InjectionMode {
    Velocity(f32),
    Displacement(f32),
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use super::*;
    use crate::water::{WaterGridConfig, WaterProperties, WaveGridConfig};

    fn test_properties() -> WaterProperties {
        WaterProperties {
            wave_speed: 4.0,
            ..Default::default()
        }
    }

    fn make_flow_grid() -> WaterGrid {
        let config = WaterGridConfig {
            cell_size: 2.0,
            dims: (5, 5),
            origin: Vector3::new(0.0, 0.0, 0.0),
            ocean_level: None,
        };
        let mut grid = WaterGrid::new(config, &test_properties());
        // Fill with water at surface level 5.0 on a flat floor.
        let cell_area = grid.cell_area();
        for j in 0..5 {
            for i in 0..5 {
                grid.add_water(i, j, 5.0 * cell_area, 0.0);
            }
        }
        grid
    }

    fn make_wave_grid(flow_grid: &WaterGrid) -> WaveGrid {
        let flow_dims = flow_grid.dims();
        let flow_cell_size = flow_grid.cell_size();
        let wave_cell_size = 0.5;
        let n = (flow_cell_size / wave_cell_size).round() as usize;
        let wave_dims = (flow_dims.0 * n, flow_dims.1 * n);

        let config = WaveGridConfig {
            cell_size: wave_cell_size,
            dims: wave_dims,
            origin: flow_grid.origin(),
            cells_per_flow_cell: n,
        };
        WaveGrid::new(config, &test_properties())
    }

    fn total_wave_energy(wave_grid: &WaveGrid) -> f32 {
        let dims = wave_grid.dims();
        let mut energy = 0.0f32;
        for wj in 0..dims.1 {
            for wi in 0..dims.0 {
                let c = wave_grid.cell(wi, wj);
                energy += c.displacement * c.displacement + c.velocity * c.velocity;
            }
        }
        energy
    }

    #[test]
    fn impact_creates_wave_disturbance() {
        let flow_grid = make_flow_grid();
        let mut wave_grid = make_wave_grid(&flow_grid);
        let mut coupler = WaveBodyCoupler::new(WaveCouplingConfig::default());

        // Body above water, first frame.
        let body_above = BodySnapshot {
            id: 1,
            position: Point3::new(5.0, 8.0, 5.0),
            velocity: Vector3::new(0.0, -5.0, 0.0),
            footprint_radius: 0.5,
            mass: 20.0,
            is_self_propelled: false,
        };
        coupler.update(&[body_above], &mut wave_grid, &flow_grid);
        assert!(
            total_wave_energy(&wave_grid) < 1e-6,
            "No disturbance when body is above water"
        );

        // Body enters water.
        let body_entering = BodySnapshot {
            id: 1,
            position: Point3::new(5.0, 4.8, 5.0),
            velocity: Vector3::new(0.0, -5.0, 0.0),
            footprint_radius: 0.5,
            mass: 20.0,
            is_self_propelled: false,
        };
        coupler.update(&[body_entering], &mut wave_grid, &flow_grid);

        let energy = total_wave_energy(&wave_grid);
        assert!(
            energy > 0.1,
            "Impact should create wave disturbance, got energy={energy}"
        );
    }

    #[test]
    fn bobbing_creates_continuous_disturbance() {
        let flow_grid = make_flow_grid();
        let mut wave_grid = make_wave_grid(&flow_grid);
        let mut coupler = WaveBodyCoupler::new(WaveCouplingConfig::default());

        // Frame 1: body submerged.
        let body = BodySnapshot {
            id: 1,
            position: Point3::new(5.0, 4.5, 5.0),
            velocity: Vector3::new(0.0, 0.5, 0.0),
            footprint_radius: 0.5,
            mass: 20.0,
            is_self_propelled: false,
        };
        coupler.update(&[body], &mut wave_grid, &flow_grid);
        let energy_after_first = total_wave_energy(&wave_grid);

        // Frame 2: still submerged, oscillating.
        let body = BodySnapshot {
            id: 1,
            position: Point3::new(5.0, 4.7, 5.0),
            velocity: Vector3::new(0.0, -0.3, 0.0),
            footprint_radius: 0.5,
            mass: 20.0,
            is_self_propelled: false,
        };
        coupler.update(&[body], &mut wave_grid, &flow_grid);
        let energy_after_second = total_wave_energy(&wave_grid);

        assert!(
            energy_after_second > energy_after_first,
            "Bobbing should accumulate wave energy: first={energy_after_first}, second={energy_after_second}"
        );
    }

    #[test]
    fn player_wake_from_horizontal_movement() {
        let flow_grid = make_flow_grid();
        let mut wave_grid = make_wave_grid(&flow_grid);
        let mut coupler = WaveBodyCoupler::new(WaveCouplingConfig::default());

        // Frame 1: establish submerged state.
        let body = BodySnapshot {
            id: 1,
            position: Point3::new(5.0, 4.5, 5.0),
            velocity: Vector3::zeros(),
            footprint_radius: 0.5,
            mass: 20.0,
            is_self_propelled: true,
        };
        coupler.update(&[body], &mut wave_grid, &flow_grid);

        // Frame 2: moving horizontally.
        let body = BodySnapshot {
            id: 1,
            position: Point3::new(5.0, 4.5, 5.0),
            velocity: Vector3::new(3.0, 0.0, 0.0),
            footprint_radius: 0.5,
            mass: 20.0,
            is_self_propelled: true,
        };
        coupler.update(&[body], &mut wave_grid, &flow_grid);

        let energy = total_wave_energy(&wave_grid);
        assert!(
            energy > 0.01,
            "Player wake should create disturbance, got energy={energy}"
        );
    }

    #[test]
    fn no_disturbance_when_above_water() {
        let flow_grid = make_flow_grid();
        let mut wave_grid = make_wave_grid(&flow_grid);
        let mut coupler = WaveBodyCoupler::new(WaveCouplingConfig::default());

        let body = BodySnapshot {
            id: 1,
            position: Point3::new(5.0, 10.0, 5.0),
            velocity: Vector3::new(0.0, -2.0, 0.0),
            footprint_radius: 0.5,
            mass: 20.0,
            is_self_propelled: false,
        };

        // Two frames above water.
        coupler.update(&[body.clone()], &mut wave_grid, &flow_grid);
        coupler.update(&[body], &mut wave_grid, &flow_grid);

        assert!(
            total_wave_energy(&wave_grid) < 1e-6,
            "No disturbance when body is always above water"
        );
    }

    #[test]
    fn stale_bodies_cleaned_up() {
        let flow_grid = make_flow_grid();
        let mut wave_grid = make_wave_grid(&flow_grid);
        let mut coupler = WaveBodyCoupler::new(WaveCouplingConfig::default());

        let body = BodySnapshot {
            id: 42,
            position: Point3::new(5.0, 4.5, 5.0),
            velocity: Vector3::zeros(),
            footprint_radius: 0.5,
            mass: 20.0,
            is_self_propelled: false,
        };
        coupler.update(&[body], &mut wave_grid, &flow_grid);
        assert!(coupler.body_states.contains_key(&42));

        // 60 frames with no bodies → should be pruned.
        for _ in 0..60 {
            coupler.update(&[], &mut wave_grid, &flow_grid);
        }
        assert!(
            !coupler.body_states.contains_key(&42),
            "Stale body state should be cleaned up"
        );
    }
}
