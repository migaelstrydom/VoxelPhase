//! Physical properties shared by all water in the game.
//!
//! These constants govern how water behaves — flow equalization rate, wave
//! propagation speed, damping, fluid density, etc. They are independent of
//! grid geometry and do not vary between water bodies.

/// Physical properties of water, shared by the flow grid and wave grid.
///
/// These are simulation constants that apply uniformly to all water in the
/// game. Grid geometry (dims, origin, cell size) is configured separately
/// in [`super::WaterGridConfig`] and [`super::WaveGridConfig`].
pub struct WaterProperties {
    /// Flow equalization rate in 1/s: the fraction of the head difference
    /// between two neighbouring flow cells that moves across their shared edge
    /// per second. Higher = faster leveling.
    ///
    /// Independent of cell size — the grid scales the transfer by cell area.
    /// Values above `1 / dt * 0.125` are clamped by the flow grid for
    /// stability, so ~7 is the practical ceiling at 60 Hz.
    pub flow_rate: f32,

    /// Surface level difference below which the flow simulation is considered
    /// settled (no significant flow remains).
    pub settle_epsilon: f32,

    /// Fluid density in kg/m³. Used by the buoyancy system to compute
    /// upward force on submerged bodies.
    pub fluid_density: f32,

    /// Wave propagation speed in m/s. Controls how fast ripples travel
    /// across the fine wave grid.
    pub wave_speed: f32,

    /// Wave damping coefficient in 1/s. Controls how quickly ripple
    /// energy dissipates.
    pub wave_damping: f32,
}

impl Default for WaterProperties {
    fn default() -> Self {
        Self {
            flow_rate: 4.0,
            settle_epsilon: 0.001,
            fluid_density: 1000.0,
            wave_speed: 2.0,
            wave_damping: 0.5,
        }
    }
}
