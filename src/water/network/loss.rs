//! Loss: minor water only (§7.7).
//!
//! Water lost to the ground and air clears up leftovers — trickles that
//! should end, puddles that should dry — without thinning rivers or lowering
//! lakes. So it applies only to *minor* stores, a flag the topology sets
//! between ticks with hysteresis. Within a tick the flag is fixed and the
//! loss is the rate times the wetted area, which rises with the store's own
//! volume: the implicit solve stays unconditionally stable.

use super::store::Store;

/// A minor store leaves minor at this multiple of its threshold.
pub const MINOR_HYSTERESIS: f32 = 1.5;

/// How minor water is lost, per level.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LossLaw {
    /// Loss per unit wetted area, m/s. Zero turns loss off.
    pub rate: f64,
    /// A basin is minor while its deepest water is shallower than this.
    pub minor_depth: f32,
    /// A reach is minor while its inflow is below this, m³/s.
    pub minor_discharge: f64,
}

impl Default for LossLaw {
    fn default() -> Self {
        Self {
            rate: 0.0,
            minor_depth: 0.1,
            minor_discharge: 0.02,
        }
    }
}

impl LossLaw {
    /// A law losing `mm_per_hour` from minor stores.
    pub fn from_mm_per_hour(mm_per_hour: f64) -> Self {
        Self {
            rate: mm_per_hour / 1000.0 / 3600.0,
            ..Self::default()
        }
    }

    /// Water a store loses at `volume`, m³/s, and its derivative in volume.
    pub fn loss(&self, store: &Store, volume: f64) -> (f64, f64) {
        if self.rate <= 0.0 || !store.is_minor() {
            return (0.0, 0.0);
        }
        match store {
            Store::Basin(b) => {
                let level = b.hypsometry.level(volume);
                let area = b.hypsometry.area(level);
                // d(area)/dV = (d area/dL) / area; approximate by a finite
                // difference on the level.
                let dl = 1e-3;
                let slope = (b.hypsometry.area(level + dl) - area) / dl as f64;
                let d = if area > 1e-9 { slope / area } else { 0.0 };
                (self.rate * area, self.rate * d)
            }
            Store::Sink | Store::Reservoir => (0.0, 0.0),
        }
    }

    /// Whether a basin should be minor, given whether it is now: its depth
    /// against the threshold, with hysteresis.
    pub fn basin_minor(&self, depth: f32, now: bool) -> bool {
        if now {
            depth < self.minor_depth * MINOR_HYSTERESIS
        } else {
            depth < self.minor_depth
        }
    }
}
