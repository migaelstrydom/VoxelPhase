//! The orifice: water through a hole in a basin's floor.
//!
//! ```text
//!   Q = gain · C_d · a · √(2 g H)
//! ```
//!
//! for a hole of area `a` under a head `H` above its lip. At low head the
//! hole cannot run full, and water spills over its rim as over a weir, so the
//! law is the smooth minimum of the orifice and a weir along the hole's
//! perimeter.

use crate::water::network::link::Link;
use crate::water::network::store::StoreView;

use super::weir::{level_slope, WEIR_COEFFICIENT};

/// Orifice discharge coefficient.
pub const ORIFICE_COEFFICIENT: f64 = 0.6;

/// Sharpness of the smooth minimum between the two laws.
const SMOOTH_MIN_POWER: f64 = 4.0;

const GRAVITY: f64 = 9.81;

/// Water through a hole in a floor.
#[derive(Debug, Clone, PartialEq)]
pub struct Orifice {
    /// The hole's rim: the floor it was blown through.
    pub lip: f32,
    /// Plan area of the hole, m².
    pub area: f64,
    /// Length of its rim, m.
    pub perimeter: f64,
    pub gain: f32,
}

impl Orifice {
    /// Discharge at `level`, and its derivative in the level.
    pub fn at(&self, level: f64) -> (f64, f64) {
        let head = level - self.lip as f64;
        if head <= 0.0 {
            return (0.0, 0.0);
        }
        let g = self.gain as f64;
        let orifice = g * ORIFICE_COEFFICIENT * self.area * (2.0 * GRAVITY * head).sqrt();
        let d_orifice = orifice / (2.0 * head);
        let weir = g * WEIR_COEFFICIENT * self.perimeter * head.powf(1.5);
        let d_weir = 1.5 * weir / head;
        // q = (o^−n + w^−n)^(−1/n)
        let n = SMOOTH_MIN_POWER;
        let s = orifice.powf(-n) + weir.powf(-n);
        let q = s.powf(-1.0 / n);
        let dq = s.powf(-1.0 / n - 1.0)
            * (orifice.powf(-n - 1.0) * d_orifice + weir.powf(-n - 1.0) * d_weir);
        (q, dq)
    }
}

impl Link for Orifice {
    fn discharge(&self, up: StoreView, _down: StoreView) -> f64 {
        self.at(up.level() as f64).0
    }

    fn jacobian(&self, up: StoreView, _down: StoreView) -> (f64, f64) {
        (self.at(up.level() as f64).1 / level_slope(up), 0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hole() -> Orifice {
        Orifice {
            lip: 0.0,
            area: 1.0,
            perimeter: 4.0,
            gain: 1.0,
        }
    }

    #[test]
    fn deep_water_runs_as_an_orifice_and_shallow_as_a_weir() {
        let deep = hole().at(4.0).0;
        let orifice = ORIFICE_COEFFICIENT * (2.0 * GRAVITY * 4.0f64).sqrt();
        assert!(
            (deep - orifice).abs() / orifice < 0.05,
            "{deep} vs {orifice}"
        );
        let shallow = hole().at(0.01).0;
        let weir = WEIR_COEFFICIENT * 4.0 * 0.01f64.powf(1.5);
        assert!((shallow - weir).abs() / weir < 0.05, "{shallow} vs {weir}");
    }

    #[test]
    fn the_derivative_matches_a_finite_difference() {
        let (h, e) = (0.3, 1e-6);
        let (_, d) = hole().at(h);
        let f = (hole().at(h + e).0 - hole().at(h - e).0) / (2.0 * e);
        assert!((d - f).abs() < 1e-4 * f, "{d} vs {f}");
    }
}
