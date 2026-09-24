//! The weir: water over a crest.
//!
//! ```text
//!   Q_free(L) = gain · C_w · w · Σ_cells max(L − h_i, 0)^1.5
//! ```
//!
//! summed over the crest's cells, each at its own saddle `h_i`, so a V-shaped
//! notch behaves as a V-notch weir (∝ H^2.5) with no special case. Between two
//! basins the weir can drown: with heads `H_up` and `H_down` over a cell,
//! Villemonte's correction `f(r) = (1 − r^1.5)^0.385` scales the free flow,
//! run linearly to zero past r = 0.98 so that its derivative stays finite, and
//! flow reverses when the downstream level is the higher.

use crate::water::geometry::COLUMN_SIZE;
use crate::water::network::link::Link;
use crate::water::network::store::StoreView;

/// Weir discharge coefficient, m^0.5/s.
pub const WEIR_COEFFICIENT: f64 = 1.7;

/// Villemonte's ratio past which the correction runs linearly to zero.
const DROWNED_KNEE: f64 = 0.98;

/// Water over one crest.
#[derive(Debug, Clone, PartialEq)]
pub struct Weir {
    /// Each crest cell's saddle, ascending.
    pub saddles: Vec<f32>,
    /// The level's `drain_gain` (§7.3).
    pub gain: f32,
    /// Whether the downstream store is a basin whose level can drown or
    /// reverse the flow.
    pub between_basins: bool,
}

impl Weir {
    pub fn new(mut saddles: Vec<f32>, gain: f32, between_basins: bool) -> Self {
        saddles.sort_by(|a, b| a.total_cmp(b));
        Self {
            saddles,
            gain,
            between_basins,
        }
    }

    /// The lowest saddle.
    pub fn lip(&self) -> f32 {
        self.saddles.first().copied().unwrap_or(f32::INFINITY)
    }

    fn coefficient(&self) -> f64 {
        self.gain as f64 * WEIR_COEFFICIENT * COLUMN_SIZE as f64
    }

    /// Free discharge at level `level`, and its derivative in the level.
    pub fn free(&self, level: f64) -> (f64, f64) {
        let mut q = 0.0;
        let mut dq = 0.0;
        for &h in &self.saddles {
            let head = level - h as f64;
            if head <= 0.0 {
                break;
            }
            q += head.powf(1.5);
            dq += 1.5 * head.sqrt();
        }
        let c = self.coefficient();
        (c * q, c * dq)
    }

    /// Discharge from the store at `high` to the store at `low` (levels,
    /// `high ≥ low`), and its derivatives in each level.
    fn drowned(&self, high: f64, low: f64) -> (f64, f64, f64) {
        let mut q = 0.0;
        let mut d_high = 0.0;
        let mut d_low = 0.0;
        for &h in &self.saddles {
            let h = h as f64;
            let head = high - h;
            if head <= 0.0 {
                break;
            }
            let tail = (low - h).max(0.0);
            let r = tail / head;
            let (f, df_dr) = villemonte(r);
            let free = head.powf(1.5);
            let d_free = 1.5 * head.sqrt();
            q += free * f;
            // r = tail / head: dr/dhigh = −r/head, dr/dlow = 1/head (if wet).
            d_high += d_free * f + free * df_dr * (-r / head);
            if tail > 0.0 {
                d_low += free * df_dr / head;
            }
        }
        let c = self.coefficient();
        (c * q, c * d_high, c * d_low)
    }
}

/// Villemonte's submergence factor and its derivative in r.
fn villemonte(r: f64) -> (f64, f64) {
    if r <= 0.0 {
        return (1.0, 0.0);
    }
    let at = |r: f64| (1.0 - r.powf(1.5)).max(0.0).powf(0.385);
    let slope = |r: f64| {
        let base = (1.0 - r.powf(1.5)).max(1e-12);
        0.385 * base.powf(0.385 - 1.0) * (-1.5 * r.sqrt())
    };
    if r <= DROWNED_KNEE {
        (at(r), slope(r))
    } else {
        // Linear from the knee to zero at r = 1.
        let knee = at(DROWNED_KNEE);
        let run = 1.0 - DROWNED_KNEE;
        let f = knee * ((1.0 - r) / run).max(0.0);
        (f, if r < 1.0 { -knee / run } else { 0.0 })
    }
}

impl Link for Weir {
    fn discharge(&self, up: StoreView, down: StoreView) -> f64 {
        let (lu, ld) = (up.level() as f64, down.level() as f64);
        if !self.between_basins || !ld.is_finite() {
            return self.free(lu).0;
        }
        if lu >= ld {
            self.drowned(lu, ld).0
        } else {
            -self.drowned(ld, lu).0
        }
    }

    fn jacobian(&self, up: StoreView, down: StoreView) -> (f64, f64) {
        let (lu, ld) = (up.level() as f64, down.level() as f64);
        let (dq_dlu, dq_dld) = if !self.between_basins || !ld.is_finite() {
            (self.free(lu).1, 0.0)
        } else if lu >= ld {
            let (_, a, b) = self.drowned(lu, ld);
            (a, b)
        } else {
            let (_, a, b) = self.drowned(ld, lu);
            (-b, -a)
        };
        (
            dq_dlu / level_slope(up),
            if dq_dld == 0.0 {
                0.0
            } else {
                dq_dld / level_slope(down)
            },
        )
    }

    fn reversible(&self) -> bool {
        self.between_basins
    }
}

/// dV/dL of a store at its current volume: its wetted area. Never below a
/// square centimetre, so an empty store's derivative stays finite.
pub fn level_slope(view: StoreView) -> f64 {
    match view.store.as_basin() {
        Some(b) => b.hypsometry.area(view.level()).max(1e-4),
        None => f64::INFINITY,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_five_metre_lip_passes_the_design_s_discharge() {
        // §22: a 5 m lip at 0.2 m³/s runs about 8 cm deep; at 1.5 m³/s, 31 cm.
        let weir = Weir::new(vec![0.0; 10], 1.0, false);
        let at = |q: f64| {
            let (mut lo, mut hi) = (0.0, 2.0);
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                if weir.free(mid).0 < q {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            hi
        };
        assert!((at(0.2) - 0.08).abs() < 0.01, "{}", at(0.2));
        assert!((at(1.5) - 0.31).abs() < 0.02, "{}", at(1.5));
    }

    #[test]
    fn a_drowned_weir_passes_less_and_reverses() {
        let weir = Weir::new(vec![0.0; 4], 1.0, true);
        let free = weir.drowned(0.5, -1.0).0;
        let drowned = weir.drowned(0.5, 0.45).0;
        assert!(drowned < free && drowned > 0.0);
        assert!(weir.drowned(0.5, 0.5).0.abs() < 1e-9);
    }

    #[test]
    fn the_drowned_derivatives_match_finite_differences() {
        let weir = Weir::new(vec![0.0, 0.05, 0.1], 1.0, true);
        let (hi, lo, e) = (0.4, 0.3, 1e-6);
        let (_, a, b) = weir.drowned(hi, lo);
        let fa = (weir.drowned(hi + e, lo).0 - weir.drowned(hi - e, lo).0) / (2.0 * e);
        let fb = (weir.drowned(hi, lo + e).0 - weir.drowned(hi, lo - e).0) / (2.0 * e);
        assert!((a - fa).abs() < 1e-4 * fa.abs().max(1.0), "{a} vs {fa}");
        assert!((b - fb).abs() < 1e-4 * fb.abs().max(1.0), "{b} vs {fb}");
    }
}
