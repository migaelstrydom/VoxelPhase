//! A source's outflow: the discharge it was authored with.

use crate::water::network::link::Link;
use crate::water::network::store::StoreView;

/// What a spring or sky source gives, whatever is upstream or down (§7.6).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FixedRate {
    /// m³/s.
    pub discharge: f64,
}

impl Link for FixedRate {
    fn discharge(&self, _up: StoreView, _down: StoreView) -> f64 {
        self.discharge
    }

    fn jacobian(&self, _up: StoreView, _down: StoreView) -> (f64, f64) {
        (0.0, 0.0)
    }
}
