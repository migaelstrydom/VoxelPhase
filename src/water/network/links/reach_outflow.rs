//! A reach's outflow: whatever its rating lets out of its downstream end.

use crate::water::network::link::Link;
use crate::water::network::store::StoreView;

/// The water leaving the bottom of a reach, by the reach's own rating
/// (§7.5). The law holds nothing: the reach answers for its release.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ReachOutflow;

impl Link for ReachOutflow {
    fn discharge(&self, up: StoreView, _down: StoreView) -> f64 {
        up.store.release(up.volume).map_or(0.0, |(q, _)| q)
    }

    fn jacobian(&self, up: StoreView, _down: StoreView) -> (f64, f64) {
        (up.store.release(up.volume).map_or(0.0, |(_, dq)| dq), 0.0)
    }
}
