//! Links: the laws that move water between stores.
//!
//! Stores are a closed set; links are where new physics is added. Each law
//! owns its parameters, so a link never has to downcast the store it drains.

use std::fmt::Debug;

use nalgebra::Point3;

use super::store::StoreView;

/// The arc a link's water follows when it leaves a lip and falls: for the
/// fall mesh, and for re-tracing when an edit touches it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FallPath {
    /// Points along the arc, from the lip to where it lands.
    pub points: Vec<Point3<f32>>,
}

/// A law moving water from an upstream store to a downstream one.
pub trait Link: Debug + Send + Sync {
    /// Discharge in m³/s from `up` to `down`. Negative means reversed flow,
    /// which only a link that says it is [`Self::reversible`] may return.
    fn discharge(&self, up: StoreView, down: StoreView) -> f64;

    /// d(discharge)/d(up volume) and d(discharge)/d(down volume), for the
    /// implicit solve.
    fn jacobian(&self, up: StoreView, down: StoreView) -> (f64, f64);

    /// Whether the discharge depends on the downstream store and can
    /// reverse. Such a link joins its two stores into one group the solver
    /// solves together.
    fn reversible(&self) -> bool {
        false
    }

    /// The arc the water follows if it leaves a lip.
    fn fall_path(&self) -> Option<&FallPath> {
        None
    }
}
