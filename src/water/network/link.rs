//! Links: the laws that move water between stores.
//!
//! Stores are a closed set; links are where new physics is added. Each law
//! owns its parameters, so a link never has to downcast the store it drains.

use std::fmt::Debug;

use nalgebra::Point3;

use crate::water::geometry::Column;

use super::store::StoreView;

/// The arc a link's water follows when it leaves a lip and falls: for the
/// fall mesh, and for re-tracing when an edit touches it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FallPath {
    /// Points along the arc, from the lip to where it lands.
    pub points: Vec<Point3<f32>>,
    /// Seconds the water takes from the lip to the landing, at each point.
    pub times: Vec<f32>,
}

impl FallPath {
    /// Whether the arc passes over any of `columns` (sorted).
    pub fn crosses(&self, columns: &[Column]) -> bool {
        self.points
            .iter()
            .any(|p| columns.binary_search(&Column::containing(p.x, p.z)).is_ok())
    }

    /// Where the water lands.
    pub fn landing(&self) -> Option<Point3<f32>> {
        self.points.last().copied()
    }

    /// Carry on along `next`, which starts where this lands: a fall onto a
    /// ledge too short to be a channel, and over its edge.
    pub fn extend(&mut self, next: FallPath) {
        let offset = self.times.last().copied().unwrap_or(0.0);
        self.points.extend(next.points);
        self.times
            .extend(next.times.into_iter().map(|t| t + offset));
    }

    /// Height the water drops, m.
    pub fn drop(&self) -> f32 {
        match (self.points.first(), self.points.last()) {
            (Some(a), Some(b)) => a.y - b.y,
            _ => 0.0,
        }
    }
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
}
