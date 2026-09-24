//! Stores: everything that holds water.
//!
//! The kinds are closed and stored contiguously, so a store is an enum. A
//! store answers only for its volume, its level at a port and its loss; the
//! laws that move water between stores live in links.

use super::basin::Basin;

/// Where a link attaches to a store. Basins ignore it; a reach has two ends.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Port {
    Upstream,
    Downstream,
}

/// Everything that holds water.
#[derive(Debug, Clone)]
pub enum Store {
    /// Still water with one level.
    Basin(Basin),
    /// Takes whatever reaches it: an authored sink, the edge of the world.
    Sink,
    /// Gives without running dry: behind a spring or a sky source.
    Reservoir,
}

impl Store {
    /// Water held, m³; infinite for sinks and reservoirs.
    pub fn volume(&self) -> f64 {
        match self {
            Store::Basin(b) => b.volume,
            Store::Sink | Store::Reservoir => f64::INFINITY,
        }
    }

    /// Whether the store holds a finite volume the solver integrates.
    pub fn is_finite(&self) -> bool {
        matches!(self, Store::Basin(_))
    }

    /// The surface level at `port` implied by `volume`.
    pub fn level_at(&self, volume: f64, _port: Port) -> f32 {
        match self {
            Store::Basin(b) => b.hypsometry.level(volume),
            Store::Sink => f32::NEG_INFINITY,
            Store::Reservoir => f32::INFINITY,
        }
    }

    pub fn as_basin(&self) -> Option<&Basin> {
        match self {
            Store::Basin(b) => Some(b),
            _ => None,
        }
    }

    pub fn as_basin_mut(&mut self) -> Option<&mut Basin> {
        match self {
            Store::Basin(b) => Some(b),
            _ => None,
        }
    }

    /// Whether the store is minor, and loses water (§7.7).
    pub fn is_minor(&self) -> bool {
        match self {
            Store::Basin(b) => b.minor,
            Store::Sink | Store::Reservoir => false,
        }
    }

    fn set_volume(&mut self, volume: f64) {
        if let Store::Basin(b) = self {
            b.volume = volume;
        }
    }

    /// Add `delta` m³. Only the ledger's callers move volume.
    pub(crate) fn adjust_volume(&mut self, delta: f64) {
        let volume = self.volume() + delta;
        self.set_volume(volume.max(0.0));
    }
}

/// A store's volume and level function, seen from one port.
#[derive(Debug, Clone, Copy)]
pub struct StoreView<'a> {
    pub store: &'a Store,
    pub volume: f64,
    pub port: Port,
}

impl StoreView<'_> {
    pub fn level(&self) -> f32 {
        self.store.level_at(self.volume, self.port)
    }
}
