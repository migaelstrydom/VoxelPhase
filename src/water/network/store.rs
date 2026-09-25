//! Stores: everything that holds water.
//!
//! The kinds are closed and stored contiguously, so a store is an enum. A
//! store answers only for its volume, its level at a port and its loss; the
//! laws that move water between stores live in links.

use super::basin::Basin;
use super::ocean::Ocean;
use super::reach::Reach;

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
    /// A stretch of running channel.
    Reach(Reach),
    /// The sea: fixed level, endless volume.
    Ocean(Ocean),
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
            Store::Reach(r) => r.storage,
            Store::Ocean(_) | Store::Sink | Store::Reservoir => f64::INFINITY,
        }
    }

    /// Whether the store holds a finite volume the solver integrates.
    pub fn is_finite(&self) -> bool {
        matches!(self, Store::Basin(_) | Store::Reach(_))
    }

    /// The surface level at `port` implied by `volume`.
    pub fn level_at(&self, volume: f64, port: Port) -> f32 {
        match self {
            Store::Basin(b) => b.hypsometry.level(volume),
            Store::Reach(r) => r.level_at_end(port == Port::Downstream),
            Store::Ocean(o) => o.level,
            Store::Sink => f32::NEG_INFINITY,
            Store::Reservoir => f32::INFINITY,
        }
    }

    /// What a store lets out of its downstream end at `volume` by its own
    /// law, and the derivative: a reach's rating. `None` for stores whose
    /// outflow is set by the link across their lip.
    pub fn release(&self, volume: f64) -> Option<(f64, f64)> {
        match self {
            Store::Reach(r) => Some(r.release(volume)),
            _ => None,
        }
    }

    pub fn as_reach(&self) -> Option<&Reach> {
        match self {
            Store::Reach(r) => Some(r),
            _ => None,
        }
    }

    pub fn as_reach_mut(&mut self) -> Option<&mut Reach> {
        match self {
            Store::Reach(r) => Some(r),
            _ => None,
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

    pub fn as_ocean(&self) -> Option<&Ocean> {
        match self {
            Store::Ocean(o) => Some(o),
            _ => None,
        }
    }

    pub fn as_ocean_mut(&mut self) -> Option<&mut Ocean> {
        match self {
            Store::Ocean(o) => Some(o),
            _ => None,
        }
    }

    /// The still surface of a store that has one: a basin's level, or the
    /// sea's.
    pub fn surface(&self) -> Option<f32> {
        match self {
            Store::Basin(b) => Some(b.level()),
            Store::Ocean(o) => Some(o.level),
            _ => None,
        }
    }

    /// The highest a store can hold water: a basin's cap, the sea's level.
    pub fn cap(&self) -> Option<f32> {
        match self {
            Store::Basin(b) => Some(b.cap),
            Store::Ocean(o) => Some(o.level),
            _ => None,
        }
    }

    /// Whether the store waits on a deferred re-flood (§9.2).
    pub fn is_frozen(&self) -> bool {
        matches!(self, Store::Basin(b) if b.frozen)
    }

    /// Whether the store is minor, and loses water (§7.7).
    pub fn is_minor(&self) -> bool {
        match self {
            Store::Basin(b) => b.minor,
            Store::Reach(r) => r.minor,
            Store::Ocean(_) | Store::Sink | Store::Reservoir => false,
        }
    }

    fn set_volume(&mut self, volume: f64) {
        match self {
            Store::Basin(b) => b.volume = volume,
            Store::Reach(r) => r.storage = volume,
            Store::Ocean(_) | Store::Sink | Store::Reservoir => {}
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
