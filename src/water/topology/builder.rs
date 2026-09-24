//! The topology builder: the only writer of the network's topology.
//!
//! ```text
//!   authored pools ──create_pool──┐
//!   terrain edits ──reregion──────┼──▶ stores, regions, span owners ──▶ solver
//!   between ticks ──settle────────┘    (every volume moved via the ledger)
//! ```
//!
//! It runs between solver ticks. The solver never changes topology, and
//! nothing but the builder writes span ownership.

use std::fmt;

use rustc_hash::FxHashSet;

use crate::water::geometry::{Column, SpanRef, WaterGeometry, ORTHOGONAL};
use crate::water::ids::StoreId;
use crate::water::network::{
    Basin, CrestKind, DepressionFinder, Flood, LossLaw, Network, Store, POTHOLE_DEPTH,
    POTHOLE_VOLUME,
};
use crate::water::solver::{Account, VolumeLedger};

use super::edit::TopologyEdit;

/// A basin within this of its flood's cap is re-flooded to find its higher
/// banks.
const CAP_MARGIN: f32 = 0.01;

/// A lake splits once it is this far below a merge saddle; a pair merges once
/// within `MERGE_LEVELS` of each other. The gap between them stops a lake
/// resting exactly at its saddle from flickering between one body and two.
pub const SPLIT_BELOW: f32 = 0.01;
pub const MERGE_LEVELS: f32 = 0.005;

/// A basin holding less than this, with nothing flowing in, has dried up.
pub const DRIED_VOLUME: f64 = 1e-3;

/// Volume above an outlet smaller than this is left where it is.
const DISCARD_EPSILON: f64 = 1e-6;

/// What the builder needs to change the network: the network itself, its
/// ledger, and the geometry that regions are flooded over.
pub struct Topology<'a> {
    pub network: &'a mut Network,
    pub ledger: &'a mut VolumeLedger,
    pub geometry: &'a mut WaterGeometry,
}

/// Why an authored pool could not be placed.
#[derive(Debug, Clone, PartialEq)]
pub enum PoolError {
    /// The seed's column has no floor under the surface.
    NoFloor { seed: (f32, f32), surface: f32 },
    /// The floor at the seed is at or above the surface.
    DryAtSeed {
        seed: (f32, f32),
        surface: f32,
        floor: f32,
    },
}

impl fmt::Display for PoolError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PoolError::NoFloor { seed, surface } => write!(
                f,
                "pool at ({:.1}, {:.1}) @ {surface:.2}: no floor under the surface",
                seed.0, seed.1
            ),
            PoolError::DryAtSeed {
                seed,
                surface,
                floor,
            } => write!(
                f,
                "pool at ({:.1}, {:.1}) @ {surface:.2}: the floor at the seed is {floor:.2}",
                seed.0, seed.1
            ),
        }
    }
}

/// The only writer of topology.
#[derive(Debug, Default)]
pub struct TopologyBuilder {
    /// Every edit applied, oldest first, when recording is on.
    log: Option<Vec<TopologyEdit>>,
}

impl TopologyBuilder {
    /// A builder that keeps a log of every edit, for tests and the harness.
    pub fn recording() -> Self {
        Self {
            log: Some(Vec::new()),
        }
    }

    /// The edits applied so far, if recording.
    pub fn log(&self) -> &[TopologyEdit] {
        self.log.as_deref().unwrap_or(&[])
    }

    fn record(&mut self, edit: TopologyEdit) {
        if let Some(log) = self.log.as_mut() {
            log.push(edit);
        }
    }

    /// Place an authored pool: flood from the seed at `surface` and fill the
    /// region to it. A seed already inside a basin places nothing more.
    pub fn create_pool(
        &mut self,
        t: &mut Topology,
        seed: (f32, f32),
        surface: f32,
    ) -> Result<StoreId, PoolError> {
        let graph = t.geometry.graph();
        let column = Column::containing(seed.0, seed.1);
        let span = graph
            .span_at(column, surface)
            .ok_or(PoolError::NoFloor { seed, surface })?;
        let floor = graph.span(span).floor_min;
        if floor >= surface {
            return Err(PoolError::DryAtSeed {
                seed,
                surface,
                floor,
            });
        }
        if let Some(existing) = graph.owner(span).body {
            return Ok(existing);
        }
        let flood = self.flood(t, None, &[span], surface);
        let basin = Basin::from_flood(flood, 0.0);
        let volume = basin.hypsometry.volume(surface);
        let id = self.add_basin(t, Basin { volume, ..basin });
        t.ledger.place(volume);
        Ok(id)
    }

    /// Re-flood every basin a terrain edit touched: any owning a span in a
    /// re-paired column or the ring around it.
    pub fn after_terrain_update(&mut self, t: &mut Topology, columns: &[Column]) {
        let graph = t.geometry.graph();
        let mut touched: FxHashSet<StoreId> = FxHashSet::default();
        for column in columns {
            for c in
                std::iter::once(*column).chain(ORTHOGONAL.iter().map(|s| column.offset(s.di, s.dk)))
            {
                for span in graph.refs(c) {
                    if let Some(body) = graph.owner(span).body {
                        touched.insert(body);
                    }
                }
            }
        }
        let mut touched: Vec<StoreId> = touched.into_iter().collect();
        touched.sort();
        for basin in touched {
            self.reregion(t, basin);
        }
    }

    /// Re-flood a basin from its surviving wet spans at its current level,
    /// keeping its volume.
    pub fn reregion(&mut self, t: &mut Topology, id: StoreId) {
        let Some(basin) = t.network.store(id).and_then(Store::as_basin) else {
            return;
        };
        let level = basin.level();
        let columns = basin.columns();
        let seeds = self.surviving_seeds(t, id, &columns, level);
        self.release(t, id, &columns);
        if seeds.is_empty() {
            // Nothing under the water survived: the bed was blown away from
            // under it, into the void.
            self.remove_basin(t, id, Account::Sunk);
            return;
        }
        let flood = self.flood(t, Some(id), &seeds, level);
        let basin = t
            .network
            .store_mut(id)
            .and_then(Store::as_basin_mut)
            .expect("checked above");
        basin.reregion(flood);
        self.claim(t, id);
        self.record(TopologyEdit::Reregion { basin: id, seeds });
    }

    /// The between-tick housekeeping of every basin: water above an outlet
    /// with nowhere to go, splits, the loss gate, drying up, and re-flooding a
    /// basin that has risen to the top of its region.
    pub fn settle(&mut self, t: &mut Topology, loss: &LossLaw) {
        for id in t.network.store_ids() {
            let Some(basin) = t.network.store(id).and_then(Store::as_basin) else {
                continue;
            };
            let level = basin.level();

            // Until outlets carry links (stage 4a), water above one has
            // nowhere to go and leaves the books as discarded.
            if let Some(spill) = basin.spill() {
                let keep = basin.hypsometry.volume(spill);
                let excess = basin.volume - keep;
                if excess > DISCARD_EPSILON {
                    self.transfer(t, Account::Store(id), Account::Discarded, excess);
                }
            }

            let basin = t.network.store(id).and_then(Store::as_basin).expect("live");
            let level = basin.level().min(level);
            if basin
                .merge_saddles
                .iter()
                .any(|m| level < m.saddle - SPLIT_BELOW)
            {
                self.split(t, id);
                continue;
            }
            if level > basin.cap - CAP_MARGIN {
                self.reregion(t, id);
            }

            let Some(basin) = t.network.store(id).and_then(Store::as_basin) else {
                continue;
            };
            if basin.volume < DRIED_VOLUME {
                self.remove_basin(t, id, Account::Lost);
                continue;
            }
            let depth = basin.level() - basin.deepest();
            let minor = loss.basin_minor(depth, basin.minor);
            if minor != basin.minor {
                if let Some(b) = t.network.store_mut(id).and_then(Store::as_basin_mut) {
                    b.minor = minor;
                }
                self.record(TopologyEdit::SetMinor { store: id, minor });
            }
        }
    }

    /// Split a basin that has drained below a merge saddle into one basin per
    /// connected part of what is still under water. Each part keeps the
    /// volume its own hypsometry puts under the level; the rounding remainder
    /// goes to the largest.
    pub fn split(&mut self, t: &mut Topology, id: StoreId) {
        let Some(basin) = t.network.store(id).and_then(Store::as_basin) else {
            return;
        };
        let level = basin.level();
        let volume = basin.volume;
        let columns = basin.columns();
        let wet: Vec<SpanRef> = basin
            .region
            .iter()
            .filter(|r| r.shape.floor_min < level)
            .map(|r| r.span)
            .collect();
        let parts = self.wet_components(t, &wet, level);
        if parts.len() < 2 {
            self.reregion(t, id);
            return;
        }

        self.release(t, id, &columns);
        let mut children: Vec<(Flood, f64)> = Vec::new();
        for part in parts {
            let flood = self.flood(t, None, &part, level);
            let hold = Basin::from_flood(flood.clone(), 0.0)
                .hypsometry
                .volume(level);
            children.push((flood, hold));
        }
        // Parts too small to be depressions dissolve into the largest.
        children.sort_by(|a, b| b.1.total_cmp(&a.1));
        let mut kept: Vec<(Flood, f64)> = Vec::new();
        let mut dissolved = 0.0;
        for (i, (flood, hold)) in children.into_iter().enumerate() {
            let bottom = flood
                .region
                .iter()
                .map(|r| r.shape.floor_min)
                .fold(f32::INFINITY, f32::min);
            if i == 0 || level - bottom > POTHOLE_DEPTH || hold > POTHOLE_VOLUME {
                kept.push((flood, hold));
            } else {
                dissolved += hold;
            }
        }
        let mut remaining = volume;
        let mut ids = Vec::new();
        let count = kept.len();
        for (i, (flood, hold)) in kept.into_iter().enumerate() {
            let share = if i == 0 { hold + dissolved } else { hold };
            let share = if i + 1 == count {
                remaining.max(0.0)
            } else {
                share.min(remaining)
            };
            let child = self.add_basin(t, Basin::from_flood(flood, 0.0));
            self.transfer(t, Account::Store(id), Account::Store(child), share);
            remaining -= share;
            ids.push(child);
        }
        // Whatever rounding left behind goes to the largest.
        if remaining.abs() > 0.0 {
            self.transfer(t, Account::Store(id), Account::Store(ids[0]), remaining);
        }
        t.network.remove_store(id);
        self.record(TopologyEdit::RemoveStore {
            store: id,
            residual_to: Account::Store(ids[0]),
        });
        for child in ids {
            self.claim(t, child);
        }
    }

    /// Move volume between accounts, stores included, through the ledger.
    pub fn transfer(&mut self, t: &mut Topology, from: Account, to: Account, volume: f64) {
        if volume == 0.0 {
            return;
        }
        t.ledger.transfer(from, to, volume);
        if let Account::Store(id) = from {
            if let Some(store) = t.network.store_mut(id) {
                store.adjust_volume(-volume);
            }
        }
        if let Account::Store(id) = to {
            if let Some(store) = t.network.store_mut(id) {
                store.adjust_volume(volume);
            }
        }
        self.record(TopologyEdit::Transfer { from, to, volume });
    }

    fn add_basin(&mut self, t: &mut Topology, basin: Basin) -> StoreId {
        let id = t.network.add_store(Store::Basin(basin));
        self.claim(t, id);
        self.record(TopologyEdit::AddStore(id));
        id
    }

    fn remove_basin(&mut self, t: &mut Topology, id: StoreId, residual_to: Account) {
        let Some(basin) = t.network.store(id).and_then(Store::as_basin) else {
            return;
        };
        let columns = basin.columns();
        let residual = basin.volume;
        self.transfer(t, Account::Store(id), residual_to, residual);
        self.release(t, id, &columns);
        t.network.remove_store(id);
        self.record(TopologyEdit::RemoveStore {
            store: id,
            residual_to,
        });
    }

    fn flood(&self, t: &Topology, store: Option<StoreId>, seeds: &[SpanRef], level: f32) -> Flood {
        let graph = t.geometry.graph();
        let owner = |span: SpanRef| graph.owner(span).body;
        DepressionFinder {
            graph,
            drainage: t.geometry.drainage(),
            store,
            owner: &owner,
        }
        .flood(seeds, level)
    }

    /// Mark a basin's region spans as its own, where no other store has them.
    fn claim(&self, t: &mut Topology, id: StoreId) {
        let Some(basin) = t.network.store(id).and_then(Store::as_basin) else {
            return;
        };
        let spans: Vec<SpanRef> = basin.region.iter().map(|r| r.span).collect();
        let graph = t.geometry.graph_mut();
        for span in spans {
            let owner = graph.owner_mut(span);
            if owner.body.is_none() {
                owner.body = Some(id);
            }
        }
    }

    /// Clear a basin's claim on every span in `columns`.
    fn release(&self, t: &mut Topology, id: StoreId, columns: &[Column]) {
        let graph = t.geometry.graph_mut();
        for &column in columns {
            let refs: Vec<SpanRef> = graph.refs(column).collect();
            for span in refs {
                let owner = graph.owner_mut(span);
                if owner.body == Some(id) {
                    owner.body = None;
                }
            }
        }
    }

    /// A basin's spans that are still under its level, found through
    /// ownership, which the span remap carries across rebuilds.
    fn surviving_seeds(
        &self,
        t: &Topology,
        id: StoreId,
        columns: &[Column],
        level: f32,
    ) -> Vec<SpanRef> {
        let graph = t.geometry.graph();
        let mut seeds = Vec::new();
        for &column in columns {
            for span in graph.refs(column) {
                if graph.owner(span).body == Some(id) && graph.span(span).floor_min < level {
                    seeds.push(span);
                }
            }
        }
        seeds
    }

    /// Connected parts of `spans`, joined below `level`.
    fn wet_components(&self, t: &Topology, spans: &[SpanRef], level: f32) -> Vec<Vec<SpanRef>> {
        let graph = t.geometry.graph();
        let set: FxHashSet<SpanRef> = spans.iter().copied().collect();
        let mut seen: FxHashSet<SpanRef> = FxHashSet::default();
        let mut parts = Vec::new();
        for &start in spans {
            if !seen.insert(start) {
                continue;
            }
            let mut part = vec![start];
            let mut cursor = 0;
            while cursor < part.len() {
                let s = part[cursor];
                cursor += 1;
                for n in graph.orthogonal_neighbours(s) {
                    if n.saddle < level && set.contains(&n.span) && seen.insert(n.span) {
                        part.push(n.span);
                    }
                }
            }
            parts.push(part);
        }
        parts
    }
}

/// Whether a flood's lowest crest is an outlet below `level`: the basin
/// stands above where it spills.
pub fn stands_above_outlet(flood: &Flood, level: f32) -> Option<f32> {
    flood
        .crests
        .iter()
        .filter(|c| c.kind == CrestKind::Outlet && c.saddle < level)
        .map(|c| c.saddle)
        .reduce(f32::min)
}
