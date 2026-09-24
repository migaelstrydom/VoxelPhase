//! The topology builder: the only writer of the network's topology.
//!
//! ```text
//!   authored pools ──create_pool──┐
//!   terrain edits ──after_edit────┼──▶ stores, links, regions, span owners ──▶ solver
//!   between ticks ──settle────────┘    (every volume moved via the ledger)
//! ```
//!
//! It runs between solver ticks. The solver never changes topology, and
//! nothing but the builder writes span ownership.
//!
//! **Links are made lazily.** A basin's crests are grouped into outflows, and
//! an outflow is linked once the basin rises to within a few centimetres of
//! its lip: until then nothing crosses it, and a weir would carry zero. Its
//! target is resolved then: the store owning the far side, or for an outlet
//! the store its drainage path reaches (§7.4, instant routing), with an empty
//! basin made in a dry depression on the way. A link whose store is removed
//! by a merge, split or drying is dropped, and its outflow relinks the same
//! way. So every topology change heals by the same rule.

use std::fmt;

use rustc_hash::{FxHashMap, FxHashSet};

use crate::water::geometry::{
    Column, Drain, GeometryUpdate, SpanRef, SpanRemap, WaterGeometry, ORTHOGONAL,
};
use crate::water::ids::{LinkId, StoreId};
use crate::water::network::links::{Orifice, Weir};
use crate::water::network::{
    is_pothole, pit_bottom, Basin, CrestKind, DepressionFinder, Flood, FloodMode, HoleColumn,
    LinkEntry, LossLaw, Network, Outflow, Port, Store, POTHOLE_DEPTH, POTHOLE_VOLUME,
};
use crate::water::solver::{account, Account, VolumeLedger};

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

/// An outflow is linked once its basin is within this of the lip.
pub const LINK_MARGIN: f32 = 0.02;

/// A weir or orifice carrying less than this closes; it reopens when its
/// free discharge passes twice this (§7.3), m³/s.
pub const Q_RETIRE: f64 = 2e-3;

/// Steps a drainage walk takes before giving up on reaching a store.
const MAX_WALK: usize = 1 << 16;

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
#[derive(Debug)]
pub struct TopologyBuilder {
    /// The level's `drain_gain`, for every weir and orifice made.
    gain: f32,
    /// The sink every open world edge drains into, made on first use.
    void: Option<StoreId>,
    /// Every edit applied, oldest first, when recording is on.
    log: Option<Vec<TopologyEdit>>,
}

impl TopologyBuilder {
    pub fn new(gain: f32) -> Self {
        Self {
            gain,
            void: None,
            log: None,
        }
    }

    /// A builder that keeps a log of every edit, for tests and the harness.
    pub fn recording(gain: f32) -> Self {
        Self {
            log: Some(Vec::new()),
            ..Self::new(gain)
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
        let flood = self.flood(t, None, &[span], surface, FloodMode::Create);
        let basin = Basin::from_flood(flood, 0.0);
        let volume = basin.hypsometry.volume(surface);
        let id = self.add_basin(t, Basin { volume, ..basin });
        t.ledger.place(volume);
        Ok(id)
    }

    /// Bring the network up to date with a terrain edit: note floors blown
    /// through into something below, then re-flood every basin that owns a
    /// span in a re-paired column or the ring around it.
    pub fn after_terrain_update(&mut self, t: &mut Topology, update: &GeometryUpdate) {
        self.note_holes(t, update);
        let graph = t.geometry.graph();
        let mut touched: FxHashSet<StoreId> = FxHashSet::default();
        for column in &update.remap.columns {
            let ring = std::iter::once(*column)
                .chain(ORTHOGONAL.iter().map(|s| column.offset(s.di, s.dk)));
            for c in ring {
                for span in graph.refs(c) {
                    if let Some(body) = graph.owner(span).body {
                        touched.insert(body);
                    }
                }
            }
        }
        for entry in &update.remap.entries {
            if let Some(body) = entry.old_owner.body {
                touched.insert(body);
            }
        }
        let mut touched: Vec<StoreId> = touched.into_iter().collect();
        touched.sort();
        for basin in touched {
            self.reregion_after(t, basin, Some(&update.remap));
        }
    }

    /// Where several old spans landed in one new span, the floor between
    /// them was blown through (§8.3): the new span went to the owner of the
    /// lowest, and every basin that owned one above it has a hole there.
    fn note_holes(&self, t: &mut Topology, update: &GeometryUpdate) {
        let mut landed: FxHashMap<(Column, u8), Vec<&crate::water::geometry::RemapEntry>> =
            FxHashMap::default();
        for entry in &update.remap.entries {
            if let Some(new) = entry.new_ordinal {
                landed.entry((entry.column, new)).or_default().push(entry);
            }
        }
        let mut keys: Vec<(Column, u8)> = landed.keys().copied().collect();
        keys.sort();
        for key in keys {
            let entries = &landed[&key];
            let lowest = entries
                .iter()
                .min_by_key(|e| e.old_ordinal)
                .expect("at least one entry landed");
            for upper in entries
                .iter()
                .filter(|e| e.old_ordinal != lowest.old_ordinal)
            {
                let Some(body) = upper.old_owner.body else {
                    continue;
                };
                if lowest.old_owner.body == Some(body) {
                    continue;
                }
                if let Some(basin) = t.network.store_mut(body).and_then(Store::as_basin_mut) {
                    basin.add_hole(HoleColumn {
                        column: key.0,
                        lip: upper.old.floor_c,
                    });
                }
            }
        }
    }

    /// Re-flood a basin from its surviving wet spans at its current level,
    /// keeping its volume. Its outflows are regrouped and relinked lazily.
    pub fn reregion(&mut self, t: &mut Topology, id: StoreId) {
        self.reregion_after(t, id, None);
    }

    /// As [`Self::reregion`], after an edit described by `remap`: a span
    /// re-paired by it seeds the flood only if water stood over it before,
    /// so a crater blown into a dry bank does not fill from the lake beside
    /// it. The basin keeps its claim through the flood, so its own spans are
    /// its own and a dry depression it can now reach is not.
    fn reregion_after(&mut self, t: &mut Topology, id: StoreId, remap: Option<&SpanRemap>) {
        let Some(basin) = t.network.store(id).and_then(Store::as_basin) else {
            return;
        };
        let level = basin.level();
        let columns = basin.columns();
        let links: Vec<LinkId> = basin.links().collect();
        let seeds = self.surviving_seeds(t, id, &columns, level, remap);
        for link in links {
            self.remove_link(t, link);
        }
        if seeds.is_empty() {
            // Nothing under the water survived: the bed was blown away from
            // under it, into the void.
            let void = self.void(t);
            self.remove_basin(t, id, Account::Store(void));
            return;
        }
        let flood = self.flood(t, Some(id), &seeds, level, FloodMode::Reregion);
        self.release(t, id, &columns);
        let basin = t
            .network
            .store_mut(id)
            .and_then(Store::as_basin_mut)
            .expect("checked above");
        basin.reregion(flood);
        self.claim(t, id);
        self.record(TopologyEdit::Reregion { basin: id, seeds });
    }

    /// The between-tick housekeeping of every basin: links to outflows it has
    /// risen to, closing and reopening them, merges, splits, the loss gate,
    /// drying up, and re-flooding a basin that has risen to the top of its
    /// region.
    pub fn settle(&mut self, t: &mut Topology, loss: &LossLaw) {
        for id in t.network.store_ids() {
            if t.network.store(id).and_then(Store::as_basin).is_none() {
                continue;
            }
            self.link_outflows(t, id);
            self.close_or_open(t, id);
            if self.merge_with_neighbour(t, id) {
                continue;
            }

            let basin = t.network.store(id).and_then(Store::as_basin).expect("live");
            let level = basin.level();
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
            if basin.volume < DRIED_VOLUME && !self.fed(t, id) {
                let residual_to = basin
                    .outflows
                    .iter()
                    .filter_map(|o| o.target)
                    .find(|target| t.network.store(*target).is_some())
                    .map_or(Account::Lost, |target| account(t.network, target));
                self.remove_basin(t, id, residual_to);
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

    /// Link every outflow the basin has risen to, and forget links whose
    /// store has gone.
    fn link_outflows(&mut self, t: &mut Topology, id: StoreId) {
        let basin = t.network.store(id).and_then(Store::as_basin).expect("live");
        let level = basin.level();
        let count = basin.outflows.len();
        for index in 0..count {
            let basin = t.network.store(id).and_then(Store::as_basin).expect("live");
            let outflow = &basin.outflows[index];
            if let Some(link) = outflow.link {
                if t.network.link(link).is_some() {
                    continue;
                }
            }
            if level + LINK_MARGIN < outflow.lip {
                continue;
            }
            let outflow = outflow.clone();
            let Some(target) = self.resolve_target(t, id, &outflow) else {
                continue;
            };
            // Two basins across one ridge share one reversible weir: a
            // second, the other way, would carry the same water twice.
            let shared = t.network.links().find_map(|(link, e)| {
                (e.up == target && e.down == id && e.law.reversible()).then_some(link)
            });
            let link = match shared {
                Some(link) => link,
                None => self.add_link(t, id, target, &outflow),
            };
            if let Some(b) = t.network.store_mut(id).and_then(Store::as_basin_mut) {
                if let Some(o) = b.outflows.get_mut(index) {
                    o.link = Some(link);
                    o.target = Some(target);
                }
            }
        }
    }

    /// Close links whose flow has fallen below `Q_RETIRE`; reopen closed ones
    /// whose free flow has passed twice it.
    fn close_or_open(&mut self, t: &mut Topology, id: StoreId) {
        let links: Vec<LinkId> = t
            .network
            .store(id)
            .and_then(Store::as_basin)
            .map(|b| b.links().collect())
            .unwrap_or_default();
        for link in links {
            let Some(entry) = t.network.link(link) else {
                continue;
            };
            let (Some(up), Some(down)) = (
                t.network.view(
                    entry.up,
                    t.network.store(entry.up).map_or(0.0, Store::volume),
                    entry.up_port,
                ),
                t.network.view(
                    entry.down,
                    t.network.store(entry.down).map_or(0.0, Store::volume),
                    entry.down_port,
                ),
            ) else {
                continue;
            };
            let q = entry.law.discharge(up, down).abs();
            let open = if entry.open {
                q >= Q_RETIRE
            } else {
                q > 2.0 * Q_RETIRE
            };
            if open != entry.open {
                if let Some(e) = t.network.link_mut(link) {
                    e.open = open;
                }
                self.record(TopologyEdit::SetLinkOpen { link, open });
            }
        }
    }

    /// Merge with a basin across an outflow once both stand over its lip at
    /// the same level (§8.1). Returns whether the basin is gone.
    fn merge_with_neighbour(&mut self, t: &mut Topology, id: StoreId) -> bool {
        let basin = t.network.store(id).and_then(Store::as_basin).expect("live");
        let level = basin.level();
        let partner = basin.outflows.iter().find_map(|o| {
            let target = o.target?;
            let other = t.network.store(target)?.as_basin()?;
            let other_level = other.level();
            (target != id
                && level > o.lip
                && other_level > o.lip
                && (level - other_level).abs() < MERGE_LEVELS)
                .then_some(target)
        });
        match partner {
            Some(other) => {
                self.merge(t, id, other);
                true
            }
            None => false,
        }
    }

    /// Two basins standing at one level over the ridge between them become
    /// one: a new store over both regions, holding both volumes.
    pub fn merge(&mut self, t: &mut Topology, a: StoreId, b: StoreId) {
        let (Some(first), Some(second)) = (
            t.network.store(a).and_then(Store::as_basin),
            t.network.store(b).and_then(Store::as_basin),
        ) else {
            return;
        };
        let level = 0.5 * (first.level() + second.level());
        let mut seeds: Vec<SpanRef> = Vec::new();
        for basin in [first, second] {
            let l = basin.level();
            seeds.extend(
                basin
                    .region
                    .iter()
                    .filter(|r| r.shape.floor_min < l)
                    .map(|r| r.span),
            );
        }
        let mut holes = first.holes.clone();
        holes.extend(second.holes.iter().copied());
        let (columns_a, columns_b) = (first.columns(), second.columns());
        let (volume_a, volume_b) = (first.volume, second.volume);
        self.release(t, a, &columns_a);
        self.release(t, b, &columns_b);

        let flood = self.flood(t, None, &seeds, level, FloodMode::Create);
        let mut union = Basin::from_flood(flood, 0.0);
        for hole in holes {
            union.add_hole(hole);
        }
        let merged = self.add_basin(t, union);
        self.transfer(t, Account::Store(a), Account::Store(merged), volume_a);
        self.transfer(t, Account::Store(b), Account::Store(merged), volume_b);
        for gone in [a, b] {
            t.network.remove_store(gone);
            self.record(TopologyEdit::RemoveStore {
                store: gone,
                residual_to: Account::Store(merged),
            });
        }
        // The union's region claims what the two released; spans either of
        // them still held are taken over now.
        self.claim(t, merged);
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
        let holes = basin.holes.clone();
        let links: Vec<LinkId> = basin.links().collect();
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

        for link in links {
            self.remove_link(t, link);
        }
        self.release(t, id, &columns);
        let mut children: Vec<(Flood, f64)> = Vec::new();
        for part in parts {
            let flood = self.flood(t, None, &part, level, FloodMode::Create);
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
            let mut child = Basin::from_flood(flood, 0.0);
            for hole in &holes {
                child.add_hole(*hole);
            }
            let child = self.add_basin(t, child);
            self.transfer(t, Account::Store(id), Account::Store(child), share);
            remaining -= share;
            ids.push(child);
        }
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

    /// Where an outflow's water goes, making an empty basin in a dry
    /// depression if that is where it lands.
    fn resolve_target(
        &mut self,
        t: &mut Topology,
        from: StoreId,
        outflow: &Outflow,
    ) -> Option<StoreId> {
        let cell = outflow
            .cells
            .iter()
            .min_by(|a, b| a.saddle.total_cmp(&b.saddle))?;
        if cell.inside == cell.outside {
            return Some(self.void(t));
        }
        let graph = t.geometry.graph();
        if let Some(owner) = graph.owner(cell.outside).body {
            if owner != from && t.network.store(owner).is_some() {
                return Some(owner);
            }
        }
        match outflow.kind {
            CrestKind::Child { .. } if outflow.hole.is_none() => {
                Some(self.empty_basin(t, cell.outside))
            }
            _ => self.follow_drainage(t, from, cell.outside),
        }
    }

    /// Walk the drainage field from `start` to the first store it reaches:
    /// a basin's region, the void, or a dry depression made a basin.
    fn follow_drainage(
        &mut self,
        t: &mut Topology,
        from: StoreId,
        start: SpanRef,
    ) -> Option<StoreId> {
        let mut span = start;
        for _ in 0..MAX_WALK {
            let (graph, drainage) = t.geometry.routing();
            if let Some(owner) = graph.owner(span).body {
                if owner != from {
                    return Some(owner);
                }
            }
            let fill = drainage.fill(graph, span);
            let floor = graph.span(span).floor_min;
            if drainage.drain(graph, span) == Drain::Outlet {
                return Some(self.void(t));
            }
            if fill > floor + 1e-3 && (!fill.is_finite() || !is_pothole(graph, span, fill)) {
                return Some(self.empty_basin(t, span));
            }
            match drainage.downstream_resolved(graph, span) {
                Some(next) => span = next,
                None => return Some(self.void(t)),
            }
        }
        log::warn!("drainage walk from {start:?} found no store");
        None
    }

    /// An empty basin in the depression holding `span`, or the store that
    /// already has it.
    fn empty_basin(&mut self, t: &mut Topology, span: SpanRef) -> StoreId {
        let graph = t.geometry.graph();
        let bottom = pit_bottom(graph, span);
        if let Some(owner) = graph.owner(bottom).body {
            if t.network.store(owner).is_some() {
                return owner;
            }
        }
        let level = graph.span(bottom).floor_min;
        let flood = self.flood(t, None, &[bottom], level, FloodMode::Create);
        self.add_basin(t, Basin::from_flood(flood, 0.0))
    }

    /// The sink every open world edge drains into.
    fn void(&mut self, t: &mut Topology) -> StoreId {
        if let Some(id) = self.void.filter(|id| t.network.store(*id).is_some()) {
            return id;
        }
        let id = t.network.add_store(Store::Sink);
        self.record(TopologyEdit::AddStore(id));
        self.void = Some(id);
        id
    }

    fn add_link(
        &mut self,
        t: &mut Topology,
        up: StoreId,
        down: StoreId,
        outflow: &Outflow,
    ) -> LinkId {
        let down_is_basin = t
            .network
            .store(down)
            .is_some_and(|s| s.as_basin().is_some());
        let law: Box<dyn crate::water::network::Link> = match outflow.hole {
            Some(hole) => Box::new(Orifice {
                lip: hole.lip,
                area: hole.area,
                perimeter: hole.perimeter,
                gain: self.gain,
            }),
            None => Box::new(Weir::new(
                outflow.cells.iter().map(|c| c.saddle).collect(),
                self.gain,
                down_is_basin,
            )),
        };
        let volume = t.network.store(up).map_or(0.0, Store::volume);
        let down_volume = t.network.store(down).map_or(0.0, Store::volume);
        let open = match (
            t.network.view(up, volume, Port::Downstream),
            t.network.view(down, down_volume, Port::Upstream),
        ) {
            (Some(u), Some(d)) => law.discharge(u, d).abs() > 2.0 * Q_RETIRE,
            _ => false,
        };
        let id = t.network.add_link(LinkEntry {
            up,
            down,
            law,
            open,
            up_port: Port::Downstream,
            down_port: Port::Upstream,
        });
        self.record(TopologyEdit::AddLink(id));
        id
    }

    fn remove_link(&mut self, t: &mut Topology, link: LinkId) {
        if t.network.remove_link(link).is_some() {
            self.record(TopologyEdit::RemoveLink(link));
        }
    }

    /// Whether anything flows into a store.
    fn fed(&self, t: &Topology, id: StoreId) -> bool {
        t.network.links().any(|(_, l)| {
            l.open && l.down == id && {
                let up = t.network.store(l.up).map_or(0.0, Store::volume);
                let down = t.network.store(id).map_or(0.0, Store::volume);
                match (
                    t.network.view(l.up, up, l.up_port),
                    t.network.view(id, down, l.down_port),
                ) {
                    (Some(u), Some(d)) => l.law.discharge(u, d) > 0.0,
                    _ => false,
                }
            }
        })
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

    fn flood(
        &self,
        t: &Topology,
        store: Option<StoreId>,
        seeds: &[SpanRef],
        level: f32,
        mode: FloodMode,
    ) -> Flood {
        let graph = t.geometry.graph();
        let owner = |span: SpanRef| graph.owner(span).body;
        DepressionFinder {
            graph,
            drainage: t.geometry.drainage(),
            store,
            owner: &owner,
            mode,
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
    /// ownership, which the span remap carries across rebuilds. In a column
    /// the edit re-paired, a span counts only if an old span of the basin's
    /// that landed in it was under water.
    fn surviving_seeds(
        &self,
        t: &Topology,
        id: StoreId,
        columns: &[Column],
        level: f32,
        remap: Option<&SpanRemap>,
    ) -> Vec<SpanRef> {
        let mut wet_before: FxHashSet<(Column, u8)> = FxHashSet::default();
        let mut repaired: FxHashSet<Column> = FxHashSet::default();
        if let Some(remap) = remap {
            repaired.extend(remap.columns.iter().copied());
            for e in &remap.entries {
                if let Some(new) = e.new_ordinal {
                    if e.old_owner.body == Some(id) && e.old.floor_min < level {
                        wet_before.insert((e.column, new));
                    }
                }
            }
        }
        let graph = t.geometry.graph();
        let mut seeds = Vec::new();
        for &column in columns {
            for span in graph.refs(column) {
                if graph.owner(span).body != Some(id) || graph.span(span).floor_min >= level {
                    continue;
                }
                if repaired.contains(&column) && !wet_before.contains(&(column, span.ordinal)) {
                    continue;
                }
                seeds.push(span);
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
