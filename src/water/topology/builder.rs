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

use nalgebra::{Point3, Vector3};
use rustc_hash::{FxHashMap, FxHashSet};

use crate::water::geometry::{
    Column, Drain, GeometryUpdate, SpanGraph, SpanRef, SpanRemap, WaterGeometry, COLUMN_SIZE,
    ORTHOGONAL,
};
use crate::water::ids::{LinkId, StoreId};
use crate::water::network::links::{FixedRate, Orifice, ReachOutflow, Weir};
use crate::water::network::{
    critical_depth, is_pothole, pit_bottom, pit_bottom_within, BackSide, Basin, ChannelOutlet,
    CrestKind, DepressionFinder, FallPath, FallTracer, Flood, FloodMode, HoleColumn, Landing,
    LinkEntry, Lip, LossLaw, Network, Ocean, Outflow, Port, Reach, Store, Trace, GRAVITY,
    POTHOLE_DEPTH, POTHOLE_VOLUME, RATING_Q_MIN,
};
use crate::water::solver::{account, Account, VolumeLedger};

use super::edit::TopologyEdit;
use super::router::{
    build_reaches, channel_heads, downstream_of, reach_footprint, rescan, standing_in, walk,
    WalkEnd, WalkLip,
};

mod shoreline;

/// Head over the lip a channel's design discharge is taken at, m.
const DESIGN_HEAD: f32 = 0.3;

/// The least design discharge a channel is built for, m³/s.
const MIN_DESIGN_DISCHARGE: f64 = 0.5;

/// A receding reach this shallow, m, has dried.
const RETIRE_DEPTH: f32 = 0.005;

/// A basin within this of its flood's cap is re-flooded to find its higher
/// banks.
pub const CAP_MARGIN: f32 = 0.01;

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

/// The discharges a channel is laid for.
#[derive(Debug, Clone, Copy)]
struct ChannelFlow {
    /// What its reaches are rated to, with headroom, m³/s.
    design: f64,
    /// What flows into it as it is laid, m³/s.
    now: f64,
}

/// Depth over a crest where a weir's water leaves it: the critical depth at
/// the design head, m.
const CRITICAL_DEPTH: f32 = 2.0 / 3.0 * DESIGN_HEAD;

/// A basin whose region holds more spans than this is re-flooded a frame
/// after the edit that touched it, not on the edit's own frame (§9.2): the
/// edit's frame already carries the terrain rebuild. A re-flood costs about
/// 0.23 µs a span, so this is about 1.8 ms.
pub const VALVE_SPANS: usize = 8000;

/// Falls chained through landings before the chain is given up to the void.
const MAX_FALLS: u32 = 256;

/// A channel outlet's point is this far above the floor it names, m, so that
/// finding its span again reads the air over the floor, not the rock under.
const OUTLET_LIFT: f32 = 0.05;

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

/// A spring or sky source: a reservoir giving a fixed discharge from a
/// point in space (§7.6). It is anchored there, so an edit moves where its
/// water lands, never where it comes from.
#[derive(Debug, Clone)]
pub struct Source {
    pub reservoir: StoreId,
    pub position: Point3<f32>,
    /// Launch velocity, m/s.
    pub velocity: Vector3<f32>,
    /// m³/s.
    pub discharge: f64,
    /// The link carrying its water, once traced and landed.
    pub link: Option<LinkId>,
    /// Sealed in rock: never traced again.
    pub buried: bool,
}

/// Where water entering the network goes: the first store it reaches, and
/// the arc it falls along on the way, if it falls.
struct Entry {
    store: StoreId,
    fall: Option<FallPath>,
    /// Where the water leaves, when that is not the lip of the link it
    /// enters by: a channel too short for a reach falls from the far side
    /// of its one cell, not from the crest before it.
    lip: Option<Lip>,
}

/// The only writer of topology.
#[derive(Debug)]
pub struct TopologyBuilder {
    /// The level's `drain_gain`, for every weir and orifice made.
    gain: f32,
    /// The sink every closed world edge drains into, made on first use.
    void: Option<StoreId>,
    /// The sea, if the level has one.
    ocean: Option<StoreId>,
    /// Every edit applied, oldest first, when recording is on.
    log: Option<Vec<TopologyEdit>>,
    /// Edits applied so far, recorded or not: how a caller tells whether a
    /// pass changed anything.
    edits: u64,
    sources: Vec<Source>,
    /// Basins frozen for a re-flood the valve deferred, and the edit's remap
    /// they re-flood after.
    deferred: Vec<StoreId>,
    deferred_remap: Option<SpanRemap>,
}

impl TopologyBuilder {
    pub fn new(gain: f32) -> Self {
        Self {
            gain,
            void: None,
            ocean: None,
            log: None,
            edits: 0,
            sources: Vec::new(),
            deferred: Vec::new(),
            deferred_remap: None,
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

    /// How many edits have been applied.
    pub fn edit_count(&self) -> u64 {
        self.edits
    }

    fn record(&mut self, edit: TopologyEdit) {
        self.edits += 1;
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

    /// Place the sea at `level`: a store claiming every span below it that the
    /// open edges reach, flooded once, here (§12).
    pub fn create_ocean(&mut self, t: &mut Topology, level: f32, swell: f32) -> StoreId {
        let id = t.network.add_store(Store::Ocean(Ocean {
            level,
            swell,
            region_version: 0,
        }));
        self.record(TopologyEdit::AddStore(id));
        self.ocean = Some(id);
        let seeds: Vec<SpanRef> = {
            let geometry = &*t.geometry;
            let (graph, outlets) = (geometry.graph(), geometry.drainage().outlets());
            let (min, max) = graph.column_bounds();
            let mut border = Vec::new();
            for k in min.k..=max.k {
                for i in min.i..=max.i {
                    let column = Column::new(i, k);
                    if outlets.on_sea_edge(graph, column) {
                        border.extend(
                            graph
                                .refs(column)
                                .filter(|s| graph.span(*s).floor_min < level),
                        );
                    }
                }
            }
            border
        };
        self.claim_below(t, id, seeds, level);
        id
    }

    /// The sea, if the level has one.
    pub fn ocean(&self) -> Option<StoreId> {
        self.ocean
    }

    /// Claim for `id` every unowned span joined to `seeds` below `level`.
    fn claim_below(&mut self, t: &mut Topology, id: StoreId, seeds: Vec<SpanRef>, level: f32) {
        let graph = t.geometry.graph_mut();
        let mut queue: Vec<SpanRef> = Vec::new();
        for seed in seeds {
            let owner = graph.owner_mut(seed);
            if owner.body.is_none() {
                owner.body = Some(id);
                queue.push(seed);
            }
        }
        while let Some(span) = queue.pop() {
            for n in graph.orthogonal_neighbours(span) {
                if n.saddle >= level {
                    continue;
                }
                let owner = graph.owner_mut(n.span);
                if owner.body.is_none() {
                    owner.body = Some(id);
                    queue.push(n.span);
                }
            }
        }
    }

    /// A lowland that has filled to sea level over its weir joins the sea:
    /// its spans below sea level become the sea's, and its water the
    /// ocean's.
    fn absorb_into_ocean(&mut self, t: &mut Topology, id: StoreId, ocean: StoreId) {
        let Some(basin) = t.network.store(id).and_then(Store::as_basin) else {
            return;
        };
        let columns = basin.columns();
        let volume = basin.volume;
        let level = t
            .network
            .store(ocean)
            .and_then(Store::surface)
            .unwrap_or(0.0);
        self.transfer(t, Account::Store(id), Account::Ocean, volume);
        let graph = t.geometry.graph_mut();
        for column in columns {
            let refs: Vec<SpanRef> = graph.refs(column).collect();
            for span in refs {
                let below = graph.span(span).floor_min < level;
                let owner = graph.owner_mut(span);
                if owner.body == Some(id) {
                    owner.body = below.then_some(ocean);
                }
            }
        }
        t.network.remove_store(id);
        self.record(TopologyEdit::RemoveStore {
            store: id,
            residual_to: Account::Ocean,
        });
        if let Some(o) = t.network.store_mut(ocean).and_then(Store::as_ocean_mut) {
            o.region_version = o.region_version.wrapping_add(1);
        }
    }

    /// Spans an edit has newly joined to the sea below sea level (a breached
    /// sea wall) become a lowland basin, empty if dry, whose crest into the
    /// sea links at once: the sea floods it over the weir, and it joins the
    /// sea once it reaches sea level (§9.1, step 5). The sea never claims
    /// spans at once.
    fn connect_lowlands(&mut self, t: &mut Topology, update: &GeometryUpdate) {
        let Some(ocean) = self.ocean else {
            return;
        };
        let Some(level) = t.network.store(ocean).and_then(Store::surface) else {
            return;
        };
        let candidates: Vec<SpanRef> = {
            let graph = t.geometry.graph();
            update
                .remap
                .columns
                .iter()
                .flat_map(|c| graph.refs(*c))
                .filter(|s| {
                    graph.owner(*s).body.is_none()
                        && graph.span(*s).floor_min < level
                        && graph
                            .orthogonal_neighbours(*s)
                            .iter()
                            .any(|n| n.saddle < level && graph.owner(n.span).body == Some(ocean))
                })
                .collect()
        };
        for span in candidates {
            let graph = t.geometry.graph();
            let bottom = pit_bottom_within(graph, span, |s| graph.owner(s).body.is_none());
            if graph.owner(bottom).body.is_none() {
                self.basin_at_bottom(t, bottom);
            }
        }
    }

    /// Bring the network up to date with a terrain edit: note floors blown
    /// through into something below, then re-flood every basin that owns a
    /// span in a re-paired column or the ring around it.
    pub fn after_terrain_update(&mut self, t: &mut Topology, update: &GeometryUpdate) {
        self.note_holes(t, update);
        self.reroute_touched(t, update);
        self.retrace_touched(t, update);
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
            let large = t
                .network
                .store(basin)
                .and_then(Store::as_basin)
                .is_some_and(|b| b.region.len() > VALVE_SPANS);
            if large {
                self.freeze(t, basin);
            } else {
                self.reregion_after(t, basin, Some(&update.remap));
            }
        }
        if !self.deferred.is_empty() {
            self.deferred_remap = Some(update.remap.clone());
        }
        // Last: a lowland made now is new, not a basin the edit touched.
        self.connect_lowlands(t, update);
    }

    /// Hold a basin still until its deferred re-flood runs.
    fn freeze(&mut self, t: &mut Topology, id: StoreId) {
        if let Some(basin) = t.network.store_mut(id).and_then(Store::as_basin_mut) {
            basin.frozen = true;
            self.deferred.push(id);
        }
    }

    /// Whether a re-flood is waiting on the valve.
    pub fn has_deferred(&self) -> bool {
        !self.deferred.is_empty()
    }

    /// Run the re-floods the valve deferred, against the edit they followed.
    /// Must run before the geometry takes another edit.
    pub fn run_deferred(&mut self, t: &mut Topology) {
        let remap = self.deferred_remap.take();
        for id in std::mem::take(&mut self.deferred) {
            if let Some(basin) = t.network.store_mut(id).and_then(Store::as_basin_mut) {
                basin.frozen = false;
                self.reregion_after(t, id, remap.as_ref());
            }
        }
    }

    /// Where several old spans landed in one new span, the floor between
    /// them was blown through (§8.3): the new span went to the owner of the
    /// lowest, and every basin that owned one above it has a hole there. A
    /// closed sliver blew nothing through, and is lowest only if nothing
    /// resting landed.
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
                .min_by_key(|e| (!e.rests, e.old_ordinal))
                .expect("at least one entry landed");
            for upper in entries
                .iter()
                .filter(|e| e.rests && e.old_ordinal != lowest.old_ordinal)
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
            // under it. The water runs on to wherever the ground there now
            // drains.
            let to = self
                .runoff(t, id, &columns, level)
                .map_or(Account::Sunk, |s| account(t.network, s));
            self.remove_basin(t, id, to);
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
        self.settle_reaches(t, loss);
        self.link_sources(t);
        for id in t.network.store_ids() {
            let settles = t
                .network
                .store(id)
                .and_then(Store::as_basin)
                .is_some_and(|b| !b.frozen);
            if !settles {
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

    /// Lay a channel from an outlet down the drainage field to the store it
    /// reaches. Returns where the outlet's weir feeds: the first reach, or
    /// for a path too short to be a channel the target itself, with the arc
    /// the water falls along if it leaves the lip at a riser.
    fn route(
        &mut self,
        t: &mut Topology,
        from: StoreId,
        outflow: &Outflow,
        level: f32,
    ) -> Option<Entry> {
        let cell = outflow
            .cells
            .iter()
            .min_by(|a, b| a.saddle.total_cmp(&b.saddle))?;
        let weir = Weir::new(
            outflow.cells.iter().map(|c| c.saddle).collect(),
            self.gain,
            false,
        );
        let flow = ChannelFlow {
            design: weir
                .free((level.max(outflow.lip) + DESIGN_HEAD) as f64)
                .0
                .max(MIN_DESIGN_DISCHARGE),
            now: weir.free(level.max(outflow.lip) as f64).0,
        };
        let lip = WalkLip {
            inside: cell.inside,
            height: cell.saddle,
        };
        self.channel(t, Some(from), Some(lip), cell.outside, flow, 0)
    }

    /// Lay a channel of reaches from `start` down the drainage field, and
    /// link it to wherever it ends: a store, the void, an empty basin in a
    /// dry depression, or over a fall to wherever that lands. `falls` counts
    /// the falls already chained above this channel.
    fn channel(
        &mut self,
        t: &mut Topology,
        from: Option<StoreId>,
        lip: Option<WalkLip>,
        start: SpanRef,
        flow: ChannelFlow,
        falls: u32,
    ) -> Option<Entry> {
        let (cells, end) = {
            let network = &*t.network;
            let levels = |s: StoreId| network.store(s).and_then(Store::surface);
            let (graph, drainage) = t.geometry.routing();
            walk(graph, drainage, network, &levels, from, lip, start)
        };
        let heads: Vec<StoreId> = from.into_iter().collect();
        let reaches = {
            let graph = t.geometry.graph();
            let standing = standing_in(graph, t.network, &heads);
            build_reaches(graph, &cells, flow.design, &standing)
        };
        // A fall leaves from the surface the channel runs at now, which is
        // where its last reach is drawn, not at the design discharge.
        let (speed, depth) =
            reaches
                .last()
                .map_or(((GRAVITY * CRITICAL_DEPTH).sqrt(), CRITICAL_DEPTH), |r| {
                    let at = r.rating.at(flow.now.max(RATING_Q_MIN));
                    (at.velocity, at.depth)
                });
        // Where the channel's water leaves it: the edge of its last cell, or
        // of the crest it left when it has none.
        let last = cells
            .last()
            .map(|c| (c.column, t.geometry.graph().span(*c).floor_c))
            .or(lip.map(|l| (l.inside.column, l.height)));
        // A channel ending in a pit or off the world's edge holds the span it
        // ends at as its last cell: there is no edge to cross, and its lip is
        // its last reach's own end.
        let end_lip = |toward: SpanRef| {
            last.filter(|(column, _)| *column != toward.column)
                .map(|(column, height)| Lip::across(column, toward.column, height))
        };
        let width = reaches
            .last()
            .map_or(COLUMN_SIZE, |r| r.rating.at(r.rating.design()).top_width);
        let thickness = critical_depth(flow.design, width);
        let (target, outlet, lip) = match end {
            WalkEnd::Store(store, span) => {
                let lip = end_lip(span);
                let arc =
                    lip.and_then(|l| self.arc(t, from, &l, l.height() + depth, speed, thickness));
                (store, Some(outlet_at(t.geometry.graph(), span, arc)), lip)
            }
            WalkEnd::Depression(span) => {
                let lip = end_lip(span);
                let arc =
                    lip.and_then(|l| self.arc(t, from, &l, l.height() + depth, speed, thickness));
                let at = outlet_at(t.geometry.graph(), span, arc);
                (self.empty_basin(t, span), Some(at), lip)
            }
            WalkEnd::Void(span) => (self.edge_store(t, span), None, end_lip(span)),
            WalkEnd::Lost => (self.void(t), None, None),
            WalkEnd::Fall { lip: edge, toward } => {
                let base = match (cells.is_empty(), lip) {
                    (true, Some(l)) => l.height,
                    _ => t.geometry.graph().span(edge).floor_c,
                };
                let fall_lip = Lip::across(edge.column, toward.column, base);
                let (launch, velocity) = fall_lip.launch(base + depth, speed);
                let entry = self.fall(t, from, launch, velocity, flow, falls + 1)?;
                let at = entry
                    .fall
                    .as_ref()
                    .and_then(FallPath::landing)
                    .unwrap_or(launch);
                (
                    entry.store,
                    Some(ChannelOutlet {
                        at,
                        fall: entry.fall,
                    }),
                    Some(fall_lip),
                )
            }
        };
        if reaches.is_empty() {
            return Some(Entry {
                store: target,
                fall: outlet.and_then(|o| o.fall),
                lip,
            });
        }
        let ids: Vec<StoreId> = reaches
            .into_iter()
            .map(|reach| {
                let id = t.network.add_store(Store::Reach(reach));
                self.record(TopologyEdit::AddStore(id));
                self.claim_reach(t, id);
                id
            })
            .collect();
        let fall = outlet.as_ref().and_then(|o| o.fall.clone());
        if let Some(last) = ids
            .last()
            .and_then(|id| t.network.store_mut(*id))
            .and_then(Store::as_reach_mut)
        {
            last.outlet = outlet;
        }
        for (i, &id) in ids.iter().enumerate() {
            let last = i + 1 == ids.len();
            let down = if last { target } else { ids[i + 1] };
            let (lip, fall) = if last {
                (lip, fall.clone())
            } else {
                (None, None)
            };
            self.link_reach(t, id, down, lip, fall);
        }
        Some(Entry {
            store: ids[0],
            fall: None,
            lip: None,
        })
    }

    /// Link a reach to the store below it. Its lip is the edge of its last
    /// column, where the channel was cut, or else the end of its centreline.
    fn link_reach(
        &mut self,
        t: &mut Topology,
        reach: StoreId,
        down: StoreId,
        lip: Option<Lip>,
        fall: Option<FallPath>,
    ) {
        let Some(lip) = lip.or_else(|| {
            t.network
                .store(reach)
                .and_then(Store::as_reach)
                .map(Reach::end_lip)
        }) else {
            return;
        };
        let link = t.network.add_link(LinkEntry {
            up: reach,
            down,
            law: Box::new(ReachOutflow),
            open: true,
            up_port: Port::Downstream,
            down_port: Port::Upstream,
            lip,
            fall,
            back: None,
        });
        self.record(TopologyEdit::AddLink(link));
    }

    /// Trace water launched at `at` and find the store it comes down in:
    /// the first whose water stands in its way (§7.9), or else a channel
    /// laid on from where it lands. `None` when `at` is sealed in rock.
    fn fall(
        &mut self,
        t: &mut Topology,
        from: Option<StoreId>,
        at: Point3<f32>,
        velocity: Vector3<f32>,
        flow: ChannelFlow,
        falls: u32,
    ) -> Option<Entry> {
        let trace = self.trace(t, from, at, velocity)?;
        let caught = trace
            .caught
            .and_then(|(span, y)| holder(t.geometry.graph(), t.network, from, span, y));
        let mut path = trace.path;
        let store = match (caught, trace.landing) {
            (Some(store), _) => store,
            (None, Landing::Void) => {
                let off = path.landing().unwrap_or(at);
                self.edge_store_at(t, off)
            }
            (None, Landing::Span(_)) if falls > MAX_FALLS => {
                log::warn!("water falling from {at:?}: more than {MAX_FALLS} falls in a chain");
                self.void(t)
            }
            (None, Landing::Span(span)) => {
                let entry = self.channel(t, from, None, span, flow, falls)?;
                if let Some(more) = entry.fall {
                    path.extend(more);
                }
                entry.store
            }
        };
        Some(Entry {
            store,
            fall: Some(path),
            lip: None,
        })
    }

    /// The arc water leaving `lip` at `surface` with `speed` follows, if a
    /// jet `thickness` deep parts from the ground there (§7.9).
    fn arc(
        &self,
        t: &Topology,
        from: Option<StoreId>,
        lip: &Lip,
        surface: f32,
        speed: f32,
        thickness: f32,
    ) -> Option<FallPath> {
        if !lip.separates(t.geometry.graph(), thickness) {
            return None;
        }
        let (at, velocity) = lip.launch(surface, speed);
        self.trace(t, from, at, velocity).map(|trace| trace.path)
    }

    /// Where an outflow's water leaves its basin: the middle of a hole, or
    /// the crest's lowest cell, facing out.
    fn outflow_lip(&self, outflow: &Outflow) -> Option<Lip> {
        if let Some(hole) = outflow.hole {
            let n = outflow.cells.len() as f32;
            let (x, z) = outflow.cells.iter().fold((0.0, 0.0), |(x, z), c| {
                let (cx, cz) = c.outside.column.centre();
                (x + cx / n, z + cz / n)
            });
            return (n > 0.0).then(|| Lip::down(Point3::new(x, hole.lip - OUTLET_LIFT, z)));
        }
        let cell = outflow
            .cells
            .iter()
            .min_by(|a, b| a.saddle.total_cmp(&b.saddle))?;
        Some(Lip::across(
            cell.inside.column,
            cell.outside.column,
            cell.saddle,
        ))
    }

    /// The arc an outflow's water leaves its crest or hole along, from
    /// `from`'s side: launched at the critical depth and speed of the
    /// design head over the crest, straight down through a hole.
    fn outflow_arc(
        &self,
        t: &Topology,
        from: StoreId,
        outflow: &Outflow,
        lip: &Lip,
    ) -> Option<FallPath> {
        let width = outflow.cells.len() as f32 * COLUMN_SIZE;
        let design = Weir::new(
            outflow.cells.iter().map(|c| c.saddle).collect(),
            self.gain,
            false,
        )
        .free((outflow.lip + DESIGN_HEAD) as f64)
        .0;
        let speed = if outflow.hole.is_some() {
            0.0
        } else {
            (GRAVITY * CRITICAL_DEPTH).sqrt()
        };
        self.arc(
            t,
            Some(from),
            lip,
            lip.height() + CRITICAL_DEPTH,
            speed,
            critical_depth(design, width),
        )
    }

    /// Sweep a fall arc to the ground, noting where a store other than
    /// `from` could first catch it.
    fn trace(
        &self,
        t: &Topology,
        from: Option<StoreId>,
        at: Point3<f32>,
        velocity: Vector3<f32>,
    ) -> Option<Trace> {
        let graph = t.geometry.graph();
        let network = &*t.network;
        let holds = |span: SpanRef, y: f32| holder(graph, network, from, span, y).is_some();
        FallTracer {
            graph,
            holds: &holds,
        }
        .trace(at, velocity)
    }

    /// The store other than `except` whose water stands over `span`, if any.
    fn standing_water(&self, t: &Topology, span: SpanRef, except: StoreId) -> Option<StoreId> {
        let graph = t.geometry.graph();
        let owner = graph.owner(span);
        if let Some((reach, _)) = owner
            .reach
            .filter(|(r, _)| *r != except && t.network.store(*r).is_some())
        {
            return Some(reach);
        }
        owner
            .body
            .filter(|_| surface_over(graph, t.network, span).is_some())
    }

    /// Place a spring or sky source: a reservoir giving `discharge` from
    /// `position`, launched at `velocity`, linked to wherever it lands.
    pub fn create_source(
        &mut self,
        t: &mut Topology,
        position: Point3<f32>,
        velocity: Vector3<f32>,
        discharge: f64,
    ) -> StoreId {
        let reservoir = t.network.add_store(Store::Reservoir);
        self.record(TopologyEdit::AddStore(reservoir));
        self.sources.push(Source {
            reservoir,
            position,
            velocity,
            discharge,
            link: None,
            buried: false,
        });
        self.link_source(t, self.sources.len() - 1);
        reservoir
    }

    /// The sources placed, in the order they were.
    pub fn sources(&self) -> &[Source] {
        &self.sources
    }

    /// Link every source whose link has gone: its landing store was
    /// replaced, or an edit touched its arc.
    fn link_sources(&mut self, t: &mut Topology) {
        for index in 0..self.sources.len() {
            let source = &self.sources[index];
            let linked = source.link.is_some_and(|l| t.network.link(l).is_some());
            if !source.buried && !linked {
                self.link_source(t, index);
            }
        }
    }

    fn link_source(&mut self, t: &mut Topology, index: usize) {
        let source = self.sources[index].clone();
        let flow = ChannelFlow {
            design: (2.0 * source.discharge).max(MIN_DESIGN_DISCHARGE),
            now: source.discharge,
        };
        let Some(entry) = self.fall(t, None, source.position, source.velocity, flow, 0) else {
            log::warn!(
                "water source at {:?} is sealed in rock: nothing flows",
                source.position
            );
            self.sources[index].buried = true;
            return;
        };
        let link = t.network.add_link(LinkEntry {
            up: source.reservoir,
            down: entry.store,
            law: Box::new(FixedRate {
                discharge: source.discharge,
            }),
            open: true,
            up_port: Port::Downstream,
            down_port: Port::Upstream,
            lip: Lip {
                at: source.position,
                direction: nalgebra::Vector2::new(source.velocity.x, source.velocity.z)
                    .try_normalize(1e-6)
                    .unwrap_or_else(nalgebra::Vector2::zeros),
            },
            fall: entry.fall,
            back: None,
        });
        self.record(TopologyEdit::AddLink(link));
        self.sources[index].link = Some(link);
    }

    /// Link the last reach of a channel again once the store it fed has been
    /// replaced (merged, split): to whatever stands at its outlet now.
    fn relink_channel_end(&mut self, t: &mut Topology, id: StoreId) {
        let Some(reach) = t.network.store(id).and_then(Store::as_reach) else {
            return;
        };
        if downstream_of(t.network, id).is_some() {
            return;
        }
        let Some(outlet) = reach.outlet.clone() else {
            let void = self.void(t);
            self.link_reach(t, id, void, None, None);
            return;
        };
        let column = Column::containing(outlet.at.x, outlet.at.z);
        let span = t
            .geometry
            .graph()
            .span_at(column, outlet.at.y + OUTLET_LIFT);
        // A channel ending in a dry pit claims the pit's first span as its
        // last cell, so a walk from there finds the channel itself: the pit
        // gets an empty basin again.
        let target = match span {
            None => Some(self.void(t)),
            Some(span) => match self.standing_water(t, span, id) {
                Some(store) => Some(store),
                None if self.in_depression(t, span) => Some(self.empty_basin(t, span)),
                None => {
                    // The water it ran into has gone from its shore: it
                    // carries on over the bed left dry (§8.2).
                    self.expose(t, id, span, None);
                    return;
                }
            },
        };
        match target.filter(|d| *d != id) {
            Some(down) => self.link_reach(t, id, down, None, outlet.fall),
            None => self.remove_channel(t, id),
        }
    }

    /// An edit across a fall's arc, in its air above the water it enters,
    /// may move where it lands (§7.9). The arc is traced again from its
    /// lip. Where it is still caught by the same store, only the arc
    /// changes; where not, the link is made again to the new store and the
    /// channel it fed recedes. The channels above keep their reaches.
    fn retrace_touched(&mut self, t: &mut Topology, update: &GeometryUpdate) {
        let mut columns = update.remap.columns.clone();
        columns.sort_unstable();
        columns.dedup();
        let touched: Vec<LinkId> = t
            .network
            .links()
            .filter(|(_, l)| {
                let above = t
                    .network
                    .store(l.down)
                    .and_then(Store::surface)
                    .unwrap_or(f32::NEG_INFINITY);
                l.fall.as_ref().is_some_and(|f| f.crosses(&columns, above))
            })
            .map(|(id, _)| id)
            .collect();
        for link in touched {
            self.retrace(t, link);
        }
    }

    /// Trace a link's arc again from its first point.
    fn retrace(&mut self, t: &mut Topology, link: LinkId) {
        let Some(entry) = t.network.link(link) else {
            return;
        };
        let (up, down) = (entry.up, entry.down);
        let Some((at, velocity)) = entry
            .fall
            .as_ref()
            .and_then(|f| Some((*f.points.first()?, f.velocity)))
        else {
            return;
        };
        let up_reach = t.network.store(up).and_then(Store::as_reach);
        let flow = ChannelFlow {
            design: up_reach.map_or(MIN_DESIGN_DISCHARGE, |r| r.rating.design()),
            now: up_reach.map_or(0.0, |r| r.outflow),
        };
        let from = t.network.store(up).and_then(Store::as_basin).map(|_| up);
        let Some(landed) = self.fall(t, from, at, velocity, flow, 0) else {
            self.remove_link(t, link);
            return;
        };
        if landed.store == down {
            if let Some(e) = t.network.link_mut(link) {
                e.fall = landed.fall;
            }
            return;
        }
        let is_reach = t.network.store(up).and_then(Store::as_reach).is_some();
        let lip = t.network.link(link).map(|e| e.lip);
        self.remove_link(t, link);
        if is_reach {
            let outlet = ChannelOutlet {
                at: landed
                    .fall
                    .as_ref()
                    .and_then(FallPath::landing)
                    .unwrap_or(at),
                fall: landed.fall.clone(),
            };
            if let Some(r) = t.network.store_mut(up).and_then(Store::as_reach_mut) {
                r.outlet = Some(outlet);
            }
            self.link_reach(t, up, landed.store, lip, landed.fall);
        }
        // A basin's outflow and a source relink by their own rules next settle.
    }

    /// Mark the spans a reach's water covers at its design discharge, each
    /// with the reach cell nearest it.
    fn claim_reach(&self, t: &mut Topology, id: StoreId) {
        let Some(reach) = t.network.store(id).and_then(Store::as_reach) else {
            return;
        };
        let footprint = reach_footprint(t.geometry.graph(), reach);
        let mut claimed: Vec<Column> = Vec::new();
        let mut best: FxHashMap<SpanRef, f32> = FxHashMap::default();
        let graph = t.geometry.graph_mut();
        for (span, cell, offset) in footprint {
            let owner = graph.owner_mut(span);
            let nearer = best.get(&span).is_none_or(|d| offset < *d);
            if owner.reach.is_none() || (nearer && owner.reach.is_some_and(|(r, _)| r == id)) {
                owner.reach = Some((id, cell));
                best.insert(span, offset);
                claimed.push(span.column);
            }
        }
        claimed.sort_unstable();
        claimed.dedup();
        if let Some(reach) = t.network.store_mut(id).and_then(Store::as_reach_mut) {
            reach.claimed = claimed;
        }
    }

    fn release_reach(&self, t: &mut Topology, id: StoreId, columns: &[Column]) {
        let graph = t.geometry.graph_mut();
        for &column in columns {
            let refs: Vec<SpanRef> = graph.refs(column).collect();
            for span in refs {
                let owner = graph.owner_mut(span);
                if owner.reach.is_some_and(|(r, _)| r == id) {
                    owner.reach = None;
                }
            }
        }
    }

    /// Move every channel's shoreline to where the water stands (§8.2),
    /// relink channel ends whose store was replaced, retire reaches whose
    /// water has run out, set their loss gate, and rescan any whose flow has
    /// outgrown its rating.
    fn settle_reaches(&mut self, t: &mut Topology, loss: &LossLaw) {
        self.move_shorelines(t);
        for id in t.network.store_ids() {
            self.relink_channel_end(t, id);
        }
        for id in t.network.store_ids() {
            let Some(reach) = t.network.store(id).and_then(Store::as_reach) else {
                continue;
            };
            // A channel dries from the top: a reach retires only once
            // nothing flows into it.
            let retire = reach.state == crate::water::network::ReachState::Receding
                && (reach.wetted() <= 0.01
                    || reach.mean_depth() < RETIRE_DEPTH
                    || reach.outflow < Q_RETIRE)
                && !self.fed(t, id);
            if retire {
                self.remove_reach(t, id);
                continue;
            }
            let minor = loss.reach_minor(reach.inflow, reach.minor);
            let outgrown = reach.inflow > reach.rating.design() * 1.05;
            if outgrown {
                let rating = {
                    let heads = channel_heads(t.network, id);
                    let graph = t.geometry.graph();
                    let standing = standing_in(graph, t.network, &heads);
                    rescan(graph, reach, reach.inflow * 2.0, &standing)
                };
                if let Some(r) = t.network.store_mut(id).and_then(Store::as_reach_mut) {
                    r.rating = rating;
                    r.version = r.version.wrapping_add(1);
                }
            }
            if let Some(r) = t.network.store_mut(id).and_then(Store::as_reach_mut) {
                if r.minor != minor {
                    r.minor = minor;
                    self.record(TopologyEdit::SetMinor { store: id, minor });
                }
            }
        }
    }

    /// Remove a reach, its water going to the store downstream of it.
    fn remove_reach(&mut self, t: &mut Topology, id: StoreId) {
        let Some(reach) = t.network.store(id).and_then(Store::as_reach) else {
            return;
        };
        let columns = reach.claimed.clone();
        let storage = reach.storage;
        let down = downstream_of(t.network, id);
        let residual_to = match down {
            Some(d) if t.network.store(d).is_some() => account(t.network, d),
            _ => Account::Sunk,
        };
        self.transfer(t, Account::Store(id), residual_to, storage);
        self.release_reach(t, id, &columns);
        t.network.remove_store(id);
        self.record(TopologyEdit::RemoveStore {
            store: id,
            residual_to,
        });
    }

    /// An edit across a channel re-routes it: each reach the edit touched is
    /// cut out with the channel above it, their water going downstream, and
    /// the outflow that fed them relinks and lays a new channel over the new
    /// ground.
    fn reroute_touched(&mut self, t: &mut Topology, update: &GeometryUpdate) {
        let mut touched: FxHashSet<StoreId> = FxHashSet::default();
        let graph = t.geometry.graph();
        for column in &update.remap.columns {
            for span in graph.refs(*column) {
                if let Some((reach, _)) = graph.owner(span).reach {
                    touched.insert(reach);
                }
            }
        }
        for entry in &update.remap.entries {
            if let Some((reach, _)) = entry.old_owner.reach {
                touched.insert(reach);
            }
        }
        let mut touched: Vec<StoreId> = touched.into_iter().collect();
        touched.sort();
        for id in touched {
            self.cut_channel(t, id);
        }
    }

    /// Cut a channel at a reach an edit touched: remove it and every reach
    /// above it, upstream first, so each one's water lands in the reach below
    /// and the cut reach's in whatever survives beneath it. The reaches below
    /// keep their water; with nothing feeding them they recede and retire
    /// (§8.2), unless the channel laid again joins them.
    fn cut_channel(&mut self, t: &mut Topology, at: StoreId) {
        if t.network.store(at).and_then(Store::as_reach).is_none() {
            return;
        }
        let mut above: Vec<StoreId> = vec![at];
        let mut cursor = 0;
        while cursor < above.len() {
            let id = above[cursor];
            cursor += 1;
            let feeders: Vec<StoreId> = t
                .network
                .links()
                .filter(|(_, l)| l.down == id)
                .map(|(_, l)| l.up)
                .filter(|up| t.network.store(*up).and_then(Store::as_reach).is_some())
                .collect();
            for up in feeders {
                if !above.contains(&up) {
                    above.push(up);
                }
            }
        }
        // `above` runs downstream to upstream.
        for id in above.into_iter().rev() {
            self.remove_reach(t, id);
        }
    }

    /// Remove a reach and every reach joined to it, up or down: the channel
    /// the outflow that fed it will lay again.
    fn remove_channel(&mut self, t: &mut Topology, start: StoreId) {
        if t.network.store(start).and_then(Store::as_reach).is_none() {
            return;
        }
        let mut channel: Vec<StoreId> = vec![start];
        let mut seen: FxHashSet<StoreId> = channel.iter().copied().collect();
        let mut cursor = 0;
        while cursor < channel.len() {
            let id = channel[cursor];
            cursor += 1;
            for (_, link) in t.network.links() {
                for other in [link.up, link.down] {
                    let joined = (link.up == id || link.down == id)
                        && t.network
                            .store(other)
                            .is_some_and(|s| s.as_reach().is_some());
                    if joined && seen.insert(other) {
                        channel.push(other);
                    }
                }
            }
        }
        // Upstream first, so each reach's water lands in the reach below,
        // still there, and the last reach's in the channel's target.
        channel.sort();
        while !channel.is_empty() {
            let top = channel
                .iter()
                .position(|&id| {
                    !t.network
                        .links()
                        .any(|(_, l)| l.down == id && channel.contains(&l.up))
                })
                .unwrap_or(0);
            let id = channel.remove(top);
            if t.network.store(id).and_then(Store::as_reach).is_some() {
                self.remove_reach(t, id);
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
            // A store standing over the lip on the far side pours in: link
            // at once, however low this side is.
            let poured = match outflow.kind {
                CrestKind::Child { owner: Some(other) } => t
                    .network
                    .store(other)
                    .and_then(Store::surface)
                    .is_some_and(|l| l > outflow.lip),
                _ => false,
            };
            if level + LINK_MARGIN < outflow.lip && !poured {
                continue;
            }
            // Nothing worth a link crosses yet: a weir made now would open
            // closed, and a channel routed now would lie empty.
            if outflow.cells.iter().any(|c| c.saddle < level)
                && outflow.hole.is_none()
                && Weir::new(
                    outflow.cells.iter().map(|c| c.saddle).collect(),
                    self.gain,
                    false,
                )
                .free(level as f64)
                .0 <= 2.0 * Q_RETIRE
                && outflow.link.is_some()
            {
                continue;
            }
            let outflow = outflow.clone();
            let routed = outflow.kind == CrestKind::Outlet
                && outflow.hole.is_none()
                && outflow.cells.iter().all(|c| c.inside != c.outside);
            let entry = if routed {
                self.route(t, id, &outflow, level)
            } else {
                self.resolve_target(t, id, &outflow).map(|store| Entry {
                    store,
                    fall: self
                        .outflow_lip(&outflow)
                        .and_then(|lip| self.outflow_arc(t, id, &outflow, &lip)),
                    lip: None,
                })
            };
            let Some(Entry {
                store: target,
                fall,
                lip,
            }) = entry.filter(|e| e.store != id)
            else {
                continue;
            };
            // Two basins across one ridge share one reversible weir: a
            // second, the other way, would carry the same water twice.
            let shared = t.network.links().find_map(|(link, e)| {
                (e.up == target && e.down == id && e.law.reversible()).then_some(link)
            });
            let link = match shared {
                Some(link) => link,
                None => self.add_link(t, id, target, &outflow, lip, fall),
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
    /// the same level (§8.1), or once one of them is full to its cap and the
    /// other stands above that cap. A full basin can rise no further, so it
    /// would never meet the other's level: it is a pocket the other has
    /// flooded, a cave under a lake with a hole blown between them. Returns
    /// whether the basin is gone.
    fn merge_with_neighbour(&mut self, t: &mut Topology, id: StoreId) -> bool {
        let basin = t.network.store(id).and_then(Store::as_basin).expect("live");
        let level = basin.level();
        let sea = basin.outflows.iter().find_map(|o| {
            let target = o.target?;
            let sea = t.network.store(target)?.as_ocean()?.level;
            (level > o.lip && sea > o.lip && (level - sea).abs() < MERGE_LEVELS).then_some(target)
        });
        if let Some(ocean) = sea {
            self.absorb_into_ocean(t, id, ocean);
            return true;
        }
        let partner = basin.outflows.iter().find_map(|o| {
            let target = o.target?;
            let other = t.network.store(target)?.as_basin()?;
            if other.frozen {
                return None;
            }
            let other_level = other.level();
            let flooded = |full: &Basin, full_level: f32, above: f32| {
                full_level > full.cap - CAP_MARGIN && above > full.cap
            };
            (target != id
                && level > o.lip
                && other_level > o.lip
                && ((level - other_level).abs() < MERGE_LEVELS
                    || flooded(basin, level, other_level)
                    || flooded(other, other_level, level)))
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
            return Some(self.edge_store(t, cell.inside));
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

    /// Where water standing over `columns` at `level` runs to once the bed
    /// under it is gone: the store the drainage from the lowest span there
    /// reaches.
    fn runoff(
        &mut self,
        t: &mut Topology,
        from: StoreId,
        columns: &[Column],
        level: f32,
    ) -> Option<StoreId> {
        let graph = t.geometry.graph();
        let start = columns
            .iter()
            .filter_map(|c| graph.span_at(*c, level))
            .min_by(|a, b| {
                graph
                    .span(*a)
                    .floor_min
                    .total_cmp(&graph.span(*b).floor_min)
            })?;
        self.follow_drainage(t, from, start).filter(|s| *s != from)
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
                return Some(self.edge_store(t, span));
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

    /// Whether `span` lies in a real depression, below where it would fill
    /// to before spilling.
    fn in_depression(&self, t: &mut Topology, span: SpanRef) -> bool {
        let (graph, drainage) = t.geometry.routing();
        let fill = drainage.fill(graph, span);
        fill > graph.span(span).floor_min + 1e-3
            && (!fill.is_finite() || !is_pothole(graph, span, fill))
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
        self.basin_at_bottom(t, bottom)
    }

    /// A new, empty basin over the pit whose bottom is `bottom`.
    fn basin_at_bottom(&mut self, t: &mut Topology, bottom: SpanRef) -> StoreId {
        let level = t.geometry.graph().span(bottom).floor_min;
        let flood = self.flood(t, None, &[bottom], level, FloodMode::Create);
        self.add_basin(t, Basin::from_flood(flood, 0.0))
    }

    /// Where water leaving the world at `span` goes: the sea, on an edge that
    /// opens onto it, or the void.
    fn edge_store(&mut self, t: &mut Topology, span: SpanRef) -> StoreId {
        let geometry = &*t.geometry;
        let sea = geometry
            .drainage()
            .outlets()
            .on_sea_edge(geometry.graph(), span.column);
        match self.ocean.filter(|_| sea) {
            Some(ocean) => ocean,
            None => self.void(t),
        }
    }

    /// As [`Self::edge_store`], for water falling off the map at `at`: the
    /// edge it crossed is the one on the side it left by.
    fn edge_store_at(&mut self, t: &mut Topology, at: Point3<f32>) -> StoreId {
        let geometry = &*t.geometry;
        let (min, max) = geometry.graph().column_bounds();
        let column = Column::containing(at.x, at.z);
        let (di, dk) = (
            (column.i > max.i) as i32 - (column.i < min.i) as i32,
            (column.k > max.k) as i32 - (column.k < min.k) as i32,
        );
        let sea = geometry.drainage().outlets().sea();
        let opens = sea.is_some_and(|s| s.opens(di, 0) || s.opens(0, dk));
        match self.ocean.filter(|_| opens) {
            Some(ocean) => ocean,
            None => self.void(t),
        }
    }

    /// The sink every closed world edge drains into.
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
        lip: Option<Lip>,
        fall: Option<FallPath>,
    ) -> LinkId {
        // Water can come back over the weir only if the far side can stand
        // above it: another basin, or the sea over a lowland's lip.
        let down_is_basin = match t.network.store(down) {
            Some(Store::Basin(_)) => true,
            Some(Store::Ocean(o)) => outflow.cells.iter().any(|c| c.saddle < o.level),
            _ => false,
        };
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
        let crest = self
            .outflow_lip(outflow)
            .unwrap_or_else(|| Lip::down(Point3::new(0.0, outflow.lip, 0.0)));
        let lip = lip.unwrap_or(crest);
        // Water running back leaves the far basin over the same crest.
        let back = law.reversible().then(|| {
            let lip = Lip {
                direction: -crest.direction,
                ..crest
            };
            BackSide {
                lip,
                fall: self.outflow_arc(t, down, outflow, &lip),
            }
        });
        let id = t.network.add_link(LinkEntry {
            up,
            down,
            law,
            open,
            up_port: Port::Downstream,
            down_port: Port::Upstream,
            lip,
            fall,
            back,
        });
        self.record(TopologyEdit::AddLink(id));
        id
    }

    fn remove_link(&mut self, t: &mut Topology, link: LinkId) {
        if t.network.remove_link(link).is_some() {
            self.record(TopologyEdit::RemoveLink(link));
        }
    }

    /// Whether anything flows into a store, or is on its way: a channel
    /// whose front has yet to reach its end feeds the store it ends in.
    fn fed(&self, t: &Topology, id: StoreId) -> bool {
        t.network.links().any(|(_, l)| {
            if !l.open || (l.down != id && l.up != id) {
                return false;
            }
            let coming = l.down == id
                && t.network
                    .store(l.up)
                    .and_then(Store::as_reach)
                    .is_some_and(|r| r.inflow > 0.0);
            let volume = |s: StoreId| t.network.store(s).map_or(0.0, Store::volume);
            let q = match (
                t.network.view(l.up, volume(l.up), l.up_port),
                t.network.view(l.down, volume(l.down), l.down_port),
            ) {
                (Some(u), Some(d)) => l.law.discharge(u, d),
                _ => 0.0,
            };
            // Into it downstream, or back up a reversible link.
            coming || (l.down == id && q > 0.0) || (l.up == id && q < 0.0)
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

/// The water surface standing over `span`: its basin's level, or its
/// reach's running depth over the floor. `None` where it is dry.
fn surface_over(graph: &SpanGraph, network: &Network, span: SpanRef) -> Option<f32> {
    let owner = graph.owner(span);
    let floor = graph.span(span);
    if let Some(reach) = owner
        .reach
        .and_then(|(r, _)| network.store(r))
        .and_then(Store::as_reach)
    {
        let depth = reach.running().depth;
        if depth > 0.0 && reach.wetted() > 0.0 {
            return Some(floor.floor_c + depth);
        }
    }
    let level = owner
        .body
        .and_then(|b| network.store(b))
        .and_then(Store::surface)?;
    (level > floor.floor_min).then_some(level)
}

/// The store other than `from` whose water stands at height `y` over
/// `span` now: a basin or the sea up to its level, or a reach up to its
/// design depth over the bed. Water landing on a basin's dry bed is not yet
/// the basin's: it runs down that bed as a channel, and the basin drowns the
/// channel as it rises (§8.2).
fn holder(
    graph: &SpanGraph,
    network: &Network,
    from: Option<StoreId>,
    span: SpanRef,
    y: f32,
) -> Option<StoreId> {
    let owner = graph.owner(span);
    let body = owner.body.filter(|b| {
        Some(*b) != from
            && network
                .store(*b)
                .and_then(Store::surface)
                .is_some_and(|level| y <= level)
    });
    let reach = owner.reach.map(|(r, _)| r).filter(|r| {
        Some(*r) != from
            && network
                .store(*r)
                .and_then(Store::as_reach)
                .is_some_and(|reach| {
                    y <= graph.span(span).floor_c + reach.rating.at(reach.rating.design()).depth
                })
    });
    body.or(reach)
}

/// A channel's outlet over `span`: its floor, lifted clear of the rock.
fn outlet_at(graph: &SpanGraph, span: SpanRef, fall: Option<FallPath>) -> ChannelOutlet {
    let (x, z) = span.column.centre();
    ChannelOutlet {
        at: Point3::new(x, graph.span(span).floor_c + OUTLET_LIFT, z),
        fall,
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
