//! Rebuilding the network after a terrain edit.
//!
//! ```text
//!   before the geometry update   every reach's water ──▶ parcels at points
//!   after it                     every basin ──▶ its surviving seeds, level, volume
//!                                clear every basin, reach and link
//!                                each pond re-flooded from its seeds, holding its volume
//!                                every outflow it stands at linked: its channel laid
//!                                each parcel poured where it stands now
//!                                every source linked again
//! ```
//!
//! Nothing of the old network is carried but its water and where it stood:
//! no store, link or channel is patched, so none can disagree with the new
//! ground. The network laid is the one the new ground and the water on it
//! give, by the same rules the settle lays one between edits. Ground an
//! edit did not touch gives the same stores and channels as before, holding
//! the same water.
//!
//! A parcel goes to whatever holds its point now: a basin, a reach, an
//! empty basin in the dry depression it has fallen into, or else a channel
//! laid down the drainage from there. So a river cut by a crater runs on
//! below it, draining, while the crater fills from above.

use nalgebra::{Point3, Vector3};
use rustc_hash::{FxHashMap, FxHashSet};

use crate::water::geometry::{Column, Drain, SpanRef, SpanRemap, COLUMN_SIZE, ORTHOGONAL};
use crate::water::ids::StoreId;
use crate::water::network::{Basin, CrestKind, FloodMode, HoleColumn, ReachState, Store};
use crate::water::solver::Account;

use super::super::edit::TopologyEdit;
use super::super::router::downstream_of;
use super::{ChannelFlow, Topology, TopologyBuilder, OUTLET_LIFT};

/// Water running down a reach before an edit, as parcels at points, so that
/// it outlives the spans under it.
#[derive(Debug, Clone)]
pub struct Runnel {
    /// Upstream first.
    parcels: Vec<Parcel>,
    /// What a channel laid for it is rated to, and carries now.
    flow: ChannelFlow,
    /// What flowed into it over the last tick, and out of it.
    inflow: f64,
    outflow: f64,
    /// The bed point its water's front had reached, short of the reach's end.
    front: Option<Point3<f32>>,
    /// The bed point its water's tail had left, below the reach's top.
    tail: Option<Point3<f32>>,
    /// Draining, with nothing flowing in.
    receding: bool,
}

/// A volume of water standing at a point.
#[derive(Debug, Clone, Copy)]
struct Parcel {
    /// Just over the floor it stood on.
    at: Point3<f32>,
    volume: f64,
}

/// A basin's water, found again on the new ground.
struct Pond {
    old: StoreId,
    level: f32,
    volume: f64,
    /// Its spans still under its level, found through ownership, which the
    /// span remap carries across the edit.
    seeds: Vec<SpanRef>,
    /// Where it stood, for water whose bed is gone.
    columns: Vec<Column>,
    holes: Vec<HoleColumn>,
    /// The basin itself, unlinked, where the edit touched no column its
    /// flood reads: flooded again it would be the same.
    kept: Option<Basin>,
}

/// What a reach was poured: how far along it water landed, and what the
/// runnels it came from carried.
#[derive(Debug, Clone, Copy)]
struct Wet {
    /// The last of its cells water landed on.
    last: usize,
    /// The greatest discharge a runnel poured into it was rated to: what it
    /// is rated to again.
    design: f64,
    /// The most that flowed into a runnel poured into it.
    inflow: f64,
    /// Where a runnel's last water landed, and what flowed out of it: the
    /// reach's own outflow, if that is its last cell.
    outflow: Option<(usize, f64)>,
    /// Every runnel poured into it was receding.
    receding: bool,
}

type Wetted = FxHashMap<StoreId, Wet>;

/// Which reach runs over each span, and which of its cells the span is:
/// exact, where a span's owner names only the reach whose water covers it
/// and its nearest cell.
#[derive(Default)]
struct Cells {
    of: FxHashMap<SpanRef, (StoreId, usize)>,
    /// The store slots the index was built over.
    slots: usize,
}

impl Cells {
    /// The reach cell at `span`, indexing again if reaches were laid since.
    fn at(&mut self, t: &Topology, span: SpanRef) -> Option<(StoreId, usize)> {
        if self.slots != t.network.store_slots() {
            self.of.clear();
            // A reach's last cell is the first of the reach below it: the
            // cell is the lower reach's.
            for (id, store) in t.network.stores() {
                if let Some(reach) = store.as_reach() {
                    for (i, cell) in reach.cells.iter().enumerate().skip(1) {
                        self.of.insert(*cell, (id, i));
                    }
                }
            }
            for (id, store) in t.network.stores() {
                if let Some(first) = store.as_reach().and_then(|r| r.cells.first()) {
                    self.of.insert(*first, (id, 0));
                }
            }
            self.slots = t.network.store_slots();
        }
        self.of.get(&span).copied()
    }
}

impl TopologyBuilder {
    /// The water in every reach, as parcels on its wetted cells, channels
    /// upstream first. Taken before the geometry takes an edit.
    pub fn runnels(&self, t: &Topology) -> Vec<Runnel> {
        let graph = t.geometry.graph();
        upstream_first(t)
            .into_iter()
            .filter_map(|id| {
                let reach = t.network.store(id)?.as_reach()?;
                if reach.storage <= 0.0 {
                    return None;
                }
                let (from, to) = match reach.state {
                    ReachState::Receding => (reach.tail, reach.front),
                    _ => (0.0, reach.front),
                };
                // Its last cell is the first of the reach below it, whose
                // water that is.
                let shared = downstream_of(t.network, id)
                    .and_then(|d| t.network.store(d)?.as_reach()?.cells.first().copied())
                    .is_some_and(|first| reach.cells.last() == Some(&first));
                let own = reach.cells.len() - shared as usize;
                let mut wet: Vec<usize> = (0..own)
                    .filter(|&i| {
                        let d = reach.cell_distance[i];
                        d >= from - 0.5 * COLUMN_SIZE && d <= to + 0.5 * COLUMN_SIZE
                    })
                    .collect();
                if wet.is_empty() {
                    wet.push(0);
                }
                let share = reach.storage / wet.len() as f64;
                let parcels = wet
                    .into_iter()
                    .map(|i| {
                        let cell = reach.cells[i];
                        let (x, z) = cell.column.centre();
                        Parcel {
                            at: Point3::new(x, graph.span(cell).floor_c + OUTLET_LIFT, z),
                            volume: share,
                        }
                    })
                    .collect();
                let bed = |d: f32| reach.centreline.point_at(d) + Vector3::y() * OUTLET_LIFT;
                Some(Runnel {
                    parcels,
                    flow: ChannelFlow {
                        design: reach.rating.design(),
                        now: reach.outflow.max(reach.inflow),
                    },
                    inflow: reach.inflow,
                    outflow: reach.outflow,
                    front: (reach.front < reach.length).then(|| bed(reach.front)),
                    tail: (reach.tail > 0.0).then(|| bed(reach.tail)),
                    receding: reach.state == ReachState::Receding,
                })
            })
            .collect()
    }

    /// Lay the network again over ground an edit has changed, as `remap`
    /// says, and pour back the water it held: the basins' from the network
    /// as it stood, the reaches' from `runnels`, taken before the edit.
    pub fn rebuild(&mut self, t: &mut Topology, remap: &SpanRemap, runnels: Vec<Runnel>) {
        let ponds = self.ponds(t, remap);
        let kept: FxHashSet<StoreId> = ponds
            .iter()
            .filter(|p| p.kept.is_some())
            .map(|p| p.old)
            .collect();
        self.clear(t, &kept);
        let (basins, runoff) = self.refill(t, ponds);
        self.connect_lowlands(t, remap);
        for &id in &basins {
            if t.network.store(id).is_some() {
                self.link_outflows(t, id);
            }
        }

        self.link_sources(t);

        let mut wetted = Wetted::default();
        let mut cells = Cells::default();
        for runnel in &runnels {
            for parcel in &runnel.parcels {
                self.pour(t, *parcel, runnel, &mut cells, &mut wetted);
            }
        }
        // Nothing under the water survived: the bed was blown away from
        // under it. The water runs on to wherever the ground there drains.
        for pond in runoff {
            let to = self
                .runoff(t, pond.old, &pond.columns, pond.level)
                .map_or(Account::Sunk, |s| {
                    crate::water::solver::account(t.network, s)
                });
            self.receive(t, to, pond.volume, None, false, &mut wetted);
        }
        let ends = ends(t, &runnels);
        self.wet_reaches(t, &wetted, &ends, &mut cells);
    }

    /// Every basin's water as it stands on the new ground.
    fn ponds(&self, t: &Topology, remap: &SpanRemap) -> Vec<Pond> {
        let mut holes = self.new_holes(t, remap);
        let touched: FxHashSet<Column> = remap
            .columns
            .iter()
            .flat_map(|c| {
                std::iter::once(*c).chain(ORTHOGONAL.iter().map(|s| c.offset(s.di, s.dk)))
            })
            .collect();
        let mut ponds = Vec::new();
        for (old, basin) in t
            .network
            .stores()
            .filter_map(|(id, s)| Some((id, s.as_basin()?)))
        {
            let level = basin.level();
            let columns = basin.columns();
            let mut kept_holes = basin.holes.clone();
            let new = holes.remove(&old).unwrap_or_default();
            let untouched = new.is_empty()
                && !columns.iter().any(|c| touched.contains(c))
                && !basin
                    .crests
                    .iter()
                    .any(|c| touched.contains(&c.outside.column));
            for hole in new {
                if !kept_holes.iter().any(|h| h.column == hole.column) {
                    kept_holes.push(hole);
                }
            }
            let kept = untouched.then(|| {
                let mut basin = basin.clone();
                for outflow in &mut basin.outflows {
                    outflow.link = None;
                    outflow.target = None;
                }
                basin
            });
            let seeds = if kept.is_some() {
                Vec::new()
            } else {
                self.surviving_seeds(t, old, &columns, level, Some(remap))
            };
            ponds.push(Pond {
                old,
                level,
                volume: basin.volume,
                seeds,
                columns,
                holes: kept_holes,
                kept,
            });
        }
        ponds
    }

    /// Remove every basin, reach and link, and their claims on the spans,
    /// but for the claims of the basins in `kept`, which go back as they
    /// were. Their water is poured again; the sea, the sinks and the
    /// sources' reservoirs stay.
    fn clear(&mut self, t: &mut Topology, kept: &FxHashSet<StoreId>) {
        let links: Vec<_> = t.network.links().map(|(id, _)| id).collect();
        for link in links {
            self.remove_link(t, link);
        }
        for id in t.network.store_ids() {
            let columns = match t.network.store(id) {
                Some(Store::Basin(b)) => b.columns(),
                Some(Store::Reach(r)) => r.claimed.clone(),
                _ => continue,
            };
            if !kept.contains(&id) {
                self.release(t, id, &columns);
                self.release_reach(t, id, &columns);
            }
            t.network.remove_store(id);
            self.record(TopologyEdit::Cleared(id));
        }
        for source in &mut self.sources {
            source.link = None;
        }
    }

    /// A basin for each pond: the one it was where the edit left it alone,
    /// else a new one flooded from its seeds at its level, its own spans one
    /// body and a dry depression the edit opened to it a depression of its
    /// own, filled over a weir. Every new basin claims its seeds before any
    /// floods, so each finds the others' water where it stands. Returns the
    /// basins, and the ponds with nothing left under their water.
    fn refill(&mut self, t: &mut Topology, ponds: Vec<Pond>) -> (Vec<StoreId>, Vec<Pond>) {
        let mut renamed: FxHashMap<StoreId, StoreId> = FxHashMap::default();
        let was_basin: FxHashSet<StoreId> = ponds.iter().map(|p| p.old).collect();
        let mut basins = Vec::new();
        let mut runoff = Vec::new();
        let mut flooding = Vec::new();
        for mut pond in ponds {
            if let Some(basin) = pond.kept.take() {
                t.network.restore(pond.old, Store::Basin(basin));
                self.record(TopologyEdit::AddStore(pond.old));
                renamed.insert(pond.old, pond.old);
                basins.push(pond.old);
            } else if pond.seeds.is_empty() {
                runoff.push(pond);
            } else {
                flooding.push(pond);
            }
        }
        let first = t.network.store_slots() as u32;
        {
            let graph = t.geometry.graph_mut();
            for (k, pond) in flooding.iter().enumerate() {
                for &seed in &pond.seeds {
                    graph.owner_mut(seed).body = Some(StoreId(first + k as u32));
                }
            }
        }
        let floods: Vec<_> = flooding
            .iter()
            .enumerate()
            .map(|(k, pond)| {
                let id = StoreId(first + k as u32);
                self.flood(t, Some(id), &pond.seeds, pond.level, FloodMode::Reregion)
            })
            .collect();
        for (pond, flood) in flooding.into_iter().zip(floods) {
            let id = self.add_basin(t, Basin::with_holes(flood, pond.volume, pond.holes));
            renamed.insert(pond.old, id);
            self.record(TopologyEdit::Reregion {
                basin: id,
                seeds: pond.seeds,
            });
            basins.push(id);
        }
        // A kept basin's crests name the basins beyond them by their old ids.
        for &id in &kept_ids(&basins, &renamed) {
            if let Some(basin) = t.network.store_mut(id).and_then(Store::as_basin_mut) {
                let rename = |kind: &mut CrestKind| {
                    if let CrestKind::Child { owner: Some(o) } = kind {
                        if was_basin.contains(o) {
                            *kind = CrestKind::Child {
                                owner: renamed.get(o).copied(),
                            };
                        }
                    }
                };
                basin.crests.iter_mut().for_each(|c| rename(&mut c.kind));
                for outflow in &mut basin.outflows {
                    rename(&mut outflow.kind);
                    outflow.cells.iter_mut().for_each(|c| rename(&mut c.kind));
                }
            }
        }
        (basins, runoff)
    }

    /// Pour a parcel of `runnel`'s where it stands now.
    fn pour(
        &mut self,
        t: &mut Topology,
        parcel: Parcel,
        runnel: &Runnel,
        cells: &mut Cells,
        wetted: &mut Wetted,
    ) {
        let column = Column::containing(parcel.at.x, parcel.at.z);
        let Some(span) = t.geometry.graph().span_at(column, parcel.at.y) else {
            let void = self.void(t);
            self.receive(t, Account::Store(void), parcel.volume, None, false, wetted);
            return;
        };
        let to = self.holder_of(t, span, runnel.flow, cells);
        let account = crate::water::solver::account(t.network, to);
        let cell = cells
            .at(t, span)
            .filter(|(reach, _)| *reach == to)
            .map(|(_, cell)| cell)
            .or_else(|| nearest_cell(t, to, parcel.at));
        let last = runnel.parcels.last().is_some_and(|p| p.at == parcel.at);
        self.receive(
            t,
            account,
            parcel.volume,
            cell.map(|c| (c, runnel)),
            last,
            wetted,
        );
    }

    /// The store water standing on `span` belongs to: the basin or sea whose
    /// water stands over it, the reach over it, an empty basin in the dry
    /// depression it lies in, the world's edge it runs off, or else a
    /// channel laid down the drainage from it, to the water below.
    fn holder_of(
        &mut self,
        t: &mut Topology,
        span: SpanRef,
        flow: ChannelFlow,
        cells: &mut Cells,
    ) -> StoreId {
        let graph = t.geometry.graph();
        let owner = graph.owner(span);
        let floor = graph.span(span).floor_min;
        let standing = owner.body.filter(|b| {
            t.network
                .store(*b)
                .and_then(Store::surface)
                .is_some_and(|level| level > floor)
        });
        if let Some(body) = standing {
            return body;
        }
        if let Some((reach, _)) = cells.at(t, span) {
            return reach;
        }
        if let Some((reach, _)) = owner.reach.filter(|(r, _)| t.network.store(*r).is_some()) {
            return reach;
        }
        let bank = owner.body.is_some_and(|b| t.network.store(b).is_some());
        if !bank && self.in_depression(t, span) {
            return self.empty_basin(t, span);
        }
        let outlet = {
            let (graph, drainage) = t.geometry.routing();
            drainage.drain(graph, span) == Drain::Outlet
        };
        if outlet {
            return self.edge_store(t, span);
        }
        match self.channel(t, None, None, span, flow, 0) {
            Some(entry) => entry.store,
            None => self.void(t),
        }
    }

    /// Credit `volume` to an account: a finite store's volume, or the
    /// ledger's for the sea and the sinks. Water from a runnel poured into a
    /// reach at one of its cells marks how far along it the water stands.
    fn receive(
        &mut self,
        t: &mut Topology,
        to: Account,
        volume: f64,
        cell: Option<(usize, &Runnel)>,
        last: bool,
        wetted: &mut Wetted,
    ) {
        match to {
            Account::Store(id) if t.network.store(id).is_some_and(Store::is_finite) => {
                let store = t.network.store_mut(id).expect("checked above");
                store.adjust_volume(volume);
                if let (Some(_), Some((cell, runnel))) = (store.as_reach(), cell) {
                    let wet = wetted.entry(id).or_insert(Wet {
                        last: cell,
                        design: 0.0,
                        inflow: 0.0,
                        outflow: None,
                        receding: true,
                    });
                    wet.last = wet.last.max(cell);
                    wet.design = wet.design.max(runnel.flow.design);
                    wet.inflow = wet.inflow.max(runnel.inflow);
                    if last && wet.outflow.is_none_or(|(c, _)| cell >= c) {
                        wet.outflow = Some((cell, runnel.outflow));
                    }
                    wet.receding &= runnel.receding;
                }
            }
            // The water stood where nothing finite holds it now: it has left
            // the stores for the sea or a sink.
            Account::Store(_) => t.ledger.transfer(to, Account::Sunk, volume),
            other => t
                .ledger
                .transfer(Account::Store(StoreId(u32::MAX)), other, volume),
        }
        self.record(TopologyEdit::Poured { into: to, volume });
    }

    /// Set each reach's front, tail and state to the water poured into it:
    /// its ends where the water's were, and running on, flowing out or
    /// receding as the water was. A reach is rated to what the water poured
    /// into it was.
    fn wet_reaches(&mut self, t: &mut Topology, wetted: &Wetted, ends: &Ends, cells: &mut Cells) {
        let mut ids: Vec<StoreId> = wetted.keys().copied().collect();
        ids.sort();
        for id in ids {
            let wet = wetted[&id];
            let Some(reach) = t.network.store(id).and_then(Store::as_reach) else {
                continue;
            };
            let last = reach.cells.len() - 1;
            let own_end = match reach.cells.last().and_then(|c| cells.at(t, *c)) {
                Some((other, _)) if other != id => last.saturating_sub(1),
                _ => last,
            };
            if (wet.design - reach.rating.design()).abs() > 1e-9 * wet.design.max(1.0) {
                self.rerate(t, id, wet.design);
            }
            let reach = t
                .network
                .store_mut(id)
                .and_then(Store::as_reach_mut)
                .expect("checked above");
            // The water's own ends where it had them, else from where it
            // landed: its tail at the top, its front at the last water, or
            // at the end if the water reached within a cell of it.
            let (front, tail) = ends.get(&id).copied().unwrap_or((None, None));
            reach.tail = tail.unwrap_or(0.0);
            reach.front = front.unwrap_or_else(|| {
                if wet.last + 1 >= own_end {
                    reach.length
                } else {
                    (reach.cell_distance[wet.last] + 0.5 * COLUMN_SIZE).min(reach.length)
                }
            });
            reach.state = if wet.receding || reach.tail > 0.0 {
                ReachState::Receding
            } else if reach.front >= reach.length {
                ReachState::Flowing
            } else {
                ReachState::Advancing
            };
            reach.inflow = if reach.state == ReachState::Receding {
                0.0
            } else {
                wet.inflow
            };
            reach.outflow = match wet.outflow {
                Some((cell, q)) if cell + 1 >= own_end => q,
                _ => reach.release(reach.storage).0,
            };
        }
    }
}

/// Where runnels' fronts and tails stood, on the reaches over those points
/// now: distances along each reach.
type Ends = FxHashMap<StoreId, (Option<f32>, Option<f32>)>;

/// Each runnel's front and tail, found on the reach over its point now.
fn ends(t: &Topology, runnels: &[Runnel]) -> Ends {
    let graph = t.geometry.graph();
    let mut ends = Ends::default();
    for runnel in runnels {
        for (at, is_front) in [(runnel.front, true), (runnel.tail, false)] {
            let Some(at) = at else {
                continue;
            };
            let Some(span) = graph.span_at(Column::containing(at.x, at.z), at.y) else {
                continue;
            };
            let Some((id, _)) = graph.owner(span).reach else {
                continue;
            };
            let Some(reach) = t.network.store(id).and_then(Store::as_reach) else {
                continue;
            };
            let d = reach.distance_at(at.x, at.z).clamp(0.0, reach.length);
            let entry = ends.entry(id).or_default();
            if is_front {
                entry.0 = Some(d);
            } else {
                entry.1 = Some(d);
            }
        }
    }
    ends
}

/// The cell of reach `id` nearest `at`, if `id` is a reach.
fn nearest_cell(t: &Topology, id: StoreId, at: Point3<f32>) -> Option<usize> {
    let reach = t.network.store(id)?.as_reach()?;
    let d = reach.distance_at(at.x, at.z);
    Some(
        reach
            .cell_distance
            .partition_point(|c| *c <= d)
            .saturating_sub(1),
    )
}

/// The basins of `basins` that kept their ids.
fn kept_ids(basins: &[StoreId], renamed: &FxHashMap<StoreId, StoreId>) -> Vec<StoreId> {
    basins
        .iter()
        .copied()
        .filter(|id| renamed.get(id) == Some(id))
        .collect()
}

/// Every reach, each after every reach that feeds it.
fn upstream_first(t: &Topology) -> Vec<StoreId> {
    let reaches: Vec<StoreId> = t
        .network
        .stores()
        .filter(|(_, s)| s.as_reach().is_some())
        .map(|(id, _)| id)
        .collect();
    let is_reach = |id: StoreId| t.network.store(id).and_then(Store::as_reach).is_some();
    let mut feeders: FxHashMap<StoreId, usize> = reaches.iter().map(|&id| (id, 0)).collect();
    for (_, link) in t.network.links() {
        if is_reach(link.up) && is_reach(link.down) {
            *feeders.entry(link.down).or_default() += 1;
        }
    }
    let mut ready: Vec<StoreId> = reaches
        .iter()
        .copied()
        .filter(|id| feeders[id] == 0)
        .collect();
    let mut order = Vec::with_capacity(reaches.len());
    while let Some(id) = ready.pop() {
        order.push(id);
        for (_, link) in t.network.links() {
            if link.up == id && is_reach(link.down) {
                let count = feeders.get_mut(&link.down).expect("a reach");
                *count -= 1;
                if *count == 0 {
                    ready.push(link.down);
                }
            }
        }
    }
    // A loop of reaches, which a walk down the drainage never lays, would
    // be left out: take what is left in id order.
    for id in reaches {
        if !order.contains(&id) {
            order.push(id);
        }
    }
    order
}
