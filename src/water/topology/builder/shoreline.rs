//! Shorelines: where a channel meets standing water, moved as the water
//! rises and falls (§8.2).
//!
//! ```text
//!   each reach ──▶ the first of its cells a body's water has drowned?
//!                  cut the reach before it: what the cut length held goes
//!                  to that body, the reach ends in it, the reaches below
//!                  recede; a reach drowned whole goes, and its feeders end
//!                  in the body instead
//!   each channel's end in a basin ──▶ its shoreline left dry?
//!                  lengthen its last reach to the water, or lay a channel
//!                  on from the shore; the new length starts empty
//! ```
//!
//! Only the boundary moves. The reaches above it keep their water, so no
//! river's storage is poured into a lake at once. Drowned moves water only
//! into the body that drowned the cells, which can only raise it and drown
//! more; Exposed moves none. Neither can set the other off, so a shoreline
//! cannot loop. Drowning is judged on a cell's highest floor and exposure on
//! its lowest, so a level hovering at a shoreline does not flicker.

use crate::water::geometry::{Column, SpanGraph, SpanRef};
use crate::water::ids::StoreId;
use crate::water::network::{critical_depth, Lip, Network, ReachState, Store};
use crate::water::solver::{account, Account};

use super::super::edit::TopologyEdit;
use super::super::router::{
    channel_heads, downstream_of, one_reach, standing_in, walk, WalkEnd, REACH_LENGTH,
};
use super::{outlet_at, ChannelFlow, Topology, TopologyBuilder};

/// A cell is drowned once water stands this far over the highest point of
/// its floor, m.
const DROWN_MARGIN: f32 = 0.05;

/// The longest a reach is lengthened to over a shoreline left dry, m; past
/// it, a channel of its own is laid on.
const MAX_EXTENDED: f32 = 2.0 * REACH_LENGTH;

impl TopologyBuilder {
    /// Move every channel's shoreline to where the water stands now.
    pub(super) fn move_shorelines(&mut self, t: &mut Topology) {
        self.drown_channels(t);
        self.expose_channel_ends(t);
    }

    /// Cut every reach at the first cell a body's water has drowned. A cut
    /// can leave the reach above ending in drowned cells too, so each pass
    /// looks again, up to one pass per store.
    fn drown_channels(&mut self, t: &mut Topology) {
        for _ in 0..t.network.store_slots() {
            let mut cut = false;
            for id in t.network.store_ids() {
                let Some(reach) = t.network.store(id).and_then(Store::as_reach) else {
                    continue;
                };
                let heads = channel_heads(t.network, id);
                if let Some((at, body)) =
                    first_drowned(t.geometry.graph(), t.network, &reach.cells, &heads)
                {
                    self.cut_reach(t, id, at, body);
                    cut = true;
                }
            }
            if !cut {
                break;
            }
        }
    }

    /// Cut a reach before cell `at`, which `body` has drowned. What it holds
    /// beyond what its shorter self holds at the same flow goes to `body`,
    /// so its outflow is continuous across the cut.
    fn cut_reach(&mut self, t: &mut Topology, id: StoreId, at: usize, body: StoreId) {
        let Some(old) = t.network.store(id).and_then(Store::as_reach).cloned() else {
            return;
        };
        let design = old.rating.design();
        let kept = {
            let heads = channel_heads(t.network, id);
            let graph = t.geometry.graph();
            let standing = standing_in(graph, t.network, &heads);
            one_reach(graph, &old.cells[..at], design, &standing)
        };
        let Some(mut kept) = kept else {
            self.drown_reach(t, id, body);
            return;
        };
        let q = match old.state {
            ReachState::Advancing => old.inflow,
            _ => old.outflow,
        };
        kept.front = old.front.min(kept.length);
        kept.tail = old.tail.min(kept.front);
        kept.state = match old.state {
            ReachState::Advancing if kept.front >= kept.length => ReachState::Flowing,
            state => state,
        };
        let holds = kept.rating.at(q).area * kept.wetted() as f64
            + kept.dead_between(kept.tail, kept.front);
        let moved = (old.storage - holds).max(0.0);
        kept.storage = old.storage;
        kept.inflow = old.inflow;
        kept.outflow = old.outflow;
        kept.minor = old.minor;
        kept.version = old.version.wrapping_add(1);

        let (last, drowned) = (old.cells[at - 1], old.cells[at]);
        let graph = t.geometry.graph();
        let lip = Lip::across(last.column, drowned.column, graph.span(last).floor_c);
        let running = kept.rating.at(q);
        let thickness = critical_depth(design, kept.rating.at(design).top_width);
        let arc = self.arc(
            t,
            None,
            &lip,
            lip.height() + running.depth,
            running.velocity,
            thickness,
        );
        kept.outlet = Some(outlet_at(t.geometry.graph(), drowned, arc.clone()));

        let outflows: Vec<_> = t
            .network
            .links()
            .filter(|(_, l)| l.up == id)
            .map(|(link, _)| link)
            .collect();
        for link in outflows {
            self.remove_link(t, link);
        }
        self.release_reach(t, id, &old.claimed);
        if let Some(store) = t.network.store_mut(id) {
            *store = Store::Reach(kept);
        }
        self.claim_reach(t, id);
        let to = account(t.network, body);
        self.transfer(t, Account::Store(id), to, moved);
        self.record(TopologyEdit::CutReach { reach: id, at });
        self.link_reach(t, id, body, Some(lip), arc);
    }

    /// Remove a reach `body` has drowned whole, its water going to `body`,
    /// and end the reaches that fed it there instead.
    fn drown_reach(&mut self, t: &mut Topology, id: StoreId, body: StoreId) {
        let Some(reach) = t.network.store(id).and_then(Store::as_reach) else {
            return;
        };
        let (columns, storage) = (reach.claimed.clone(), reach.storage);
        let entry = reach.cells.first().copied();
        let feeders: Vec<StoreId> = t
            .network
            .links()
            .filter(|(_, l)| l.down == id)
            .map(|(_, l)| l.up)
            .filter(|up| t.network.store(*up).and_then(Store::as_reach).is_some())
            .collect();
        let to = account(t.network, body);
        self.transfer(t, Account::Store(id), to, storage);
        self.release_reach(t, id, &columns);
        t.network.remove_store(id);
        self.record(TopologyEdit::RemoveStore {
            store: id,
            residual_to: to,
        });
        for feeder in feeders {
            if let (Some(span), Some(r)) = (
                entry,
                t.network.store_mut(feeder).and_then(Store::as_reach_mut),
            ) {
                r.outlet = Some(outlet_at(t.geometry.graph(), span, None));
            }
            self.link_reach(t, feeder, body, None, None);
        }
    }

    /// Carry on every channel whose shoreline the basin it ran into has left
    /// dry, from that shoreline to where the water stands now.
    fn expose_channel_ends(&mut self, t: &mut Topology) {
        for id in t.network.store_ids() {
            if let Some((shore, down)) = exposed_shore(t, id) {
                self.expose(t, id, shore, Some(down));
            }
        }
    }

    /// Carry reach `id` on from its dry shoreline `shore`, where it ran into
    /// `down` (or into a store since replaced). Where the walk from there
    /// reaches standing water within `MAX_EXTENDED` of channel, the reach is
    /// lengthened in place, its front left where it was, to advance over the
    /// new length; otherwise a channel is laid on from the shore. The walk
    /// passes over the reach's own claim on its last cell.
    pub(super) fn expose(
        &mut self,
        t: &mut Topology,
        id: StoreId,
        shore: SpanRef,
        down: Option<StoreId>,
    ) {
        let (cells, end) = {
            let network = &*t.network;
            let levels = |s: StoreId| network.store(s).and_then(Store::surface);
            let (graph, drainage) = t.geometry.routing();
            walk(graph, drainage, network, &levels, Some(id), None, shore)
        };
        let Some(old) = t.network.store(id).and_then(Store::as_reach).cloned() else {
            return;
        };
        let outflows: Vec<_> = t
            .network
            .links()
            .filter(|(_, l)| l.up == id)
            .map(|(link, _)| link)
            .collect();
        let longer = match end {
            WalkEnd::Store(store, span) if down.is_none_or(|d| d == store) => {
                let mut all = old.cells.clone();
                all.extend(cells.iter().copied());
                let heads = channel_heads(t.network, id);
                let graph = t.geometry.graph();
                let standing = standing_in(graph, t.network, &heads);
                one_reach(graph, &all, old.rating.design(), &standing)
                    .filter(|r| r.length <= MAX_EXTENDED)
                    .map(|r| (r, span, store))
            }
            _ => None,
        };
        for link in outflows {
            self.remove_link(t, link);
        }
        let Some((mut longer, span, into)) = longer else {
            let flow = ChannelFlow {
                design: old.rating.design(),
                now: old.outflow,
            };
            match self.channel(t, Some(id), None, shore, flow, 0) {
                Some(on) if on.store != id => self.link_reach(t, id, on.store, on.lip, on.fall),
                _ => match down {
                    Some(down) => self.link_reach(t, id, down, None, None),
                    None => self.remove_channel(t, id),
                },
            }
            return;
        };
        let graph = t.geometry.graph();
        let last = *longer.cells.last().expect("at least two cells");
        let lip = Lip::across(last.column, span.column, graph.span(last).floor_c);
        let design = old.rating.design();
        let running = longer.rating.at(old.outflow.max(old.inflow));
        let thickness = critical_depth(design, longer.rating.at(design).top_width);
        let arc = self.arc(
            t,
            None,
            &lip,
            lip.height() + running.depth,
            running.velocity,
            thickness,
        );
        longer.storage = old.storage;
        longer.front = old.front.min(longer.length);
        longer.tail = old.tail.min(longer.front);
        longer.state = match old.state {
            ReachState::Flowing if longer.front < longer.length => ReachState::Advancing,
            state => state,
        };
        longer.inflow = old.inflow;
        longer.outflow = old.outflow;
        longer.minor = old.minor;
        longer.version = old.version.wrapping_add(1);
        longer.outlet = Some(outlet_at(t.geometry.graph(), span, arc.clone()));
        self.release_reach(t, id, &old.claimed);
        if let Some(store) = t.network.store_mut(id) {
            *store = Store::Reach(longer);
        }
        self.claim_reach(t, id);
        self.record(TopologyEdit::ExtendReach {
            reach: id,
            cells: cells.len(),
        });
        self.link_reach(t, id, into, Some(lip), arc);
    }
}

/// The dry shoreline a reach's end has been left on, and the basin it ran
/// into: its outlet's span, once the basin stands below that span's lowest
/// floor. A channel ending over a fall has no shoreline.
fn exposed_shore(t: &Topology, id: StoreId) -> Option<(SpanRef, StoreId)> {
    let reach = t.network.store(id)?.as_reach()?;
    let outlet = reach.outlet.as_ref().filter(|o| o.fall.is_none())?;
    let down = downstream_of(t.network, id)?;
    let level = t.network.store(down)?.as_basin()?.level();
    let graph = t.geometry.graph();
    let shore = graph.span_at(Column::containing(outlet.at.x, outlet.at.z), outlet.at.y)?;
    (level < graph.span(shore).floor_min).then_some((shore, down))
}

/// The first of `cells` over which a body other than one of `heads` stands
/// more than `DROWN_MARGIN` above its highest floor, and that body. A lake
/// standing over its own outlet covers the first cells of the channel
/// leaving it, and does not drown the channel it feeds.
fn first_drowned(
    graph: &SpanGraph,
    network: &Network,
    cells: &[SpanRef],
    heads: &[StoreId],
) -> Option<(usize, StoreId)> {
    cells.iter().enumerate().find_map(|(i, cell)| {
        let body = graph.owner(*cell).body.filter(|b| !heads.contains(b))?;
        let level = network.store(body)?.surface()?;
        (level > graph.span(*cell).floor_max + DROWN_MARGIN).then_some((i, body))
    })
}
