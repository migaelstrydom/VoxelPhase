//! Opening a level at rest (§13, `settle: Steady`).
//!
//! ```text
//!   repeat until nothing moves:
//!     settle topology              link outflows, route channels, merge, re-region
//!     for each store fed by a source, nearest the source first:
//!       reach  ── storage for its inflow, running full length
//!       basin  ── the volume where inflow = outflow + loss, others held
//!                 (or its cap, if nothing lets out enough yet: the next
//!                 settle finds its higher banks and links its outflow)
//! ```
//!
//! This is Fill–Spill–Merge run to completion, as Gauss–Seidel sweeps over
//! the network. Water brought in comes from the sources' reservoirs through
//! the ledger, so the level opens with `emitted` holding what the sources
//! filled. Pools no source feeds keep their authored volume.

use std::collections::VecDeque;

use crate::water::ids::StoreId;
use crate::water::network::{LossLaw, Network, Reach, ReachState, Store};
use crate::water::solver::{account, Account};

use super::builder::{Topology, TopologyBuilder, CAP_MARGIN};

/// Sweeps before giving up on reaching rest.
pub const MAX_SWEEPS: usize = 400;

/// A sweep that moves no store's volume by more than this share of it (or
/// of 1 m³, for small stores) and changes no topology is at rest.
const REST_TOLERANCE: f64 = 1e-5;

/// Bisection steps for a basin's steady volume.
const BISECTION_STEPS: usize = 60;

/// A basin within this of where it is held below its cap is standing there,
/// m.
const LEVEL_TOLERANCE: f32 = 1e-4;

/// How the settle went.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SteadyReport {
    pub sweeps: usize,
    /// Whether the network came to rest within `MAX_SWEEPS`.
    pub converged: bool,
}

/// Bring every store fed by a source to steady state.
pub fn settle_steady(
    builder: &mut TopologyBuilder,
    t: &mut Topology,
    loss: &LossLaw,
) -> SteadyReport {
    for sweep in 0..MAX_SWEEPS {
        let edits = builder.edit_count();
        builder.settle(t, loss);
        let mut moved = 0.0f64;
        for id in fed_upstream_first(t.network) {
            let before = t.network.store(id).map_or(0.0, Store::volume);
            let target = match t.network.store(id) {
                Some(Store::Reach(_)) => steady_reach(t.network, id),
                Some(Store::Basin(_)) => steady_basin(t.network, loss, id),
                _ => None,
            };
            let Some(target) = target else {
                continue;
            };
            let q = inflow(t.network, id);
            builder.transfer(t, Account::Emitted, account(t.network, id), target - before);
            if let Some(reach) = t.network.store_mut(id).and_then(Store::as_reach_mut) {
                run_full(reach, q);
            }
            moved = moved.max((target - before).abs() / before.max(1.0));
        }
        if builder.edit_count() == edits && moved <= REST_TOLERANCE {
            return SteadyReport {
                sweeps: sweep + 1,
                converged: true,
            };
        }
    }
    SteadyReport {
        sweeps: MAX_SWEEPS,
        converged: false,
    }
}

/// Finite stores a source's water reaches, in the order it reaches them.
fn fed_upstream_first(network: &Network) -> Vec<StoreId> {
    let mut seen = vec![false; network.store_slots()];
    let mut queue: VecDeque<StoreId> = network
        .stores()
        .filter(|(_, s)| matches!(s, Store::Reservoir))
        .map(|(id, _)| id)
        .collect();
    for id in &queue {
        seen[id.0 as usize] = true;
    }
    let mut order = Vec::new();
    while let Some(id) = queue.pop_front() {
        if network.store(id).is_some_and(Store::is_finite) {
            order.push(id);
        }
        for (_, link) in network.links() {
            let next = if link.up == id {
                Some(link.down)
            } else if link.down == id && link.law.reversible() {
                Some(link.up)
            } else {
                None
            };
            if let Some(next) = next {
                if !seen[next.0 as usize] {
                    seen[next.0 as usize] = true;
                    queue.push_back(next);
                }
            }
        }
    }
    order
}

/// What flows into a store now, m³/s, from every link at current volumes.
fn inflow(network: &Network, id: StoreId) -> f64 {
    let mut total = 0.0;
    for (_, link) in network.links() {
        if link.up != id && link.down != id {
            continue;
        }
        let (Some(up), Some(down)) = (
            network.view(
                link.up,
                network.store(link.up).map_or(0.0, Store::volume),
                link.up_port,
            ),
            network.view(
                link.down,
                network.store(link.down).map_or(0.0, Store::volume),
                link.down_port,
            ),
        ) else {
            continue;
        };
        let q = link.law.discharge(up, down);
        if link.down == id && q > 0.0 {
            total += q;
        } else if link.up == id && q < 0.0 {
            total -= q;
        }
    }
    total
}

/// Net flow into a store, m³/s, with its own volume at `volume` (or as it
/// stands) and every other store as it stands. Loss is left out.
fn net_flow(network: &Network, id: StoreId, volume: Option<f64>) -> f64 {
    let at = |s: StoreId| {
        let store = network.store(s);
        match volume.filter(|_| s == id) {
            Some(v) => v,
            None => store.map_or(0.0, Store::volume),
        }
    };
    let mut net = 0.0;
    for (_, link) in network.links() {
        if link.up != id && link.down != id {
            continue;
        }
        let (Some(up), Some(down)) = (
            network.view(link.up, at(link.up), link.up_port),
            network.view(link.down, at(link.down), link.down_port),
        ) else {
            continue;
        };
        let q = link.law.discharge(up, down);
        if link.down == id {
            net += q;
        }
        if link.up == id {
            net -= q;
        }
    }
    net
}

/// A reach's storage when running full length at its inflow.
fn steady_reach(network: &Network, id: StoreId) -> Option<f64> {
    let reach = network.store(id)?.as_reach()?;
    let q = inflow(network, id);
    if q <= 0.0 {
        return None;
    }
    let area = reach.rating.at(q).area;
    Some(area * reach.length as f64 + reach.dead_between(0.0, reach.length))
}

/// Put a reach in the state its steady storage implies: flowing over its
/// whole length at `q`.
fn run_full(reach: &mut Reach, q: f64) {
    reach.front = reach.length;
    reach.tail = 0.0;
    reach.state = ReachState::Flowing;
    reach.inflow = q;
    reach.outflow = q;
}

/// The volume at which what flows into a basin equals what flows out and
/// what it loses, with every other store held where it is.
///
/// A basin nothing yet drains fast enough rises to just under the margin at
/// which it is re-flooded: high enough that every lip within
/// `LINK_MARGIN` of it links on the next settle, low enough that its links
/// are not dropped by a re-flood. Only once it already stands there, its
/// links made and still not enough, does it rise to its cap, and the next
/// settle re-floods it to find its higher banks.
fn steady_basin(network: &Network, loss: &LossLaw, id: StoreId) -> Option<f64> {
    let basin = network.store(id)?.as_basin()?;
    let store = network.store(id)?;
    let balance = |v: f64| net_flow(network, id, Some(v)) - loss.loss(store, v).0;
    let held = basin.cap - 2.0 * CAP_MARGIN;
    let top = if basin.level() >= held - LEVEL_TOLERANCE {
        basin.cap
    } else {
        held
    };
    let (mut lo, mut hi) = (0.0f64, basin.hypsometry.volume(top));
    if balance(hi) >= 0.0 {
        return Some(hi);
    }
    if balance(lo) <= 0.0 {
        return Some(lo);
    }
    for _ in 0..BISECTION_STEPS {
        let mid = 0.5 * (lo + hi);
        if balance(mid) > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    Some(0.5 * (lo + hi))
}
