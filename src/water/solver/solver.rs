//! The hydrology solver: integrates volumes, never changes topology.
//!
//! ```text
//!   tick():
//!     for group in stores, upstream first:     one store, or a cycle of basins
//!         solve the group implicitly for V'
//!     commit every link's transfer and every loss through the ledger
//!     check the ledger
//! ```
//!
//! It runs at a fixed 1/60 s from an accumulator. Integration costs
//! microseconds, so a slower tick would save nothing, and would put
//! centimetre steps into small pools and the bodies floating in them.

use crate::water::ids::StoreId;
use crate::water::network::{Network, Store};

use super::implicit::{solve_group, view};
use super::ledger::{Account, Balance, VolumeLedger};
use crate::water::network::LossLaw;

/// The solver's fixed tick, seconds.
pub const TICK: f64 = 1.0 / 60.0;

/// Most ticks run in one frame; a longer frame drops the rest rather than
/// spiral.
pub const MAX_TICKS_PER_FRAME: u32 = 4;

/// Integrates the network's volumes at a fixed tick.
#[derive(Debug, Default)]
pub struct HydrologySolver {
    accumulator: f64,
}

impl HydrologySolver {
    /// Run as many ticks as `frame_dt` has accumulated. Returns how many ran.
    pub fn advance(
        &mut self,
        network: &mut Network,
        ledger: &mut VolumeLedger,
        loss: &LossLaw,
        frame_dt: f64,
    ) -> u32 {
        self.accumulator = (self.accumulator + frame_dt).min(TICK * MAX_TICKS_PER_FRAME as f64);
        let mut ticks = 0;
        while self.accumulator >= TICK - 1e-9 {
            self.accumulator -= TICK;
            tick(network, ledger, loss, TICK);
            ticks += 1;
        }
        ticks
    }
}

/// One tick of `dt` seconds.
pub fn tick(network: &mut Network, ledger: &mut VolumeLedger, loss: &LossLaw, dt: f64) -> Balance {
    let slots = network.store_slots();
    let start: Vec<f64> = (0..slots)
        .map(|i| network.store(StoreId(i as u32)).map_or(0.0, Store::volume))
        .collect();
    let mut volumes = start.clone();
    for group in groups_upstream_first(network) {
        solve_group(network, loss, &group, &start, &mut volumes, dt);
    }
    let flows = commit(network, ledger, loss, &start, &volumes, dt);
    advance_reaches(network, &flows, dt);
    ledger.check(network.held_volume())
}

/// What moved through each store over a tick, m³/s, by store slot.
struct Flows {
    inflow: Vec<f64>,
    outflow: Vec<f64>,
}

/// Move every reach's front and tail by what flowed through it.
fn advance_reaches(network: &mut Network, flows: &Flows, dt: f64) {
    for id in network.store_ids() {
        if let Some(reach) = network.store_mut(id).and_then(Store::as_reach_mut) {
            reach.inflow = flows.inflow[id.0 as usize];
            reach.outflow = flows.outflow[id.0 as usize];
            reach.advance(dt as f32);
        }
    }
}

/// Book every link's transfer and every loss at the solved volumes, scaled
/// so that no store gives more than it held at the start of the tick.
fn commit(
    network: &mut Network,
    ledger: &mut VolumeLedger,
    loss: &LossLaw,
    start: &[f64],
    volumes: &[f64],
    dt: f64,
) -> Flows {
    // (source, destination, amount) with every amount positive.
    let mut moves: Vec<(StoreId, StoreId, f64)> = Vec::new();
    for (_, link) in network.flowing_links() {
        let up = view(network, link.up, link.up_port, volumes);
        let down = view(network, link.down, link.down_port, volumes);
        let amount = link.law.discharge(up, down) * dt;
        if amount > 0.0 {
            moves.push((link.up, link.down, amount));
        } else if amount < 0.0 {
            moves.push((link.down, link.up, -amount));
        }
    }
    let mut losses: Vec<(StoreId, f64)> = Vec::new();
    for (id, store) in network.stores() {
        let (lost, _) = loss.loss(store, volumes[id.0 as usize]);
        if lost > 0.0 {
            losses.push((id, lost * dt));
        }
    }

    // Guard: (Q_out + Q_loss) · dt ≤ V for every finite store.
    let mut giving = vec![0.0f64; start.len()];
    for &(from, _, amount) in &moves {
        giving[from.0 as usize] += amount;
    }
    for &(id, amount) in &losses {
        giving[id.0 as usize] += amount;
    }
    let scale = |id: StoreId, network: &Network| {
        let finite = network.store(id).is_some_and(Store::is_finite);
        let given = giving[id.0 as usize];
        let held = start[id.0 as usize];
        if finite && given > held && given > 0.0 {
            held / given
        } else {
            1.0
        }
    };

    let mut flows = Flows {
        inflow: vec![0.0; start.len()],
        outflow: vec![0.0; start.len()],
    };
    for (from, to, amount) in moves {
        let amount = amount * scale(from, network);
        let (from_account, to_account) = (account(network, from), account(network, to));
        ledger.transfer(from_account, to_account, amount);
        adjust(network, from, -amount);
        adjust(network, to, amount);
        flows.outflow[from.0 as usize] += amount / dt;
        flows.inflow[to.0 as usize] += amount / dt;
    }
    for (id, amount) in losses {
        let amount = amount * scale(id, network);
        ledger.transfer(Account::Store(id), Account::Lost, amount);
        adjust(network, id, -amount);
    }
    flows
}

/// The ledger account a store's volume is booked against.
pub fn account(network: &Network, id: StoreId) -> Account {
    match network.store(id) {
        Some(Store::Sink) => Account::Sunk,
        Some(Store::Reservoir) => Account::Emitted,
        Some(Store::Ocean(_)) => Account::Ocean,
        _ => Account::Store(id),
    }
}

fn adjust(network: &mut Network, id: StoreId, delta: f64) {
    if let Some(store) = network.store_mut(id) {
        if store.is_finite() {
            store.adjust_volume(delta);
        }
    }
}

/// Finite stores grouped into strongly connected components of the open
/// links, upstream groups first. Ties follow store id.
fn groups_upstream_first(network: &Network) -> Vec<Vec<StoreId>> {
    let slots = network.store_slots();
    let mut edges: Vec<Vec<usize>> = vec![Vec::new(); slots];
    for (_, link) in network.flowing_links() {
        let finite = |id: StoreId| network.store(id).is_some_and(Store::is_finite);
        if finite(link.up) && finite(link.down) {
            edges[link.up.0 as usize].push(link.down.0 as usize);
            if link.law.reversible() {
                edges[link.down.0 as usize].push(link.up.0 as usize);
            }
        }
    }
    let nodes: Vec<usize> = network
        .stores()
        .filter(|(_, s)| s.is_finite())
        .map(|(id, _)| id.0 as usize)
        .collect();
    let mut groups = tarjan(&nodes, &edges);
    // Tarjan emits components downstream first.
    groups.reverse();
    groups
        .into_iter()
        .map(|g| g.into_iter().map(|i| StoreId(i as u32)).collect())
        .collect()
}

/// Strongly connected components, each sorted, in reverse topological order.
fn tarjan(nodes: &[usize], edges: &[Vec<usize>]) -> Vec<Vec<usize>> {
    struct State {
        index: Vec<Option<usize>>,
        low: Vec<usize>,
        on_stack: Vec<bool>,
        stack: Vec<usize>,
        next: usize,
        out: Vec<Vec<usize>>,
    }
    fn visit(v: usize, edges: &[Vec<usize>], s: &mut State) {
        s.index[v] = Some(s.next);
        s.low[v] = s.next;
        s.next += 1;
        s.stack.push(v);
        s.on_stack[v] = true;
        for &w in &edges[v] {
            match s.index[w] {
                None => {
                    visit(w, edges, s);
                    s.low[v] = s.low[v].min(s.low[w]);
                }
                Some(iw) if s.on_stack[w] => s.low[v] = s.low[v].min(iw),
                _ => {}
            }
        }
        if Some(s.low[v]) == s.index[v] {
            let mut component = Vec::new();
            while let Some(w) = s.stack.pop() {
                s.on_stack[w] = false;
                component.push(w);
                if w == v {
                    break;
                }
            }
            component.sort_unstable();
            s.out.push(component);
        }
    }
    let n = edges.len();
    let mut state = State {
        index: vec![None; n],
        low: vec![0; n],
        on_stack: vec![false; n],
        stack: Vec::new(),
        next: 0,
        out: Vec::new(),
    };
    for &v in nodes {
        if state.index[v].is_none() {
            visit(v, edges, &mut state);
        }
    }
    state.out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn components_come_upstream_first() {
        // 0 → 1 ⇄ 2 → 3
        let edges = vec![vec![1], vec![2], vec![1, 3], vec![]];
        let mut groups = tarjan(&[0, 1, 2, 3], &edges);
        groups.reverse();
        assert_eq!(groups, vec![vec![0], vec![1, 2], vec![3]]);
    }
}
