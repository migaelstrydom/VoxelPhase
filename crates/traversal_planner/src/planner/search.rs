use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use smallvec::SmallVec;

use crate::params::GaitParams;
use crate::path::PathVec;
use crate::plan::{BodyState, Contact, Limb, Motion, Phase, Plan};
use crate::section::{Footing, SectionQuery};

use super::cost::FootfallCosts;
use super::settings::PlannerSettings;
use super::state::PlanState;
use super::stride::Stride;
use super::transition::{Candidate, ExpandContext, TransitionModel};

/// Width across the path of the cells footfalls are merged in, in metres.
const FOOTFALL_CELL_WIDTH: f32 = 0.05;

/// What one plan is asked for.
#[derive(Clone, Copy, Debug)]
pub struct PlanRequest<'a> {
    pub body: BodyState,
    /// The foot that lands next. The other is taken to be planted under the
    /// body, on fixed ground at its natural offset.
    pub lead: Limb,
    /// The speed along the path the player asks for, in m/s.
    pub target_speed: f32,
    /// The last frame's plan in this frame's path coordinates, its landed
    /// footfalls dropped: its committed footfall is kept if it still can be,
    /// and moving its tentative ones costs churn.
    pub previous: Option<&'a Plan>,
}

/// A plan that carries the body to the horizon.
#[derive(Clone, Debug, PartialEq)]
pub struct Planned {
    pub plan: Plan,
    pub cost: f32,
    /// Whether the last plan's committed footfall could no longer be kept,
    /// so the plan started over from the body.
    pub diverged: bool,
}

/// No plan reaches the horizon on this heading.
#[derive(Clone, Debug, PartialEq)]
pub struct Refusal {
    /// The cheapest plan to the furthest point any plan reached: where the
    /// way runs out.
    pub partial: Plan,
    pub diverged: bool,
}

/// Searches for the cheapest plan to the horizon.
///
/// Footfalls are candidates on a lattice: every `footfall_spacing` along the
/// path, on each lane. Footfalls only move forward, so the lattice is
/// searched in order of `s`, keeping the cheapest way to each cell (a
/// footfall, its foot and where the body landed with it): a dynamic programme
/// whose cost is bounded by the lattice's size.
pub struct Planner {
    pub params: GaitParams,
    pub settings: PlannerSettings,
    models: Vec<Box<dyn TransitionModel>>,
}

/// A footfall cell: where along and across the path, which foot, and where
/// the body landed with it, which decides where it can go next.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
struct CellKey {
    along: i64,
    across: i32,
    limb: Limb,
    body: i64,
}

struct Node {
    state: PlanState,
    cost: f32,
    parent: Option<usize>,
    /// The phases from the parent's footfall to this one.
    phases: SmallVec<[Phase; 2]>,
}

/// A node waiting to be expanded, nearest the body first.
struct Pending {
    s: f32,
    node: usize,
}

impl PartialEq for Pending {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for Pending {}

impl PartialOrd for Pending {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Pending {
    fn cmp(&self, other: &Self) -> Ordering {
        other.s.total_cmp(&self.s).then(other.node.cmp(&self.node))
    }
}

impl Planner {
    pub fn new(params: GaitParams, settings: PlannerSettings) -> Self {
        Self {
            params,
            settings,
            models: vec![Box::new(Stride)],
        }
    }

    pub fn plan(
        &self,
        query: &dyn SectionQuery,
        request: &PlanRequest,
    ) -> Result<Planned, Refusal> {
        let context = ExpandContext {
            query,
            params: &self.params,
            settings: &self.settings,
            target_speed: request.target_speed,
        };
        let costs = FootfallCosts {
            weights: &self.settings.costs,
        };
        let previous: Vec<Contact> = request
            .previous
            .map(|plan| plan.footfalls().map(|(_, contact)| *contact).collect())
            .unwrap_or_default();
        let committed = request
            .previous
            .filter(|plan| plan.committed > 0)
            .and_then(|_| previous.first().copied());

        let mut nodes = vec![Node {
            state: self.start(request),
            cost: 0.0,
            parent: None,
            phases: SmallVec::new(),
        }];
        let mut cheapest: HashMap<CellKey, usize> = HashMap::new();
        let mut pending = BinaryHeap::from([Pending { s: 0.0, node: 0 }]);
        let goal = request.body.s + self.settings.horizon;
        let (mut finished, mut furthest) = (None::<usize>, 0);
        let mut diverged = false;
        let mut candidates = Vec::new();

        while let Some(Pending { node: index, .. }) = pending.pop() {
            let node = &nodes[index];
            if index != 0 && cheapest.get(&self.cell(&node.state)) != Some(&index) {
                continue;
            }
            if node.state.body.s >= goal {
                if finished.map_or(true, |best| node.cost < nodes[best].cost) {
                    finished = Some(index);
                }
                continue;
            }
            if node.state.body.s > nodes[furthest].state.body.s {
                furthest = index;
            }

            candidates.clear();
            for model in &self.models {
                model.expand(&node.state, &context, &mut candidates);
            }
            if index == 0 {
                if let Some(committed) = committed {
                    let kept = |candidate: &Candidate| {
                        let moved = candidate.landing.stance.at - committed.at;
                        candidate.landing.stance.limb == committed.limb
                            && moved.length() <= self.settings.commit_tolerance
                    };
                    if candidates.iter().any(kept) {
                        candidates.retain(kept);
                    } else {
                        diverged = true;
                    }
                }
            }

            let base = node.cost;
            for candidate in candidates.drain(..) {
                let landed = candidate.landing.stance;
                if !costs.admits(landed.footing) {
                    continue;
                }
                let number = candidate.landing.footfalls - 1;
                let churn = previous
                    .get(number)
                    .filter(|_| !(number == 0 && committed.is_some() && !diverged))
                    .map_or(0.0, |before| costs.churn(&landed, before));
                let cost = base + candidate.cost + costs.of(&landed) + churn;
                let key = self.cell(&candidate.landing);
                if cheapest
                    .get(&key)
                    .is_some_and(|&other| nodes[other].cost <= cost)
                {
                    continue;
                }
                if !candidate
                    .phases
                    .iter()
                    .all(|phase| query.clears(phase, &self.params))
                {
                    continue;
                }
                let s = candidate.landing.stance.at.s;
                nodes.push(Node {
                    state: candidate.landing,
                    cost,
                    parent: Some(index),
                    phases: candidate.phases,
                });
                let added = nodes.len() - 1;
                cheapest.insert(key, added);
                pending.push(Pending { s, node: added });
            }
        }

        match finished {
            Some(index) => Ok(Planned {
                plan: Self::plan_to(&nodes, index),
                cost: nodes[index].cost,
                diverged,
            }),
            None => Err(Refusal {
                partial: Self::plan_to(&nodes, furthest),
                diverged,
            }),
        }
    }

    /// The state the plan starts from: the body, and the other foot planted
    /// under it.
    fn start(&self, request: &PlanRequest) -> PlanState {
        let planted = request.lead.other();
        PlanState {
            time: 0.0,
            body: request.body,
            stance: Contact {
                limb: planted,
                at: PathVec::new(
                    request.body.s,
                    self.params.foot.natural_offset(planted),
                    request.body.z - self.params.body.radius,
                ),
                normal: PathVec::UP,
                footing: Footing::Fixed,
            },
            footfalls: 0,
        }
    }

    fn cell(&self, state: &PlanState) -> CellKey {
        CellKey {
            along: (state.stance.at.s / self.settings.footfall_spacing).floor() as i64,
            across: (state.stance.at.d / FOOTFALL_CELL_WIDTH).round() as i32,
            limb: state.stance.limb,
            body: (state.body.s / self.settings.footfall_spacing).floor() as i64,
        }
    }

    /// The plan from the start to `last`: every stride's phases, then the
    /// last footfall landing. The first stride is committed.
    fn plan_to(nodes: &[Node], last: usize) -> Plan {
        let mut chain = vec![last];
        while let Some(parent) = nodes[*chain.last().expect("non-empty")].parent {
            chain.push(parent);
        }
        chain.reverse();
        let mut phases: Vec<Phase> = chain
            .iter()
            .flat_map(|&index| nodes[index].phases.iter().cloned())
            .collect();
        let committed = chain.get(1).map_or(0, |&first| nodes[first].phases.len());
        if chain.len() > 1 {
            let end = &nodes[last].state;
            phases.push(Phase {
                start: end.time,
                duration: 0.0,
                contacts: SmallVec::from_slice(&[end.stance]),
                motion: Motion::Grounded {
                    from: end.body.s,
                    to: end.body.s,
                    start_speed: end.body.speed,
                    end_speed: end.body.speed,
                },
            });
        }
        Plan { phases, committed }
    }
}
