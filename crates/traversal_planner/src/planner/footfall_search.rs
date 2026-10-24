use std::collections::BinaryHeap;

use rustc_hash::FxHashMap;

use crate::params::GaitParams;
use crate::plan::{Contact, Limb};
use crate::section::{Foothold, FootholdWindow, SectionQuery};

use super::body_path::{BodyPath, BodySegment};
use super::cheapest_first::Pending;
use super::cost::FootfallCosts;
use super::rules::PUSH_OFF_TOLERANCE;
use super::settings::PlannerSettings;

/// Length along the path of the bands the body's landing is merged in, in
/// metres.
const BODY_CELL: f32 = 0.25;
/// Width across the path of the cells footfalls are merged in, in metres.
const FOOTFALL_CELL_WIDTH: f32 = 0.05;
/// Slack on matching positions along the path, in metres.
const TOLERANCE: f32 = 1.0e-3;

/// A foot landing, and where the body is when it does.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Footfall {
    pub contact: Contact,
    /// Where along the path the body is as the foot lands.
    pub body_s: f32,
}

/// Footfalls that carry the body along its whole path, and their cost.
pub struct FootfallPlan {
    /// In order, the planted foot not among them.
    pub footfalls: Vec<Footfall>,
    pub cost: f32,
}

/// No footfalls carry the body along its whole path.
pub struct FootfallsRefused {
    /// The cheapest footfalls to the furthest the body was carried.
    pub footfalls: Vec<Footfall>,
}

/// The cheapest footfalls along a body path already chosen.
///
/// Each footfall is a foot landing on a foothold as the body passes a point
/// of its path. Every hop ends with one, pushed off from a stance foot at
/// or behind the takeoff; none lands while the body rolls off ground; on ridden stretches the body is behind the landing
/// foot by one of the `landing_leads`. The body's height is the path's, so
/// a candidate costs no physics: only reach, rise and drop, edge margins,
/// footing, whether the body rests on ground a foot is on, and churn.
pub struct FootfallSearch<'a> {
    pub query: &'a dyn SectionQuery,
    pub params: &'a GaitParams,
    pub settings: &'a PlannerSettings,
    /// The speed along the path the player asks for, in m/s.
    pub target_speed: f32,
    /// Spacing of candidate footfalls along the path, in metres.
    pub spacing: f32,
    pub body: &'a BodyPath,
    /// The last plan's footfalls after the planted foot, in order: moving
    /// one costs churn.
    pub previous: &'a [Contact],
}

/// A flight as the footfalls see it.
#[derive(Clone, Copy)]
struct Flight {
    takeoff: f32,
    takeoff_z: f32,
    landing: f32,
    landing_z: f32,
}

#[derive(Clone, Copy)]
struct Node {
    footfall: Footfall,
    /// Footfalls planned so far, the planted foot not counted.
    count: usize,
    /// The first flight not yet landed.
    next_flight: usize,
    cost: f32,
    parent: Option<usize>,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
struct CellKey {
    along: i64,
    across: i32,
    limb: Limb,
    body: i64,
}

impl FootfallSearch<'_> {
    pub fn search(&self, planted: Footfall) -> Result<FootfallPlan, FootfallsRefused> {
        let flights: Vec<Flight> = self
            .body
            .hops()
            .filter_map(|segment| match *segment {
                BodySegment::Flight {
                    takeoff,
                    landing,
                    landing_z,
                    ..
                } => Some(Flight {
                    takeoff: takeoff.s,
                    takeoff_z: takeoff.z,
                    landing,
                    landing_z,
                }),
                BodySegment::Ride { .. } => None,
            })
            .collect();
        let body_end = self.body.end().unwrap_or(planted.body_s);
        let natural = self.settings.natural_step(self.target_speed, self.params);
        let goal = flights
            .last()
            .map_or(f32::MIN, |flight| flight.landing)
            .max(body_end - 0.5 * natural);

        let mut nodes = vec![Node {
            footfall: planted,
            count: 0,
            next_flight: 0,
            cost: 0.0,
            parent: None,
        }];
        let mut cheapest: FxHashMap<CellKey, usize> = FxHashMap::default();
        let mut pending = BinaryHeap::from([Pending { cost: 0.0, node: 0 }]);
        let mut furthest = 0;
        let mut candidates = Vec::new();

        while let Some(Pending { node: index, .. }) = pending.pop() {
            let node = nodes[index];
            if index != 0 && cheapest.get(&self.cell(&node.footfall)) != Some(&index) {
                continue;
            }
            if node.next_flight == flights.len() && node.footfall.body_s >= goal - TOLERANCE {
                return Ok(FootfallPlan {
                    footfalls: Self::chain(&nodes, index),
                    cost: node.cost,
                });
            }
            let best = &nodes[furthest];
            if (node.footfall.body_s, -node.cost) > (best.footfall.body_s, -best.cost) {
                furthest = index;
            }

            candidates.clear();
            self.expand(
                &node,
                &flights,
                body_end,
                &cheapest,
                &nodes,
                &mut candidates,
            );
            for mut candidate in candidates.drain(..) {
                candidate.parent = Some(index);
                let key = self.cell(&candidate.footfall);
                if cheapest
                    .get(&key)
                    .is_some_and(|&other| nodes[other].cost <= candidate.cost)
                {
                    continue;
                }
                nodes.push(candidate);
                cheapest.insert(key, nodes.len() - 1);
                pending.push(Pending {
                    cost: candidate.cost,
                    node: nodes.len() - 1,
                });
            }
        }
        Err(FootfallsRefused {
            footfalls: Self::chain(&nodes, furthest),
        })
    }

    /// The next footfalls from `node`: landing as the body rides on, or at
    /// the next flight's landing.
    fn expand(
        &self,
        node: &Node,
        flights: &[Flight],
        body_end: f32,
        cheapest: &FxHashMap<CellKey, usize>,
        nodes: &[Node],
        out: &mut Vec<Node>,
    ) {
        let params = self.params;
        let stance = node.footfall.contact;
        let limb = stance.limb.other();
        let mut footholds = Vec::new();
        self.query.footholds(
            &FootholdWindow {
                s: stance.at.s + params.stride.min_length..stance.at.s + params.stride.max_length,
                d: params.foot.reach(limb),
                spacing: self.spacing,
            },
            &mut footholds,
        );
        let next_flight = flights.get(node.next_flight);
        let ridden_until = next_flight.map_or(body_end, |flight| flight.takeoff);
        let worth = |footfall: &Footfall, cost: f32| {
            cheapest
                .get(&self.cell(footfall))
                .map_or(true, |&other| nodes[other].cost > cost)
        };

        for foothold in &footholds {
            let Some(base) = self.footfall_cost(node, limb, foothold) else {
                continue;
            };
            let contact = Contact {
                limb,
                at: foothold.at,
                normal: foothold.normal,
                footing: foothold.footing,
            };
            let step = foothold.at.s - stance.at.s;

            // Landing as the body rides on, before any flight.
            for lead in &self.settings.landing_leads {
                let body_s = foothold.at.s - lead * step;
                if body_s <= node.footfall.body_s + TOLERANCE
                    || body_s > ridden_until + TOLERANCE
                    || self.rolling_at(body_s)
                {
                    continue;
                }
                let footfall = Footfall { contact, body_s };
                if !worth(&footfall, base) {
                    continue;
                }
                let Some(body_z) = self.body.height_at(body_s, self.query) else {
                    continue;
                };
                if !params.foot_reaches(body_z, contact.at.z)
                    || !params.foot_reaches(body_z, stance.at.z)
                {
                    continue;
                }
                let cost = base + self.unheld_cost(&contact, &stance, body_s);
                if worth(&footfall, cost) {
                    out.push(Node {
                        footfall,
                        count: node.count + 1,
                        next_flight: node.next_flight,
                        cost,
                        parent: None,
                    });
                }
            }

            // Landing at the end of the next flight, pushed off from the
            // stance foot with the body over or past it.
            if let Some(flight) = next_flight {
                let landing = flight.landing;
                let lands_ahead = foothold.at.s >= landing - TOLERANCE
                    && foothold.at.s <= 2.0 * landing - stance.at.s + TOLERANCE;
                if flight.takeoff < stance.at.s - PUSH_OFF_TOLERANCE || !lands_ahead {
                    continue;
                }
                let footfall = Footfall {
                    contact,
                    body_s: landing,
                };
                if !worth(&footfall, base)
                    || !params.foot_reaches(flight.landing_z, contact.at.z)
                    || !params.foot_reaches(flight.takeoff_z, stance.at.z)
                {
                    continue;
                }
                let cost = base + self.unheld_cost(&contact, &stance, landing);
                if worth(&footfall, cost) {
                    out.push(Node {
                        footfall,
                        count: node.count + 1,
                        next_flight: node.next_flight + 1,
                        cost,
                        parent: None,
                    });
                }
            }
        }
    }

    /// Whether the body is rolling off ground at `body_s`, so no foot lands.
    fn rolling_at(&self, body_s: f32) -> bool {
        self.body
            .roll_offs()
            .any(|(takeoff, landing)| body_s > takeoff + TOLERANCE && body_s < landing - TOLERANCE)
    }

    /// What a footfall on `foothold` costs after `node`, whatever the body
    /// does meanwhile; `None` where no foot may land.
    fn footfall_cost(&self, node: &Node, limb: Limb, foothold: &Foothold) -> Option<f32> {
        let (params, weights) = (self.params, &self.settings.costs);
        let costs = FootfallCosts { weights };
        let stance = node.footfall.contact.at;
        let rise = foothold.at.z - stance.z;
        if !costs.admits(foothold.footing)
            || foothold.edge_margin < params.foot.min_edge_margin
            || rise > params.stride.max_rise
            || -rise > params.stride.max_drop
        {
            return None;
        }
        let natural = self.settings.natural_step(self.target_speed, params);
        let length_off = (foothold.at.s - stance.s - natural) / natural;
        let reach = params.foot.reach_in.max(params.foot.reach_out);
        let sideways_off = (foothold.at.d - params.foot.natural_offset(limb)) / reach;
        let near_edge = (weights.comfortable_edge_margin - foothold.edge_margin).max(0.0)
            / weights.comfortable_edge_margin;
        let contact = Contact {
            limb,
            at: foothold.at,
            normal: foothold.normal,
            footing: foothold.footing,
        };
        let churn = self
            .previous
            .get(node.count)
            .map_or(0.0, |before| costs.churn(&contact, before));
        Some(
            node.cost
                + weights.step_length * length_off * length_off
                + weights.sideways * sideways_off * sideways_off
                + weights.edge * near_edge * near_edge
                + costs.of(&contact)
                + churn,
        )
    }

    /// What it costs for the body at `body_s` to rest on ground neither foot
    /// is on.
    fn unheld_cost(&self, landed: &Contact, stance: &Contact, body_s: f32) -> f32 {
        let support = self.query.floor(body_s).map_or(body_s, |floor| {
            body_s - self.params.body.radius * floor.normal.s
        });
        let held = [landed.at, stance.at]
            .iter()
            .any(|foot| self.query.ground_reaches(foot, support));
        if held {
            0.0
        } else {
            self.settings.costs.unheld_body
        }
    }

    fn cell(&self, footfall: &Footfall) -> CellKey {
        let at = footfall.contact.at;
        CellKey {
            along: (at.s / self.spacing).floor() as i64,
            across: (at.d / FOOTFALL_CELL_WIDTH).round() as i32,
            limb: footfall.contact.limb,
            body: (footfall.body_s / BODY_CELL).floor() as i64,
        }
    }

    /// The footfalls from the start to `last`, the planted foot not among
    /// them.
    fn chain(nodes: &[Node], last: usize) -> Vec<Footfall> {
        let mut footfalls = Vec::new();
        let mut index = Some(last);
        while let Some(at) = index {
            if nodes[at].parent.is_some() {
                footfalls.push(nodes[at].footfall);
            }
            index = nodes[at].parent;
        }
        footfalls.reverse();
        footfalls
    }
}
