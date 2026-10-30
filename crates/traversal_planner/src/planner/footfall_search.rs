use std::collections::BinaryHeap;

use rustc_hash::FxHashMap;

use crate::params::GaitParams;
use crate::plan::{Contact, FootfallId, Limb};
use crate::section::SectionQuery;

use super::body_path::{BodyPath, BodySegment};
use super::cheapest_first::Pending;
use super::cost::FootfallCosts;
use super::footfall_candidates::{Candidate, CandidateSpec, FootfallCandidates};
use super::rules::{FOOT_SLACK, PUSH_OFF_TOLERANCE};
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
/// or behind the takeoff; none lands while the body rolls off ground; on
/// ridden stretches the body is behind the landing foot by one of the
/// `landing_leads`, and the foot stays within reach until the body has
/// passed it by the gait's plant-ahead distance, where it lifts. The body's
/// height is the path's, so a candidate costs no physics: only reach, rise
/// and drop, edge margins, footing, whether the body rests on ground a foot
/// is on, and churn.
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
    /// The footfall a foot is already swinging to, if one is: moving it
    /// costs more.
    pub swinging: Option<FootfallId>,
}

/// What every expansion of one search reads.
struct Expansion<'a> {
    flights: &'a [Flight],
    body_end: f32,
    costs: &'a FootfallCosts<'a>,
    /// The left foot's, then the right's.
    candidates: &'a [FootfallCandidates; 2],
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
            .flights()
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
                BodySegment::Ride { .. } | BodySegment::Leap { .. } => None,
            })
            .collect();
        // Feet carry the body as far as it rides: to its end, or to a leap
        // it ends in, which they push off for and nothing lands.
        let body_end = self
            .body
            .leap()
            .map(BodySegment::from)
            .or(self.body.end())
            .unwrap_or(planted.body_s);
        let natural = self.settings.natural_step(self.target_speed, self.params);
        let goal = flights
            .last()
            .map_or(f32::MIN, |flight| flight.landing)
            .max(body_end - 0.5 * natural);

        let costs = FootfallCosts {
            weights: &self.settings.costs,
            swinging: self.swinging,
        };
        let candidates = [Limb::LeftFoot, Limb::RightFoot].map(|limb| {
            FootfallCandidates::new(&self.candidate_spec(&costs, &flights, body_end, limb))
        });
        let search = Expansion {
            flights: &flights,
            body_end,
            costs: &costs,
            candidates: &candidates,
        };

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
        let mut children = Vec::new();

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

            children.clear();
            self.expand(&node, &search, &cheapest, &nodes, &mut children);
            for mut candidate in children.drain(..) {
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
        search: &Expansion,
        cheapest: &FxHashMap<CellKey, usize>,
        nodes: &[Node],
        out: &mut Vec<Node>,
    ) {
        let params = self.params;
        let stance = node.footfall.contact;
        let limb = stance.limb.other();
        let candidates = &search.candidates[usize::from(limb == Limb::RightFoot)];
        let next_flight = search.flights.get(node.next_flight);
        let ridden_until = next_flight.map_or(search.body_end, |flight| flight.takeoff);
        let worth = |footfall: &Footfall, cost: f32| {
            cheapest
                .get(&self.cell(footfall))
                .map_or(true, |&other| nodes[other].cost > cost)
        };

        let stride = stance.at.s + params.stride.min_length..stance.at.s + params.stride.max_length;
        for candidate in candidates.within(stride) {
            let Some(base) = self.footfall_cost(node, limb, candidate, 0.0, search.costs) else {
                continue;
            };
            let foothold = &candidate.foothold;
            let contact = Contact {
                limb,
                footfall: None,
                at: foothold.at,
                normal: foothold.normal,
                footing: foothold.footing,
            };

            // Landing as the body rides on, before any flight.
            let plant_ahead = params.plant_ahead(self.target_speed, foothold.at.s - stance.at.s);
            let lifts = (foothold.at.s + plant_ahead).min(ridden_until);
            for lead in &self.settings.landing_leads {
                let body_s = foothold.at.s - lead * plant_ahead;
                if body_s <= node.footfall.body_s + TOLERANCE || body_s > ridden_until + TOLERANCE {
                    continue;
                }
                let footfall = Footfall { contact, body_s };
                if !worth(&footfall, base) {
                    continue;
                }
                let Some(body_z) = self.body.height_at(body_s, self.query) else {
                    continue;
                };
                if !params.foot_reaches(body_s, body_z, &contact.at)
                    || !self.reaches_until_lifted(&contact, body_s, lifts)
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
        }

        // Landing at the end of the next flight, pushed off from the stance
        // foot with the body over or past it, within a foot's slack of where
        // the body comes down: a running leap, its step judged by the ground
        // either side of the flight.
        let Some(flight) = next_flight else {
            return;
        };
        let landing = flight.landing;
        if flight.takeoff < stance.at.s - PUSH_OFF_TOLERANCE {
            return;
        }
        let flown = landing - flight.takeoff;
        let farthest = landing + FOOT_SLACK;
        for candidate in candidates.within(landing - TOLERANCE..farthest + TOLERANCE) {
            let Some(base) = self.footfall_cost(node, limb, candidate, flown, search.costs) else {
                continue;
            };
            let foothold = &candidate.foothold;
            let contact = Contact {
                limb,
                footfall: None,
                at: foothold.at,
                normal: foothold.normal,
                footing: foothold.footing,
            };
            let footfall = Footfall {
                contact,
                body_s: landing,
            };
            if !worth(&footfall, base)
                || !params.foot_reaches(landing, flight.landing_z, &contact.at)
                || !params.foot_reaches(flight.takeoff, flight.takeoff_z, &stance.at)
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

    /// Whether a foot landed with the body at `body_s` is still within its
    /// reach when the body is at `lifts`, where the foot lifts: the lower of
    /// the two stations binds, and within reach at both it is in between.
    fn reaches_until_lifted(&self, contact: &Contact, body_s: f32, lifts: f32) -> bool {
        if lifts <= body_s + TOLERANCE {
            return true;
        }
        self.body
            .height_at(lifts, self.query)
            .is_some_and(|body_z| self.params.foot_reaches(lifts, body_z, &contact.at))
    }

    /// What a footfall on `candidate` costs after `node`, whatever the body
    /// does meanwhile, `flown` metres of the step a jump rather than a
    /// stride; `None` where no foot may land.
    fn footfall_cost(
        &self,
        node: &Node,
        limb: Limb,
        candidate: &Candidate,
        flown: f32,
        costs: &FootfallCosts,
    ) -> Option<f32> {
        let fixed = candidate.fixed?;
        let (params, weights) = (self.params, &self.settings.costs);
        let foothold = &candidate.foothold;
        let stance = node.footfall.contact.at;
        let rise = foothold.at.z - stance.z;
        if rise > params.stride.max_rise || -rise > params.stride.max_drop {
            return None;
        }
        let length_off = (foothold.at.s - stance.s - flown - fixed.natural) / fixed.natural;
        let contact = Contact {
            limb,
            footfall: None,
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
                + fixed.sideways
                + fixed.edge
                + fixed.footing
                + churn,
        )
    }

    /// What the candidates for `limb`'s footfalls are read for: as far as a
    /// footfall can land, a stride's plant-ahead past where the body rides
    /// to, or a foot's slack past the last flight's landing.
    fn candidate_spec<'a>(
        &'a self,
        costs: &'a FootfallCosts<'a>,
        flights: &[Flight],
        body_end: f32,
        limb: Limb,
    ) -> CandidateSpec<'a> {
        let lead = self
            .settings
            .landing_leads
            .iter()
            .fold(0.0_f32, |most, &lead| most.max(lead));
        let plant_ahead = self
            .params
            .plant_ahead(self.target_speed, self.params.stride.max_length);
        let ridden = body_end + lead * plant_ahead;
        let landed = flights
            .last()
            .map_or(f32::NEG_INFINITY, |flight| flight.landing + FOOT_SLACK);
        CandidateSpec {
            query: self.query,
            params: self.params,
            settings: self.settings,
            costs,
            target_speed: self.target_speed,
            spacing: self.spacing,
            limb,
            until: ridden.max(landed) + 2.0 * TOLERANCE,
        }
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
