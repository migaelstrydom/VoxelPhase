use std::collections::BinaryHeap;

use rustc_hash::FxHashMap;
use smallvec::SmallVec;

use crate::params::GaitParams;
use crate::plan::BodyState;
use crate::planner::body_path::{BodyPath, BodySegment};
use crate::planner::cheapest_first::Pending;
use crate::planner::rules::{recovery_time_lost, FOOT_SLACK, MIN_PLANNING_SPEED};
use crate::planner::settings::PlannerSettings;
use crate::section::SectionQuery;

use super::feet::FootReach;
use super::previous::{FlightSpan, PreviousPath};

/// Width of the speed bands the body's states are merged in, in m/s.
const SPEED_BAND: f32 = 0.25;
/// Slack on reaching a station, in metres.
pub(super) const STATION_TOLERANCE: f32 = 1.0e-4;

/// The cheapest path for the body alone: where it rides, where it flies and
/// how fast, out to the plan's goal. Footfalls are placed along it after.
///
/// The path is searched over stations every `body_step` along the path, each
/// with the body's speed. From a station the body rides to the next, pressing
/// toward the asked-for speed, or flies to a landing up to a step ahead:
/// flights are tried only where the ride ahead breaks, gets rough or falls
/// away, so open ground is walked at speed. The feet are asked only what they can
/// do near a station: it is ridden through only with a foothold within half
/// a step, so the body is never carried over a void propped on the corners of
/// stones either side; a flight needs a foothold just behind its takeoff and
/// just past its landing, and is short enough for one step to span them.
///
/// New ways for the body to move (climbing, hanging) are new kinds of edge
/// out of a station.
pub struct BodySearch<'a> {
    pub query: &'a dyn SectionQuery,
    pub params: &'a GaitParams,
    pub settings: &'a PlannerSettings,
    /// The speed along the path the player asks for, in m/s.
    pub target_speed: f32,
    /// Flights the feet could not follow: no flight spanning one is tried.
    pub vetoed: &'a [FlightSpan],
    /// Where along the path the body's path must reach.
    pub goal: f32,
    /// Where along the path the foot holding the body is planted: no takeoff
    /// before it, since that foot pushes off.
    pub planted: f32,
    /// The last plan's body path, if there was a last plan: leaving the
    /// flights it settled on costs churn, and its rides are offered again as
    /// they were, so it stays within reach whatever the search's own steps.
    pub previous: Option<&'a PreviousPath>,
}

/// A body path that reaches the goal, and what it costs.
pub struct BodyPlan {
    pub path: BodyPath,
    pub cost: f32,
}

/// A way on out of a station: where it leads, and what it costs.
pub(super) struct Edge {
    pub station: usize,
    /// The body's speed along the path there.
    pub speed: f32,
    /// Height of the bottom-sphere centre there.
    pub z: f32,
    pub cost: f32,
    /// How the body gets there.
    pub arrival: BodySegment,
}

/// The ways on out of a station.
pub(super) type Edges = SmallVec<[Edge; 16]>;

/// The body at a station, as the search reached it.
pub(super) struct Node {
    pub station: usize,
    pub speed: f32,
    /// Height of the bottom-sphere centre at the station.
    pub z: f32,
    cost: f32,
    parent: Option<usize>,
    /// How the body got here from the parent.
    arrival: Option<BodySegment>,
    /// Where along the path the body last touched down: a foot landed at or
    /// ahead of it, so a takeoff pushes off from a foothold no further back.
    pub landed: f32,
    /// Whether it touched down within a foot's slack of here, where pushing
    /// off depends on where.
    just_landed: bool,
    /// Whether it has flown since the start: until then the planted foot,
    /// whose place is known, is the one that pushes off.
    pub flown: bool,
}

/// How far apart two rides' accelerations may be, in m/s², for them to be
/// one: a ride's phase holds its acceleration, so pressing to the asked-for
/// speed and holding it there stay two.
const SAME_ACCELERATION: f32 = 0.05;

/// Whether two rides keep the same acceleration, so one phase can hold both.
fn same_acceleration(a: &BodySegment, b: &BodySegment) -> bool {
    match (a.ride_acceleration(), b.ride_acceleration()) {
        (Some(a), Some(b)) => (a - b).abs() <= SAME_ACCELERATION,
        _ => false,
    }
}

/// The speed band `speed` falls in.
fn speed_band(speed: f32) -> i32 {
    (speed / SPEED_BAND).round() as i32
}

/// What the search keeps between expansions.
struct Frontier {
    nodes: Vec<Node>,
    cheapest: FxHashMap<(usize, i32, bool), usize>,
    pending: BinaryHeap<Pending>,
}

impl Frontier {
    fn key(node: &Node) -> (usize, i32, bool) {
        (node.station, speed_band(node.speed), node.just_landed)
    }

    /// Whether a state with `key` reached at `cost` would be the cheapest yet.
    fn could_improve(&self, key: &(usize, i32, bool), cost: f32) -> bool {
        self.cheapest
            .get(key)
            .is_none_or(|&other| self.nodes[other].cost > cost)
    }

    /// Adds `node` if it is the cheapest way yet to its station and speed,
    /// queued by its cost plus `estimate`, a lower bound on what it has still
    /// to pay.
    fn relax(&mut self, node: Node, estimate: f32) {
        let key = Self::key(&node);
        if self
            .cheapest
            .get(&key)
            .is_some_and(|&other| self.nodes[other].cost <= node.cost)
        {
            return;
        }
        let cost = node.cost + estimate;
        self.nodes.push(node);
        let index = self.nodes.len() - 1;
        self.cheapest.insert(key, index);
        self.pending.push(Pending { cost, node: index });
    }

    fn is_current(&self, index: usize) -> bool {
        self.cheapest.get(&Self::key(&self.nodes[index])) == Some(&index)
    }
}

impl BodySearch<'_> {
    /// The cheapest body path from `start` to the goal, or, if none reaches
    /// it, the cheapest path to the furthest station reached.
    pub fn search(&self, start: BodyState) -> Result<BodyPlan, BodyPath> {
        let step = self.settings.body_step;
        let goal = self.goal - start.s;
        let flight_reach = self.params.longest_hop(start.speed.max(self.target_speed));
        let natural = self.natural_step();
        // As far as the ground was sampled, for a flight off an edge to land
        // or to leap into what lies past it.
        let searched = (goal + flight_reach + natural).max(self.query.sampled_to() - start.s);
        let stations = (searched / step).ceil() as usize + 1;
        let mut known = FootReach::new(stations + 1);
        let airborne = self
            .query
            .floor(start.s)
            .is_none_or(|floor| start.z > floor.z + self.params.body.skim_height);

        let mut frontier = Frontier {
            nodes: Vec::new(),
            cheapest: FxHashMap::default(),
            pending: BinaryHeap::new(),
        };
        frontier.relax(
            Node {
                station: 0,
                speed: start.speed.max(MIN_PLANNING_SPEED),
                z: start.z,
                cost: 0.0,
                parent: None,
                arrival: None,
                landed: self.planted,
                just_landed: start.s - self.planted < FOOT_SLACK,
                flown: false,
            },
            0.0,
        );
        let mut furthest = 0;

        while let Some(Pending { node: index, .. }) = frontier.pending.pop() {
            if !frontier.is_current(index) {
                continue;
            }
            let (station, speed, cost, landed, flown) = {
                let node = &frontier.nodes[index];
                (node.station, node.speed, node.cost, node.landed, node.flown)
            };
            let s = self.station_s(start.s, station);
            // At the goal, and a step past the last landing: what the plan
            // commits to is chosen knowing it goes on from there.
            let stepped_on = !flown || s - landed >= natural - STATION_TOLERANCE;
            if s - start.s >= goal - STATION_TOLERANCE && stepped_on {
                return Ok(BodyPlan {
                    path: Self::path_to(&frontier.nodes, index),
                    cost: cost + self.recovery_due(s, speed),
                });
            }
            let further = (station, -cost);
            let best = (
                frontier.nodes[furthest].station,
                -frontier.nodes[furthest].cost,
            );
            if further.0 > best.0 || (further.0 == best.0 && further.1 > best.1) {
                furthest = index;
            }

            let mut edges = Edges::new();
            // A body in the air flies on the arc it is on until it meets
            // the floor: nothing else is open to it.
            if station == 0 && airborne {
                edges.extend(self.carried_flight(start, known.len()));
            } else {
                self.ride(start.s, station, speed, &mut known, &mut edges);
                self.ride_again(start.s, station, speed, &mut known, &mut edges);
                let at = &frontier.nodes[index];
                self.roll_off(start.s, at, &mut known, &mut edges);
                // A flight is checked for clearance only if it could be the
                // cheapest way yet to its landing: it lands just landed.
                let worth = |landing: usize, landing_speed: f32, edge_cost: f32| {
                    frontier.could_improve(
                        &(landing, speed_band(landing_speed), true),
                        cost + edge_cost,
                    )
                };
                self.fly(start.s, at, &worth, &mut known, &mut edges);
                self.leap(start.s, at, &mut known, &mut edges);
            }
            for edge in edges {
                let next_s = self.station_s(start.s, edge.station);
                let (landed, flown) = match edge.arrival {
                    BodySegment::Flight {
                        landing,
                        pushed: true,
                        ..
                    } => (landing, true),
                    _ => (landed, flown),
                };
                frontier.relax(
                    Node {
                        station: edge.station,
                        speed: edge.speed,
                        z: edge.z,
                        cost: cost + edge.cost,
                        parent: Some(index),
                        arrival: Some(edge.arrival),
                        landed,
                        just_landed: next_s - landed < FOOT_SLACK,
                        flown,
                    },
                    self.recovery_due(next_s, edge.speed),
                );
            }
        }
        Err(Self::path_to(&frontier.nodes, furthest))
    }

    /// Where station `station` is along the path. Station 0 is the body;
    /// the rest lie on a grid fixed along the path, not to the body, so the
    /// next frame's search, from a body a little further on, tries the same
    /// takeoffs and landings.
    pub(super) fn station_s(&self, origin: f32, station: usize) -> f32 {
        if station == 0 {
            origin
        } else {
            (self.grid_index(origin) + station as i64) as f32 * self.settings.body_step
        }
    }

    /// The grid station at or behind `origin`.
    pub(super) fn grid_index(&self, origin: f32) -> i64 {
        ((origin + STATION_TOLERANCE) / self.settings.body_step).floor() as i64
    }

    /// Seconds a body at `speed` at `s` has still to lose regaining the
    /// speed the drive presses toward there: a lower bound on what the way
    /// on costs, and what a plan arriving slow at its goal is charged.
    pub(super) fn recovery_due(&self, s: f32, speed: f32) -> f32 {
        let target = self.ground_target(s, s + self.settings.body_step);
        recovery_time_lost(speed, target, self.params.body.max_acceleration)
    }

    pub(super) fn natural_step(&self) -> f32 {
        self.settings.natural_step(self.target_speed, self.params)
    }

    /// The path from the start to `last`, consecutive rides merged.
    fn path_to(nodes: &[Node], last: usize) -> BodyPath {
        let mut arrivals = Vec::new();
        let mut index = Some(last);
        while let Some(at) = index {
            if let Some(arrival) = nodes[at].arrival {
                arrivals.push(arrival);
            }
            index = nodes[at].parent;
        }
        arrivals.reverse();
        let mut segments: Vec<BodySegment> = Vec::new();
        for arrival in arrivals {
            let merges = segments
                .last()
                .is_some_and(|last| same_acceleration(last, &arrival));
            match (segments.last_mut(), arrival) {
                (
                    Some(BodySegment::Ride { to, end_speed, .. }),
                    BodySegment::Ride {
                        to: next_to,
                        end_speed: next_end,
                        ..
                    },
                ) if merges => {
                    *to = next_to;
                    *end_speed = next_end;
                }
                _ => segments.push(arrival),
            }
        }
        BodyPath { segments }
    }
}
