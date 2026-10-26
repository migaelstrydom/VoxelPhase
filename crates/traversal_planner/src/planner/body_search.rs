use std::collections::BinaryHeap;

use rustc_hash::FxHashMap;
use smallvec::SmallVec;

use crate::params::GaitParams;
use crate::path::PathVec;
use crate::plan::{climb_reaching, flight_at, BodyState, Motion, Phase};
use crate::section::{FootholdWindow, Footing, SectionQuery};

use super::body_path::{BodyPath, BodySegment};
use super::cheapest_first::Pending;
use super::rules::{
    landed_speed, pressed_speed, recovery_time_lost, time_lost, FOOT_SLACK, MIN_PLANNING_SPEED,
    PUSH_OFF_TOLERANCE,
};
use super::settings::PlannerSettings;

/// Width of the speed bands the body's states are merged in, in m/s.
const SPEED_BAND: f32 = 0.25;
/// Spacing of the footholds looked for to tell whether a station is within a
/// foot's reach of any, in metres.
const SUPPORT_SPACING: f32 = 0.1;
/// How far below the arc of a body flying free the floor may fall and still
/// be ridden where the body could not ride out of where it comes down, in
/// metres: a capsule's contacts absorb a sample's rounding, but a dip's far
/// side is struck.
const FALL_AWAY_SLACK: f32 = 0.005;
/// How far past where a body skims the floor it must be able to ride on, in
/// metres: past it the body is back down.
const SKIM_RUN: f32 = 0.5;
/// How far past where a rolling body's arc meets the floor it may be carried
/// up onto ground it can ride on, in metres: up a gap's far brow.
const MOUNT_REACH: f32 = 0.1;
/// Slack on reaching a station, in metres.
const STATION_TOLERANCE: f32 = 1.0e-4;

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

/// The last plan's body path, as the next search reads it.
#[derive(Clone, Debug, PartialEq)]
pub struct PreviousPath {
    /// The flights it settled on.
    pub flights: Vec<FlightSpan>,
    pub rides: Vec<RideSpan>,
    /// How far along the path it settled, in metres: flying from further on
    /// costs no churn, as it chose nothing there knowing what lay past it.
    pub settled_until: f32,
}

/// A stretch the body rode, along the path, and its speed at the end.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RideSpan {
    pub from: f32,
    pub to: f32,
    pub end_speed: f32,
}

/// Where a flight takes off and lands, along the path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlightSpan {
    pub takeoff: f32,
    pub landing: f32,
}

impl FlightSpan {
    /// Whether a flight from `takeoff` to `landing` spans this one.
    fn is_spanned_by(&self, takeoff: f32, landing: f32) -> bool {
        takeoff <= self.takeoff + STATION_TOLERANCE && landing >= self.landing - STATION_TOLERANCE
    }
}

/// A body path that reaches the goal, and what it costs.
pub struct BodyPlan {
    pub path: BodyPath,
    pub cost: f32,
}

struct Node {
    station: usize,
    speed: f32,
    /// Height of the bottom-sphere centre at the station.
    z: f32,
    cost: f32,
    parent: Option<usize>,
    /// How the body got here from the parent.
    arrival: Option<BodySegment>,
    /// Where along the path the body last touched down: a foot landed at or
    /// ahead of it, so a takeoff pushes off from a foothold no further back.
    landed: f32,
    /// Whether it touched down within a foot's slack of here, where pushing
    /// off depends on where.
    just_landed: bool,
    /// Whether it has flown since the start: until then the planted foot,
    /// whose place is known, is the one that pushes off.
    flown: bool,
}

/// How near a flight's takeoff and landing must be to the last plan's for
/// it to be the same flight, in metres.
const CHURN_MATCH: f32 = 0.1;

/// What the feet need of a station.
#[derive(Clone, Copy, PartialEq, Eq)]
enum FootNeed {
    /// To ride through or land on it: a foothold within half a step.
    Support,
    /// To take off from it: a foothold just behind, to push off from.
    PushOff,
    /// To land on it: a foothold just ahead, to land on.
    TouchDown,
}

/// What the feet can do at each station, as it is worked out.
struct FootReach {
    answers: [Vec<Option<bool>>; 3],
}

impl FootReach {
    fn new(stations: usize) -> Self {
        Self {
            answers: std::array::from_fn(|_| vec![None; stations]),
        }
    }

    fn len(&self) -> usize {
        self.answers[0].len()
    }
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
            .map_or(true, |&other| self.nodes[other].cost > cost)
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
            let (station, speed, z, cost, landed, flown) = {
                let node = &frontier.nodes[index];
                (
                    node.station,
                    node.speed,
                    node.z,
                    node.cost,
                    node.landed,
                    node.flown,
                )
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

            let mut edges: SmallVec<[(usize, f32, f32, f32, BodySegment); 16]> = SmallVec::new();
            // A body in the air flies on the arc it is on until it meets
            // the floor: nothing else is open to it.
            if station == 0 && airborne {
                edges.extend(self.carried_flight(start, known.len()));
            } else {
                self.ride(start.s, station, speed, &mut known, &mut edges);
                self.ride_again(start.s, station, speed, &mut known, &mut edges);
                self.roll_off(
                    start.s,
                    station,
                    speed,
                    z,
                    (landed, flown),
                    &mut known,
                    &mut edges,
                );
                // A flight is checked for clearance only if it could be the
                // cheapest way yet to its landing: it lands just landed.
                let worth = |landing: usize, landing_speed: f32, edge_cost: f32| {
                    frontier.could_improve(
                        &(landing, speed_band(landing_speed), true),
                        cost + edge_cost,
                    )
                };
                self.fly(
                    start.s,
                    station,
                    speed,
                    z,
                    (landed, flown),
                    &worth,
                    &mut known,
                    &mut edges,
                );
                self.leap(
                    start.s,
                    station,
                    speed,
                    z,
                    (landed, flown),
                    &mut known,
                    &mut edges,
                );
            }
            for (next, next_speed, next_z, edge_cost, arrival) in edges {
                let next_s = self.station_s(start.s, next);
                let (landed, flown) = match arrival {
                    BodySegment::Flight {
                        landing,
                        pushed: true,
                        ..
                    } => (landing, true),
                    _ => (landed, flown),
                };
                frontier.relax(
                    Node {
                        station: next,
                        speed: next_speed,
                        z: next_z,
                        cost: cost + edge_cost,
                        parent: Some(index),
                        arrival: Some(arrival),
                        landed,
                        just_landed: next_s - landed < FOOT_SLACK,
                        flown,
                    },
                    self.recovery_due(next_s, next_speed),
                );
            }
        }
        Err(Self::path_to(&frontier.nodes, furthest))
    }

    /// Where station `station` is along the path. Station 0 is the body;
    /// the rest lie on a grid fixed along the path, not to the body, so the
    /// next frame's search, from a body a little further on, tries the same
    /// takeoffs and landings.
    fn station_s(&self, origin: f32, station: usize) -> f32 {
        if station == 0 {
            origin
        } else {
            (self.grid_index(origin) + station as i64) as f32 * self.settings.body_step
        }
    }

    /// The grid station at or behind `origin`.
    fn grid_index(&self, origin: f32) -> i64 {
        ((origin + STATION_TOLERANCE) / self.settings.body_step).floor() as i64
    }

    /// Riding on to the next station, pressing toward the asked-for speed.
    fn ride(
        &self,
        origin: f32,
        station: usize,
        speed: f32,
        known: &mut FootReach,
        out: &mut SmallVec<[(usize, f32, f32, f32, BodySegment); 16]>,
    ) {
        let next = station + 1;
        if next >= known.len() || !self.feet_can(origin, next, FootNeed::Support, known) {
            return;
        }
        let (from, to) = (
            self.station_s(origin, station),
            self.station_s(origin, next),
        );
        let ride = self.query.ride(from, to);
        if ride.end < to - STATION_TOLERANCE || self.falls_away(from, to, speed) {
            return;
        }
        let Some(floor) = self.query.floor(to) else {
            return;
        };
        let pressed_toward = self.ground_target(from, to);
        let (end_speed, met_loss) = self.ridden_speed(from, to, speed, pressed_toward);
        let cost = self.settings.costs.ride_travel * ride.vertical_travel
            + met_loss
            + self.ride_churn(from, to)
            + time_lost(speed, end_speed, to - from, pressed_toward);
        out.push((
            next,
            end_speed,
            floor.z,
            cost,
            BodySegment::Ride {
                from,
                to,
                start_speed: speed,
                end_speed,
            },
        ));
    }

    /// Riding again a stretch the last plan rode from this station, to where
    /// it ended.
    fn ride_again(
        &self,
        origin: f32,
        station: usize,
        speed: f32,
        known: &mut FootReach,
        out: &mut SmallVec<[(usize, f32, f32, f32, BodySegment); 16]>,
    ) {
        let Some(previous) = self.previous else {
            return;
        };
        let step = self.settings.body_step;
        let from = self.station_s(origin, station);
        for ride in &previous.rides {
            if (ride.from - from).abs() > 0.5 * step || ride.to <= from + 0.5 * step {
                continue;
            }
            let next = station + ((ride.to - from) / step).round().max(1.0) as usize;
            if next >= known.len()
                || !(station + 1..=next)
                    .all(|passed| self.feet_can(origin, passed, FootNeed::Support, known))
            {
                continue;
            }
            let to = self.station_s(origin, next);
            let ridden = self.query.ride(from, to);
            let Some(floor) = self
                .query
                .floor(to)
                .filter(|_| ridden.end >= to - STATION_TOLERANCE)
                .filter(|_| !self.falls_away(from, to, speed))
            else {
                continue;
            };
            let pressed_toward = self.ground_target(from, to);
            let (end_speed, met_loss) = self.ridden_speed(from, to, speed, pressed_toward);
            let cost = self.settings.costs.ride_travel * ridden.vertical_travel
                + met_loss
                + time_lost(speed, end_speed, to - from, pressed_toward);
            out.push((
                next,
                end_speed,
                floor.z,
                cost,
                BodySegment::Ride {
                    from,
                    to,
                    start_speed: speed,
                    end_speed,
                },
            ));
        }
    }

    /// The speed across the ground the drive presses toward riding from
    /// `from` to `to`: the asked-for speed, less what the floor's grade
    /// between them costs.
    fn ground_target(&self, from: f32, to: f32) -> f32 {
        match (self.query.floor(from), self.query.floor(to)) {
            (Some(low), Some(high)) => self
                .params
                .body
                .ground_speed(self.target_speed, (high.z - low.z) / (to - from)),
            _ => self.target_speed,
        }
    }

    /// The grade the body leaves the floor at `s` with: the floor's over the
    /// station before, level where there is none.
    fn leaving_grade(&self, s: f32) -> f32 {
        let step = self.settings.body_step;
        match (self.query.floor(s - step), self.query.floor(s)) {
            (Some(before), Some(at)) => (at.z - before.z) / step,
            _ => 0.0,
        }
    }

    /// Where a body leaving the floor at `from` at `speed`, climbing as the
    /// floor did, is `along` metres on: free of it, gravity pulls it down no
    /// faster than this. Leaving while rising, as off a brow, it rises on for
    /// [`BodyDims::rise_held`](crate::params::BodyDims::rise_held) at most,
    /// then falls from where the drive cancelled the rise.
    fn free_height(&self, from: f32, z: f32, speed: f32, along: f32) -> f32 {
        let speed = speed.max(MIN_PLANNING_SPEED);
        let leaving = BodyState {
            s: from,
            z,
            speed,
            climb: speed * self.leaving_grade(from),
        };
        let rise_held = Some(self.params.body.rise_held);
        flight_at(&leaving, self.params.gravity, rise_held, along / speed).0
    }

    /// The speed a body riding from `from` to `to` at `speed` ends with,
    /// station by station meeting the floor's faces and pressing toward
    /// `target`, and what the speed its faces took costs.
    fn ridden_speed(&self, from: f32, to: f32, speed: f32, target: f32) -> (f32, f32) {
        let step = self.settings.body_step;
        let stations = ((to - from) / step).round().max(1.0) as usize;
        let run = (to - from) / stations as f32;
        let (mut speed, mut cost) = (speed, 0.0);
        for index in 0..stations {
            let at = from + index as f32 * run;
            let met = self.met_speed(at, at + run, speed);
            cost += self.settings.costs.landing_loss * (speed - met).powi(2);
            speed = pressed_speed(met, run, target, self.params);
        }
        (speed, cost)
    }

    /// The speed along the path a body riding from `from` to `to` at `speed`
    /// keeps where the floor there turns up more steeply than at `from`: it
    /// meets it as a face, which takes what runs into it and leaves what runs
    /// along it ([`landed_speed`]). Read from the floor's normals, which a
    /// corner between stations does not smear.
    fn met_speed(&self, from: f32, to: f32, speed: f32) -> f32 {
        let (Some(before), Some(after)) = (self.query.floor(from), self.query.floor(to)) else {
            return speed;
        };
        let grade = |normal: PathVec| -normal.s / normal.z.max(f32::EPSILON);
        let (riding, meeting) = (grade(before.normal), grade(after.normal));
        if meeting <= riding {
            return speed;
        }
        landed_speed(speed, speed * riding, after.normal)
    }

    /// Whether the floor between `from` and `to` drops away from the body
    /// riding it at `speed` faster than gravity can follow: past `from` the
    /// body flies, over a gap or off a step down, rather than dipping. Over
    /// a brow, or an arch's corners, it skims by up to the body's skim height
    /// and rides on, where it can ride on from where it comes down.
    ///
    /// A body leaving while rising is followed past `to`, up to a stride,
    /// until its arc is back down on the floor: off a brow the floor falls
    /// away under the climb it carries.
    fn falls_away(&self, from: f32, to: f32, speed: f32) -> bool {
        let Some(start) = self.query.floor(from) else {
            return false;
        };
        let step = self.settings.body_step;
        let stations = ((to - from) / step).round().max(1.0) as usize;
        let followed = if self.leaving_grade(from) > 0.0 {
            stations.max((self.params.stride.max_length / step).round() as usize)
        } else {
            stations
        };
        for index in 1..=followed {
            let along = index as f32 * step;
            let Some(floor) = self.query.floor(from + along) else {
                continue;
            };
            let below = self.free_height(from, start.z, speed, along) - floor.z;
            if below > self.params.body.skim_height
                || (below > FALL_AWAY_SLACK && !self.rides_on(from + along, SKIM_RUN))
            {
                return true;
            }
            if index >= stations && below <= FALL_AWAY_SLACK {
                break;
            }
        }
        false
    }

    /// Whether the floor can be ridden on for `run` past `s`: no face too
    /// steep to ride, as a gap's far side, rises across it.
    fn rides_on(&self, s: f32, run: f32) -> bool {
        self.query.ride(s, s + run).end >= s + run - STATION_TOLERANCE
    }

    /// Rolling off where the floor falls away: no push, the body flies on
    /// the speed and climb it rode with until its arc meets the floor again,
    /// as far as that is. Its feet are a hop's: the last near the brow, the
    /// next at the landing. Meeting the far side of a gap
    /// below its brow, it is carried up onto it, as a capsule rides up a
    /// rounded corner, where it meets it at a slope the drive rides: it
    /// lands on the first station near there that it can ride on from, its
    /// climb set to reach it.
    fn roll_off(
        &self,
        origin: f32,
        station: usize,
        speed: f32,
        z: f32,
        landed: (f32, bool),
        known: &mut FootReach,
        out: &mut SmallVec<[(usize, f32, f32, f32, BodySegment); 16]>,
    ) {
        let (params, settings) = (self.params, self.settings);
        let step = settings.body_step;
        let from = self.station_s(origin, station);
        if !self.falls_away(from, from + step, speed)
            || !self.pushes_off(origin, station, z, landed, known)
        {
            return;
        }
        let Some(start) = self.query.floor(from) else {
            return;
        };
        let last = known.len() - 1;
        let Some(meets) = (station + 1..=last).find(|&next| {
            let along = self.station_s(origin, next) - from;
            self.query
                .floor(from + along)
                .is_some_and(|floor| floor.z >= self.free_height(from, start.z, speed, along))
        }) else {
            return;
        };
        let steepest = params.body.steepest_ride().cos();
        // Where the arc meets the floor the cast's normal is the contact's:
        // too steep, and the body strikes the far side rather than mounts it.
        if self
            .query
            .floor(self.station_s(origin, meets))
            .is_none_or(|floor| floor.normal.z < steepest)
        {
            return;
        }
        let carried = (MOUNT_REACH / step).round() as usize;
        let Some(landing_station) = (meets..=(meets + carried).min(last)).find(|&next| {
            let landing = self.station_s(origin, next);
            self.query.floor(landing).is_some_and(|floor| {
                floor.normal.z >= steepest
                    && self.query.ride(landing, landing + step).end
                        >= landing + step - STATION_TOLERANCE
            }) && self.feet_can(origin, next, FootNeed::TouchDown, known)
        }) else {
            return;
        };
        let landing = self.station_s(origin, landing_station);
        let Some(floor) = self.query.floor(landing) else {
            return;
        };
        let (duration, gravity) = ((landing - from) / speed, params.gravity);
        // Only a body leaving while rising has a rise for the drive to cut:
        // one carried up a gap's far brow is held to it, not flying free.
        let rise_held = (self.leaving_grade(from) > 0.0).then_some(params.body.rise_held);
        let takeoff = BodyState {
            s: from,
            z: start.z,
            speed,
            climb: climb_reaching(floor.z - start.z, duration, gravity, rise_held),
        };
        let phase = Phase {
            start: 0.0,
            duration,
            contacts: SmallVec::new(),
            motion: Motion::Flight {
                takeoff,
                gravity,
                rise_held,
            },
        };
        if !self.query.clears(&phase) {
            return;
        }
        // Unheld as in any flight, though nothing pushed it.
        let cost = settings.costs.flight
            + settings.costs.ride_travel * (start.z - floor.z).abs()
            + self.flight_churn(from, landing)
            + time_lost(speed, speed, landing - from, self.target_speed);
        let landing_climb = flight_at(&takeoff, gravity, rise_held, duration).1;
        let landed = landed_speed(speed, landing_climb, floor.normal).max(MIN_PLANNING_SPEED);
        let cost = cost + settings.costs.landing_loss * (speed - landed).powi(2);
        out.push((
            landing_station,
            landed,
            floor.z,
            cost,
            BodySegment::Flight {
                takeoff,
                landing,
                landing_z: floor.z,
                gravity,
                rise_held,
                pushed: false,
            },
        ));
    }

    /// Flying from this station, where the ride ahead breaks or gets rough
    /// within a hop's reach, to any landing the climb it asks for allows: a
    /// running leap, its feet a push-off and a landing, not a stride.
    fn fly(
        &self,
        origin: f32,
        station: usize,
        speed: f32,
        z: f32,
        (landed, flown): (f32, bool),
        worth: &dyn Fn(usize, f32, f32) -> bool,
        known: &mut FootReach,
        out: &mut SmallVec<[(usize, f32, f32, f32, BodySegment); 16]>,
    ) {
        let (params, settings) = (self.params, self.settings);
        let from = self.station_s(origin, station);
        if !self.trouble_ahead(from, speed)
            || !self.pushes_off(origin, station, z, (landed, flown), known)
        {
            return;
        }
        let steepest = params.body.steepest_ride().cos();
        let stride = (settings.flight_step / settings.body_step).round().max(1.0) as usize;
        if (self.grid_index(origin) + station as i64) % stride as i64 != 0 {
            return;
        }
        // A push adds to the climb the body leaves with, never takes from it.
        let carried = (speed * self.leaving_grade(from)).max(0.0);
        // As far as was sampled: the climb a landing asks for bounds a hop.
        let first = station
            + ((settings.min_flight - STATION_TOLERANCE) / settings.body_step).ceil() as usize;
        for landing_station in (first..known.len()).step_by(stride) {
            let landing = self.station_s(origin, landing_station);
            if self
                .vetoed
                .iter()
                .any(|span| span.is_spanned_by(from, landing))
            {
                continue;
            }
            let Some(floor) = self.query.floor(landing) else {
                continue;
            };
            if floor.normal.z < steepest
                || !self.feet_can(origin, landing_station, FootNeed::TouchDown, known)
            {
                continue;
            }
            let duration = (landing - from) / speed;
            let gravity = params.gravity;
            let climb = (floor.z - z + 0.5 * gravity * duration * duration) / duration;
            if !(carried..=params.flight.max_takeoff_climb).contains(&climb) {
                continue;
            }
            let takeoff = BodyState {
                s: from,
                z,
                speed,
                climb,
            };
            let phase = Phase {
                start: 0.0,
                duration,
                contacts: SmallVec::new(),
                motion: Motion::Flight {
                    takeoff,
                    gravity,
                    rise_held: None,
                },
            };
            let cost = settings.costs.flight
                + settings.costs.takeoff_climb * climb * climb
                + self.flight_churn(from, landing)
                + time_lost(speed, speed, landing - from, self.target_speed);
            let landed = landed_speed(speed, climb - gravity * duration, floor.normal)
                .max(MIN_PLANNING_SPEED);
            let cost = cost + settings.costs.landing_loss * (speed - landed).powi(2);
            if !worth(landing_station, landed, cost) || !self.query.clears(&phase) {
                continue;
            }
            out.push((
                landing_station,
                landed,
                floor.z,
                cost,
                BodySegment::Flight {
                    takeoff,
                    landing,
                    landing_z: floor.z,
                    gravity,
                    rise_held: None,
                    pushed: true,
                },
            ));
        }
    }

    /// The flight a body `start`ing in the air is on: from where it is, at
    /// its own climb, unpushed, to the first of the `stations` where its arc
    /// meets the floor, its climb set to land there; past the last without
    /// meeting one, a leap into what lies out of sight.
    fn carried_flight(
        &self,
        start: BodyState,
        stations: usize,
    ) -> Option<(usize, f32, f32, f32, BodySegment)> {
        let gravity = self.params.gravity;
        let speed = start.speed.max(MIN_PLANNING_SPEED);
        let takeoff = BodyState { speed, ..start };
        let meets = (1..stations).find_map(|station| {
            let s = self.station_s(start.s, station);
            let t = (s - start.s) / speed;
            let floor = self.query.floor(s)?;
            (floor.z >= flight_at(&takeoff, gravity, None, t).0).then_some((station, s, floor))
        });
        let Some((station, landing, floor)) = meets else {
            let last = stations - 1;
            let until = self.station_s(start.s, last);
            let t = (until - start.s) / speed;
            let end_z = flight_at(&takeoff, gravity, None, t).0;
            return Some((
                last,
                speed,
                end_z,
                0.0,
                BodySegment::Leap {
                    takeoff,
                    until,
                    gravity,
                },
            ));
        };
        let duration = (landing - start.s) / speed;
        let takeoff = BodyState {
            climb: climb_reaching(floor.z - start.z, duration, gravity, None),
            ..takeoff
        };
        let landing_climb = flight_at(&takeoff, gravity, None, duration).1;
        let landed = landed_speed(speed, landing_climb, floor.normal).max(MIN_PLANNING_SPEED);
        Some((
            station,
            landed,
            floor.z,
            0.0,
            BodySegment::Flight {
                takeoff,
                landing,
                landing_z: floor.z,
                gravity,
                rise_held: None,
                pushed: false,
            },
        ))
    }

    /// Whether a foot behind the takeoff at `station` can push the body off:
    /// the planted one, until the first flight; after it, one that landed at
    /// or after the body last touched down.
    fn pushes_off(
        &self,
        origin: f32,
        station: usize,
        z: f32,
        (landed, flown): (f32, bool),
        known: &mut FootReach,
    ) -> bool {
        let from = self.station_s(origin, station);
        if from < landed - PUSH_OFF_TOLERANCE {
            false
        } else if from - landed >= FOOT_SLACK {
            self.feet_can(origin, station, FootNeed::PushOff, known)
        } else {
            !flown || self.foothold_within(landed..from + PUSH_OFF_TOLERANCE, from, z)
        }
    }

    /// Leaping off a brow, where the floor is seen to drop away within a
    /// stride and the body can ride no further, into what lies out of sight: pushed off with
    /// `leap_climb`, its arc meets no floor up to where the ground was
    /// sampled, so the body is taken to fall on past it, as off a drop too
    /// deep to see the bottom of. The path ends in the air there; replanned
    /// as the ground below comes into sight, the leap becomes a hop that
    /// lands. Where nothing is sampled past the brow, or the floor ahead
    /// rises, there is nothing seen to leap off, and a body already in the
    /// air has no brow under it to push off.
    fn leap(
        &self,
        origin: f32,
        station: usize,
        speed: f32,
        z: f32,
        landed: (f32, bool),
        known: &mut FootReach,
        out: &mut SmallVec<[(usize, f32, f32, f32, BodySegment); 16]>,
    ) {
        let (params, settings) = (self.params, self.settings);
        let last = known.len() - 1;
        if station >= last {
            return;
        }
        let (from, next, until) = (
            self.station_s(origin, station),
            self.station_s(origin, station + 1),
            self.station_s(origin, last),
        );
        let rides_on = self.query.ride(from, next).end >= next - STATION_TOLERANCE
            && !self.falls_away(from, next, speed);
        let on_brow = self
            .query
            .floor(from)
            .is_some_and(|floor| (z - floor.z).abs() <= params.body.skim_height);
        let stride = (params.stride.max_length / settings.body_step).ceil() as usize;
        let drops = on_brow
            && (station + 1..=(station + stride).min(last)).any(|ahead| {
                self.query
                    .floor(self.station_s(origin, ahead))
                    .is_some_and(|below| below.z < z - params.body.skim_height)
            });
        if rides_on
            || !drops
            || self
                .vetoed
                .iter()
                .any(|span| span.is_spanned_by(from, until))
            || !self.pushes_off(origin, station, z, landed, known)
        {
            return;
        }
        let carried = (speed * self.leaving_grade(from)).max(0.0);
        let climb = settings
            .leap_climb
            .max(carried)
            .min(params.flight.max_takeoff_climb);
        let takeoff = BodyState {
            s: from,
            z,
            speed,
            climb,
        };
        let gravity = params.gravity;
        let phase = Phase {
            start: 0.0,
            duration: (until - from) / speed,
            contacts: SmallVec::new(),
            motion: Motion::Flight {
                takeoff,
                gravity,
                rise_held: None,
            },
        };
        let Some(end_z) = phase.flight_height(until) else {
            return;
        };
        if !self.query.clears(&phase) {
            return;
        }
        // Its climb is set, not chosen: it is not traded against a hop's.
        let cost = settings.costs.flight + self.flight_churn(from, until);
        out.push((
            last,
            speed,
            end_z,
            cost,
            BodySegment::Leap {
                takeoff,
                until,
                gravity,
            },
        ));
    }

    /// What flying from `takeoff` to `landing` costs in churn: nothing if
    /// the last plan flew there too, or settled nothing there.
    fn flight_churn(&self, takeoff: f32, landing: f32) -> f32 {
        let Some(previous) = self
            .previous
            .filter(|previous| takeoff < previous.settled_until)
        else {
            return 0.0;
        };
        let matches = previous.flights.iter().any(|span| {
            (span.takeoff - takeoff).abs() <= CHURN_MATCH
                && (span.landing - landing).abs() <= CHURN_MATCH
        });
        if matches {
            0.0
        } else {
            self.settings.costs.churn
        }
    }

    /// What riding from `from` to `to` costs in churn: riding on past where
    /// the last plan took off.
    fn ride_churn(&self, from: f32, to: f32) -> f32 {
        let passes_takeoff = self.previous.is_some_and(|previous| {
            previous.flights.iter().any(|span| {
                let left_behind = span.takeoff + CHURN_MATCH;
                from < left_behind && to >= left_behind
            })
        });
        if passes_takeoff {
            self.settings.costs.churn
        } else {
            0.0
        }
    }

    /// Whether a foothold in `window` along the path is in reach of the body
    /// at height `body_z`, and one a foot may land on.
    fn foothold_within(&self, window: std::ops::Range<f32>, body_s: f32, body_z: f32) -> bool {
        let mut footholds = Vec::new();
        self.query.footholds(
            &FootholdWindow {
                s: window,
                d: f32::MIN..f32::MAX,
                spacing: SUPPORT_SPACING,
            },
            &mut footholds,
        );
        footholds.iter().any(|foothold| {
            foothold.edge_margin >= self.params.foot.min_edge_margin
                && !matches!(foothold.footing, Footing::Loose { resting: false, .. })
                && self.params.foot_reaches(body_s, body_z, &foothold.at)
        })
    }

    /// Whether the ride within a hop's reach ahead of `from` breaks, gets
    /// rough, or falls away from a body at `speed`: only then is flying
    /// worth trying.
    fn trouble_ahead(&self, from: f32, speed: f32) -> bool {
        let reach = self.params.longest_hop(speed);
        let ahead = self.query.ride(from, from + reach);
        ahead.end < from + reach - STATION_TOLERANCE
            || ahead.vertical_travel >= self.settings.rough_ride
            || self.falls_away(from, ahead.end, speed)
    }

    /// Seconds a body at `speed` at `s` has still to lose regaining the
    /// speed the drive presses toward there: a lower bound on what the way
    /// on costs, and what a plan arriving slow at its goal is charged.
    fn recovery_due(&self, s: f32, speed: f32) -> f32 {
        let target = self.ground_target(s, s + self.settings.body_step);
        recovery_time_lost(speed, target, self.params.body.max_acceleration)
    }

    fn natural_step(&self) -> f32 {
        self.settings.natural_step(self.target_speed, self.params)
    }

    /// Whether the feet can do what `need` asks at a station, worked out once
    /// per station: a foothold in reach of the body resting there, in the
    /// window `need` names, that a foot may land on.
    fn feet_can(&self, origin: f32, station: usize, need: FootNeed, known: &mut FootReach) -> bool {
        let slot = &mut known.answers[need as usize][station];
        if let Some(answer) = *slot {
            return answer;
        }
        let s = self.station_s(origin, station);
        let half = 0.5 * self.params.stride.max_length;
        let window = match need {
            FootNeed::Support => s - half..s + half,
            FootNeed::PushOff => s - FOOT_SLACK..s + PUSH_OFF_TOLERANCE,
            FootNeed::TouchDown => s - STATION_TOLERANCE..s + FOOT_SLACK,
        };
        let answer = (station == 0 && need == FootNeed::Support)
            || self
                .query
                .floor(s)
                .is_some_and(|floor| self.foothold_within(window, s, floor.z));
        known.answers[need as usize][station] = Some(answer);
        answer
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
