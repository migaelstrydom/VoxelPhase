use std::collections::BinaryHeap;

use rustc_hash::FxHashMap;
use smallvec::SmallVec;

use crate::params::GaitParams;
use crate::plan::{BodyState, Motion, Phase};
use crate::section::{FootholdWindow, Footing, SectionQuery};

use super::body_path::{BodyPath, BodySegment};
use super::cheapest_first::Pending;
use super::rules::{slowdown_cost, Pace, MIN_PLANNING_SPEED, PUSH_OFF_TOLERANCE};
use super::settings::PlannerSettings;

/// Width of the speed bands the body's states are merged in, in m/s: fine
/// enough that a stride's worth of braking ahead of a takeoff spans a band.
const SPEED_BAND: f32 = 0.25;
/// Spacing of the footholds looked for to tell whether a station is within a
/// foot's reach of any, in metres.
const SUPPORT_SPACING: f32 = 0.1;
/// How far below the arc of a body flying free the floor may fall and still
/// be ridden, in metres: a capsule's contacts absorb a sample's rounding,
/// not a dip.
const FALL_AWAY_SLACK: f32 = 0.005;
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
/// toward the asked-for speed or braking, or flies to a landing up to a
/// step ahead: braking and flights are tried only where the ride ahead
/// breaks or gets rough, so open ground is walked at speed. The feet are asked only what they can
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
    /// The last plan's body path, if there was a last plan: one the feet
    /// followed, so leaving it costs churn, and its rides are offered again
    /// as they were, so it stays within reach whatever the search's own
    /// steps.
    pub previous: Option<&'a PreviousPath>,
}

/// The last plan's body path, as the next search reads it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PreviousPath {
    pub flights: Vec<FlightSpan>,
    pub rides: Vec<RideSpan>,
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
/// Stretches the slowdown still due is summed over.
const HEURISTIC_PIECES: usize = 4;
/// Share of the slowdown still due the search counts on, below one so the
/// estimate stays a lower bound.
const HEURISTIC_SLACK: f32 = 0.9;
/// How far a foot may be behind a takeoff or past a landing, in metres.
const FOOT_SLACK: f32 = 0.15;

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
        let flight_reach = self.params.stride.max_length;
        let stations = ((goal + flight_reach) / step).ceil() as usize + 1;
        let mut known = FootReach::new(stations + 1);

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
            if s - start.s >= goal - STATION_TOLERANCE {
                return Ok(BodyPlan {
                    path: Self::path_to(&frontier.nodes, index),
                    cost,
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
            self.ride(start.s, station, speed, &mut known, &mut edges);
            self.ride_again(start.s, station, speed, &mut known, &mut edges);
            self.roll_off(start.s, station, speed, &mut known, &mut edges);
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
            for (next, next_speed, next_z, edge_cost, arrival) in edges {
                let next_s = self.station_s(start.s, next);
                let remaining = goal - (next_s - start.s);
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
                    self.slowdown_still_due(next_speed, remaining),
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

    /// Riding on, pressing toward the asked-for speed to the next station,
    /// or braking over far enough to slow by about a speed band: braking one
    /// station at a time would never leave its band, and lose to pressing.
    fn ride(
        &self,
        origin: f32,
        station: usize,
        speed: f32,
        known: &mut FootReach,
        out: &mut SmallVec<[(usize, f32, f32, f32, BodySegment); 16]>,
    ) {
        let step = self.settings.body_step;
        let natural = self.natural_step();
        // Braking a band's worth, or half that: half still leaves the band
        // at the speeds that matter, and the two together fit the distance
        // left to a takeoff far more closely than either alone.
        let band_span = (SPEED_BAND * speed / (self.params.body.max_acceleration * step))
            .ceil()
            .max(1.0) as usize;
        let mut paces: SmallVec<[(Pace, usize); 8]> = SmallVec::from_slice(&[(Pace::Press, 1)]);
        if self.trouble_ahead(self.station_s(origin, station)) {
            paces.extend((band_span.div_ceil(2)..=band_span).map(|span| (Pace::Brake, span)));
        }
        for &(pace, span) in &paces {
            let next = station + span;
            if next >= known.len()
                || !(station + 1..=next)
                    .all(|passed| self.feet_can(origin, passed, FootNeed::Support, known))
            {
                continue;
            }
            let (from, to) = (
                self.station_s(origin, station),
                self.station_s(origin, next),
            );
            let ride = self.query.ride(from, to);
            if ride.end < to - STATION_TOLERANCE || self.falls_away(from, to, speed) {
                continue;
            }
            let Some(floor) = self.query.floor(to) else {
                continue;
            };
            let pressed_toward = self.ground_target(from, to);
            let end_speed = pace.speed_after(speed, to - from, pressed_toward, self.params);
            if pace == Pace::Brake && end_speed >= speed {
                continue;
            }
            let cost = self.settings.costs.ride_travel * ride.vertical_travel
                + self.ride_churn(from, to)
                + slowdown_cost(
                    end_speed,
                    self.target_speed,
                    to - from,
                    natural,
                    self.settings,
                );
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

    /// Riding again a stretch the last plan rode from this station, to where
    /// it ended and as near its end speed as the drive allows from here.
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
            let change = self.params.body.max_acceleration * (to - from) / speed;
            let end_speed = ride
                .end_speed
                .clamp(speed - change, speed + change)
                .max(MIN_PLANNING_SPEED);
            let cost = self.settings.costs.ride_travel * ridden.vertical_travel
                + slowdown_cost(
                    end_speed,
                    self.target_speed,
                    to - from,
                    self.natural_step(),
                    self.settings,
                );
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
    /// station before, where that falls, and level where it rises or there
    /// is none, since the drive holds the body down off a brow rather than
    /// launch it as off a ramp.
    fn leaving_grade(&self, s: f32) -> f32 {
        let step = self.settings.body_step;
        match (self.query.floor(s - step), self.query.floor(s)) {
            (Some(before), Some(at)) => ((at.z - before.z) / step).min(0.0),
            _ => 0.0,
        }
    }

    /// Where a body leaving the floor at `from` at `speed`, climbing as the
    /// floor did, is `along` metres on: free of it, gravity pulls it down no
    /// faster than this.
    fn free_height(&self, from: f32, z: f32, speed: f32, along: f32) -> f32 {
        let time = along / speed.max(MIN_PLANNING_SPEED);
        z + self.leaving_grade(from) * along - 0.5 * self.params.gravity * time * time
    }

    /// Whether the floor between `from` and `to` drops away from the body
    /// riding it at `speed` faster than gravity can follow: past `from` the
    /// body flies, over a gap or off a step down, rather than dipping.
    fn falls_away(&self, from: f32, to: f32, speed: f32) -> bool {
        let Some(start) = self.query.floor(from) else {
            return false;
        };
        let step = self.settings.body_step;
        let stations = ((to - from) / step).round().max(1.0) as usize;
        (1..=stations).any(|index| {
            let along = index as f32 * step;
            self.query.floor(from + along).is_some_and(|floor| {
                floor.z < self.free_height(from, start.z, speed, along) - FALL_AWAY_SLACK
            })
        })
    }

    /// Rolling off where the floor falls away: no push, the body flies on
    /// the speed and climb it rode with until its arc meets the floor again,
    /// within a step for the feet to span. Meeting the far side of a gap
    /// below its brow, it is carried up onto it, as a capsule rides up a
    /// rounded corner, where it meets it at a slope the drive rides: it
    /// lands on the first station near there that it can ride on from, its
    /// climb set to reach it.
    fn roll_off(
        &self,
        origin: f32,
        station: usize,
        speed: f32,
        known: &mut FootReach,
        out: &mut SmallVec<[(usize, f32, f32, f32, BodySegment); 16]>,
    ) {
        let (params, settings) = (self.params, self.settings);
        let step = settings.body_step;
        let from = self.station_s(origin, station);
        if !self.falls_away(from, from + step, speed) {
            return;
        }
        let Some(start) = self.query.floor(from) else {
            return;
        };
        let longest = params.stride.max_length - 2.0 * FOOT_SLACK;
        let last = (station + ((longest + STATION_TOLERANCE) / step).floor() as usize)
            .min(known.len() - 1);
        let Some(meets) = (station + 1..=last).find(|&next| {
            let along = self.station_s(origin, next) - from;
            self.query
                .floor(from + along)
                .is_some_and(|floor| floor.z >= self.free_height(from, start.z, speed, along))
        }) else {
            return;
        };
        let steepest = params.body.max_ground_slope_degrees.to_radians().cos();
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
        let takeoff = BodyState {
            s: from,
            z: start.z,
            speed,
            climb: (floor.z - start.z + 0.5 * gravity * duration * duration) / duration,
        };
        let phase = Phase {
            start: 0.0,
            duration,
            contacts: SmallVec::new(),
            motion: Motion::Flight { takeoff, gravity },
        };
        if !self.query.clears(&phase) {
            return;
        }
        // Unheld as in any flight, though nothing pushed it.
        let cost = settings.costs.flight
            + settings.costs.ride_travel * (start.z - floor.z).abs()
            + self.flight_churn(from, landing)
            + slowdown_cost(
                speed,
                self.target_speed,
                landing - from,
                self.natural_step(),
                settings,
            );
        out.push((
            landing_station,
            speed,
            floor.z,
            cost,
            BodySegment::Flight {
                takeoff,
                landing,
                landing_z: floor.z,
                gravity,
                pushed: false,
            },
        ));
    }

    /// Flying from this station to a landing up to a step ahead, where the
    /// ride ahead breaks or gets rough.
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
        let reach = params.stride.max_length;
        if !self.trouble_ahead(from) {
            return;
        }
        // Pushing off from a foot behind the takeoff: the planted one, until
        // the first flight; after it, one that landed at or after the body
        // last touched down.
        let can_push_off = if from < landed - PUSH_OFF_TOLERANCE {
            false
        } else if from - landed >= FOOT_SLACK {
            self.feet_can(origin, station, FootNeed::PushOff, known)
        } else {
            !flown || self.foothold_within(landed..from + PUSH_OFF_TOLERANCE, z)
        };
        if !can_push_off {
            return;
        }
        let steepest = params.body.max_ground_slope_degrees.to_radians().cos();
        let stride = (settings.flight_step / settings.body_step).round().max(1.0) as usize;
        if (self.grid_index(origin) + station as i64) % stride as i64 != 0 {
            return;
        }
        // The step from the foot pushing off to the foot landing spans the
        // flight and a foot's slack at either end.
        let longest = reach - 2.0 * FOOT_SLACK;
        let first = station
            + ((settings.min_flight - STATION_TOLERANCE) / settings.body_step).ceil() as usize;
        let last = station + ((longest + STATION_TOLERANCE) / settings.body_step).floor() as usize;
        for landing_station in (first..=last.min(known.len() - 1)).step_by(stride) {
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
            if !(0.0..=params.flight.max_takeoff_climb).contains(&climb) {
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
                motion: Motion::Flight { takeoff, gravity },
            };
            let cost = settings.costs.flight
                + settings.costs.takeoff_climb * climb * climb
                + self.flight_churn(from, landing)
                + slowdown_cost(
                    speed,
                    self.target_speed,
                    landing - from,
                    self.natural_step(),
                    settings,
                );
            if !worth(landing_station, speed, cost) || !self.query.clears(&phase) {
                continue;
            }
            out.push((
                landing_station,
                speed,
                floor.z,
                cost,
                BodySegment::Flight {
                    takeoff,
                    landing,
                    landing_z: floor.z,
                    gravity,
                    pushed: true,
                },
            ));
        }
    }

    /// What flying from `takeoff` to `landing` costs in churn: nothing if
    /// the last plan flew there too.
    fn flight_churn(&self, takeoff: f32, landing: f32) -> f32 {
        let Some(previous) = self.previous else {
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
    fn foothold_within(&self, window: std::ops::Range<f32>, body_z: f32) -> bool {
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
                && self.params.foot_reaches(body_z, foothold.at.z)
        })
    }

    /// Whether the ride within a step ahead of `from` breaks or gets rough:
    /// only then is flying or braking worth trying.
    fn trouble_ahead(&self, from: f32) -> bool {
        let reach = self.params.stride.max_length;
        let ahead = self.query.ride(from, from + reach);
        ahead.end < from + reach - STATION_TOLERANCE
            || ahead.vertical_travel >= self.settings.rough_ride
    }

    /// A lower bound on the slowdown a body at `speed` has still to pay over
    /// the `remaining` metres: it can regain speed no faster than the drive
    /// allows, and pays for running slow all the way back up. Taken at the
    /// fast end of each stretch, so it never overstates; the search's
    /// per-station speed-ups run a little ahead of the continuous rate, which
    /// `HEURISTIC_SLACK` allows for.
    fn slowdown_still_due(&self, speed: f32, remaining: f32) -> f32 {
        let target = self.target_speed.max(MIN_PLANNING_SPEED);
        if speed >= target || remaining <= 0.0 {
            return 0.0;
        }
        let acceleration = self.params.body.max_acceleration;
        let recovery = ((target * target - speed * speed) / (2.0 * acceleration)).min(remaining);
        let piece = recovery / HEURISTIC_PIECES as f32;
        let natural = self.natural_step();
        let due: f32 = (1..=HEURISTIC_PIECES)
            .map(|index| {
                let reached = (speed * speed + 2.0 * acceleration * piece * index as f32).sqrt();
                slowdown_cost(reached, target, piece, natural, self.settings)
            })
            .sum();
        HEURISTIC_SLACK * due
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
                .is_some_and(|floor| self.foothold_within(window, floor.z));
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
            match (segments.last_mut(), arrival) {
                (
                    Some(BodySegment::Ride { to, end_speed, .. }),
                    BodySegment::Ride {
                        to: next_to,
                        end_speed: next_end,
                        ..
                    },
                ) => {
                    *to = next_to;
                    *end_speed = next_end;
                }
                _ => segments.push(arrival),
            }
        }
        BodyPath { segments }
    }
}
