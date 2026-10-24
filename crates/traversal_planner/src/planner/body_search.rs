use std::collections::BinaryHeap;

use rustc_hash::FxHashMap;
use smallvec::SmallVec;

use crate::params::GaitParams;
use crate::plan::{BodyState, Motion, Phase};
use crate::section::{FootholdWindow, Footing, SectionQuery};

use super::body_path::{BodyPath, BodySegment};
use super::cheapest_first::Pending;
use super::rules::{reaches, slowdown_cost, Pace, MIN_PLANNING_SPEED};
use super::settings::PlannerSettings;

/// Width of the speed bands the body's states are merged in, in m/s.
const SPEED_BAND: f32 = 0.5;
/// Spacing of the footholds looked for to tell whether a station is within a
/// foot's reach of any, in metres.
const SUPPORT_SPACING: f32 = 0.1;
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
}

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

/// What the search keeps between expansions.
struct Frontier {
    nodes: Vec<Node>,
    cheapest: FxHashMap<(usize, i32), usize>,
    pending: BinaryHeap<Pending>,
}

impl Frontier {
    fn key(node: &Node) -> (usize, i32) {
        (node.station, (node.speed / SPEED_BAND).round() as i32)
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
        let goal = self.settings.body_goal(self.target_speed, self.params);
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
            },
            0.0,
        );
        let mut furthest = 0;

        while let Some(Pending { node: index, .. }) = frontier.pending.pop() {
            if !frontier.is_current(index) {
                continue;
            }
            let (station, speed, z, cost) = {
                let node = &frontier.nodes[index];
                (node.station, node.speed, node.z, node.cost)
            };
            let s = start.s + station as f32 * step;
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
            self.fly(start.s, station, speed, z, &mut known, &mut edges);
            for (next, next_speed, next_z, edge_cost, arrival) in edges {
                let remaining = goal - next as f32 * step;
                frontier.relax(
                    Node {
                        station: next,
                        speed: next_speed,
                        z: next_z,
                        cost: cost + edge_cost,
                        parent: Some(index),
                        arrival: Some(arrival),
                    },
                    self.slowdown_still_due(next_speed, remaining),
                );
            }
        }
        Err(Self::path_to(&frontier.nodes, furthest))
    }

    fn station_s(&self, origin: f32, station: usize) -> f32 {
        origin + station as f32 * self.settings.body_step
    }

    /// Riding on, pressing toward the asked-for speed to the next station,
    /// or braking as far as it takes to slow by a speed band: braking one
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
        let braking_span = (SPEED_BAND * speed / (self.params.body.max_acceleration * step))
            .ceil()
            .max(1.0) as usize;
        let paces: &[(Pace, usize)] = if self.trouble_ahead(self.station_s(origin, station)) {
            &[(Pace::Press, 1), (Pace::Brake, braking_span)]
        } else {
            &[(Pace::Press, 1)]
        };
        for &(pace, span) in paces {
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
            if ride.end < to - STATION_TOLERANCE {
                continue;
            }
            let Some(floor) = self.query.floor(to) else {
                continue;
            };
            let end_speed = pace.speed_after(speed, to - from, self.target_speed, self.params);
            if pace == Pace::Brake && end_speed >= speed {
                continue;
            }
            let cost = self.settings.costs.ride_travel * ride.vertical_travel
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

    /// Flying from this station to a landing up to a step ahead, where the
    /// ride ahead breaks or gets rough.
    fn fly(
        &self,
        origin: f32,
        station: usize,
        speed: f32,
        z: f32,
        known: &mut FootReach,
        out: &mut SmallVec<[(usize, f32, f32, f32, BodySegment); 16]>,
    ) {
        let (params, settings) = (self.params, self.settings);
        let from = self.station_s(origin, station);
        let reach = params.stride.max_length;
        if !self.trouble_ahead(from) {
            return;
        }
        if !self.feet_can(origin, station, FootNeed::PushOff, known) {
            return;
        }
        let steepest = params.body.max_ground_slope_degrees.to_radians().cos();
        let stride = (settings.flight_step / settings.body_step).round().max(1.0) as usize;
        if station % stride != 0 {
            return;
        }
        // The step from the foot pushing off to the foot landing spans the
        // flight and a foot's slack at either end.
        let longest = reach - 2.0 * FOOT_SLACK;
        let first = station + (settings.min_flight / settings.body_step).ceil() as usize;
        let last = station + (longest / settings.body_step).floor() as usize;
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
            if !self.query.clears(&phase) {
                continue;
            }
            let cost = settings.costs.flight
                + settings.costs.takeoff_climb * climb * climb
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
                },
            ));
        }
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
            FootNeed::PushOff => s - FOOT_SLACK..s + STATION_TOLERANCE,
            FootNeed::TouchDown => s - STATION_TOLERANCE..s + FOOT_SLACK,
        };
        let answer = (station == 0 && need == FootNeed::Support)
            || self.query.floor(s).is_some_and(|floor| {
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
                        && reaches(floor.z, foothold.at.z, self.params)
                })
            });
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
