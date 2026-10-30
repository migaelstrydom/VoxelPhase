use smallvec::SmallVec;

use crate::plan::{climb_reaching, flight_at, BodyState, Motion, Phase};
use crate::planner::body_path::BodySegment;
use crate::planner::rules::{landed_speed, time_lost, MIN_PLANNING_SPEED};

use super::feet::{FootNeed, FootReach};
use super::search::{BodySearch, Edge, Edges, Node, STATION_TOLERANCE};

/// How far past where a rolling body's arc meets the floor it may be carried
/// up onto ground it can ride on, in metres: up a gap's far brow.
const MOUNT_REACH: f32 = 0.1;

impl BodySearch<'_> {
    /// Rolling off where the floor falls away: no push, the body flies on
    /// the speed and climb it rode with until its arc meets the floor again,
    /// as far as that is. Its feet are a hop's: the last near the brow, the
    /// next at the landing. Meeting the far side of a gap
    /// below its brow, it is carried up onto it, as a capsule rides up a
    /// rounded corner, where it meets it at a slope the drive rides: it
    /// lands on the first station near there that it can ride on from, its
    /// climb set to reach it.
    pub(super) fn roll_off(&self, origin: f32, at: &Node, known: &mut FootReach, out: &mut Edges) {
        let (station, speed) = (at.station, at.speed);
        let (params, settings) = (self.params, self.settings);
        let step = settings.body_step;
        let from = self.station_s(origin, station);
        if !self.falls_away(from, from + step, speed) || !self.pushes_off(origin, at, known) {
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
            + settings.costs.airborne * duration
            + settings.costs.ride_travel * (start.z - floor.z).abs()
            + self.flight_churn(from, landing)
            + time_lost(speed, speed, landing - from, self.target_speed);
        let landing_climb = flight_at(&takeoff, gravity, rise_held, duration).1;
        let landed = landed_speed(speed, landing_climb, floor.normal).max(MIN_PLANNING_SPEED);
        let cost = cost + settings.costs.landing_loss * (speed - landed).powi(2);
        out.push(Edge {
            station: landing_station,
            speed: landed,
            z: floor.z,
            cost,
            arrival: BodySegment::Flight {
                takeoff,
                landing,
                landing_z: floor.z,
                gravity,
                rise_held,
                pushed: false,
            },
        });
    }

    /// Flying from this station, where the ride ahead breaks or gets rough
    /// within a hop's reach, to any landing the climb it asks for allows: a
    /// running leap, its feet a push-off and a landing, not a stride.
    pub(super) fn fly(
        &self,
        origin: f32,
        at: &Node,
        worth: &dyn Fn(usize, f32, f32) -> bool,
        known: &mut FootReach,
        out: &mut Edges,
    ) {
        let (station, speed, z) = (at.station, at.speed, at.z);
        let (params, settings) = (self.params, self.settings);
        let from = self.station_s(origin, station);
        if !self.trouble_ahead(from, speed) || !self.pushes_off(origin, at, known) {
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
                + settings.costs.airborne * duration
                + settings.costs.takeoff_climb * climb * climb
                + self.flight_churn(from, landing)
                + time_lost(speed, speed, landing - from, self.target_speed);
            let landed = landed_speed(speed, climb - gravity * duration, floor.normal)
                .max(MIN_PLANNING_SPEED);
            let cost = cost + settings.costs.landing_loss * (speed - landed).powi(2);
            if !worth(landing_station, landed, cost) || !self.query.clears(&phase) {
                continue;
            }
            out.push(Edge {
                station: landing_station,
                speed: landed,
                z: floor.z,
                cost,
                arrival: BodySegment::Flight {
                    takeoff,
                    landing,
                    landing_z: floor.z,
                    gravity,
                    rise_held: None,
                    pushed: true,
                },
            });
        }
    }

    /// The flight a body `start`ing in the air is on: from where it is, at
    /// its own climb, unpushed, to the first of the `stations` where its arc
    /// meets the floor, its climb set to land there; past the last without
    /// meeting one, a leap into what lies out of sight.
    pub(super) fn carried_flight(&self, start: BodyState, stations: usize) -> Option<Edge> {
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
            return Some(Edge {
                station: last,
                speed,
                z: end_z,
                cost: 0.0,
                arrival: BodySegment::Leap {
                    takeoff,
                    until,
                    gravity,
                },
            });
        };
        let duration = (landing - start.s) / speed;
        let takeoff = BodyState {
            climb: climb_reaching(floor.z - start.z, duration, gravity, None),
            ..takeoff
        };
        let landing_climb = flight_at(&takeoff, gravity, None, duration).1;
        let landed = landed_speed(speed, landing_climb, floor.normal).max(MIN_PLANNING_SPEED);
        Some(Edge {
            station,
            speed: landed,
            z: floor.z,
            cost: 0.0,
            arrival: BodySegment::Flight {
                takeoff,
                landing,
                landing_z: floor.z,
                gravity,
                rise_held: None,
                pushed: false,
            },
        })
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
    pub(super) fn leap(&self, origin: f32, at: &Node, known: &mut FootReach, out: &mut Edges) {
        let (station, speed, z) = (at.station, at.speed, at.z);
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
            || !self.pushes_off(origin, at, known)
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
        // Its climb is set, not chosen: it is not traded against a hop's. Its
        // landing is out of sight, so it is charged the time it is known to
        // spend in the air, rising and falling back to where it left, wherever
        // it leaves from.
        let known_airborne = 2.0 * climb / gravity;
        let cost = settings.costs.flight
            + settings.costs.airborne * known_airborne
            + self.flight_churn(from, until);
        out.push(Edge {
            station: last,
            speed,
            z: end_z,
            cost,
            arrival: BodySegment::Leap {
                takeoff,
                until,
                gravity,
            },
        });
    }
}
