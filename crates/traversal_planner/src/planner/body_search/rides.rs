use crate::path::PathVec;
use crate::plan::{flight_at, BodyState};
use crate::planner::body_path::BodySegment;
use crate::planner::rules::{landed_speed, pressed_speed, time_lost, MIN_PLANNING_SPEED};

use super::feet::{FootNeed, FootReach};
use super::search::{BodySearch, Edge, Edges, STATION_TOLERANCE};

/// How far below the arc of a body flying free the floor may fall and still
/// be ridden where the body could not ride out of where it comes down, in
/// metres: a capsule's contacts absorb a sample's rounding, but a dip's far
/// side is struck.
const FALL_AWAY_SLACK: f32 = 0.005;
/// How far past where a body skims the floor it must be able to ride on, in
/// metres: past it the body is back down.
const SKIM_RUN: f32 = 0.5;

impl BodySearch<'_> {
    /// Riding on to the next station, pressing toward the asked-for speed.
    pub(super) fn ride(
        &self,
        origin: f32,
        station: usize,
        speed: f32,
        known: &mut FootReach,
        out: &mut Edges,
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
        out.push(Edge {
            station: next,
            speed: end_speed,
            z: floor.z,
            cost,
            arrival: BodySegment::Ride {
                from,
                to,
                start_speed: speed,
                end_speed,
            },
        });
    }

    /// Riding again a stretch the last plan rode from this station, to where
    /// it ended.
    pub(super) fn ride_again(
        &self,
        origin: f32,
        station: usize,
        speed: f32,
        known: &mut FootReach,
        out: &mut Edges,
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
            out.push(Edge {
                station: next,
                speed: end_speed,
                z: floor.z,
                cost,
                arrival: BodySegment::Ride {
                    from,
                    to,
                    start_speed: speed,
                    end_speed,
                },
            });
        }
    }

    /// The speed across the ground the drive presses toward riding from
    /// `from` to `to`: the asked-for speed, less what the floor's grade
    /// between them costs.
    pub(super) fn ground_target(&self, from: f32, to: f32) -> f32 {
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
    pub(super) fn leaving_grade(&self, s: f32) -> f32 {
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
    pub(super) fn free_height(&self, from: f32, z: f32, speed: f32, along: f32) -> f32 {
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
    pub(super) fn ridden_speed(&self, from: f32, to: f32, speed: f32, target: f32) -> (f32, f32) {
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
    pub(super) fn met_speed(&self, from: f32, to: f32, speed: f32) -> f32 {
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
    pub(super) fn falls_away(&self, from: f32, to: f32, speed: f32) -> bool {
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
    pub(super) fn rides_on(&self, s: f32, run: f32) -> bool {
        self.query.ride(s, s + run).end >= s + run - STATION_TOLERANCE
    }

    /// Whether the ride within a hop's reach ahead of `from` breaks, gets
    /// rough, or falls away from a body at `speed`: only then is flying
    /// worth trying.
    pub(super) fn trouble_ahead(&self, from: f32, speed: f32) -> bool {
        let reach = self.params.longest_hop(speed);
        let ahead = self.query.ride(from, from + reach);
        ahead.end < from + reach - STATION_TOLERANCE
            || ahead.vertical_travel >= self.settings.rough_ride
            || self.falls_away(from, ahead.end, speed)
    }
}
