//! The path a platform patrols: an ordered list of waypoints and which one it
//! is currently travelling to.
//!
//! Pure geometry and one index. Nothing here knows about velocity, thrust or
//! physics — it answers where the current leg runs, how much of it is left, and
//! when to move on to the next one.

use nalgebra::Vector3;
use serde::Deserialize;

/// What happens when a platform runs out of waypoints.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
pub enum RouteLoop {
    /// Turn around and walk the list back the other way. The two-waypoint case
    /// is the shuttle every platform used to be.
    Shuttle,
    /// Wrap to the first waypoint and go round again.
    Circuit,
}

/// An ordered patrol route, and the leg of it currently being travelled.
///
/// ```text
///        leg_start        target
///   w0 ──────●══════▓═══════●────── w2
///                   ▲       │
///              platform   remaining (projected on to the leg,
///                          so an overshoot reads negative)
/// ```
///
/// Legs are advanced by a **plane test**: the platform has arrived when it
/// crosses the plane through the waypoint perpendicular to the leg. A capture
/// sphere can be missed by a body moving fast enough, and a platform that
/// slingshots past one can circle it forever without ever entering it; a plane
/// cannot be stepped over, because being past it is a permanent condition.
#[derive(Debug, Clone)]
pub struct Route {
    /// The waypoints, in order. At least two; a shorter list is padded at
    /// construction so every accessor has a leg to talk about.
    waypoints: Vec<Vector3<f32>>,
    /// Index of the waypoint being travelled to.
    target: usize,
    /// Index of the waypoint departed from. The leg is `from` → `target`.
    from: usize,
    /// Direction of travel through the list: `+1` forward, `-1` back. Only a
    /// [`RouteLoop::Shuttle`] ever sets it to `-1`.
    step: isize,
    /// What to do at the end of the list.
    looping: RouteLoop,
}

impl Route {
    /// A route through `waypoints`, starting at the first and heading for the
    /// second.
    ///
    /// A list shorter than two waypoints has no leg to travel, so it is padded
    /// with a repeat of what it does have (or the origin, if it has nothing).
    /// Such a route is degenerate and reports a zero travel direction, which
    /// every caller already has to handle for coincident waypoints anyway.
    pub fn new(mut waypoints: Vec<Vector3<f32>>, looping: RouteLoop) -> Self {
        match waypoints.len() {
            0 => waypoints = vec![Vector3::zeros(), Vector3::zeros()],
            1 => waypoints.push(waypoints[0]),
            _ => {}
        }
        Self {
            waypoints,
            target: 1,
            from: 0,
            step: 1,
            looping,
        }
    }

    /// The classic two-point shuttle.
    pub fn shuttle(from: Vector3<f32>, to: Vector3<f32>) -> Self {
        Self::new(vec![from, to], RouteLoop::Shuttle)
    }

    pub fn waypoints(&self) -> &[Vector3<f32>] {
        &self.waypoints
    }

    pub fn looping(&self) -> RouteLoop {
        self.looping
    }

    /// Index of the waypoint being travelled to.
    ///
    /// A leg change is a change in this, which is how an observer detects a
    /// turn. It generalises the old `heading` flag: a circuit has no heading to
    /// flip, but it always has a waypoint it is heading for.
    pub fn target_index(&self) -> usize {
        self.target
    }

    /// The waypoint currently being travelled to.
    pub fn target_point(&self) -> Vector3<f32> {
        self.waypoints[self.target]
    }

    /// The waypoint the current leg departed from.
    pub fn leg_start(&self) -> Vector3<f32> {
        self.waypoints[self.from]
    }

    /// Unit vector along the current leg. Zero if the leg is degenerate.
    pub fn travel_direction(&self) -> Vector3<f32> {
        (self.target_point() - self.leg_start())
            .try_normalize(1e-6)
            .unwrap_or_else(Vector3::zeros)
    }

    /// Distance still to run on this leg, measured along the leg itself.
    ///
    /// Projecting rather than taking the raw distance means a platform that
    /// sails past its waypoint reads negative and moves on, instead of chasing
    /// a target it has already passed. Lateral displacement does not count
    /// against the leg, so a platform knocked sideways still has to travel the
    /// full length of it.
    pub fn remaining(&self, position: &Vector3<f32>) -> f32 {
        (self.target_point() - position).dot(&self.travel_direction())
    }

    /// Displacement from the leg line, measured at the platform's position.
    ///
    /// Points from the platform back on to the line, so it is the correction
    /// the motor owes. Along-leg progress is excluded by construction.
    pub fn off_leg(&self, position: &Vector3<f32>) -> Vector3<f32> {
        let direction = self.travel_direction();
        let to_target = self.target_point() - position;
        to_target - direction * to_target.dot(&direction)
    }

    /// Move on to the next leg if the platform has crossed the target
    /// waypoint's plane. Returns whether it did.
    pub fn advance_if_reached(&mut self, position: &Vector3<f32>) -> bool {
        if self.travel_direction() == Vector3::zeros() || self.remaining(position) > 0.0 {
            return false;
        }
        self.advance();
        true
    }

    /// Step to the next leg unconditionally.
    fn advance(&mut self) {
        let last = self.waypoints.len() - 1;
        self.from = self.target;
        match self.looping {
            RouteLoop::Circuit => {
                self.target = (self.target + 1) % self.waypoints.len();
            }
            RouteLoop::Shuttle => {
                if self.target == last {
                    self.step = -1;
                } else if self.target == 0 {
                    self.step = 1;
                }
                self.target = self.target.saturating_add_signed(self.step);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(x: f32) -> Vector3<f32> {
        Vector3::new(x, 0.0, 0.0)
    }

    fn straight_route(looping: RouteLoop) -> Route {
        Route::new(vec![at(0.0), at(10.0), at(20.0)], looping)
    }

    #[test]
    fn a_new_route_runs_from_the_first_waypoint_to_the_second() {
        let route = straight_route(RouteLoop::Shuttle);
        assert_eq!(route.leg_start(), at(0.0));
        assert_eq!(route.target_point(), at(10.0));
    }

    /// The plane test, and the whole reason for it: arrival is a permanent
    /// condition once crossed, at any speed and any overshoot.
    #[test]
    fn a_leg_advances_when_the_waypoint_plane_is_crossed() {
        let mut route = straight_route(RouteLoop::Shuttle);
        assert!(!route.advance_if_reached(&at(9.99)));
        assert!(route.advance_if_reached(&at(10.01)));
        assert_eq!(route.target_point(), at(20.0));
    }

    /// A platform moving fast enough to step clean over a capture sphere is
    /// still past the plane, and still arrives.
    #[test]
    fn a_waypoint_overshot_by_miles_still_counts_as_reached() {
        let mut route = straight_route(RouteLoop::Shuttle);
        assert!(route.advance_if_reached(&at(500.0)));
        assert_eq!(route.target_point(), at(20.0));
    }

    /// Off to one side is not progress. This is what stops a platform blasted
    /// sideways from registering an arrival it never made.
    #[test]
    fn lateral_displacement_is_not_progress() {
        let mut route = straight_route(RouteLoop::Shuttle);
        let beside = Vector3::new(5.0, 0.0, 50.0);
        assert!(!route.advance_if_reached(&beside));
        assert_eq!(route.remaining(&beside), 5.0);
        assert_eq!(route.off_leg(&beside), Vector3::new(0.0, 0.0, -50.0));
    }

    #[test]
    fn a_shuttle_walks_the_list_and_turns_around_at_each_end() {
        let mut route = straight_route(RouteLoop::Shuttle);
        let mut visited = vec![route.target_point().x];
        for _ in 0..5 {
            route.advance();
            visited.push(route.target_point().x);
        }
        assert_eq!(visited, vec![10.0, 20.0, 10.0, 0.0, 10.0, 20.0]);
    }

    #[test]
    fn a_circuit_wraps_to_the_start() {
        let mut route = straight_route(RouteLoop::Circuit);
        let mut visited = vec![route.target_point().x];
        for _ in 0..4 {
            route.advance();
            visited.push(route.target_point().x);
        }
        assert_eq!(visited, vec![10.0, 20.0, 0.0, 10.0, 20.0]);
    }

    /// A shuttle's leg start must follow it round the turn, or the leg after a
    /// turnaround would be measured from the wrong end and read as already
    /// complete.
    #[test]
    fn the_leg_start_follows_the_turnaround() {
        let mut route = Route::shuttle(at(0.0), at(10.0));
        route.advance();
        assert_eq!(route.leg_start(), at(10.0));
        assert_eq!(route.target_point(), at(0.0));
        assert_eq!(route.travel_direction(), -Vector3::x());
        assert_eq!(route.remaining(&at(10.0)), 10.0);
    }

    /// Degenerate routes must not panic or divide by zero — a level can author
    /// anything.
    #[test]
    fn a_degenerate_route_has_no_direction_and_never_advances() {
        for waypoints in [vec![], vec![at(3.0)], vec![at(3.0), at(3.0)]] {
            let mut route = Route::new(waypoints, RouteLoop::Shuttle);
            assert_eq!(route.travel_direction(), Vector3::zeros());
            assert!(!route.advance_if_reached(&at(3.0)));
        }
    }
}
