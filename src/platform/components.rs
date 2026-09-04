//! Components for powered moving platforms.

use nalgebra::Vector3;
use specs::{Component, DenseVecStorage};

/// Default distance from an endpoint at which the platform turns around.
///
/// Wide enough that a frame of travel at ordinary cruise speeds cannot step
/// clean over the endpoint, small enough to be invisible.
pub const DEFAULT_ARRIVAL_RADIUS: f32 = 0.1;

/// Default stiffness of the correction back on to the route, in 1/s.
///
/// Chosen an order of magnitude below the per-frame stability limit (a drive
/// target refreshed at 60 Hz diverges above 120) and stiff enough that gravity
/// costs a horizontal run millimetres rather than metres.
pub const DEFAULT_ROUTE_GAIN: f32 = 10.0;

/// A platform that shuttles between two fixed world points under its own power.
///
/// The platform is an ordinary dynamic body with no anchor to the world. Its
/// motor is a velocity drive (see [`crate::drive::Actuator`]), so it
/// commands *speed* and never position — but the speed it commands is aimed at
/// the endpoint it is currently travelling to, recomputed from wherever the
/// platform actually is:
///
/// ```text
///   from ●───────────────────────────────● to
///             ↑ correction (across the route, proportional to error)
///             ● platform ──► cruise (along the route, always at `speed`)
/// ```
///
/// That makes the endpoints hard and the path between them soft. A grenade can
/// shove the platform anywhere it likes and stall it for as long as it out-pushes
/// the motor, but once free the platform flies straight at its target and
/// resumes the shuttle. Only losing the drive entirely — the trick `DeathSystem`
/// uses on the player — takes it out of service for good.
#[derive(Component, Debug)]
#[storage(DenseVecStorage)]
pub struct MovingPlatform {
    /// One end of the shuttle, in world space. The platform spawns here.
    pub from: Vector3<f32>,
    /// The other end, in world space.
    pub to: Vector3<f32>,
    /// Cruise speed, in m/s.
    pub speed: f32,
    /// Which endpoint the platform is currently travelling to: `+1.0` for
    /// [`to`](Self::to), `-1.0` for [`from`](Self::from). Flips on arrival.
    pub heading: f32,
    /// How close to the target endpoint counts as arrived, in metres.
    pub arrival_radius: f32,
    /// Stiffness of the pull back on to the route, in 1/s: off-route
    /// displacement is closed at `route_gain` metres per second per metre.
    ///
    /// This is what holds a horizontal run at its authored height. The motor is
    /// a velocity drive, so gravity is a *velocity* disturbance of roughly
    /// `g · frame_dt`; a proportional correction leaves a standing error of
    /// that disturbance divided by this gain — about 15 mm at the default,
    /// against nearly a metre if the route were aimed at by direction alone.
    ///
    /// Raise it for a stiffer platform, but the drive target is refreshed once
    /// per frame, so `route_gain · frame_dt` approaching 1 rings and past 2
    /// diverges.
    pub route_gain: f32,
}

impl MovingPlatform {
    /// A platform shuttling from `from` to `to` at `speed`.
    pub fn new(from: Vector3<f32>, to: Vector3<f32>, speed: f32) -> Self {
        Self {
            from,
            to,
            speed,
            heading: 1.0,
            arrival_radius: DEFAULT_ARRIVAL_RADIUS,
            route_gain: DEFAULT_ROUTE_GAIN,
        }
    }

    /// The endpoint currently being travelled to.
    pub fn target_point(&self) -> Vector3<f32> {
        if self.heading > 0.0 {
            self.to
        } else {
            self.from
        }
    }

    /// Unit vector along the current leg, `from` → `to` or the reverse.
    ///
    /// Zero for a degenerate route whose endpoints coincide.
    pub fn travel_direction(&self) -> Vector3<f32> {
        (self.to - self.from)
            .try_normalize(1e-6)
            .map(|d| d * self.heading)
            .unwrap_or_else(Vector3::zeros)
    }

    /// Distance still to run on this leg, measured along the leg itself.
    ///
    /// Projecting rather than taking the raw distance means a platform that
    /// sails past its endpoint reads negative and turns around, instead of
    /// chasing a target it has already overshot. Lateral displacement does not
    /// count against the leg, so a platform knocked sideways still has to travel
    /// the full length of it.
    pub fn remaining(&self, position: &Vector3<f32>) -> f32 {
        (self.target_point() - position).dot(&self.travel_direction())
    }

    /// Displacement from the route line, measured at the platform's position.
    ///
    /// Points from the platform back on to the line, so it is the correction the
    /// motor owes. Along-route progress is excluded by construction.
    pub fn off_route(&self, position: &Vector3<f32>) -> Vector3<f32> {
        let direction = self.travel_direction();
        let to_target = self.target_point() - position;
        to_target - direction * to_target.dot(&direction)
    }

    /// The velocity the motor should aim for, from where the platform actually
    /// is.
    ///
    /// Two independent terms, which is the whole design:
    ///
    /// - **cruise** along the leg at [`speed`](Self::speed), unconditionally;
    /// - **correction** across it, proportional to how far off the route the
    ///   platform has ended up, capped at `speed` so a blast is answered
    ///   promptly rather than violently.
    ///
    /// Splitting them means progress along the route is never traded away to
    /// pay for a disturbance, and a disturbance is closed at a rate that does
    /// not depend on how far from the endpoint the platform happens to be.
    pub fn target_velocity(&self, position: &Vector3<f32>) -> Vector3<f32> {
        let direction = self.travel_direction();
        if direction == Vector3::zeros() {
            return Vector3::zeros();
        }

        let mut correction = self.off_route(position) * self.route_gain;
        let correction_speed = correction.norm();
        if correction_speed > self.speed {
            correction *= self.speed / correction_speed;
        }

        direction * self.speed + correction
    }

    /// Flip `heading` if the platform has reached the endpoint it was aiming
    /// for.
    pub fn update_heading(&mut self, position: &Vector3<f32>) {
        if self.travel_direction() == Vector3::zeros() {
            return;
        }
        if self.remaining(position) <= self.arrival_radius {
            self.heading = -self.heading;
        }
    }
}
