//! Components for powered moving platforms.

use nalgebra::Vector3;
use specs::{Component, DenseVecStorage};

use super::route::{Route, RouteLoop};
use super::seek::{SeekMotion, SeekState};

/// Default stiffness of the correction back on to the route, in 1/s.
///
/// Chosen an order of magnitude below the per-frame stability limit (a drive
/// target refreshed at 60 Hz diverges above 120) and stiff enough that gravity
/// costs a horizontal run millimetres rather than metres.
pub const DEFAULT_ROUTE_GAIN: f32 = 10.0;

/// A platform that patrols a route under its own power.
///
/// The platform is an ordinary dynamic body with no anchor to the world. Its
/// motor is a velocity drive (see [`crate::drive::Actuator`]), so it commands
/// *speed* and never position. Three pieces decide what speed:
///
/// ```text
///   Route ──────────► SeekMotion ─────────► + ───► linear_target
///   (which waypoint,   (thrust at it,       ▲
///    which leg)         drag, momentum)     │
///                                           │
///   route_gain × off_leg ────────────────────
///   (stiff correction back on to the line)
/// ```
///
/// Two terms, and the split is the whole design:
///
/// - **seek** thrusts at the target waypoint and lets drag settle it at cruise.
///   Momentum is what smooths the motion: the thrust direction still steps the
///   instant a waypoint is crossed, but velocity is its integral, so the
///   platform swings through a corner and eases through a turnaround instead of
///   inverting in a frame.
/// - **correction** across the leg, proportional to how far off the line the
///   platform has ended up, capped at cruise speed so a blast is answered
///   promptly rather than violently.
///
/// The correction is measured from the *leg*, not from the waypoint, and that
/// matters: seek alone weakens as a restoring force the longer the leg, because
/// the vertical component of a unit vector aimed at something far away is
/// nearly nothing. A long horizontal run would sag. Measured from the line, the
/// correction closes at a rate that does not depend on how far along the leg
/// the platform happens to be, which is what holds the run at its authored
/// height.
///
/// Nothing schedules the patrol. There is no clock to fall out of step with the
/// platform, so a disturbance needs no recovery rule: thrust points at the
/// target waypoint from wherever the platform actually is. A grenade can shove
/// it anywhere it likes and stall it for as long as it out-pushes the motor,
/// and when it is free it flies back at the same waypoint it was already
/// heading for. Only losing the drive entirely — the trick `DeathSystem` uses
/// on the player — takes it out of service for good.
#[derive(Component, Debug)]
#[storage(DenseVecStorage)]
pub struct MovingPlatform {
    /// The waypoints and which leg is being run.
    pub route: Route,
    /// Cruise speed and how hard the platform corners.
    pub motion: SeekMotion,
    /// The commanded velocity, carried between frames.
    pub seek: SeekState,
    /// Stiffness of the pull back on to the leg, in 1/s: off-route displacement
    /// is closed at `route_gain` metres per second per metre.
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
    /// A platform patrolling `waypoints`.
    pub fn new(waypoints: Vec<Vector3<f32>>, looping: RouteLoop, motion: SeekMotion) -> Self {
        Self {
            route: Route::new(waypoints, looping),
            motion,
            seek: SeekState::default(),
            route_gain: DEFAULT_ROUTE_GAIN,
        }
    }

    /// The two-point shuttle every platform used to be.
    pub fn shuttle(from: Vector3<f32>, to: Vector3<f32>, speed: f32) -> Self {
        Self::new(
            vec![from, to],
            RouteLoop::Shuttle,
            SeekMotion {
                speed,
                ..SeekMotion::default()
            },
        )
    }

    /// Cruise speed, in m/s.
    pub fn speed(&self) -> f32 {
        self.motion.speed
    }

    /// Index of the waypoint currently being travelled to. A change in it is a
    /// leg change, which is how an observer detects a turn.
    pub fn target_index(&self) -> usize {
        self.route.target_index()
    }

    /// Move on to the next leg if the target waypoint's plane has been crossed.
    pub fn update_heading(&mut self, position: &Vector3<f32>) {
        self.route.advance_if_reached(position);
    }

    /// Advance the motion model one frame and report the velocity the motor
    /// should aim for.
    ///
    /// Call once per frame: the seek carries state, so calling it twice in a
    /// frame spins the platform up twice as fast, and not at all leaves the
    /// command stale.
    pub fn target_velocity(&mut self, position: &Vector3<f32>, dt: f32) -> Vector3<f32> {
        let to_target = self.route.target_point() - position;
        self.motion.advance(&mut self.seek, to_target, dt);

        if self.route.travel_direction() == Vector3::zeros() {
            return Vector3::zeros();
        }

        let mut correction = self.route.off_leg(position) * self.route_gain;
        let correction_speed = correction.norm();
        if correction_speed > self.motion.speed {
            correction *= self.motion.speed / correction_speed;
        }

        self.seek.commanded + correction
    }
}
