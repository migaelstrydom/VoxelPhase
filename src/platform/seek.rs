//! The motion model: constant thrust at a target, linear drag, and whatever
//! velocity those two settle on.
//!
//! ```text
//!   dv/dt = thrust · û  −  k · v        û = unit vector at the target
//! ```
//!
//! Authored as a cruise speed and a spin-up time, from which the thrust and the
//! drag follow: `k = 1/tau` and `thrust = speed/tau`, giving a terminal velocity
//! of exactly `speed`. Both dials mean something on their own, and the two
//! coefficients that actually appear in the equation never have to be guessed
//! at.
//!
//! This is the whole reason the motion is smooth. The thrust *direction* still
//! steps the moment a waypoint is reached, but velocity is its integral, so the
//! velocity the platform is asked for is continuous however sharp the corner.
//! A platform that used to invert its target in one frame now swings through
//! the turn, decelerating into it and accelerating out, because opposed thrust
//! is what deceleration is.
//!
//! What comes out is a *commanded* velocity, handed to the motor as a drive
//! target. The real body tracks it through `Actuator::medium`, whose own
//! acceleration budget is set by what the platform must do against gravity and
//! is deliberately not this model's business: `thrust` here is a feel dial that
//! cannot starve a lift of the authority to hold itself up.

use nalgebra::Vector3;
use serde::Deserialize;

/// How a platform gets up to speed and how tightly it corners.
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct SeekMotion {
    /// Cruise speed, in m/s. The terminal velocity of thrust against drag.
    pub speed: f32,
    /// Spin-up time, in seconds: the time constant of the approach to cruise,
    /// so about `3 · tau` to arrive there.
    ///
    /// It is also the cornering dial, and the two cannot be separated — they
    /// are the same lag. A corner is rounded over a radius of about
    /// `speed · tau`, and a turnaround overshoots its waypoint by about
    /// `0.31 · speed · tau` before coming back. Author routes with that much
    /// clearance past an endpoint.
    pub tau: f32,
}

impl Default for SeekMotion {
    /// The one place a platform's motion defaults live.
    ///
    /// `tau` is enough lag to read as a deceleration into a turn without the
    /// platform wandering far off the line the author drew: half a metre of
    /// corner at the default cruise speed, and 15 cm of overshoot at a
    /// turnaround. `MovingPlatformDef`'s serde defaults delegate here rather
    /// than repeating the numbers.
    fn default() -> Self {
        Self {
            speed: 2.0,
            tau: 0.25,
        }
    }
}

/// The commanded velocity, integrated frame to frame.
#[derive(Debug, Clone, Copy, Default)]
pub struct SeekState {
    /// What the motor is currently being asked for, in m/s.
    pub commanded: Vector3<f32>,
}

impl SeekMotion {
    /// Advance the commanded velocity one frame, thrusting along `direction`.
    ///
    /// `direction` need not be normalised; a zero vector means no thrust, and
    /// the command then simply drags down towards nothing.
    ///
    /// Integrated in closed form rather than by an Euler step. The equation is
    /// a linear relaxation with an exact solution, and using it means a long
    /// frame cannot overshoot or ring the way `v += (desired − v) · dt/tau`
    /// does once `dt` approaches `tau`.
    pub fn advance(&self, state: &mut SeekState, direction: Vector3<f32>, dt: f32) {
        if self.tau <= 0.0 || dt <= 0.0 {
            state.commanded =
                direction.try_normalize(1e-6).unwrap_or_else(Vector3::zeros) * self.speed;
            return;
        }
        let desired = direction
            .try_normalize(1e-6)
            .map(|d| d * self.speed)
            .unwrap_or_else(Vector3::zeros);
        let decay = (-dt / self.tau).exp();
        state.commanded = desired + (state.commanded - desired) * decay;
    }

    /// The thrust this model applies, in m/s². Reported for diagnostics and to
    /// make the derivation visible to anything that wants to check it.
    pub fn thrust(&self) -> f32 {
        if self.tau > 0.0 {
            self.speed / self.tau
        } else {
            f32::INFINITY
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 60.0;

    /// Rounds the step count: truncating loses a whole frame at exactly the
    /// durations a time-constant test wants to check.
    fn run(motion: &SeekMotion, direction: Vector3<f32>, seconds: f32) -> SeekState {
        let mut state = SeekState::default();
        for _ in 0..(seconds / DT).round() as usize {
            motion.advance(&mut state, direction, DT);
        }
        state
    }

    /// Terminal velocity is the authored cruise speed, which is the whole point
    /// of deriving the thrust from it.
    #[test]
    fn drag_settles_the_command_at_the_cruise_speed() {
        let motion = SeekMotion {
            speed: 2.0,
            tau: 0.25,
        };
        let state = run(&motion, Vector3::x(), 5.0);
        assert!((state.commanded.x - 2.0).abs() < 1e-3);
        assert!((state.commanded.norm() - 2.0).abs() < 1e-3);
    }

    /// One time constant is one time constant.
    #[test]
    fn tau_is_the_time_constant_of_the_spin_up() {
        let motion = SeekMotion {
            speed: 2.0,
            tau: 0.25,
        };
        let state = run(&motion, Vector3::x(), 0.25);
        let expected = 2.0 * (1.0 - (-1.0f32).exp());
        assert!(
            (state.commanded.x - expected).abs() < 0.02,
            "after one tau: {} against {expected}",
            state.commanded.x
        );
    }

    /// The property the whole design rests on: reversing the thrust reverses
    /// the command *through zero*, never in one step. This is what a turnaround
    /// costs and why it reads as deceleration.
    #[test]
    fn a_reversal_passes_through_zero_rather_than_jumping() {
        let motion = SeekMotion {
            speed: 2.0,
            tau: 0.25,
        };
        let mut state = run(&motion, Vector3::x(), 5.0);

        let mut samples = Vec::new();
        for _ in 0..60 {
            motion.advance(&mut state, -Vector3::x(), DT);
            samples.push(state.commanded.x);
        }

        let biggest_step = samples
            .windows(2)
            .map(|w| (w[1] - w[0]).abs())
            .fold(0.0f32, f32::max);
        assert!(
            biggest_step < 0.5,
            "the command should ease across, not jump: biggest step {biggest_step:.3} m/s"
        );
        assert!(
            samples.iter().any(|v| v.abs() < 0.2),
            "the command should pass through zero on its way back"
        );
        assert!(
            (samples.last().unwrap() + 2.0).abs() < 0.1,
            "and arrive at cruise the other way: {}",
            samples.last().unwrap()
        );
    }

    /// A long frame must not overshoot. An Euler step with `dt > tau` would
    /// sail past the target and oscillate; the closed form cannot.
    #[test]
    fn a_frame_longer_than_tau_does_not_overshoot() {
        let motion = SeekMotion {
            speed: 2.0,
            tau: 0.05,
        };
        let mut state = SeekState::default();
        motion.advance(&mut state, Vector3::x(), 1.0);
        assert!(
            state.commanded.x <= 2.0 + 1e-5,
            "overshot to {}",
            state.commanded.x
        );
    }

    #[test]
    fn thrust_is_derived_from_the_authored_dials() {
        let motion = SeekMotion {
            speed: 2.0,
            tau: 0.25,
        };
        assert!((motion.thrust() - 8.0).abs() < 1e-5);
    }

    /// No thrust means drag alone, and drag alone means a stop.
    #[test]
    fn no_direction_drags_the_command_to_rest() {
        let motion = SeekMotion {
            speed: 2.0,
            tau: 0.25,
        };
        let mut state = run(&motion, Vector3::x(), 5.0);
        for _ in 0..120 {
            motion.advance(&mut state, Vector3::zeros(), DT);
        }
        assert!(state.commanded.norm() < 0.01, "{}", state.commanded.norm());
    }
}
