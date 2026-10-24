use serde::{Deserialize, Serialize};
use smallvec::SmallVec;

use crate::path::PathVec;
use crate::section::Footing;

use super::body::BodyState;

/// A limb that can hold the body: the feet for now, the hands when climbing
/// arrives.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Limb {
    LeftFoot,
    RightFoot,
}

impl Limb {
    /// The foot that steps after this one.
    pub fn other(self) -> Self {
        match self {
            Self::LeftFoot => Self::RightFoot,
            Self::RightFoot => Self::LeftFoot,
        }
    }
}

/// A limb holding something for the length of a phase.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Contact {
    pub limb: Limb,
    /// Where it holds, in path coordinates: for a foot, the sole's contact
    /// point, at its own `d`.
    pub at: PathVec,
    /// Unit ground normal there.
    pub normal: PathVec,
    /// What it holds on to.
    pub footing: Footing,
}

/// How the body moves during a phase. Its variant is the phase's kind.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum Motion {
    /// On its contacts: the bottom-sphere centre rides the body floor from `s`
    /// `from` to `to`, its speed along the path changing evenly in time.
    Grounded {
        from: f32,
        to: f32,
        /// Speed along the path at `from`.
        start_speed: f32,
        /// Speed along the path at `to`.
        end_speed: f32,
    },
    /// Ballistic from `takeoff` for the phase's duration.
    Flight {
        takeoff: BodyState,
        /// Downward acceleration, in m/s².
        gravity: f32,
    },
}

/// One stretch of the plan: who holds the body, and how it moves meanwhile.
///
/// A footfall is a limb that appears in a phase's contacts and not in the
/// phase before. Walking alternates double support (both feet) and single
/// support; running alternates single support and flight; a hop from plank to
/// plank is a stride with a flight in it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Phase {
    /// Seconds from the plan's start.
    pub start: f32,
    /// Seconds.
    pub duration: f32,
    /// The limbs holding the body; empty in flight.
    pub contacts: SmallVec<[Contact; 2]>,
    pub motion: Motion,
}

impl Phase {
    /// Where along the path the body is when the phase ends.
    pub fn end_s(&self) -> f32 {
        match self.motion {
            Motion::Grounded { to, .. } => to,
            Motion::Flight { takeoff, .. } => takeoff.s + takeoff.speed * self.duration,
        }
    }

    /// Where along the path the body is `elapsed` seconds into the phase, and
    /// its speed along the path then.
    pub fn progress(&self, elapsed: f32) -> (f32, f32) {
        match self.motion {
            Motion::Grounded {
                from,
                start_speed,
                end_speed,
                ..
            } => {
                let acceleration = if self.duration > 0.0 {
                    (end_speed - start_speed) / self.duration
                } else {
                    0.0
                };
                (
                    from + start_speed * elapsed + 0.5 * acceleration * elapsed * elapsed,
                    start_speed + acceleration * elapsed,
                )
            }
            Motion::Flight { takeoff, .. } => (takeoff.s + takeoff.speed * elapsed, takeoff.speed),
        }
    }

    /// Drops the first `elapsed` seconds, so the phase starts where the body
    /// is then.
    pub fn clip_start(&mut self, elapsed: f32) {
        let elapsed = elapsed.clamp(0.0, self.duration);
        if elapsed <= 0.0 {
            return;
        }
        let (s, speed) = self.progress(elapsed);
        match &mut self.motion {
            Motion::Grounded {
                from, start_speed, ..
            } => {
                *from = s;
                *start_speed = speed;
            }
            Motion::Flight { takeoff, gravity } => {
                takeoff.z += takeoff.climb * elapsed - 0.5 * *gravity * elapsed * elapsed;
                takeoff.climb -= *gravity * elapsed;
                takeoff.s = s;
            }
        }
        self.start += elapsed;
        self.duration -= elapsed;
    }

    /// Height of the bottom-sphere centre at `s` in a flight; `None` for a
    /// grounded phase, whose height is the floor's.
    pub fn flight_height(&self, s: f32) -> Option<f32> {
        match self.motion {
            Motion::Grounded { .. } => None,
            Motion::Flight { takeoff, gravity } => {
                let t = (s - takeoff.s) / takeoff.speed;
                Some(takeoff.z + takeoff.climb * t - 0.5 * gravity * t * t)
            }
        }
    }
}
