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
        /// How long the body rises before the drive cancels the rise, in
        /// seconds, for a flight nothing pushed: off a brow it is held for
        /// grounded that long. `None` for a hop, whose rise runs its course.
        #[serde(default)]
        rise_held: Option<f32>,
    },
}

/// Height and climb `t` seconds into a flight from `takeoff` under
/// `gravity`, its rise cut off after `rise_held` seconds, if it has one.
pub fn flight_at(takeoff: &BodyState, gravity: f32, rise_held: Option<f32>, t: f32) -> (f32, f32) {
    let ballistic = |t: f32| {
        (
            takeoff.z + takeoff.climb * t - 0.5 * gravity * t * t,
            takeoff.climb - gravity * t,
        )
    };
    match rise_held {
        Some(held) if t > held && takeoff.climb - gravity * held > 0.0 => {
            let fallen = t - held;
            (
                ballistic(held).0 - 0.5 * gravity * fallen * fallen,
                -gravity * fallen,
            )
        }
        _ => ballistic(t),
    }
}

/// The takeoff climb that brings a flight `rise` metres up (down, if
/// negative) in `duration` seconds under `gravity`, its rise cut off after
/// `rise_held` seconds, if it has one.
pub fn climb_reaching(rise: f32, duration: f32, gravity: f32, rise_held: Option<f32>) -> f32 {
    let ballistic = (rise + 0.5 * gravity * duration * duration) / duration;
    match rise_held {
        Some(held) if duration > held && held > 0.0 => {
            let fallen = duration - held;
            let cut = (rise + 0.5 * gravity * (held * held + fallen * fallen)) / held;
            if cut - gravity * held > 0.0 {
                cut
            } else {
                ballistic
            }
        }
        _ => ballistic,
    }
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
            Motion::Flight {
                takeoff,
                gravity,
                rise_held,
            } => {
                (takeoff.z, takeoff.climb) = flight_at(takeoff, *gravity, *rise_held, elapsed);
                takeoff.s = s;
                *rise_held = rise_held.map(|held| (held - elapsed).max(0.0));
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
            Motion::Flight {
                takeoff,
                gravity,
                rise_held,
            } => {
                let t = (s - takeoff.s) / takeoff.speed;
                Some(flight_at(&takeoff, gravity, rise_held, t).0)
            }
        }
    }
}
