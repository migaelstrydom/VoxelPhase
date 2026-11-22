use super::input::LocomotionInput;
use super::on_foot::OnFoot;
use super::rule::MovementRule;
use super::state::LocomotionState;
use crate::character::attitude::AttitudeHold;

/// How a knocked-down character lies and gets up again.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DownConfig {
    /// Below this speed, m/s, a body lying on something lies still and may
    /// get up.
    pub still_speed: f32,
    /// Seconds past its time a body lying on something but never still (on a
    /// slope it slides down, a raft that rocks) waits before getting up
    /// anyway.
    pub settle_limit: f32,
    /// How sprung the attitude is while the body rights itself, rad per N·m:
    /// the larger, the slower it stands. Stiff enough to outpull gravity's
    /// tipping at every lean (about 275 N·m per radian on the player), or
    /// the body hangs part way up. The spring is barely damped, so standing
    /// ends at its first pass within `upright_tilt`, before it can swing past.
    pub rise_compliance: f32,
    /// Within this lean of upright, rad, a body getting up is on its feet;
    /// the rigid hold that takes over stands it the rest of the way at once.
    pub upright_tilt: f32,
    /// Seconds a body may take to right itself before it is stood up
    /// regardless: one wedged where it cannot turn.
    pub rise_limit: f32,
    /// How fast, rad/s, a body knocked down where it stands pitches forward
    /// as its legs give way. The capsule has no legs to buckle: balanced on
    /// its round end, it would otherwise stand limp.
    pub buckle_rate: f32,
}

impl Default for DownConfig {
    fn default() -> Self {
        Self {
            still_speed: 0.5,
            settle_limit: 2.0,
            rise_compliance: 3.0e-4,
            upright_tilt: 20f32.to_radians(),
            rise_limit: 1.5,
            buckle_rate: 3.0,
        }
    }
}

/// Knocked down by a heavy blow: no control while the body tumbles where the
/// blow threw it, then getting up.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Down {
    pub phase: DownPhase,
}

/// Where in being knocked down a body is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DownPhase {
    /// Limp for `remaining` seconds more at least, and then until it lies
    /// still on something.
    Limp { remaining: f32 },
    /// Righting itself, `elapsed` seconds in, still with no control.
    Rising { elapsed: f32 },
}

impl Down {
    /// Knocked down from `from` for `seconds`. A body already limp stays
    /// down for whichever is longer; one getting up goes down again.
    pub fn knocked(from: &LocomotionState, seconds: f32) -> Self {
        let remaining = match from {
            LocomotionState::Down(Down {
                phase: DownPhase::Limp { remaining },
            }) => remaining.max(seconds),
            _ => seconds,
        };
        Self {
            phase: DownPhase::Limp { remaining },
        }
    }

    /// Limp until its time is up and it lies still on something; then
    /// getting up until it stands within `upright_tilt` or has tried for
    /// `rise_limit`, when it is on its feet.
    pub fn tick(self, input: &LocomotionInput) -> LocomotionState {
        let (facts, config, dt) = (&input.facts, &input.config.down, input.dt);
        let phase = match self.phase {
            DownPhase::Limp { remaining } => {
                let remaining = remaining - dt;
                let settled = facts.velocity.norm() <= config.still_speed
                    || remaining <= -config.settle_limit;
                if remaining <= 0.0 && facts.grounded && settled {
                    DownPhase::Rising { elapsed: 0.0 }
                } else {
                    DownPhase::Limp { remaining }
                }
            }
            DownPhase::Rising { elapsed } => {
                if facts.tilt <= config.upright_tilt || elapsed >= config.rise_limit {
                    return LocomotionState::OnFoot(OnFoot::default());
                }
                DownPhase::Rising {
                    elapsed: elapsed + dt,
                }
            }
        };
        LocomotionState::Down(Self { phase })
    }

    /// No authority of any kind, getting up included: the body is righted by
    /// its attitude, not driven.
    pub fn movement_rule(&self) -> MovementRule {
        MovementRule::limp()
    }

    /// Let go while limp; sprung upright while getting up.
    pub fn attitude_hold(&self, config: &DownConfig) -> AttitudeHold {
        match self.phase {
            DownPhase::Limp { .. } => AttitudeHold::Released,
            DownPhase::Rising { .. } => AttitudeHold::Sprung {
                compliance: config.rise_compliance,
            },
        }
    }

    /// The water takes a body whose time down is up: it swims. One still
    /// limp floats as it lies.
    pub fn water_takes(&self) -> bool {
        match self.phase {
            DownPhase::Limp { remaining } => remaining <= 0.0,
            DownPhase::Rising { .. } => true,
        }
    }
}
