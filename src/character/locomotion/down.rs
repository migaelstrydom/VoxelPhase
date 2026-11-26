use std::f32::consts::FRAC_PI_2;

use super::input::LocomotionInput;
use super::on_foot::OnFoot;
use super::rule::MovementRule;
use super::state::LocomotionState;
use crate::character::attitude::{approach, AttitudeHold};

/// How a knocked-down character lies and gets up again.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DownConfig {
    /// Below this speed, m/s, a body lying on something lies still and
    /// gathers itself.
    pub still_speed: f32,
    /// Past this lean, rad, a body is lying down ([`lies`](Self::lies)).
    /// One knocked down where it stands is still upright, however still: it
    /// buckles over first.
    pub lying_tilt: f32,
    /// Seconds past its time a body lying on something but never still (on a
    /// slope it slides down, a raft that rocks) waits before gathering itself
    /// anyway.
    pub settle_limit: f32,
    /// Seconds of its time down a body lying still has left when it gathers
    /// itself. Until then it lies where it fell, dazed: a harder blow keeps
    /// the body down longer, and it spends the longer time lying, not
    /// waiting on its knees.
    pub gather_lead: f32,
    /// Seconds a body gathering itself takes at least before it gets up:
    /// time for one lying on its side to turn onto its front or its back.
    pub gather_time: f32,
    /// How fast, rad/s, a body getting up is pitched from lying along its
    /// heading to standing.
    pub rise_rate: f32,
    /// How sprung the attitude is while the body gathers itself, rad per
    /// N·m: soft, as a stiffer spring turns a body lying on its side over
    /// faster and rocks it further past its front or back.
    pub gather_compliance: f32,
    /// How sprung the attitude is while the body gets up, rad per N·m: stiff
    /// enough to keep up with `rise_rate`, so the body arrives upright
    /// without swinging past it and popping off its feet, and to outpull
    /// gravity's tipping at every lean (`PLAYER_TIPPING` on the player).
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

/// How hard gravity tips the player's body over as it leans, N·m per
/// radian: its weight times the height of its centre over the end it
/// stands on. The default springs are tuned against it.
const PLAYER_TIPPING: f32 = 275.0;

impl DownConfig {
    /// The player's, for a body gravity tips over `tipping` N·m per radian
    /// of lean: its springs as stiff against its own weight as the player's
    /// are against theirs, so a heavier or taller body gathers itself and
    /// gets up as the player does.
    pub fn tipped_at(tipping: f32) -> Self {
        let defaults = Self::default();
        let softer = if tipping > 0.0 {
            PLAYER_TIPPING / tipping
        } else {
            1.0
        };
        Self {
            gather_compliance: defaults.gather_compliance * softer,
            rise_compliance: defaults.rise_compliance * softer,
            ..defaults
        }
    }

    /// Whether a body leaning `tilt` off upright, rad, is lying down: the
    /// one measure the state and the figure drawn for it share.
    pub fn lies(&self, tilt: f32) -> bool {
        tilt >= self.lying_tilt
    }
}

impl Default for DownConfig {
    fn default() -> Self {
        Self {
            still_speed: 0.5,
            lying_tilt: 60f32.to_radians(),
            settle_limit: 2.0,
            gather_lead: 1.0,
            gather_time: 0.6,
            rise_rate: 2.5,
            gather_compliance: 3.0e-4,
            rise_compliance: 8.0e-5,
            upright_tilt: 8f32.to_radians(),
            rise_limit: 1.5,
            buckle_rate: 3.0,
        }
    }
}

/// Knocked down by a heavy blow: no control while the body tumbles where the
/// blow threw it, then gathering itself on its front or its back and getting
/// up.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Down {
    pub phase: DownPhase,
}

/// Where in being knocked down a body is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DownPhase {
    /// Limp until it lies still on something with `gather_lead` or less of
    /// its time down left, `remaining` seconds.
    Limp { remaining: f32 },
    /// Lying still, held lying along its heading on its front, or on its back
    /// if it came to rest `face_up`, which turns a body lying on its side
    /// onto whichever is nearer: `elapsed` seconds in, with `remaining`
    /// seconds of its time down left.
    Gathered {
        remaining: f32,
        elapsed: f32,
        face_up: bool,
    },
    /// Getting up, `elapsed` seconds in, still with no control: off its knees
    /// from its front, or sitting up from its back if it is `face_up`.
    Rising { elapsed: f32, face_up: bool },
}

impl Down {
    /// Knocked down from `from` for `seconds`. A body already down stays
    /// down for whichever is longer; one getting up goes down again.
    pub fn knocked(from: &LocomotionState, seconds: f32) -> Self {
        let remaining = match from {
            LocomotionState::Down(Down {
                phase: DownPhase::Limp { remaining } | DownPhase::Gathered { remaining, .. },
            }) => remaining.max(seconds),
            _ => seconds,
        };
        Self {
            phase: DownPhase::Limp { remaining },
        }
    }

    /// Limp until it lies down, still, on something, and its time is nearly
    /// up (`gather_lead`); then gathered until its
    /// time is up and it has had `gather_time` to turn onto its front or its
    /// back (limp again if the ground goes from under it); then getting up
    /// until it stands within `upright_tilt` or has tried for `rise_limit`,
    /// when it is on its feet. One come to rest leaning on something rather
    /// than lying gets up from where it leans once its time is up.
    pub fn tick(self, input: &LocomotionInput) -> LocomotionState {
        let (facts, config, dt) = (&input.facts, &input.config.down, input.dt);
        let phase = match self.phase {
            DownPhase::Limp { remaining } => {
                let remaining = remaining - dt;
                let settled = facts.velocity.norm() <= config.still_speed
                    || remaining <= -config.settle_limit;
                let at_rest = facts.grounded && settled;
                if at_rest && config.lies(facts.tilt) && remaining <= config.gather_lead {
                    DownPhase::Gathered {
                        remaining,
                        elapsed: 0.0,
                        face_up: facts.face_up,
                    }
                } else if at_rest && remaining <= 0.0 {
                    DownPhase::Rising {
                        elapsed: 0.0,
                        face_up: facts.face_up,
                    }
                } else {
                    DownPhase::Limp { remaining }
                }
            }
            DownPhase::Gathered {
                remaining,
                elapsed,
                face_up,
            } => {
                let (remaining, elapsed) = (remaining - dt, elapsed + dt);
                if !facts.grounded {
                    DownPhase::Limp { remaining }
                } else if remaining <= 0.0 && elapsed >= config.gather_time {
                    DownPhase::Rising {
                        elapsed: 0.0,
                        face_up,
                    }
                } else {
                    DownPhase::Gathered {
                        remaining,
                        elapsed,
                        face_up,
                    }
                }
            }
            DownPhase::Rising { elapsed, face_up } => {
                if facts.tilt <= config.upright_tilt || elapsed >= config.rise_limit {
                    return LocomotionState::OnFoot(OnFoot::default());
                }
                DownPhase::Rising {
                    elapsed: elapsed + dt,
                    face_up,
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

    /// Let go while limp; sprung to its [`pitch`](Self::pitch) while it
    /// gathers itself and gets up.
    pub fn attitude_hold(&self, config: &DownConfig) -> AttitudeHold {
        match self.phase {
            DownPhase::Limp { .. } => AttitudeHold::Released,
            DownPhase::Gathered { .. } => AttitudeHold::Sprung {
                compliance: config.gather_compliance,
            },
            DownPhase::Rising { .. } => AttitudeHold::Sprung {
                compliance: config.rise_compliance,
            },
        }
    }

    /// The pitch to hold the body at this frame, rad, from `current`: while
    /// limp, standing, as nothing holds it, so one that gets up from where
    /// it leans starts from there; gathered, lying along its heading, on its
    /// front (+90°) or its back (−90°, its heading toward its feet), at once;
    /// getting up, pitched toward standing at `rise_rate`, which stands a
    /// body on its front up off its knees and sits one on its back up.
    pub fn pitch(&self, current: f32, config: &DownConfig, dt: f32) -> f32 {
        match self.phase {
            DownPhase::Limp { .. } => 0.0,
            DownPhase::Gathered { face_up, .. } => {
                if face_up {
                    -FRAC_PI_2
                } else {
                    FRAC_PI_2
                }
            }
            DownPhase::Rising { .. } => approach(current, 0.0, config.rise_rate, dt),
        }
    }

    /// The water takes a body whose time down is up: it swims. One with
    /// time down left floats as it lies.
    pub fn water_takes(&self) -> bool {
        match self.phase {
            DownPhase::Limp { remaining } | DownPhase::Gathered { remaining, .. } => {
                remaining <= 0.0
            }
            DownPhase::Rising { .. } => true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A body gravity tips over twice as hard as the player is sprung twice
    /// as stiffly; one tipped as the player is, as the player is.
    #[test]
    fn a_body_is_sprung_as_stiffly_against_its_weight_as_the_player() {
        let player = DownConfig::default();
        assert_eq!(DownConfig::tipped_at(PLAYER_TIPPING), player);
        let heavier = DownConfig::tipped_at(2.0 * PLAYER_TIPPING);
        assert!((heavier.gather_compliance - 0.5 * player.gather_compliance).abs() < 1e-12);
        assert!((heavier.rise_compliance - 0.5 * player.rise_compliance).abs() < 1e-12);
        assert_eq!(heavier.rise_rate, player.rise_rate);
    }
}
