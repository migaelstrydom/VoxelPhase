use nalgebra::Vector3;

use super::effects::EntryEffects;
use super::input::{Facts, LocomotionInput, Requests};
use super::ledge::StickOnEdge;
use super::on_foot::OnFoot;
use super::rule::{AirSteering, MovementRule, RuleSpeeds};
use super::state::LocomotionState;
use super::takeoff::TakeoffPhase;
use crate::character::config::LocomotionConfig;
use crate::character::ledge::{LedgeConfig, SensedEdge};

/// What kind of jump the player pressed for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JumpKind {
    /// A jump from standing or running; releasing the key early cuts it.
    Jump,
    /// Crouch, then jump: low, long and committed.
    LongJump,
    /// From swimming at the surface: up out of the water, onto a low bank
    /// or back in. Lower than a jump from the ground; releasing the key
    /// early cuts it the same way.
    OutOfWater,
    /// Off a ledge, out from the wall it hangs on. Releasing the key early
    /// cuts it as a jump's.
    BackJump,
}

/// A jump the player pressed for. The plan has no say in its arc or its
/// landing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Jump {
    pub kind: JumpKind,
    pub phase: TakeoffPhase,
    /// Responsive for a jump; a long jump's committed velocity, until its
    /// lock lapses.
    pub steering: AirSteering,
}

impl Jump {
    /// The jump `requests` press for, taking off in `phase`.
    pub fn pressed(requests: &Requests, phase: TakeoffPhase, config: &LocomotionConfig) -> Self {
        match requests.move_dir.try_normalize(1.0e-3) {
            Some(dir) if requests.long_jump_armed => {
                let speed = config.walk_speed * config.long_jump_speed_mul;
                Self {
                    kind: JumpKind::LongJump,
                    phase,
                    steering: AirSteering::Locked {
                        velocity: Vector3::new(dir.x * speed, 0.0, dir.z * speed),
                        remaining: config.long_jump_air_lock_duration,
                    },
                }
            }
            _ => Self {
                kind: JumpKind::Jump,
                phase,
                steering: AirSteering::Responsive,
            },
        }
    }

    /// Up out of the water: in flight at once, as nothing holds a swimmer
    /// up to launch from.
    pub fn out_of_water() -> Self {
        Self {
            kind: JumpKind::OutOfWater,
            phase: TakeoffPhase::Flight,
            steering: AirSteering::Responsive,
        }
    }

    /// Off the ledge `sensed`, already in flight: straight out from the wall,
    /// unsteered for a while, with the stick neutral or pushed at the wall;
    /// steered by the stick otherwise.
    pub fn back_jump(requests: &Requests, sensed: &SensedEdge, config: &LedgeConfig) -> Self {
        let outward = sensed.at.outward.into_inner();
        let steering = match StickOnEdge::of(requests.move_dir, outward, config) {
            StickOnEdge::Neutral | StickOnEdge::Towards => AirSteering::Locked {
                velocity: outward * config.back_jump_speed,
                remaining: config.back_jump_lock,
            },
            StickOnEdge::Away | StickOnEdge::Along => AirSteering::Responsive,
        };
        Self {
            kind: JumpKind::BackJump,
            phase: TakeoffPhase::Flight,
            steering,
        }
    }

    /// Down: on its feet.
    pub fn tick(self, input: &LocomotionInput) -> LocomotionState {
        let grounded = input.facts.grounded;
        if self.phase.down(grounded) {
            return LocomotionState::OnFoot(OnFoot::default());
        }
        LocomotionState::Jump(Self {
            phase: self.phase.advanced(grounded, input.dt),
            steering: self.steering.decayed(input.dt),
            ..self
        })
    }

    /// The impulse and the air speed it leaves with; a tap already released
    /// is cut up front.
    pub fn entry(
        &self,
        facts: &Facts,
        requests: &Requests,
        config: &LocomotionConfig,
    ) -> EntryEffects {
        let launch = EntryEffects {
            consume_jump: true,
            cancel_forgiveness: true,
            ..EntryEffects::default()
        };
        let cut_rise = !requests.jump_held;
        let air_speed = Some(facts.horizontal_speed().max(config.walk_speed));
        match self.kind {
            JumpKind::Jump => EntryEffects {
                vertical_speed: Some(config.jump_speed),
                air_speed,
                cut_rise,
                ..launch
            },
            JumpKind::OutOfWater => EntryEffects {
                vertical_speed: Some(config.swim.jump_speed),
                air_speed,
                cut_rise,
                ..launch
            },
            JumpKind::LongJump => EntryEffects {
                vertical_speed: Some(config.jump_speed * config.long_jump_vertical_mul),
                air_speed: Some(config.walk_speed * config.long_jump_speed_mul),
                ..launch
            },
            // Pushed off the edge, which takes the push back: the whole launch
            // is set at once, a tap's cut rise included, rather than conjured
            // as a jump with nothing under it would be.
            JumpKind::BackJump => {
                let push = config.ledge.back_jump_speed;
                let rise = if cut_rise {
                    config.jump_speed * config.jump_cutoff_factor
                } else {
                    config.jump_speed
                };
                EntryEffects {
                    launch: requests
                        .edge
                        .map(|sensed| sensed.at.outward.into_inner() * push + Vector3::y() * rise),
                    push_against: requests.edge,
                    air_speed: Some(push.max(config.walk_speed)),
                    ..launch
                }
            }
        }
    }

    /// Locked, the committed velocity; otherwise the ground speed while
    /// still in contact and the air speed after.
    pub fn movement_rule(&self, move_dir: Vector3<f32>, speeds: &RuleSpeeds) -> MovementRule {
        match (self.steering, self.phase) {
            (AirSteering::Responsive, TakeoffPhase::Launch { .. }) => {
                MovementRule::steered(move_dir, speeds.ground, speeds.air_accel)
            }
            (steering, _) => MovementRule::airborne(steering, move_dir, speeds),
        }
    }

    /// The water does not take a jump while it rises, so a jump out of the
    /// water is not caught by the water it left.
    pub fn water_takes(&self, facts: &Facts) -> bool {
        !facts.rising()
    }

    /// Releasing the key cuts a jump's rise, from the ground or out of the
    /// water; a long jump is committed.
    pub fn allows_cutoff(&self) -> bool {
        match self.kind {
            JumpKind::Jump | JumpKind::OutOfWater | JumpKind::BackJump => true,
            JumpKind::LongJump => false,
        }
    }
}
