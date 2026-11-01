use nalgebra::Vector3;

use super::effects::EntryEffects;
use super::input::{Facts, LocomotionInput, Requests};
use super::on_foot::OnFoot;
use super::rule::{AirSteering, MovementRule, RuleSpeeds};
use super::state::LocomotionState;
use super::takeoff::TakeoffPhase;
use crate::character::config::LocomotionConfig;

/// What kind of jump the player pressed for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JumpKind {
    /// A jump from standing or running; releasing the key early cuts it.
    Jump,
    /// Crouch, then jump: low, long and committed.
    LongJump,
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
        match self.kind {
            JumpKind::Jump => EntryEffects {
                vertical_speed: Some(config.jump_speed),
                air_speed: Some(facts.horizontal_speed().max(config.walk_speed)),
                cut_rise: !requests.jump_held,
                ..launch
            },
            JumpKind::LongJump => EntryEffects {
                vertical_speed: Some(config.jump_speed * config.long_jump_vertical_mul),
                air_speed: Some(config.walk_speed * config.long_jump_speed_mul),
                ..launch
            },
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

    /// Releasing the key cuts a jump's rise; a long jump is committed.
    pub fn allows_cutoff(&self) -> bool {
        match self.kind {
            JumpKind::Jump => true,
            JumpKind::LongJump => false,
        }
    }
}
