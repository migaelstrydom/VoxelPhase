use nalgebra::Vector3;

use super::input::LocomotionInput;
use super::on_foot::OnFoot;
use super::rule::{MovementRule, RuleSpeeds};
use super::state::LocomotionState;

/// In the air without a jump or a plan: walked off an edge, missed a Hop's
/// landing, or left the water over a drop.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Fall;

impl Fall {
    /// Down: on its feet.
    pub fn tick(self, input: &LocomotionInput) -> LocomotionState {
        if input.facts.grounded {
            LocomotionState::OnFoot(OnFoot::default())
        } else {
            LocomotionState::Fall(self)
        }
    }

    /// Steered at the air speed.
    pub fn movement_rule(&self, move_dir: Vector3<f32>, speeds: &RuleSpeeds) -> MovementRule {
        MovementRule::steered(move_dir, speeds.air, speeds.air_accel)
    }

    /// The water takes a body falling into it.
    pub fn water_takes(&self) -> bool {
        true
    }
}
