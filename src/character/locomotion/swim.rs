use nalgebra::Vector3;

use super::fall::Fall;
use super::input::LocomotionInput;
use super::jump::Jump;
use super::on_foot::OnFoot;
use super::rule::{MovementRule, RuleSpeeds};
use super::state::LocomotionState;

/// In water too deep to stand in. Steered at the swim rate relative to the
/// water; buoyancy owns vertical, except for a jump out of it.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Swim;

impl Swim {
    /// A Jump, pressed at the surface. On its feet once they are down in
    /// water it can wade, or out of it. A Fall only once out of the water
    /// with nothing under it: the water gone, or the body risen clear of it.
    /// A swimmer standing up in wadeable water is still swimming until its
    /// feet are down.
    pub fn tick(self, input: &LocomotionInput) -> LocomotionState {
        let (facts, swim) = (&input.facts, &input.config.swim);
        if input.requests.jump && swim.can_jump(&facts.immersion, facts.position.y) {
            return LocomotionState::Jump(Jump::out_of_water());
        }
        let out = swim.out_of_water(&facts.immersion, facts.position.y);
        if facts.grounded && (out || swim.wadeable(&facts.immersion)) {
            LocomotionState::OnFoot(OnFoot::default())
        } else if out {
            LocomotionState::Fall(Fall)
        } else {
            LocomotionState::Swim(self)
        }
    }

    /// At the swim speed and rate, relative to the water, which the caller
    /// adds.
    pub fn movement_rule(&self, move_dir: Vector3<f32>, speeds: &RuleSpeeds) -> MovementRule {
        MovementRule::steered(move_dir, speeds.swim, speeds.swim_accel)
    }

    /// Already in it.
    pub fn water_takes(&self) -> bool {
        false
    }
}
