use nalgebra::Vector3;

use super::fall::Fall;
use super::input::LocomotionInput;
use super::on_foot::OnFoot;
use super::rule::{MovementRule, RuleSpeeds};
use super::state::LocomotionState;

/// In water too deep to stand in. Steered at the swim rate relative to the
/// water; buoyancy owns vertical, and a jump does nothing.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Swim;

impl Swim {
    /// On its feet once they are down in water it can wade, or out of it.
    /// A Fall only once out of the water with nothing under it: the water
    /// gone, or the body risen clear of it. A swimmer standing up in
    /// wadeable water is still swimming until its feet are down.
    pub fn tick(self, input: &LocomotionInput) -> LocomotionState {
        let (facts, swim) = (&input.facts, &input.config.swim);
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
