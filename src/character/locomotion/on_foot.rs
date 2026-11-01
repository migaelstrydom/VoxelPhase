use nalgebra::Vector3;

use super::fall::Fall;
use super::hop::Hop;
use super::input::LocomotionInput;
use super::jump::Jump;
use super::rule::{MovementRule, RuleSpeeds};
use super::state::LocomotionState;
use super::takeoff::TakeoffPhase;

/// On the ground, with a gait chosen from the intent and the speed.
///
/// `grace` is the time left after contact is lost: still on foot, still able
/// to jump or long jump, with any rise cancelled. When it runs out the body
/// falls.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct OnFoot {
    /// Seconds of grace left since contact was lost; `None` in contact.
    pub grace: Option<f32>,
}

impl OnFoot {
    /// Jump pressed: a Jump, already in flight from grace. A plan's takeoff:
    /// a Hop. Grace running out: a Fall.
    pub fn tick(self, input: &LocomotionInput) -> LocomotionState {
        let (facts, requests, config) = (&input.facts, &input.requests, input.config);
        if requests.jump {
            let phase = match self.grace {
                Some(_) => TakeoffPhase::Flight,
                None => TakeoffPhase::launch(config.launch_window),
            };
            return LocomotionState::Jump(Jump::pressed(requests, phase, config));
        }
        if let Some(takeoff) = requests.takeoff {
            return LocomotionState::Hop(Hop::takeoff(takeoff, facts, config));
        }
        let grace = match (facts.grounded, self.grace) {
            (true, _) => None,
            (false, None) => Some(config.ground_grace_period),
            (false, Some(left)) if left - input.dt > 0.0 => Some(left - input.dt),
            (false, Some(_)) => return LocomotionState::Fall(Fall),
        };
        LocomotionState::OnFoot(OnFoot { grace })
    }

    /// At the ground speed. Grace bridges brief losses of contact (terrain
    /// seams) as well as real walk-offs, so it keeps the ground speed:
    /// asking for the air cap there perturbs the velocity at the rate seams
    /// are crossed, enough to entrain the gait. It cancels any rise, so a
    /// walk-off falls at once.
    pub fn movement_rule(&self, move_dir: Vector3<f32>, speeds: &RuleSpeeds) -> MovementRule {
        MovementRule {
            clamp_up: self.grace.is_some(),
            ..MovementRule::steered(move_dir, speeds.ground, speeds.air_accel)
        }
    }

    /// The water takes a body on its feet once it is chest deep.
    pub fn water_takes(&self) -> bool {
        true
    }
}
