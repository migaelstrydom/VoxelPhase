use crate::character::LocomotionState;

/// What the traversal planner does for the character's locomotion state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlanRole {
    /// Nothing at all: no strip, no plan, the intent left as it was written.
    Off,
    /// Plans, hands the legs their footfalls, paces the drive and asks for
    /// takeoffs and a Hop's landing.
    On,
}

impl PlanRole {
    /// The planner's role in `state`: on the ground and in the flights it
    /// took off on; off in a jump the player pressed, a fall and the water.
    pub fn of(state: &LocomotionState) -> Self {
        match state {
            LocomotionState::OnFoot(_) | LocomotionState::Hop(_) => Self::On,
            LocomotionState::Jump(_)
            | LocomotionState::Fall(_)
            | LocomotionState::Swim(_)
            | LocomotionState::Ledge(_) => Self::Off,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::locomotion::{Fall, Hop, Jump, JumpKind, OnFoot, Swim, TakeoffPhase};
    use crate::character::AirSteering;

    #[test]
    fn the_plan_runs_on_foot_and_in_its_own_flights_only() {
        let hop = LocomotionState::Hop(Hop {
            phase: TakeoffPhase::Flight,
            landing: None,
            rise_held: None,
        });
        let jump = LocomotionState::Jump(Jump {
            kind: JumpKind::Jump,
            phase: TakeoffPhase::Flight,
            steering: AirSteering::Responsive,
        });
        let grace = LocomotionState::OnFoot(OnFoot { grace: Some(0.05) });
        for (state, role) in [
            (LocomotionState::default(), PlanRole::On),
            (grace, PlanRole::On),
            (hop, PlanRole::On),
            (jump, PlanRole::Off),
            (LocomotionState::Fall(Fall), PlanRole::Off),
            (LocomotionState::Swim(Swim), PlanRole::Off),
        ] {
            assert_eq!(PlanRole::of(&state), role, "{state:?}");
        }
    }
}
