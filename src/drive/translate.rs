//! Translation from the command channel to the engine's drive.
//!
//! This is the gameplay side of the seam. It takes what an entity asked for
//! (`DriveIntent`) and what it declared about itself (`Actuator`) and produces
//! the single command the physics engine is asked to act on.
//!
//! Nothing here decides *how* a verb is delivered. A jump is a speed along the
//! support normal, and which normal that is — and whether it is delivered as
//! an impulse exchange with the floor or conjured out of the actuator's
//! allowance — is the engine's answer, because only the engine knows what is
//! holding the body up when the frame is solved. This module's whole job is to
//! put the two halves of one entity's frame into one struct.

use crate::drive::components::{Actuator, DriveIntent};
use crate::physics::{DriveCommand, NormalVerbs, PhysicsWorld, RigidBodyHandle};

/// Hand one body's frame of command to the engine: what it may grip where
/// nothing holds it up, and what it drives toward. Consumes the frame's
/// discrete verbs, on gameplay's side of the seam.
///
/// The one path every actuated body takes into the engine, in the game and
/// in any harness that drives a body without it.
pub fn apply_drive(
    physics: &mut PhysicsWorld,
    body: RigidBodyHandle,
    intent: &mut DriveIntent,
    actuator: &Actuator,
) {
    let verbs = intent.take_normal_verbs();
    let _ = physics.set_body_non_support_grip(body, actuator.non_support_grip);
    let _ = physics.set_body_drive(body, &resolve_drive(intent, verbs, actuator));
}

/// Fold one frame's command into a drive command.
pub fn resolve_drive(
    intent: &DriveIntent,
    verbs: NormalVerbs,
    actuator: &Actuator,
) -> DriveCommand {
    DriveCommand {
        anchor: actuator.anchor,
        linear_target: intent.linear_target,
        angular_target: intent.angular_target,
        max_accel: actuator.max_accel,
        angular_max_accel: actuator.angular_max_accel,
        drive_gain: actuator.drive_gain,
        patch_radius: actuator.support_patch_radius,
        allowance: actuator.allowance_command(verbs, intent.steer_accel),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::{Allowance, ReactionAnchor};
    use nalgebra::Vector3;

    fn walking() -> DriveIntent {
        DriveIntent {
            linear_target: Vector3::new(5.0, -2.0, 0.0),
            steer_accel: Some(8.0),
            ..Default::default()
        }
    }

    /// The continuous target crosses untouched. Nothing on this side of the
    /// seam knows which way is up, and nothing needs to.
    #[test]
    fn the_target_crosses_the_seam_unedited() {
        let intent = walking();
        let command = resolve_drive(&intent, NormalVerbs::default(), &Actuator::character());
        assert_eq!(command.linear_target, intent.linear_target);
    }

    /// A jump is carried as a verb rather than folded into the target: the
    /// engine decides what it is delivered against.
    #[test]
    fn a_jump_crosses_as_a_verb() {
        let mut intent = walking();
        intent.jump(7.0);
        let verbs = intent.take_normal_verbs();
        let command = resolve_drive(&intent, verbs, &Actuator::character());
        assert_eq!(command.linear_target, Vector3::new(5.0, -2.0, 0.0));
        assert_eq!(command.allowance.verbs.impulse, Some(7.0));
    }

    #[test]
    fn taking_the_verbs_consumes_them() {
        let mut intent = walking();
        intent.jump(7.0);
        intent.cut_normal(0.4);
        let _ = intent.take_normal_verbs();
        assert!(intent.take_normal_verbs().is_inert());
    }

    #[test]
    fn two_cutoffs_on_one_frame_compose_as_a_product() {
        let mut intent = walking();
        intent.jump(10.0);
        intent.cut_normal(0.5);
        intent.cut_normal(0.5);
        let verbs = intent.take_normal_verbs();
        let command = resolve_drive(&intent, verbs, &Actuator::character());
        assert_eq!(command.allowance.verbs.projection.scale, 0.25);
    }

    #[test]
    fn the_actuator_supplies_the_reaction_anchor() {
        let character = resolve_drive(
            &DriveIntent::default(),
            NormalVerbs::default(),
            &Actuator::character(),
        );
        let platform = resolve_drive(
            &DriveIntent::default(),
            NormalVerbs::default(),
            &Actuator::medium(40.0, 0.0),
        );

        assert_eq!(character.anchor, ReactionAnchor::Support);
        assert_eq!(platform.anchor, ReactionAnchor::Medium);
    }

    #[test]
    fn the_actuator_supplies_the_acceleration_budget() {
        let command = resolve_drive(
            &DriveIntent::default(),
            NormalVerbs::default(),
            &Actuator::medium(40.0, 0.0),
        );
        assert_eq!(command.max_accel, 40.0);
        assert_eq!(command.angular_max_accel, 0.0);
    }

    /// The budget is the entity's declaration and the rate is the frame's
    /// request; both cross together so neither can arrive without the other.
    #[test]
    fn the_actuator_supplies_the_allowance_and_the_intent_the_rate() {
        let ungranted = resolve_drive(&walking(), NormalVerbs::default(), &Actuator::character());
        assert_eq!(ungranted.allowance.budget, None);

        let granted = resolve_drive(
            &walking(),
            NormalVerbs::default(),
            &Actuator::character().with_allowance(Allowance::character(8.0, 500.0, 7.0)),
        );
        assert_eq!(granted.allowance.budget.unwrap().air_accel, 8.0);
        assert_eq!(granted.allowance.steer_accel, Some(8.0));
    }
}
