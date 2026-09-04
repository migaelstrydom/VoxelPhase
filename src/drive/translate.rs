//! Translation from the command channel to the engine's drive.
//!
//! This is the gameplay side of the seam. It takes what an entity asked for
//! (`DriveIntent`) and what it declared about itself (`Actuator`) and produces
//! the single target the physics engine is asked to accelerate toward.
//!
//! The discrete verbs — jumping, and the shaping that follows it — are
//! delivered here as an **Allowance**: non-conservative authority applied to
//! the driven body alone. That is the general rule from the start (D2a), not a
//! coyote-time special case: a jump with no support to push off is an
//! Allowance whatever the state machine calls it, and until the Support Set
//! exists no jump has support to push off.

use nalgebra::UnitVector3;

use crate::drive::components::{Actuator, DriveIntent, NormalVerbs};
use crate::physics::DriveCommand;

/// Fold one frame's command into a drive command.
///
/// `support_normal` is the axis the discrete verbs act along. Until the
/// Support Set lands it is the world's up, read from gravity; `None` — a world
/// with no gravity — leaves the verbs inert, because there is then no axis a
/// jump could mean anything along.
pub fn resolve_drive(
    intent: &DriveIntent,
    verbs: NormalVerbs,
    actuator: &Actuator,
    support_normal: Option<UnitVector3<f32>>,
) -> DriveCommand {
    let mut linear = intent.linear_target;

    if let Some(normal) = support_normal.filter(|_| !verbs.is_inert()) {
        let normal = normal.into_inner();
        let current = linear.dot(&normal);
        let mut along = verbs.impulse.unwrap_or(current);
        if along > 0.0 {
            along *= verbs.projection.scale;
            if verbs.projection.clamp_positive {
                along = 0.0;
            }
        }
        linear += normal * (along - current);
    }

    DriveCommand {
        anchor: actuator.anchor,
        linear_target: linear,
        angular_target: intent.angular_target,
        max_accel: actuator.max_accel,
        angular_max_accel: actuator.angular_max_accel,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drive::components::NormalProjection;
    use crate::physics::ReactionAnchor;
    use nalgebra::Vector3;

    fn up() -> Option<UnitVector3<f32>> {
        Some(UnitVector3::new_normalize(Vector3::y()))
    }

    fn walking() -> DriveIntent {
        DriveIntent {
            linear_target: Vector3::new(5.0, -2.0, 0.0),
            ..Default::default()
        }
    }

    #[test]
    fn a_silent_frame_passes_the_target_through_untouched() {
        let intent = walking();
        let target = resolve_drive(
            &intent,
            NormalVerbs::default(),
            &Actuator::character(),
            up(),
        );
        assert_eq!(target.linear_target, intent.linear_target);
    }

    #[test]
    fn a_jump_replaces_the_normal_component_and_leaves_the_plane_alone() {
        let mut intent = walking();
        intent.jump(7.0);
        let verbs = intent.take_normal_verbs();
        let target = resolve_drive(&intent, verbs, &Actuator::character(), up());
        assert_eq!(target.linear_target, Vector3::new(5.0, 7.0, 0.0));
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
        let target = resolve_drive(&intent, verbs, &Actuator::character(), up());
        assert_eq!(target.linear_target.y, 2.5);
    }

    #[test]
    fn a_cutoff_does_not_hurry_a_fall() {
        let intent = walking();
        let verbs = NormalVerbs {
            impulse: None,
            projection: NormalProjection {
                scale: 0.4,
                clamp_positive: false,
            },
        };
        let target = resolve_drive(&intent, verbs, &Actuator::character(), up());
        assert_eq!(target.linear_target.y, -2.0);
    }

    #[test]
    fn a_walk_off_cancels_the_rise_but_not_the_fall() {
        let rising = DriveIntent {
            linear_target: Vector3::new(5.0, 1.5, 0.0),
            ..Default::default()
        };
        let mut falling = walking();
        falling.clamp_normal_rise();
        let verbs = falling.take_normal_verbs();

        let mut rising_intent = rising.clone();
        rising_intent.clamp_normal_rise();
        let rising_verbs = rising_intent.take_normal_verbs();

        assert_eq!(
            resolve_drive(&rising, rising_verbs, &Actuator::character(), up())
                .linear_target
                .y,
            0.0
        );
        assert_eq!(
            resolve_drive(&falling, verbs, &Actuator::character(), up())
                .linear_target
                .y,
            -2.0
        );
    }

    #[test]
    fn a_jump_along_a_tilted_normal_keeps_its_speed() {
        let normal = UnitVector3::new_normalize(Vector3::new(1.0, 1.0, 0.0));
        let mut intent = DriveIntent::default();
        intent.jump(7.0);
        let verbs = intent.take_normal_verbs();
        let target = resolve_drive(&intent, verbs, &Actuator::character(), Some(normal));
        assert!((target.linear_target.magnitude() - 7.0).abs() < 1e-5);
        assert!((target.linear_target.dot(&normal) - 7.0).abs() < 1e-5);
    }

    #[test]
    fn a_world_with_no_gravity_has_no_axis_to_jump_along() {
        let mut intent = walking();
        intent.jump(7.0);
        let verbs = intent.take_normal_verbs();
        let target = resolve_drive(&intent, verbs, &Actuator::character(), None);
        assert_eq!(target.linear_target, Vector3::new(5.0, -2.0, 0.0));
    }

    #[test]
    fn the_actuator_supplies_the_reaction_anchor() {
        let character = resolve_drive(
            &DriveIntent::default(),
            NormalVerbs::default(),
            &Actuator::character(),
            up(),
        );
        let platform = resolve_drive(
            &DriveIntent::default(),
            NormalVerbs::default(),
            &Actuator::medium(40.0, 0.0),
            up(),
        );

        assert_eq!(character.anchor, ReactionAnchor::Support);
        assert_eq!(platform.anchor, ReactionAnchor::Medium);
    }

    #[test]
    fn the_actuator_supplies_the_acceleration_budget() {
        let target = resolve_drive(
            &DriveIntent::default(),
            NormalVerbs::default(),
            &Actuator::medium(40.0, 0.0),
            up(),
        );
        assert_eq!(target.max_accel, 40.0);
        assert_eq!(target.angular_max_accel, 0.0);
    }
}
