use traversal_planner::params::{BodyDims, FootDims, NaturalStep, NaturalSteps, StrideLimits};
use traversal_planner::GaitParams;

use crate::animation::humanoid::Gait;
use crate::animation::CharacterRigConfig;
use crate::character::LocomotionConfig;

/// Speeds the natural step is read off the animation at, in m/s.
const STEP_SPEEDS: std::ops::RangeInclusive<u32> = 1..=20;
/// Spacing of those speeds, in m/s.
const STEP_SPEED_SPACING: f32 = 0.5;

/// The player's gait limits: the capsule its locomotion builds, what the
/// drive was measured to do (`character_viewer contract`), and its legs as
/// the animation plans them: the steps they take, walking up to the walk
/// speed and sprinting above, the longest and shortest, where a foot lands
/// across the path and how far it reaches below the body.
pub fn player_gait(config: &LocomotionConfig, rig: &CharacterRigConfig) -> GaitParams {
    let natural = STEP_SPEEDS
        .map(|index| {
            let speed = index as f32 * STEP_SPEED_SPACING;
            let gait = if speed <= config.walk_speed {
                Gait::Walk
            } else {
                Gait::Sprint
            };
            NaturalStep {
                speed,
                length: rig.level_step(speed, gait),
                plant_ahead: rig.level_plant_ahead(speed, gait),
            }
        })
        .collect();
    GaitParams {
        body: BodyDims {
            radius: config.collider_radius,
            height: 2.0 * config.collider_half_height,
            rise_held: config.ground_forgiveness_window,
            ..BodyDims::default()
        },
        foot: FootDims {
            stance_half_width: rig.hip_width,
            reach_out: 0.5 * rig.level_reach(),
            reach_in: 0.5 * rig.level_reach(),
            hip_height: rig.leg_dims().standing_height,
            leg_reach: rig.leg_reach(),
            ..FootDims::default()
        },
        stride: StrideLimits {
            min_length: rig.foot_placer.settle_trigger,
            max_length: rig.longest_step(),
            natural: NaturalSteps(natural),
            ..StrideLimits::default()
        },
        ..GaitParams::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The planner's own default legs, which its tools and fixtures plan
    /// with, are the player's: when the animation's change, so must they.
    #[test]
    fn the_planners_default_legs_are_the_players() {
        let player = player_gait(&LocomotionConfig::player(), &CharacterRigConfig::default());
        let default = NaturalSteps::default();
        for index in 1..=40 {
            let speed = index as f32 * 0.25;
            let (theirs, ours) = (player.stride.natural.at(speed), default.at(speed));
            assert!(
                (theirs.length - ours.length).abs() < 0.005,
                "at {speed} m/s the animation steps {} m, the planner's default {} m",
                theirs.length,
                ours.length
            );
            assert!(
                (theirs.plant_ahead - ours.plant_ahead).abs() < 0.005,
                "at {speed} m/s the animation plants {} m ahead, the planner's default {} m",
                theirs.plant_ahead,
                ours.plant_ahead
            );
        }
        let close = |theirs: f32, ours: f32, what: &str| {
            assert!(
                (theirs - ours).abs() < 0.005,
                "{what}: the animation's {theirs} m, the planner's default {ours} m"
            );
        };
        let (foot, ours) = (&player.foot, FootDims::default());
        close(
            foot.stance_half_width,
            ours.stance_half_width,
            "stance half width",
        );
        close(foot.reach_out, ours.reach_out, "reach out");
        close(foot.reach_in, ours.reach_in, "reach in");
        close(foot.hip_height, ours.hip_height, "hip height");
        close(foot.leg_reach, ours.leg_reach, "leg reach");
        let (stride, ours) = (&player.stride, StrideLimits::default());
        close(stride.min_length, ours.min_length, "shortest step");
        close(stride.max_length, ours.max_length, "longest step");
    }
}
