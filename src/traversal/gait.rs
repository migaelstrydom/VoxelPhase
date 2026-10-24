use traversal_planner::params::BodyDims;
use traversal_planner::GaitParams;

use crate::character::LocomotionConfig;

/// The player's gait limits: the capsule its locomotion builds, and what the
/// drive was measured to do (`character_viewer contract`).
pub fn player_gait(config: &LocomotionConfig) -> GaitParams {
    GaitParams {
        body: BodyDims {
            radius: config.collider_radius,
            height: 2.0 * config.collider_half_height,
            ..BodyDims::default()
        },
        ..GaitParams::default()
    }
}
