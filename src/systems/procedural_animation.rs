//! System for updating procedural character animation.
//!
//! This system computes foot placement from IKTargets and derives pelvis target height
//! from foot positions. It runs BEFORE collision resolution, so it sets the desired
//! position that collision will then validate against terrain.

use nalgebra::{Point3, Vector3};
use specs::{Entities, Join, Read, ReadStorage, System, Write, WriteStorage};

use crate::components::{IKTargets, PelvisTarget, Position, ProbePurpose, Rotation, Velocity};
use crate::debug::DebugLines;
use crate::player::Player;
use crate::skeleton::{SpringBipedCharacter, SpringBipedSkeleton};
use crate::time::Time;

/// Updates spring biped character animation each frame.
///
/// Pipeline role:
/// 1. Reads IKTargets (ground contact points from terrain probes)
/// 2. Updates gait and foot positions based on IK targets
/// 3. Computes target pelvis height from foot positions
/// 4. Updates Position to target (collision will resolve against terrain)
pub struct ProceduralAnimationSystem;

impl<'a> System<'a> for ProceduralAnimationSystem {
    type SystemData = (
        Read<'a, Time>,
        Entities<'a>,
        ReadStorage<'a, Player>,
        ReadStorage<'a, Velocity>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, IKTargets>,
        ReadStorage<'a, Position>,
        WriteStorage<'a, SpringBipedCharacter>,
        WriteStorage<'a, PelvisTarget>,
        Write<'a, DebugLines>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            time,
            entities,
            players,
            velocities,
            rotations,
            ik_targets,
            positions,
            mut characters,
            mut pelvis_targets,
            mut debug_lines,
        ) = data;

        let dt = time.delta_seconds();

        for (entity, _player, vel, rot, pos, character, pelvis_target) in (
            &entities,
            &players,
            &velocities,
            &rotations,
            &positions,
            &mut characters,
            &mut pelvis_targets,
        )
            .join()
        {
            // Update skeleton facing direction based on player rotation
            let facing = Vector3::new(rot.0.sin(), 0.0, rot.0.cos());
            character.skeleton.set_facing(facing);

            // Sync pelvis position from current Position (start of frame)
            let pelvis_pos = Point3::new(pos.0.x, pos.0.y, pos.0.z);
            character.skeleton.set_pelvis_position(pelvis_pos);

            // Extract foot ground heights from IKTargets
            let (left_target, right_target, has_contact) =
                extract_foot_targets(ik_targets.get(entity), &character.skeleton);

            // Get horizontal velocity for gait
            let horizontal_vel = Vector3::new(vel.0.x, 0.0, vel.0.z);

            // Update gait (foot targeting and swing animation) when grounded
            if character.grounded {
                character
                    .skeleton
                    .update_gait(horizontal_vel, dt, left_target, right_target);
                debug_lines.add("Grounded", "true");
            }

            // Solve leg IK to position knees correctly
            character.skeleton.solve_ik_only();

            // Compute target pelvis height from foot positions
            // The pelvis should be positioned so legs are at a comfortable extension
            let target_pelvis_y =
                compute_target_pelvis_height(&character.skeleton, left_target, right_target);

            pelvis_target.target_y = target_pelvis_y;
            pelvis_target.has_contact = has_contact;

            character.mark_dirty();
        }
    }
}

/// Extract ground heights for left and right feet from IKTargets.
fn extract_foot_targets(
    ik_targets: Option<&IKTargets>,
    skeleton: &SpringBipedSkeleton,
) -> (Option<Point3<f32>>, Option<Point3<f32>>, bool) {
    let Some(targets) = ik_targets else {
        return (None, None, false);
    };

    let mut left_target = None;
    let mut right_target = None;

    for target in &targets.targets {
        match target.purpose {
            ProbePurpose::FootLeft => {
                left_target = Some(target.target);
            }
            ProbePurpose::FootRight => {
                right_target = Some(target.target);
            }
            _ => {}
        }
    }

    let has_contact = left_target.is_some() || right_target.is_some();

    // If we didn't get IK targets, use skeleton's current foot positions as fallback
    if left_target.is_none() {
        left_target = Some(skeleton.left_leg.foot_position);
    }
    if right_target.is_none() {
        right_target = Some(skeleton.right_leg.foot_position);
    }

    (left_target, right_target, has_contact)
}

/// Compute target pelvis height from foot positions.
///
/// When grounded, the pelvis should be at a height that keeps the legs
/// at a comfortable extension (not fully stretched, not too bent).
fn compute_target_pelvis_height(
    skeleton: &SpringBipedSkeleton,
    left_target: Option<Point3<f32>>,
    right_target: Option<Point3<f32>>,
) -> f32 {
    // Get the higher of the two foot positions (for standing on slopes)
    let left_foot_y = left_target
        .map(|t| t.y)
        .unwrap_or(skeleton.left_leg.foot_position.y);
    let right_foot_y = right_target
        .map(|t| t.y)
        .unwrap_or(skeleton.right_leg.foot_position.y);
    let ground_y = left_foot_y.max(right_foot_y);

    // Target pelvis height is ground + comfortable standing height
    // Use ~85% of max leg extension for a slightly bent knee stance
    let leg_length = skeleton.leg_length();
    let standing_height = leg_length * 0.85;

    ground_y + standing_height
}
