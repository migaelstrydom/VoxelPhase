//! Steering primitives.
//!
//! Each function returns a desired *direction* — a unit vector in the XZ plane,
//! or zero for "no preference" — except [`separate`], whose length also says
//! how pressing it is. Speed is not a steering concern: it belongs to
//! [`LocomotionConfig`](crate::character::LocomotionConfig), which the shared
//! `CharacterControlSystem` already applies. Keeping these pure and
//! speed-agnostic is what lets a brain compose several of them per frame
//! without any of them knowing what kind of creature they are driving.

use nalgebra::{Point3, Vector3};

/// Below this length a direction is treated as no preference rather than
/// normalised into a direction dominated by floating-point noise.
const MIN_MEANINGFUL_LENGTH: f32 = 1e-4;

/// Flatten to the XZ plane and normalise, or return zero if degenerate.
fn planar_direction(v: Vector3<f32>) -> Vector3<f32> {
    let planar = Vector3::new(v.x, 0.0, v.z);
    planar
        .try_normalize(MIN_MEANINGFUL_LENGTH)
        .unwrap_or_else(Vector3::zeros)
}

/// Head straight at the target.
pub fn seek(from: Point3<f32>, to: Point3<f32>) -> Vector3<f32> {
    planar_direction(to - from)
}

/// Head directly away from the threat.
pub fn flee(from: Point3<f32>, threat: Point3<f32>) -> Vector3<f32> {
    -seek(from, threat)
}

/// Close to `preferred` range and hold there.
///
/// Inside a `deadband` around the preferred distance the result is zero, so a
/// ranged creature settles instead of oscillating across its ideal standoff.
pub fn keep_distance(
    from: Point3<f32>,
    target: Point3<f32>,
    preferred: f32,
    deadband: f32,
) -> Vector3<f32> {
    let offset = Vector3::new(target.x - from.x, 0.0, target.z - from.z);
    let distance = offset.magnitude();
    if (distance - preferred).abs() <= deadband {
        return Vector3::zeros();
    }
    if distance < preferred {
        flee(from, target)
    } else {
        seek(from, target)
    }
}

/// Circle the target at the current radius. `clockwise` picks the direction,
/// which a brain should hold for a while rather than reroll each frame.
pub fn strafe(from: Point3<f32>, target: Point3<f32>, clockwise: bool) -> Vector3<f32> {
    let toward = seek(from, target);
    if toward == Vector3::zeros() {
        return Vector3::zeros();
    }
    let side = Vector3::y().cross(&toward);
    if clockwise {
        -side
    } else {
        side
    }
}

/// Where something is and how it moves.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mover {
    /// Its centre.
    pub at: Point3<f32>,
    /// Its velocity, m/s.
    pub velocity: Vector3<f32>,
}

/// Away from each of `others` that comes nearer than `spacing` (centre to
/// centre) within `ahead` seconds, if it and `me` go on as they are going:
/// from where the two will be nearest, the more the further inside
/// `spacing` that is, from nothing at it to a unit's worth at the same
/// spot, summed over them. Not normalised, so blended with where a
/// creature is going it turns it aside only as far as it is crowded.
pub fn separate(
    me: Mover,
    others: impl IntoIterator<Item = Mover>,
    spacing: f32,
    ahead: f32,
) -> Vector3<f32> {
    let planar = |v: Vector3<f32>| Vector3::new(v.x, 0.0, v.z);
    others
        .into_iter()
        .filter_map(|other| {
            let apart = planar(me.at - other.at);
            let closing = planar(me.velocity - other.velocity);
            let soonest = closing
                .try_normalize(MIN_MEANINGFUL_LENGTH)
                .map_or(0.0, |_| {
                    (-apart.dot(&closing) / closing.norm_squared()).clamp(0.0, ahead)
                });
            let nearest = apart + closing * soonest;
            let distance = nearest.magnitude();
            if distance >= spacing {
                return None;
            }
            let away = nearest
                .try_normalize(MIN_MEANINGFUL_LENGTH)
                .or_else(|| apart.try_normalize(MIN_MEANINGFUL_LENGTH))?;
            Some(away * (1.0 - distance / spacing))
        })
        .sum()
}

/// Blend weighted steering directions into one direction.
///
/// Weights are relative, not normalised — a `seek` at 1.0 blended with an
/// `avoid` at 3.0 means avoidance wins while still bending toward the target,
/// which is what makes a creature round an obstacle instead of stopping dead
/// at it.
pub fn blend(contributions: &[(Vector3<f32>, f32)]) -> Vector3<f32> {
    let sum = contributions
        .iter()
        .map(|(dir, weight)| dir * *weight)
        .sum::<Vector3<f32>>();
    planar_direction(sum)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(x: f32, z: f32) -> Point3<f32> {
        Point3::new(x, 0.0, z)
    }

    fn still(x: f32, z: f32) -> Mover {
        Mover {
            at: point(x, z),
            velocity: Vector3::zeros(),
        }
    }

    #[test]
    fn separate_turns_away_from_neighbours_inside_the_spacing_only() {
        let alone = separate(still(0.0, 0.0), [still(3.0, 0.0)], 2.0, 0.5);
        assert_eq!(alone, Vector3::zeros());
        let near = separate(still(0.0, 0.0), [still(1.5, 0.0)], 2.0, 0.5);
        let nearer = separate(still(0.0, 0.0), [still(0.5, 0.0)], 2.0, 0.5);
        assert!(near.x < 0.0 && near.z.abs() < 1e-6, "{near:?}");
        assert!(nearer.x < near.x, "{nearer:?} against {near:?}");
        let between = separate(
            still(0.0, 0.0),
            [still(1.0, 0.0), still(-1.0, 0.0)],
            2.0,
            0.5,
        );
        assert!(between.magnitude() < 1e-6, "{between:?}");
    }

    /// One coming fast from out of the spacing is made way for before it
    /// arrives, aside from the line it comes along; one going away, or that
    /// passes wide, is not.
    #[test]
    fn separate_makes_way_for_one_coming_before_it_arrives() {
        let coming = Mover {
            at: point(3.0, 0.3),
            velocity: Vector3::new(-6.0, 0.0, 0.0),
        };
        let way = separate(still(0.0, 0.0), [coming], 1.5, 0.5);
        assert!(way.z < -0.3, "{way:?}");
        assert_eq!(
            separate(still(0.0, 0.0), [coming], 1.5, 0.2),
            Vector3::zeros()
        );

        let going = Mover {
            velocity: Vector3::new(6.0, 0.0, 0.0),
            ..coming
        };
        assert_eq!(
            separate(still(0.0, 0.0), [going], 1.5, 0.5),
            Vector3::zeros()
        );
        let wide = Mover {
            at: point(3.0, 2.0),
            ..coming
        };
        assert_eq!(
            separate(still(0.0, 0.0), [wide], 1.5, 0.5),
            Vector3::zeros()
        );
    }

    #[test]
    fn seek_points_at_the_target_and_ignores_height() {
        let dir = seek(Point3::new(0.0, 10.0, 0.0), Point3::new(0.0, -5.0, 4.0));
        assert!((dir - Vector3::new(0.0, 0.0, 1.0)).magnitude() < 1e-5);
    }

    #[test]
    fn seek_on_top_of_the_target_has_no_preference() {
        assert_eq!(seek(point(3.0, 3.0), point(3.0, 3.0)), Vector3::zeros());
    }

    #[test]
    fn flee_is_the_opposite_of_seek() {
        let from = point(1.0, 2.0);
        let threat = point(5.0, 9.0);
        assert!((flee(from, threat) + seek(from, threat)).magnitude() < 1e-5);
    }

    #[test]
    fn keep_distance_settles_inside_the_deadband() {
        // 5m away, wants 5m, tolerance 1m — already good, so no movement.
        let held = keep_distance(point(0.0, 0.0), point(0.0, 5.0), 5.0, 1.0);
        assert_eq!(held, Vector3::zeros());
    }

    #[test]
    fn keep_distance_advances_when_too_far_and_backs_off_when_too_close() {
        let me = point(0.0, 0.0);
        let target = point(0.0, 10.0);
        let advance = keep_distance(me, target, 4.0, 0.5);
        assert!(advance.z > 0.9, "too far away should close the gap");

        let retreat = keep_distance(me, point(0.0, 1.0), 4.0, 0.5);
        assert!(retreat.z < -0.9, "too close should back off");
    }

    #[test]
    fn strafe_is_perpendicular_to_the_target_and_reverses() {
        let me = point(0.0, 0.0);
        let target = point(0.0, 5.0);
        let cw = strafe(me, target, true);
        let ccw = strafe(me, target, false);
        assert!(cw.dot(&seek(me, target)).abs() < 1e-5, "perpendicular");
        assert!((cw + ccw).magnitude() < 1e-5, "directions are opposites");
    }

    #[test]
    fn blend_lets_the_heavier_contribution_win_while_still_bending() {
        let toward_z = Vector3::new(0.0, 0.0, 1.0);
        let toward_x = Vector3::new(1.0, 0.0, 0.0);
        let result = blend(&[(toward_z, 1.0), (toward_x, 3.0)]);
        assert!(result.x > result.z, "the weighted direction dominates");
        assert!(result.z > 0.0, "but the lighter one still bends the result");
        assert!(
            (result.magnitude() - 1.0).abs() < 1e-5,
            "result is a unit dir"
        );
    }

    #[test]
    fn opposing_contributions_cancel_to_no_preference() {
        let z = Vector3::new(0.0, 0.0, 1.0);
        assert_eq!(blend(&[(z, 1.0), (-z, 1.0)]), Vector3::zeros());
    }
}
