//! A two-bone leg, from hip to foot.
//!
//! The bones and the knee between them. What drives the foot — a gait, a
//! canned pose, a physics constraint — is somewhere else entirely; this
//! only answers "given a hip and a foot, where does the knee go".

use nalgebra::{Point3, Vector3};

use crate::skeleton::fabrik::{FABRIKSolver, IKChain, IKTarget};

/// How far the knee is nudged along the bend direction before the law of
/// cosines places it properly. Breaks the tie when the chain comes out of
/// the solver perfectly straight.
const BEND_BIAS: f32 = 0.05;

/// `target` pulled back on to the sphere the limb can actually reach.
///
/// A hair under the full length, so the IK solver is never handed a
/// perfectly straight chain it has to pick a knee plane for.
pub fn within_reach(joint: Point3<f32>, target: Point3<f32>, length: f32) -> Point3<f32> {
    const REACHABLE: f32 = 0.999;
    let offset = target - joint;
    let distance = offset.magnitude();
    let limit = length * REACHABLE;
    if distance <= limit {
        return target;
    }
    joint + offset * (limit / distance.max(1e-6))
}

/// Solve one leg and return where its knee belongs.
///
/// `bend_dir` is the direction the knee should break toward — forward for
/// a knee, backward for a hock.
pub fn solve_knee(
    solver: &mut FABRIKSolver,
    hip: Point3<f32>,
    foot: Point3<f32>,
    current_knee: Point3<f32>,
    upper_length: f32,
    lower_length: f32,
    bend_dir: Vector3<f32>,
) -> Point3<f32> {
    let mut positions = vec![hip, current_knee, foot];
    let chain = IKChain::new(vec![0, 1, 2], &positions);
    let target = IKTarget::new(foot);

    solver.solve(&mut positions, &chain, &target, true);

    let mut knee = positions[1] + bend_dir * BEND_BIAS;
    constrain_knee(&mut knee, &hip, &foot, upper_length, lower_length, bend_dir);

    knee
}

/// Move the knee on to the circle where both bones keep their length.
fn constrain_knee(
    knee: &mut Point3<f32>,
    hip: &Point3<f32>,
    foot: &Point3<f32>,
    upper: f32,
    lower: f32,
    bend_dir: Vector3<f32>,
) {
    let hip_to_foot = foot - hip;
    let dist = hip_to_foot.magnitude();

    if dist < 0.001 {
        // Foot at hip - put knee forward
        *knee = *hip + bend_dir * upper;
        return;
    }

    let dist = dist.clamp(0.01, upper + lower - 0.01);

    let cos_angle =
        ((upper * upper + dist * dist - lower * lower) / (2.0 * upper * dist)).clamp(-1.0, 1.0);
    let angle = cos_angle.acos();

    let forward = hip_to_foot.normalize();

    let bend_axis = forward.cross(&bend_dir);
    let bend_dir_orth = if bend_axis.magnitude() > 0.01 {
        bend_axis.cross(&forward).normalize()
    } else {
        Vector3::y()
    };

    let knee_offset = forward * (angle.cos() * upper) + bend_dir_orth * (angle.sin() * upper);
    *knee = hip + knee_offset;
}

/// One leg's joints, and the bones that fix the distances between them.
///
/// A rig owns one of these per leg and feeds it a hip and a foot each
/// frame; the knee follows.
#[derive(Clone, Debug)]
pub struct Leg {
    pub hip: Point3<f32>,
    pub knee: Point3<f32>,
    pub foot: Point3<f32>,
    /// Horizontal direction the toe points.
    pub foot_forward: Vector3<f32>,
    /// Sole normal. World up on flat ground.
    pub foot_up: Vector3<f32>,
    upper_length: f32,
    lower_length: f32,
}

impl Leg {
    /// A leg at rest: straight down from `hip`, knee broken a little
    /// toward `bend_dir`.
    pub fn new(
        hip: Point3<f32>,
        upper_length: f32,
        lower_length: f32,
        facing: Vector3<f32>,
        bend_dir: Vector3<f32>,
    ) -> Self {
        let foot = hip - Vector3::y() * (upper_length + lower_length);
        let knee = hip - Vector3::y() * upper_length + bend_dir * BEND_BIAS;
        Self {
            hip,
            knee,
            foot,
            foot_forward: facing,
            foot_up: Vector3::y(),
            upper_length,
            lower_length,
        }
    }

    /// Hip-to-foot distance with the leg straight.
    pub fn reach(&self) -> f32 {
        self.upper_length + self.lower_length
    }

    /// Put the hip where the body says, pull the foot to where animation
    /// asks (as far as the bones allow), and solve the knee.
    ///
    /// Nothing upstream refuses an unreachable foot — the placer's
    /// overstretch release is a request to step, not a veto — so without
    /// the reach clamp the leg would be drawn to wherever the foot was
    /// asked for and the limb would visibly separate.
    pub fn solve(
        &mut self,
        solver: &mut FABRIKSolver,
        hip: Point3<f32>,
        foot_target: Point3<f32>,
        bend_dir: Vector3<f32>,
    ) {
        self.hip = hip;
        self.foot = within_reach(hip, foot_target, self.reach());
        self.knee = solve_knee(
            solver,
            self.hip,
            self.foot,
            self.knee,
            self.upper_length,
            self.lower_length,
            bend_dir,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Nothing upstream refuses an unreachable foot, so this is the last
    /// place a leg can be kept the length it is.
    #[test]
    fn a_foot_within_reach_is_left_exactly_where_it_was_asked_for() {
        let hip = Point3::new(1.0, 1.0, 0.0);
        let foot = Point3::new(1.2, 0.6, 0.1);
        assert_eq!(within_reach(hip, foot, 0.5), foot);
    }

    #[test]
    fn an_unreachable_foot_is_pulled_on_to_the_leg_without_turning_it() {
        let hip = Point3::new(0.0, 1.0, 0.0);
        let foot = Point3::new(0.0, -1.0, 0.0);
        let clamped = within_reach(hip, foot, 0.5);

        let reach = (clamped - hip).magnitude();
        assert!(reach <= 0.5, "leg drawn {reach} long against a 0.5 m leg");
        assert!(reach > 0.49, "and not shortened further than it had to be");
        let direction = (clamped - hip).normalize().dot(&(foot - hip).normalize());
        assert!(direction > 0.999, "the leg still points at the target");
    }

    /// A leg is a pair of bones: wherever the foot is asked to go, the
    /// knee has to sit where both bones keep their length.
    #[test]
    fn the_knee_keeps_both_bones_the_length_they_are() {
        let mut solver = FABRIKSolver::default();
        let mut leg = Leg::new(
            Point3::new(0.0, 1.0, 0.0),
            0.25,
            0.25,
            Vector3::z(),
            Vector3::z(),
        );

        for foot in [
            Point3::new(0.0, 0.6, 0.2),
            Point3::new(0.1, 0.8, -0.1),
            Point3::new(0.0, 0.55, 0.0),
        ] {
            leg.solve(&mut solver, Point3::new(0.0, 1.0, 0.0), foot, Vector3::z());
            let upper = (leg.knee - leg.hip).magnitude();
            let lower = (leg.foot - leg.knee).magnitude();
            assert!((upper - 0.25).abs() < 1e-3, "upper bone {upper}");
            assert!((lower - 0.25).abs() < 2e-2, "lower bone {lower}");
        }
    }
}
