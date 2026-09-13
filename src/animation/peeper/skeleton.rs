//! Where every part of a peeper is this frame.
//!
//! ```text
//!   pelvis ──haunch──► haunch top ──neck(pitch)──► eye ──► feeler anchors
//!     │                                             │
//!     ├── hip L ──► Leg L ──► foot L                └──► gaze
//!     └── hip R ──► Leg R ──► foot R
//! ```
//!
//! Pure placement. What drives the feet is the gait, what drives the neck
//! pitch is the creature's swing, and what draws the result is `mesh`.

use nalgebra::{Point3, Vector3};

use crate::animation::rig::{right_vector, DroopPair, Frame, Leg};
use crate::animation::FootSide;
use crate::skeleton::fabrik::FABRIKSolver;

use super::config::PeeperRigConfig;

/// How much of the eye's radius the feelers are kept clear of, so they can
/// rest against it without being shoved off.
const EYE_CLEARANCE_FRACTION: f32 = 0.96;

/// What a foot is doing, as the gait decided it.
#[derive(Clone, Copy, Debug)]
pub struct FootPose {
    pub position: Point3<f32>,
    /// Horizontal direction the toe points.
    pub forward: Vector3<f32>,
    /// Sole normal.
    pub up: Vector3<f32>,
}

/// The peeper's joints.
pub struct PeeperSkeleton {
    /// Where the legs meet, at the bottom of the haunch.
    pub pelvis: Point3<f32>,
    /// Top of the haunch, where the neck leaves the body.
    pub shoulder: Point3<f32>,
    /// Centre of the eye.
    pub eye: Point3<f32>,
    /// Which way the eye is looking — square out of the front of the head,
    /// so it swings down with the neck when the creature pecks.
    pub gaze: Vector3<f32>,
    /// Horizontal direction the creature faces.
    pub facing: Vector3<f32>,
    /// Left leg, then right.
    pub legs: [Leg; 2],
    pub feelers: DroopPair,

    solver: FABRIKSolver,
}

impl PeeperSkeleton {
    pub fn new(config: &PeeperRigConfig, pelvis: Point3<f32>, facing: Vector3<f32>) -> Self {
        let facing = facing.try_normalize(1e-4).unwrap_or_else(Vector3::z);
        let right = right_vector(facing);
        let bend = knee_bend(config, facing);

        let legs = [FootSide::Left, FootSide::Right].map(|side| {
            Leg::new(
                pelvis + right * (side.side_sign() * config.hip_width),
                config.upper_leg_length,
                config.lower_leg_length,
                facing,
                bend,
            )
        });

        let shoulder = pelvis + Vector3::y() * config.haunch_height;
        let (eye, gaze) = head_at(config, shoulder, facing, config.rest_pitch);
        let frame = Frame::upright(eye, facing);
        let feelers = DroopPair::new(config.feeler, feeler_anchors(config), &frame);

        Self {
            pelvis,
            shoulder,
            eye,
            gaze,
            facing,
            legs,
            feelers,
            solver: FABRIKSolver::default(),
        }
    }

    /// Place the body, pitch the neck, and solve the legs under it.
    pub fn update(
        &mut self,
        dt: f32,
        config: &PeeperRigConfig,
        pelvis: Point3<f32>,
        facing: Vector3<f32>,
        neck_pitch: f32,
        feet: [FootPose; 2],
    ) {
        self.facing = facing.try_normalize(1e-4).unwrap_or(self.facing);
        self.pelvis = pelvis;
        self.shoulder = pelvis + Vector3::y() * config.haunch_height;

        let (eye, gaze) = head_at(config, self.shoulder, self.facing, neck_pitch);
        self.eye = eye;
        self.gaze = gaze;

        let right = right_vector(self.facing);
        let bend = knee_bend(config, self.facing);

        for (index, side) in [FootSide::Left, FootSide::Right].into_iter().enumerate() {
            let hip = self.pelvis + right * (side.side_sign() * config.hip_width);
            let foot = feet[index];
            self.legs[index].solve(&mut self.solver, hip, foot.position, bend);
            self.legs[index].foot_forward = foot.forward;
            self.legs[index].foot_up = foot.up;
        }

        let frame = Frame::upright(self.eye, self.facing);
        self.feelers.update(
            dt,
            feeler_anchors(config),
            &frame,
            (self.eye, config.eye_radius * EYE_CLEARANCE_FRACTION),
        );
    }

    /// The head's frame, for placing parts authored in its local space.
    pub fn head_frame(&self) -> Frame {
        Frame::upright(self.eye, self.facing)
    }
}

/// Where the head sits and what it is looking at, for a neck pitched
/// `pitch` radians forward of vertical.
///
/// The gaze is the neck direction turned a quarter of a turn forward, not
/// the neck direction itself: the eye is on the *front* of the head, so a
/// creature standing upright looks at the horizon and one mid-peck looks
/// down its own strike.
fn head_at(
    config: &PeeperRigConfig,
    shoulder: Point3<f32>,
    facing: Vector3<f32>,
    pitch: f32,
) -> (Point3<f32>, Vector3<f32>) {
    let (sin, cos) = pitch.sin_cos();
    let neck = Vector3::y() * cos + facing * sin;
    let gaze = facing * cos - Vector3::y() * sin;
    (shoulder + neck * config.neck_length, gaze)
}

/// Where the feelers root, in the head's local frame.
fn feeler_anchors(config: &PeeperRigConfig) -> [Vector3<f32>; 2] {
    let (x, y, z) = config.feeler_anchor;
    let (x, y, z) = (
        x * config.eye_radius,
        y * config.eye_radius,
        z * config.eye_radius,
    );
    [Vector3::new(-x, y, z), Vector3::new(x, y, z)]
}

/// Which way the knees break. Backward, like a bird's hock.
fn knee_bend(config: &PeeperRigConfig, facing: Vector3<f32>) -> Vector3<f32> {
    (facing * config.knee_bend)
        .try_normalize(1e-4)
        .unwrap_or(facing)
}
