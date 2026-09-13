//! Where every part of a heart critter is this frame.
//!
//! ```text
//!   pelvis ──stub──► torso centre ──► ear anchors ──► DroopPair
//!     │
//!     ├── hip L ──► Leg L ──► foot L
//!     └── hip R ──► Leg R ──► foot R
//! ```
//!
//! Pure placement. What drives the feet is the gait; what draws the result
//! is `mesh`.

use nalgebra::{Point3, Vector3};

use crate::animation::rig::{right_vector, DroopPair, Frame, Leg};
use crate::animation::FootSide;
use crate::skeleton::fabrik::FABRIKSolver;

use super::config::CritterRigConfig;

/// Where an ear meets the head, in the torso's local frame, as fractions
/// of the heart's own width and height. The lobes of a heart are the only
/// place an ear can sit without looking stuck on.
const EAR_ANCHOR_WIDTH_FRACTION: f32 = 0.26;
const EAR_ANCHOR_HEIGHT_FRACTION: f32 = 0.36;

/// How much of the torso's half-width the ears are kept clear of.
/// Slightly under 1 so an ear can rest against the head without being
/// shoved off it.
const HEAD_CLEARANCE_FRACTION: f32 = 0.92;

/// What a foot is doing, as the gait decided it.
#[derive(Clone, Copy, Debug)]
pub struct FootPose {
    pub position: Point3<f32>,
    /// Horizontal direction the toe points.
    pub forward: Vector3<f32>,
    /// Sole normal.
    pub up: Vector3<f32>,
}

/// The critter's joints.
pub struct CritterSkeleton {
    /// Where the legs meet, at the bottom of the pelvis stub.
    pub pelvis: Point3<f32>,
    /// Centre of the heart.
    pub torso: Point3<f32>,
    /// Horizontal direction the critter faces.
    pub facing: Vector3<f32>,
    /// Left leg, then right.
    pub legs: [Leg; 2],
    pub ears: DroopPair,

    solver: FABRIKSolver,
}

impl CritterSkeleton {
    pub fn new(config: &CritterRigConfig, pelvis: Point3<f32>, facing: Vector3<f32>) -> Self {
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

        let torso = pelvis + Vector3::y() * config.torso_rise();
        let frame = Frame::upright(torso, facing);
        let ears = DroopPair::new(config.ear, ear_anchors(config), &frame);

        Self {
            pelvis,
            torso,
            facing,
            legs,
            ears,
            solver: FABRIKSolver::default(),
        }
    }

    /// Place the body and solve the legs under it.
    pub fn update(
        &mut self,
        dt: f32,
        config: &CritterRigConfig,
        pelvis: Point3<f32>,
        facing: Vector3<f32>,
        feet: [FootPose; 2],
    ) {
        self.facing = facing.try_normalize(1e-4).unwrap_or(self.facing);
        self.pelvis = pelvis;
        self.torso = pelvis + Vector3::y() * config.torso_rise();

        let right = right_vector(self.facing);
        let bend = knee_bend(config, self.facing);

        for (index, side) in [FootSide::Left, FootSide::Right].into_iter().enumerate() {
            let hip = self.pelvis + right * (side.side_sign() * config.hip_width);
            let foot = feet[index];
            self.legs[index].solve(&mut self.solver, hip, foot.position, bend);
            self.legs[index].foot_forward = foot.forward;
            self.legs[index].foot_up = foot.up;
        }

        let frame = Frame::upright(self.torso, self.facing);
        self.ears.update(
            dt,
            ear_anchors(config),
            &frame,
            (
                self.torso,
                config.torso_width * 0.5 * HEAD_CLEARANCE_FRACTION,
            ),
        );
    }

    /// The torso's frame, for placing parts authored in its local space.
    pub fn torso_frame(&self) -> Frame {
        Frame::upright(self.torso, self.facing)
    }
}

/// Where the ears meet the head, in the torso's local frame.
fn ear_anchors(config: &CritterRigConfig) -> [Vector3<f32>; 2] {
    let x = config.torso_width * EAR_ANCHOR_WIDTH_FRACTION;
    let y = config.torso_height * EAR_ANCHOR_HEIGHT_FRACTION;
    [Vector3::new(-x, y, 0.0), Vector3::new(x, y, 0.0)]
}

/// Which way the knees break. A hock (negative `knee_bend`) reads as an
/// animal; a forward knee reads as a person in a costume.
fn knee_bend(config: &CritterRigConfig, facing: Vector3<f32>) -> Vector3<f32> {
    (facing * config.knee_bend)
        .try_normalize(1e-4)
        .unwrap_or(facing)
}
