//! Where every joint of a stilt walker is this frame.
//!
//! ```text
//!   pelvis ──haunch──► shoulder ──neck(pitch)──► head ──► gaze
//!     │
//!     ├── hip L ──► knee L ──► foot L
//!     └── hip R ──► knee R ──► foot R
//! ```
//!
//! Pure placement. What drives the feet is the gait, what drives the neck
//! pitch is the creature's strike, and what is drawn on the joints is the
//! walker's [`Dress`](super::Dress).

use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::animation::rig::{right_vector, Leg};
use crate::animation::FootSide;
use crate::model::Transform;

use super::config::StiltRigConfig;

/// What a foot is doing, as the gait decided it.
#[derive(Clone, Copy, Debug)]
pub struct FootPose {
    pub position: Point3<f32>,
    /// Horizontal direction the toe points.
    pub forward: Vector3<f32>,
    /// Sole normal.
    pub up: Vector3<f32>,
}

/// The walker's joints.
pub struct StiltSkeleton {
    /// Where the legs meet, at the bottom of the haunch.
    pub pelvis: Point3<f32>,
    /// Top of the haunch, where the neck leaves the body.
    pub shoulder: Point3<f32>,
    /// Middle of the head.
    pub head: Point3<f32>,
    /// Which way the head looks: square out of its front, so it swings
    /// down with the neck when the creature strikes.
    pub gaze: Vector3<f32>,
    /// Horizontal direction the creature faces.
    pub facing: Vector3<f32>,
    /// Left leg, then right.
    pub legs: [Leg; 2],
}

/// A pose at `origin` with +Y along `up` and +Z as near `forward` as that
/// allows.
pub fn pose(origin: Point3<f32>, up: Vector3<f32>, forward: Vector3<f32>) -> Transform {
    let up = up.try_normalize(1e-6).unwrap_or_else(Vector3::y);
    let forward = (forward - up * forward.dot(&up))
        .try_normalize(1e-6)
        .unwrap_or_else(|| {
            // `forward` ran along `up`: any direction square to it will do.
            let spare = if up.x.abs() < 0.9 {
                Vector3::x()
            } else {
                Vector3::z()
            };
            (spare - up * spare.dot(&up)).normalize()
        });
    let right = up.cross(&forward);
    Transform {
        translation: origin.coords,
        rotation: UnitQuaternion::from_basis_unchecked(&[right, up, forward]),
        scale: Vector3::repeat(1.0),
    }
}

/// A pose at `origin` with +Z exactly along `forward` and +Y as near `up`
/// as that allows: for something that points, as an eye does.
pub fn aim(origin: Point3<f32>, forward: Vector3<f32>, up: Vector3<f32>) -> Transform {
    let forward = forward.try_normalize(1e-6).unwrap_or_else(Vector3::z);
    let right = up.cross(&forward).try_normalize(1e-6).unwrap_or_else(|| {
        // `up` ran along `forward`: any direction square to it will do.
        let spare = if forward.y.abs() < 0.9 {
            Vector3::y()
        } else {
            Vector3::x()
        };
        spare.cross(&forward).normalize()
    });
    let up = forward.cross(&right);
    Transform {
        translation: origin.coords,
        rotation: UnitQuaternion::from_basis_unchecked(&[right, up, forward]),
        scale: Vector3::repeat(1.0),
    }
}

impl StiltSkeleton {
    pub fn new(config: &StiltRigConfig, pelvis: Point3<f32>, facing: Vector3<f32>) -> Self {
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
        let (head, gaze) = head_at(config, shoulder, facing, config.rest_pitch);

        Self {
            pelvis,
            shoulder,
            head,
            gaze,
            facing,
            legs,
        }
    }

    /// Place the body, pitch the neck, and solve the legs under it.
    pub fn update(
        &mut self,
        config: &StiltRigConfig,
        pelvis: Point3<f32>,
        facing: Vector3<f32>,
        neck_pitch: f32,
        feet: [FootPose; 2],
    ) {
        self.facing = facing.try_normalize(1e-4).unwrap_or(self.facing);
        self.pelvis = pelvis;
        self.shoulder = pelvis + Vector3::y() * config.haunch_height;

        let (head, gaze) = head_at(config, self.shoulder, self.facing, neck_pitch);
        self.head = head;
        self.gaze = gaze;

        let right = right_vector(self.facing);
        let bend = knee_bend(config, self.facing);

        for (index, side) in [FootSide::Left, FootSide::Right].into_iter().enumerate() {
            let hip = self.pelvis + right * (side.side_sign() * config.hip_width);
            let foot = feet[index];
            self.legs[index].solve(hip, foot.position, bend, Vector3::y());
            self.legs[index].foot_forward = foot.forward;
            self.legs[index].foot_up = foot.up;
        }
    }

    fn leg(&self, side: FootSide) -> &Leg {
        match side {
            FootSide::Left => &self.legs[0],
            FootSide::Right => &self.legs[1],
        }
    }

    /// Level over the hips, facing as the walker faces.
    pub fn hips(&self) -> Transform {
        pose(self.pelvis, Vector3::y(), self.facing)
    }

    /// At the shoulder, +Y to the head.
    pub fn neck(&self) -> Transform {
        pose(self.shoulder, self.head - self.shoulder, self.gaze)
    }

    /// At the head's middle, looking along its gaze: it tips with the
    /// neck, so what the head wears rears and spears with it.
    pub fn head(&self) -> Transform {
        let side = self.gaze.cross(&Vector3::y());
        let crown = side.cross(&self.gaze);
        pose(self.head, crown, self.gaze)
    }

    /// At the hip, +Y from the knee to the hip.
    pub fn thigh(&self, side: FootSide) -> Transform {
        let leg = self.leg(side);
        pose(leg.hip, leg.hip - leg.knee, self.facing)
    }

    /// At the knee, +Y from the foot to the knee.
    pub fn shin(&self, side: FootSide) -> Transform {
        let leg = self.leg(side);
        pose(leg.knee, leg.knee - leg.foot, self.facing)
    }

    /// At the foot, +Z to its toes and +Y off its sole.
    pub fn foot(&self, side: FootSide) -> Transform {
        let leg = self.leg(side);
        pose(leg.foot, leg.foot_up, leg.foot_forward)
    }
}

/// Where the head sits and what it is looking at, for a neck pitched
/// `pitch` radians forward of vertical.
///
/// The gaze is the neck direction turned a quarter of a turn forward, not
/// the neck direction itself: the face is on the *front* of the head, so a
/// creature standing upright looks at the horizon and one mid-strike looks
/// down its own blow.
fn head_at(
    config: &StiltRigConfig,
    shoulder: Point3<f32>,
    facing: Vector3<f32>,
    pitch: f32,
) -> (Point3<f32>, Vector3<f32>) {
    let (sin, cos) = pitch.sin_cos();
    let neck = Vector3::y() * cos + facing * sin;
    let gaze = facing * cos - Vector3::y() * sin;
    (shoulder + neck * config.neck_length, gaze)
}

/// Which way the knees break.
fn knee_bend(config: &StiltRigConfig, facing: Vector3<f32>) -> Vector3<f32> {
    (facing * config.knee_bend)
        .try_normalize(1e-4)
        .unwrap_or(facing)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_pose_is_a_rotation_with_y_up_and_z_forward() {
        let made = pose(
            Point3::new(1.0, 2.0, 3.0),
            Vector3::new(0.2, 1.0, 0.1),
            Vector3::new(0.3, 0.2, 1.0),
        );
        let up = made.rotation * Vector3::y();
        let forward = made.rotation * Vector3::z();
        assert!((up - Vector3::new(0.2, 1.0, 0.1).normalize()).norm() < 1e-5);
        assert!(forward.dot(&Vector3::new(0.3, 0.2, 1.0)) > 0.9);
        assert!(up.dot(&forward).abs() < 1e-5);
        // A rotation, not a mirror: +X is +Y × +Z.
        assert!((made.rotation * Vector3::x() - up.cross(&forward)).norm() < 1e-5);
    }

    #[test]
    fn a_pose_asked_to_face_along_its_own_up_still_holds_together() {
        let made = pose(Point3::origin(), Vector3::y(), Vector3::y());
        let up = made.rotation * Vector3::y();
        assert!((up - Vector3::y()).norm() < 1e-5);
        assert!((made.rotation * Vector3::z()).norm() > 0.99);
    }

    #[test]
    fn a_standing_walkers_bones_hang_from_their_joints() {
        let config = StiltRigConfig::default();
        let skeleton = StiltSkeleton::new(&config, Point3::new(0.0, 1.0, 0.0), Vector3::z());
        let thigh = skeleton.thigh(FootSide::Left);
        let leg = &skeleton.legs[0];
        // A point one thigh's length down the bone is the knee.
        let knee =
            thigh
                .to_matrix()
                .transform_point(&Point3::new(0.0, -config.upper_leg_length, 0.0));
        // A leg at rest is set only roughly; solved, it is exact.
        assert!((knee - leg.knee).norm() < 0.01, "{knee} vs {}", leg.knee);
        // The head looks the way the walker faces.
        let look = skeleton.head().rotation * Vector3::z();
        assert!(look.dot(&Vector3::z()) > 0.9);
    }
}
