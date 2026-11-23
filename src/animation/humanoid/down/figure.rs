//! A knocked-down figure from frame to frame: how its body lies and turns,
//! the posture it is in, and its pose, blended once a frame.

use std::f32::consts::FRAC_PI_2;

use nalgebra::{Point3, UnitQuaternion, Vector3};

use super::ground::GroundPlane;
use super::limbs::{Lie, TrunkShape};
use super::posture::{posture_of, Posture, PostureWeights, Turning};
use crate::animation::config::CharacterRigConfig;
use crate::animation::humanoid::body_frame::BodyFrame;
use crate::character::locomotion::{DownConfig, DownPhase};

/// What the animator is told about a knocked-down body each frame.
#[derive(Debug, Clone, Copy)]
pub struct Lying {
    /// The body's rotation.
    pub rotation: UnitQuaternion<f32>,
    /// The ground under it, while it rests on some.
    pub ground: Option<GroundPlane>,
    /// How the state judges a body down: whether it is lying, by the same
    /// measure ([`DownConfig::lies`]).
    pub judged: DownConfig,
}

/// A knocked-down figure's pose this frame: what the lower body and the
/// arms draw.
#[derive(Debug, Clone, Copy)]
pub struct DownPose {
    /// How far the pelvis is lowered from where the body puts it, world
    /// space.
    pub pelvis_offset: Vector3<f32>,
    /// How far the torso leans over the body's frame, toward its front, rad.
    pub torso_pitch: f32,
    /// How far the head bows over the chest, m.
    pub head_bow: f32,
    /// The feet, `[left, right]`, world space.
    pub feet: [Point3<f32>; 2],
    /// The hands, `[left, right]`, world space.
    pub hands: [Point3<f32>; 2],
}

/// How long a body keeps to the ground it was last on, s: one bouncing
/// along the ground is not in the air between bounces.
const GROUND_GRACE: f32 = 0.1;
/// Time constant the measured spin is eased with, s: a frame's turn is
/// noisy where the body bounces.
const SPIN_EASE: f32 = 0.05;
/// Time constant a posture's weight is eased toward being drawn or not, s.
const POSTURE_BLEND: f32 = 0.12;

/// A knocked-down figure: how its body lies and turns, how far into each
/// posture it is drawn, and its pose.
#[derive(Debug, Clone, Copy)]
pub struct DownFigure {
    /// The body's frame, as it lies.
    pub frame: BodyFrame,
    /// The pose drawn this frame.
    pub pose: DownPose,
    /// The body's rotation, which the next frame's spin is measured from.
    rotation: UnitQuaternion<f32>,
    /// The ground under it, kept for [`GROUND_GRACE`] after it was last
    /// there.
    ground: Option<GroundPlane>,
    /// Seconds since the ground was last under it.
    off_ground: f32,
    /// The body's spin, rad/s, world space, eased.
    spin: Vector3<f32>,
    /// How far it has turned end over end since it began tumbling, rad.
    tumbled: f32,
    /// The posture it is in.
    posture: Posture,
    /// How much of each posture is drawn.
    weights: PostureWeights,
}

impl DownFigure {
    /// The figure this frame, from the last one's (`None` on the frame the
    /// body went down): the body `lying` as it does, in `phase`, its rig
    /// drawn with the pelvis where the body puts it at `pelvis`.
    pub fn advance(
        last: Option<&DownFigure>,
        lying: &Lying,
        phase: &DownPhase,
        rig: &CharacterRigConfig,
        pelvis: Point3<f32>,
        dt: f32,
    ) -> Self {
        let frame = BodyFrame::of(&lying.rotation);
        let spin = match last {
            Some(last) if dt > 0.0 => {
                let turned = (lying.rotation * last.rotation.inverse()).scaled_axis() / dt;
                last.spin + (turned - last.spin) * (1.0 - (-dt / SPIN_EASE).exp())
            }
            Some(last) => last.spin,
            None => Vector3::zeros(),
        };
        let (ground, off_ground) = match (lying.ground, last) {
            (Some(ground), _) => (Some(ground), 0.0),
            (None, Some(last)) => {
                let off_ground = last.off_ground + dt;
                let kept = last.ground.filter(|_| off_ground < GROUND_GRACE);
                (kept, off_ground)
            }
            (None, None) => (None, f32::INFINITY),
        };
        let turning = Turning::of(
            spin,
            &frame,
            ground.as_ref(),
            last.map_or(0.0, |last| last.tumbled),
            dt,
        );
        let posture = posture_of(
            last.map(|last| last.posture),
            phase,
            &frame,
            ground.as_ref(),
            &turning,
            &lying.judged,
        );
        let weights = match last {
            Some(last) => last
                .weights
                .eased_toward(posture, 1.0 - (-dt / POSTURE_BLEND).exp()),
            None => PostureWeights::only(posture),
        };
        let up = ground.map_or_else(Vector3::y, |ground| ground.normal);
        let lie = Lie {
            frame,
            ground,
            risen: (frame.up.dot(&up).clamp(-1.0, 1.0).asin() / FRAC_PI_2).max(0.0),
            rig,
        };
        Self {
            frame,
            pose: posed(&lie, &weights, pelvis),
            rotation: lying.rotation,
            ground,
            off_ground,
            spin,
            tumbled: turning.tumbled,
            posture,
            weights,
        }
    }

    /// The posture the figure is in, by name, for reports.
    pub fn posture_tag(&self) -> &'static str {
        self.posture.tag()
    }
}

/// The pose of a figure lying as `lie` says, drawn in each posture by
/// `weights`, its pelvis where the body puts it at `pelvis`: the trunk
/// blended first, then each posture's feet and hands placed from that one
/// trunk and blended as offsets from the pelvis.
fn posed(lie: &Lie, weights: &PostureWeights, pelvis: Point3<f32>) -> DownPose {
    let shape = weights
        .drawn()
        .fold(TrunkShape::STRAIGHT, |shape, (posture, weight)| {
            shape.plus(lie.trunk_shape(posture, pelvis), weight)
        });
    let trunk = lie.trunk(pelvis + shape.drop, shape.lean);
    let mut feet = [Vector3::zeros(); 2];
    let mut hands = [Vector3::zeros(); 2];
    for (posture, weight) in weights.drawn() {
        let limbs = lie.limbs(posture, &trunk);
        for side in 0..2 {
            feet[side] += (limbs.feet[side] - pelvis) * weight;
            hands[side] += (limbs.hands[side] - pelvis) * weight;
        }
    }
    DownPose {
        pelvis_offset: shape.drop,
        torso_pitch: shape.lean,
        head_bow: shape.bow,
        feet: feet.map(|offset| pelvis + offset),
        hands: hands.map(|offset| pelvis + offset),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::animation::humanoid::down::ground::level_ground;

    const DT: f32 = 1.0 / 60.0;

    fn limp() -> DownPhase {
        DownPhase::Limp { remaining: 1.0 }
    }

    fn gathered(face_up: bool) -> DownPhase {
        DownPhase::Gathered {
            remaining: 1.0,
            elapsed: 0.5,
            face_up,
        }
    }

    /// The player's body lying along +z on level ground, on its front or its
    /// back, its axis a capsule's radius up; and where that puts its pelvis.
    fn lying_on(face_up: bool) -> (Lying, Point3<f32>) {
        let pitch = if face_up { -FRAC_PI_2 } else { FRAC_PI_2 };
        let rotation = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), pitch);
        let lying = Lying {
            rotation,
            ground: Some(level_ground()),
            judged: DownConfig::default(),
        };
        let pelvis = Point3::new(0.0, 0.25, 0.0) - (rotation * Vector3::y()) * 0.075;
        (lying, pelvis)
    }

    /// The figure after `frames` frames lying as `lying` does, in `phase`.
    fn settled(lying: &Lying, phase: &DownPhase, pelvis: Point3<f32>, frames: usize) -> DownFigure {
        let rig = CharacterRigConfig::default();
        let mut figure = DownFigure::advance(None, lying, phase, &rig, pelvis, DT);
        for _ in 0..frames {
            figure = DownFigure::advance(Some(&figure), lying, phase, &rig, pelvis, DT);
        }
        figure
    }

    /// At rest on its front, gathered, the figure is on its hands and knees:
    /// its feet and hands on the ground, its knees a thigh under its hips.
    #[test]
    fn gathered_on_its_front_it_kneels_on_the_ground() {
        let rig = CharacterRigConfig::default();
        let (lying, pelvis) = lying_on(false);
        let figure = settled(&lying, &gathered(false), pelvis, 60);
        let pose = figure.pose;
        for limb in pose.feet.iter().chain(&pose.hands) {
            assert!(limb.y.abs() < 1e-3, "off the ground: {limb}");
        }
        for (foot, side) in pose
            .feet
            .iter()
            .zip([figure.frame.left(), figure.frame.right])
        {
            let hip = pelvis + side * rig.hip_width;
            let thigh_and_shin = rig.upper_leg_length.hypot(rig.lower_leg_length);
            let reach = (foot - hip).norm();
            assert!(
                (reach - thigh_and_shin).abs() < 0.02,
                "foot {reach:.3} from its hip"
            );
            assert!(foot.z < hip.z, "the foot trails behind the knee");
        }
    }

    /// At rest on its back, gathered, the figure sits back on its hands: its
    /// pelvis lowered onto the ground, its torso leaned up, its feet ahead
    /// of its hips and its hands behind its shoulders, all on the ground.
    #[test]
    fn gathered_on_its_back_it_sits_back_on_its_hands() {
        let rig = CharacterRigConfig::default();
        let (lying, pelvis) = lying_on(true);
        let figure = settled(&lying, &gathered(true), pelvis, 60);
        let pose = figure.pose;
        assert!(((pelvis + pose.pelvis_offset).y - rig.pelvis_radius).abs() < 1e-3);
        assert!(pose.torso_pitch > 0.5, "leaned up {}", pose.torso_pitch);
        let head = figure.frame.up;
        for (foot, hand) in pose.feet.iter().zip(&pose.hands) {
            assert!(foot.y.abs() < 1e-3 && hand.y.abs() < 1e-3);
            assert!((foot - pelvis).dot(&head) < 0.0, "feet toward the feet");
            assert!((hand - pelvis).dot(&head) > rig.torso_height * 0.5);
        }
    }

    /// A figure far from the world's origin is posed as one at it, moved: its
    /// limbs are blended about its pelvis, not the origin.
    #[test]
    fn a_figure_is_posed_the_same_wherever_it_is() {
        let (near, pelvis) = lying_on(false);
        let away = Vector3::new(300.0, 40.0, -250.0);
        let far = Lying {
            ground: near.ground.map(|ground| GroundPlane {
                point: ground.point + away,
                ..ground
            }),
            ..near
        };
        let phase = limp();
        let changing = |lying: &Lying, pelvis: Point3<f32>| {
            let rig = CharacterRigConfig::default();
            let mut figure = DownFigure::advance(None, lying, &gathered(false), &rig, pelvis, DT);
            for _ in 0..60 {
                figure = DownFigure::advance(Some(&figure), lying, &phase, &rig, pelvis, DT);
            }
            figure.pose
        };
        let (here, there) = (changing(&near, pelvis), changing(&far, pelvis + away));
        for side in 0..2 {
            assert!((there.feet[side] - away - here.feet[side]).norm() < 1e-3);
            assert!((there.hands[side] - away - here.hands[side]).norm() < 1e-3);
        }
    }

    /// A body bouncing along the ground keeps to it between bounces, and is
    /// in the air once it has been off it for longer than `GROUND_GRACE`.
    #[test]
    fn a_body_bouncing_along_the_ground_keeps_to_it() {
        let rig = CharacterRigConfig::default();
        let (on_ground, pelvis) = lying_on(false);
        let bounced = Lying {
            ground: None,
            ..on_ground
        };
        let mut figure = settled(&on_ground, &limp(), pelvis, 10);
        assert_eq!(figure.posture, Posture::Spread);
        let bounce = |figure: &DownFigure| {
            DownFigure::advance(Some(figure), &bounced, &limp(), &rig, pelvis, DT)
        };
        figure = bounce(&figure);
        assert_eq!(figure.posture, Posture::Spread, "between bounces");
        for _ in 0..(GROUND_GRACE / DT).ceil() as usize {
            figure = bounce(&figure);
        }
        assert_eq!(figure.posture, Posture::Sprawled, "in the air");
    }

    /// A body spinning end over end through the air, its turn read frame to
    /// frame, comes to be drawn tucked, eased in over a few frames.
    #[test]
    fn a_body_tumbling_through_the_air_tucks() {
        let rig = CharacterRigConfig::default();
        let pelvis = Point3::new(0.0, 3.0, 0.0);
        let turned = |frame: usize| Lying {
            rotation: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 6.0 * DT * frame as f32),
            ground: None,
            judged: DownConfig::default(),
        };
        let mut figure = DownFigure::advance(None, &turned(0), &limp(), &rig, pelvis, DT);
        assert_eq!(figure.posture, Posture::Sprawled);
        for frame in 1..40 {
            figure = DownFigure::advance(Some(&figure), &turned(frame), &limp(), &rig, pelvis, DT);
        }
        assert_eq!(figure.posture, Posture::Tucked);
        let tucked = figure
            .weights
            .drawn()
            .find(|&(posture, _)| posture == Posture::Tucked);
        assert!(tucked.is_some_and(|(_, weight)| weight > 0.95));
    }
}
