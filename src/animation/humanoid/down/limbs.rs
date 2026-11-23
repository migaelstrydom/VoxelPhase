//! Where each posture puts a knocked-down figure's trunk, feet and hands.

use std::f32::consts::FRAC_PI_2;

use nalgebra::{Point3, Vector3};

use super::ground::GroundPlane;
use super::posture::Posture;
use crate::animation::config::CharacterRigConfig;
use crate::animation::humanoid::body_frame::BodyFrame;
use crate::animation::humanoid::pose_state::smoothstep;

/// How a figure lies as its limbs are placed: what every posture reads.
pub(super) struct Lie<'a> {
    /// The body's frame.
    pub(super) frame: BodyFrame,
    /// The ground under it, while it rests on some.
    pub(super) ground: Option<GroundPlane>,
    /// How far it has pitched up off the ground: 0 lying along it, 1
    /// standing.
    pub(super) risen: f32,
    /// The rig drawn.
    pub(super) rig: &'a CharacterRigConfig,
}

/// How a posture holds the trunk.
#[derive(Debug, Clone, Copy)]
pub(super) struct TrunkShape {
    /// How far the pelvis is lowered from where the body puts it, world
    /// space.
    pub(super) drop: Vector3<f32>,
    /// How far the torso leans over the body's frame, toward its front, rad.
    pub(super) lean: f32,
    /// How far the head bows over the chest, m.
    pub(super) bow: f32,
}

impl TrunkShape {
    /// The trunk as the body holds it.
    pub(super) const STRAIGHT: Self = Self {
        drop: Vector3::new(0.0, 0.0, 0.0),
        lean: 0.0,
        bow: 0.0,
    };

    /// This shape and `other`, `weight` of it, added.
    pub(super) fn plus(self, other: Self, weight: f32) -> Self {
        Self {
            drop: self.drop + other.drop * weight,
            lean: self.lean + other.lean * weight,
            bow: self.bow + other.bow * weight,
        }
    }
}

/// The trunk a posture's limbs are placed from.
pub(super) struct Trunk {
    /// The body's frame.
    frame: BodyFrame,
    /// The hips, `[left, right]`.
    hips: [Point3<f32>; 2],
    /// The shoulders, `[left, right]`, on the leaned torso.
    shoulders: [Point3<f32>; 2],
    /// Out to each side, `[left, right]`.
    sides: [Vector3<f32>; 2],
}

/// Where a posture puts the feet and hands, world space.
#[derive(Debug, Clone, Copy)]
pub(super) struct Limbs {
    /// The feet, `[left, right]`.
    pub(super) feet: [Point3<f32>; 2],
    /// The hands, `[left, right]`.
    pub(super) hands: [Point3<f32>; 2],
}

/// A stretch of getting up, from `from` of the way up to `to`, eased in and
/// out.
struct Span {
    from: f32,
    to: f32,
}

impl Span {
    /// How far through the stretch a body `risen` of the way up is, 0 to 1.
    fn at(&self, risen: f32) -> f32 {
        smoothstep(self.from, self.to, risen)
    }
}

/// A limb's reach as a share of its length when it is set on the ground:
/// just short of straight, so the elbow or knee gives.
const GROUND_REACH: f32 = 0.95;
/// A limb's length as a share of its own when it trails or stretches along
/// the body.
const STRETCH: f32 = 0.97;
/// How much further apart than its hips a sprawled body's feet trail, m.
const SPRAWL_FEET_SPLAY: f32 = 0.06;
/// How much further apart than its shoulders a sprawled body's hands trail,
/// m.
const SPRAWL_HANDS_SPLAY: f32 = 0.1;
/// How far a tucked torso curls over its knees, rad.
const TUCK_CURL: f32 = 0.5;
/// How far a tucked body's head bows, m.
const TUCK_HEAD_BOW: f32 = 0.05;
/// How far forward of its hips a tucked body draws its feet, m.
const TUCK_FEET_FORWARD: f32 = 0.12;
/// How far down its length from its hips a tucked body draws its feet, m.
const TUCK_FEET_DOWN: f32 = 0.08;
/// How far forward of its hips a tucked body holds its hands, on its
/// shins, m.
const TUCK_HANDS_FORWARD: f32 = 0.2;
/// How far up its length from its hips a tucked body holds its hands, m.
const TUCK_HANDS_UP: f32 = 0.06;
/// How far in from its hips a tucked body holds its hands, m.
const TUCK_HANDS_IN: f32 = 0.02;
/// How far in from its hips and shoulders a log-rolling body holds its feet
/// and hands, m: together.
const TOGETHER: f32 = 0.03;
/// How much of the way a spread body's legs go is out to the side.
const SPREAD_LEGS_OUT: f32 = 0.45;
/// How much of the way a spread body's legs go is down its length.
const SPREAD_LEGS_DOWN: f32 = 0.9;
/// How much of the way a spread body's arms go is out to the side.
const SPREAD_ARMS_OUT: f32 = 0.8;
/// How much of the way a spread body's arms go is up its length.
const SPREAD_ARMS_UP: f32 = 0.5;
/// How far a body sat on the ground leans its torso up off lying along it,
/// rad: back onto its hands.
const SIT_LEAN: f32 = 0.7;
/// How far from under its hips a body sat back sets its feet, toward them,
/// m: its knees up.
const SAT_FEET_AHEAD: f32 = 0.3;
/// How far from under its shoulders a body sat back sets its hands, behind
/// and out, m.
const SAT_HANDS_BEHIND: f32 = 0.2;
/// How far out to the side, for each unit behind, a body sat back sets its
/// hands.
const SAT_HANDS_OUT: f32 = 0.4;
/// Over what of getting up the feet are drawn in under the hips.
const FEET_IN: Span = Span { from: 0.2, to: 0.9 };
/// Over what of getting up the hands push off the ground and come to hang.
const HANDS_OFF: Span = Span {
    from: 0.15,
    to: 0.55,
};

impl Lie<'_> {
    /// How `posture` holds the trunk, the pelvis where the body puts it at
    /// `pelvis`.
    pub(super) fn trunk_shape(&self, posture: Posture, pelvis: Point3<f32>) -> TrunkShape {
        match (posture, self.ground) {
            (Posture::Tucked, _) => TrunkShape {
                lean: TUCK_CURL,
                bow: TUCK_HEAD_BOW,
                ..TrunkShape::STRAIGHT
            },
            (Posture::SatBack, Some(ground)) => {
                let seat = (ground.height_of(pelvis) - self.rig.pelvis_radius).max(0.0);
                TrunkShape {
                    drop: -ground.normal * seat * (1.0 - self.risen),
                    lean: (SIT_LEAN - self.risen * FRAC_PI_2).max(0.0),
                    bow: 0.0,
                }
            }
            _ => TrunkShape::STRAIGHT,
        }
    }

    /// The trunk with its pelvis at `pelvis` and its torso leaned `lean`.
    pub(super) fn trunk(&self, pelvis: Point3<f32>, lean: f32) -> Trunk {
        let frame = self.frame;
        let torso = frame.leaned(lean);
        let chest = pelvis + torso.up * self.rig.torso_height;
        let sides = [frame.left(), frame.right];
        Trunk {
            frame,
            hips: sides.map(|side| pelvis + side * self.rig.hip_width),
            shoulders: [torso.left(), torso.right]
                .map(|side| chest + side * self.rig.shoulder_width),
            sides,
        }
    }

    /// The feet and hands `posture` places from `trunk`. A posture that sets
    /// them on the ground, with none under the body, sprawls.
    pub(super) fn limbs(&self, posture: Posture, trunk: &Trunk) -> Limbs {
        let (leg, arm) = (self.rig.leg_length(), self.rig.arm_length());
        let (up, front) = (trunk.frame.up, trunk.frame.front);
        let each = |place: &dyn Fn(usize) -> Point3<f32>| [place(0), place(1)];
        let sprawled = || Limbs {
            feet: each(&|side| {
                trunk.hips[side] + trunk.sides[side] * SPRAWL_FEET_SPLAY - up * leg * STRETCH
            }),
            hands: each(&|side| {
                trunk.shoulders[side] + trunk.sides[side] * SPRAWL_HANDS_SPLAY - up * arm * STRETCH
            }),
        };
        match (posture, self.ground) {
            (Posture::Sprawled, _) => sprawled(),
            (Posture::Tucked, _) => Limbs {
                feet: each(&|side| {
                    trunk.hips[side] + front * TUCK_FEET_FORWARD - up * TUCK_FEET_DOWN
                }),
                hands: each(&|side| {
                    trunk.hips[side] + front * TUCK_HANDS_FORWARD + up * TUCK_HANDS_UP
                        - trunk.sides[side] * TUCK_HANDS_IN
                }),
            },
            (Posture::LogRoll, _) => Limbs {
                feet: each(&|side| {
                    trunk.hips[side] - up * leg * STRETCH - trunk.sides[side] * TOGETHER
                }),
                hands: each(&|side| {
                    trunk.shoulders[side] + up * arm * STRETCH - trunk.sides[side] * TOGETHER
                }),
            },
            (Posture::Buckling, Some(ground)) => {
                let ahead = ground.along(front).unwrap_or(trunk.frame.heading);
                Limbs {
                    feet: each(&|side| {
                        ground.under(trunk.hips[side]) + trunk.sides[side] * SPRAWL_FEET_SPLAY
                    }),
                    hands: each(&|side| {
                        ground.reached(trunk.shoulders[side], ahead, arm, arm * GROUND_REACH)
                    }),
                }
            }
            (Posture::Spread, Some(ground)) => Limbs {
                feet: each(&|side| {
                    let out = trunk.sides[side] * SPREAD_LEGS_OUT - up * SPREAD_LEGS_DOWN;
                    ground.reached(trunk.hips[side], out, leg, leg * GROUND_REACH)
                }),
                hands: each(&|side| {
                    let out = trunk.sides[side] * SPREAD_ARMS_OUT + up * SPREAD_ARMS_UP;
                    ground.reached(trunk.shoulders[side], out, arm, arm * GROUND_REACH)
                }),
            },
            (Posture::Kneeling, Some(ground)) => {
                let ahead = ground.along(up).or_else(|| ground.along(front));
                let ahead = ahead.unwrap_or(trunk.frame.heading);
                let kneeling = Limbs {
                    feet: each(&|side| {
                        ground.under(trunk.hips[side]) - ahead * self.rig.lower_leg_length
                    }),
                    hands: each(&|side| {
                        ground.reached(trunk.shoulders[side], ahead, arm, arm * GROUND_REACH)
                    }),
                };
                self.getting_up(kneeling, trunk, &ground)
            }
            (Posture::SatBack, Some(ground)) => {
                let headward = ground.along(up).unwrap_or(-trunk.frame.heading);
                let sat = Limbs {
                    feet: each(&|side| {
                        let reach = leg * GROUND_REACH;
                        ground.reached(trunk.hips[side], -headward, SAT_FEET_AHEAD, reach)
                    }),
                    hands: each(&|side| {
                        let out = headward + trunk.sides[side] * SAT_HANDS_OUT;
                        let reach = arm * GROUND_REACH;
                        ground.reached(trunk.shoulders[side], out, SAT_HANDS_BEHIND, reach)
                    }),
                };
                self.getting_up(sat, trunk, &ground)
            }
            (Posture::Buckling | Posture::Spread | Posture::Kneeling | Posture::SatBack, None) => {
                sprawled()
            }
        }
    }

    /// The feet and hands as the body pitches up off `gathered`, set from its
    /// hips and shoulders as they are: the feet drawn in under the hips, and
    /// the hands, pushing off the ground at first, let hang at the sides.
    /// Lying along the ground, `gathered` as it is.
    fn getting_up(&self, gathered: Limbs, trunk: &Trunk, ground: &GroundPlane) -> Limbs {
        let arm = self.rig.arm_length();
        let (drawn_in, let_go) = (FEET_IN.at(self.risen), HANDS_OFF.at(self.risen));
        Limbs {
            feet: [0, 1].map(|side| {
                let under_hip = ground.under(trunk.hips[side]);
                gathered.feet[side] + (under_hip - gathered.feet[side]) * drawn_in
            }),
            hands: [0, 1].map(|side| {
                let hanging = trunk.shoulders[side] - trunk.frame.up * arm * STRETCH
                    + trunk.sides[side] * SPRAWL_HANDS_SPLAY;
                gathered.hands[side] + (hanging - gathered.hands[side]) * let_go
            }),
        }
    }
}
