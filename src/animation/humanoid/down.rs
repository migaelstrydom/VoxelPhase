//! A knocked-down body, drawn as it lies and as it moves: buckling where it
//! was knocked down standing, tucked while it tumbles end over end, a log
//! roll along the ground, spread out as it slides, then gathered on its knees
//! or sat back on its hands, and getting up from there as the body pitches
//! up.
//!
//! ```text
//!   Lying (the body's turn, the ground under it) + DownPhase
//!        │  DownFigure::advance: its spin, the posture it is in (with
//!        │  hysteresis), each posture's weight eased toward that one
//!        ▼
//!   DownFigure ──▶ trunk: the pelvis lowered, the torso leaned
//!              ──▶ limbs: each posture's feet and hands, placed from that
//!                  trunk in the body's current frame, blended by weight
//! ```
//!
//! The capsule the body is drawn in is round and half a metre thick, and the
//! figure is a thin one: laid flat along the capsule's axis it would hang a
//! fifth of a metre over the ground. So no posture on the ground lies flat:
//! the limbs reach down to the ground the capsule rests on and hold the
//! figure up where the capsule is.

use std::f32::consts::{FRAC_PI_2, PI};

use nalgebra::{Point3, UnitQuaternion, Vector3};

use super::body_frame::BodyFrame;
use super::pose_state::smoothstep;
use crate::animation::config::CharacterRigConfig;
use crate::character::locomotion::DownPhase;

/// The ground a lying body rests on, taken as flat under it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GroundPlane {
    /// A point on the ground.
    pub point: Point3<f32>,
    /// Up from the ground, unit length.
    pub normal: Vector3<f32>,
}

impl GroundPlane {
    /// The ground a capsule `half_height` long each way from `centre`
    /// (caps included) and `radius` thick rests on, its axis along `axis`:
    /// level with the capsule's lowest point, facing `normal`.
    pub fn under_capsule(
        centre: Point3<f32>,
        axis: Vector3<f32>,
        normal: Vector3<f32>,
        half_height: f32,
        radius: f32,
    ) -> Self {
        let lowest = radius + (half_height - radius) * axis.dot(&normal).abs();
        Self {
            point: centre - normal * lowest,
            normal,
        }
    }

    fn height_of(&self, point: Point3<f32>) -> f32 {
        (point - self.point).dot(&self.normal)
    }

    fn under(&self, point: Point3<f32>) -> Point3<f32> {
        point - self.normal * self.height_of(point)
    }

    /// `direction` laid along the ground, unit length; `None` where it stands
    /// square to it.
    fn along(&self, direction: Vector3<f32>) -> Option<Vector3<f32>> {
        (direction - self.normal * direction.dot(&self.normal)).try_normalize(1e-3)
    }

    /// Where a limb `reach` long from `root` sets its end on the ground: out
    /// along `toward` (laid along the ground) up to `wanted` from under the
    /// root, as far as the limb goes. Too high to reach the ground, it hangs
    /// toward it at full stretch.
    fn reached(
        &self,
        root: Point3<f32>,
        toward: Vector3<f32>,
        wanted: f32,
        reach: f32,
    ) -> Point3<f32> {
        let height = self.height_of(root).max(0.0);
        if height >= reach {
            return root - self.normal * reach;
        }
        let across = (reach * reach - height * height).sqrt();
        let out = self.along(toward).unwrap_or_else(Vector3::zeros);
        self.under(root) + out * wanted.min(across)
    }
}

/// What the animator is told about a knocked-down body each frame.
#[derive(Debug, Clone, Copy)]
pub struct Lying {
    /// The body's rotation.
    pub rotation: UnitQuaternion<f32>,
    /// The ground under it, while it rests on some.
    pub ground: Option<GroundPlane>,
}

/// How a knocked-down body holds its limbs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Posture {
    /// Knees giving way over feet planted under the hips, hands reaching
    /// ahead for the ground: still upright on the ground, falling over.
    Buckling,
    /// Limbs trailing along the body: thrown through the air, not spinning.
    Sprawled,
    /// Knees to the chest, hands on the shins: tumbling end over end,
    /// through the air or along the ground.
    Tucked,
    /// Legs together, arms stretched past the head: rolling along the ground
    /// about its length.
    LogRoll,
    /// Arms and legs out across the ground: sliding, or come to rest before
    /// it gathers itself.
    Spread,
    /// On its hands and knees: gathered on its front, and getting up off
    /// them.
    Kneeling,
    /// Sat back on its hands with its knees up: gathered on its back, and
    /// sitting up from there.
    SatBack,
}

const POSTURES: [Posture; 7] = [
    Posture::Buckling,
    Posture::Sprawled,
    Posture::Tucked,
    Posture::LogRoll,
    Posture::Spread,
    Posture::Kneeling,
    Posture::SatBack,
];

/// Spin end over end, rad/s, past which a body tumbling tucks, and below
/// which a tucked one lets go again.
const TUCK_SPIN: (f32, f32) = (4.0, 2.5);
/// How far a body on the ground turns end over end before it is tumbling
/// rather than falling over, rad: half a turn, over its head or its heels.
const TUMBLE_TURN: f32 = PI;
/// Spin about its own length, rad/s, past which a body on the ground is
/// rolling, and below which a rolling one has stopped.
const ROLL_SPIN: (f32, f32) = (3.0, 1.5);
/// Time constant the measured spin is eased with, s: a frame's turn is
/// noisy where the body bounces.
const SPIN_EASE: f32 = 0.05;
/// Time constant a posture's weight is eased toward being drawn or not, s.
const POSTURE_BLEND: f32 = 0.12;
/// How long a body keeps to the ground it was last on, s.
const GROUND_GRACE: f32 = 0.1;
/// How far a body on the ground stands up off it, as the cosine of its lean,
/// past which it is still upright rather than lying.
const UPRIGHT: f32 = 0.5;

/// How far a tucked torso curls over its knees, rad.
const TUCK_CURL: f32 = 0.5;
/// How far back from upright a body sat on the ground leans its torso,
/// rad: back onto its hands.
const SIT_LEAN: f32 = 0.7;
/// How far from under its hips a body sat back sets its feet, toward
/// them, m: its knees up.
const SAT_FEET_AHEAD: f32 = 0.3;
/// How far from under its shoulders a body sat back sets its hands,
/// behind and out, m.
const SAT_HANDS_BEHIND: f32 = 0.2;
/// A limb's reach as a share of its length when it is set on the ground:
/// just short of straight, so the elbow or knee gives.
const GROUND_REACH: f32 = 0.95;
/// A limb's length as a share of its own when it trails or stretches along
/// the body.
const STRETCH: f32 = 0.97;

/// How much further apart than its hips a sprawled body's feet trail, m.
const SPRAWL_SPLAY: f32 = 0.06;
/// How much further apart than its shoulders a sprawled body's hands trail,
/// m.
const SPRAWL_ARM_SPLAY: f32 = 0.1;
/// A tucked body's feet from its hips, m: forward and down its length.
const TUCK_FEET: (f32, f32) = (0.12, 0.08);
/// A tucked body's hands from its hips, m: forward, up its length, and in
/// toward each other.
const TUCK_HANDS: (f32, f32, f32) = (0.2, 0.06, 0.02);
/// How far a tucked body's head bows, m.
const TUCK_HEAD_BOW: f32 = 0.05;
/// How far in from its hips and shoulders a log-rolling body holds its feet
/// and hands, m: together.
const TOGETHER: f32 = 0.03;
/// Which way a spread body's legs go: out to the side and down its length.
const SPREAD_LEGS: (f32, f32) = (0.45, 0.9);
/// Which way a spread body's arms go: out to the side and up its length.
const SPREAD_ARMS: (f32, f32) = (0.8, 0.5);
/// How far out to the side, for each unit behind, a body sat back sets its
/// hands.
const SAT_HANDS_OUT: f32 = 0.4;
/// Over what of getting up the feet are drawn in under the hips.
const FEET_IN: (f32, f32) = (0.2, 0.9);
/// Over what of getting up the hands push off the ground and come to hang.
const HANDS_OFF: (f32, f32) = (0.15, 0.55);

/// The trunk a posture's limbs are placed from.
struct Trunk {
    frame: BodyFrame,
    hips: [Point3<f32>; 2],
    shoulders: [Point3<f32>; 2],
    /// Out to each side, `[left, right]`.
    sides: [Vector3<f32>; 2],
}

/// A knocked-down figure: how its body lies, and how far into each posture
/// it is drawn.
#[derive(Debug, Clone, Copy)]
pub struct DownFigure {
    /// The body's frame, as it lies.
    pub frame: BodyFrame,
    rotation: UnitQuaternion<f32>,
    /// The ground under it, kept for [`GROUND_GRACE`] after it was last
    /// there: a body bouncing along it is not in the air between bounces.
    ground: Option<GroundPlane>,
    /// Seconds since the ground was last under it.
    off_ground: f32,
    /// The body's spin, rad/s, world space, eased.
    spin: Vector3<f32>,
    /// How far it has turned end over end, rad, since it began spinning so
    /// past [`TUCK_SPIN`]'s lower bound.
    tumbled: f32,
    posture: Posture,
    /// How much of each posture is drawn, in [`POSTURES`]' order, summing
    /// to one.
    weights: [f32; 7],
    /// How far the body has pitched up off the ground: 0 lying along it, 1
    /// standing.
    risen: f32,
}

impl DownFigure {
    /// The figure this frame, from the last one's (`None` on the frame the
    /// body went down), the body `lying` as it does, in `phase`.
    pub fn advance(last: Option<&DownFigure>, lying: &Lying, phase: &DownPhase, dt: f32) -> Self {
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
        let about_length = spin.dot(&frame.up);
        let end_over_end = (spin - frame.up * about_length).norm();
        let tumbled = match last {
            Some(last) if end_over_end > TUCK_SPIN.1 => last.tumbled + end_over_end * dt,
            _ => 0.0,
        };
        let turning = Turning {
            about_length,
            end_over_end,
            tumbled,
        };
        let posture = posture_of(
            last.map(|last| last.posture),
            phase,
            &frame,
            ground.as_ref(),
            &turning,
        );
        let weights = match last {
            Some(last) => {
                let ease = 1.0 - (-dt / POSTURE_BLEND).exp();
                let mut weights = last.weights;
                for (weight, held) in weights.iter_mut().zip(POSTURES) {
                    let target = if held == posture { 1.0 } else { 0.0 };
                    *weight += (target - *weight) * ease;
                }
                weights
            }
            None => POSTURES.map(|held| if held == posture { 1.0 } else { 0.0 }),
        };
        let up = ground.map_or_else(Vector3::y, |ground| ground.normal);
        let risen = (frame.up.dot(&up).clamp(-1.0, 1.0).asin() / FRAC_PI_2).max(0.0);
        Self {
            frame,
            rotation: lying.rotation,
            ground,
            off_ground,
            spin,
            tumbled,
            posture,
            weights,
            risen,
        }
    }

    /// How far the figure's pelvis is lowered from where the body puts it,
    /// world space, and how far its torso leans over the body's frame, rad.
    pub fn trunk(&self, rig: &CharacterRigConfig, pelvis: Point3<f32>) -> (Vector3<f32>, f32) {
        self.weighted()
            .map(|(posture, weight)| {
                let (drop, lean) = self.posture_trunk(posture, rig, pelvis);
                (drop * weight, lean * weight)
            })
            .fold((Vector3::zeros(), 0.0), |(drop, lean), (d, l)| {
                (drop + d, lean + l)
            })
    }

    /// Where the feet and the hands go, each `[left, right]`, world space:
    /// each posture's, blended.
    pub fn limbs(
        &self,
        rig: &CharacterRigConfig,
        pelvis: Point3<f32>,
    ) -> ([Point3<f32>; 2], [Point3<f32>; 2]) {
        let (drop, lean) = self.trunk(rig, pelvis);
        let trunk = self.trunk_at(rig, pelvis + drop, lean);
        let mut feet = [Vector3::zeros(); 2];
        let mut hands = [Vector3::zeros(); 2];
        for (posture, weight) in self.weighted() {
            let (posture_feet, posture_hands) = self.posture_limbs(posture, &trunk, rig);
            for side in 0..2 {
                feet[side] += posture_feet[side].coords * weight;
                hands[side] += posture_hands[side].coords * weight;
            }
        }
        (feet.map(Point3::from), hands.map(Point3::from))
    }

    /// The posture the figure is in, by name, for reports.
    pub fn posture_tag(&self) -> &'static str {
        match self.posture {
            Posture::Buckling => "buckling",
            Posture::Sprawled => "sprawled",
            Posture::Tucked => "tucked",
            Posture::LogRoll => "log_roll",
            Posture::Spread => "spread",
            Posture::Kneeling => "kneeling",
            Posture::SatBack => "sat_back",
        }
    }

    /// How far a tucked figure's head bows over its chest, m.
    pub fn head_bow(&self) -> f32 {
        self.weights[Posture::Tucked as usize] * TUCK_HEAD_BOW
    }

    /// The postures drawn, with their weights.
    fn weighted(&self) -> impl Iterator<Item = (Posture, f32)> + '_ {
        let total: f32 = self.weights.iter().sum();
        POSTURES
            .into_iter()
            .zip(self.weights)
            .filter(|&(_, weight)| weight > 1e-3)
            .map(move |(posture, weight)| (posture, weight / total))
    }

    /// The trunk's lowering and lean in `posture`.
    fn posture_trunk(
        &self,
        posture: Posture,
        rig: &CharacterRigConfig,
        pelvis: Point3<f32>,
    ) -> (Vector3<f32>, f32) {
        match (posture, self.ground) {
            (Posture::Tucked, _) => (Vector3::zeros(), TUCK_CURL),
            (Posture::SatBack, Some(ground)) => {
                let seat = (ground.height_of(pelvis) - rig.pelvis_radius).max(0.0);
                let sat = 1.0 - self.risen;
                let lean = (SIT_LEAN - self.risen * FRAC_PI_2).max(0.0);
                (-ground.normal * seat * sat, lean)
            }
            _ => (Vector3::zeros(), 0.0),
        }
    }

    /// The trunk with its pelvis at `pelvis` and its torso leaned `lean`.
    fn trunk_at(&self, rig: &CharacterRigConfig, pelvis: Point3<f32>, lean: f32) -> Trunk {
        let frame = self.frame;
        let torso = frame.leaned(lean);
        let chest = pelvis + torso.up * rig.torso_height;
        let sides = [frame.left(), frame.right];
        Trunk {
            frame,
            hips: sides.map(|side| pelvis + side * rig.hip_width),
            shoulders: [torso.left(), torso.right].map(|side| chest + side * rig.shoulder_width),
            sides,
        }
    }

    /// The feet and hands, each `[left, right]`, of `posture` from `trunk`.
    fn posture_limbs(
        &self,
        posture: Posture,
        trunk: &Trunk,
        rig: &CharacterRigConfig,
    ) -> ([Point3<f32>; 2], [Point3<f32>; 2]) {
        let (leg, arm) = (rig.leg_length(), rig.arm_length());
        let (up, front) = (trunk.frame.up, trunk.frame.front);
        let each = |place: &dyn Fn(usize) -> Point3<f32>| [place(0), place(1)];
        let sprawled = || {
            (
                each(&|side| {
                    trunk.hips[side] + trunk.sides[side] * SPRAWL_SPLAY - up * leg * STRETCH
                }),
                each(&|side| {
                    trunk.shoulders[side] + trunk.sides[side] * SPRAWL_ARM_SPLAY
                        - up * arm * STRETCH
                }),
            )
        };
        match (posture, self.ground) {
            (Posture::Sprawled, _) => sprawled(),
            (Posture::Buckling, Some(ground)) => {
                let ahead = ground.along(front).unwrap_or_else(|| trunk.frame.heading);
                (
                    each(&|side| ground.under(trunk.hips[side]) + trunk.sides[side] * SPRAWL_SPLAY),
                    each(&|side| {
                        ground.reached(trunk.shoulders[side], ahead, arm, arm * GROUND_REACH)
                    }),
                )
            }
            (Posture::Tucked, _) => (
                each(&|side| trunk.hips[side] + front * TUCK_FEET.0 - up * TUCK_FEET.1),
                each(&|side| {
                    trunk.hips[side] + front * TUCK_HANDS.0 + up * TUCK_HANDS.1
                        - trunk.sides[side] * TUCK_HANDS.2
                }),
            ),
            (Posture::LogRoll, _) => (
                each(&|side| trunk.hips[side] - up * leg * STRETCH - trunk.sides[side] * TOGETHER),
                each(&|side| {
                    trunk.shoulders[side] + up * arm * STRETCH - trunk.sides[side] * TOGETHER
                }),
            ),
            (Posture::Spread, Some(ground)) => (
                each(&|side| {
                    let out = trunk.sides[side] * SPREAD_LEGS.0 - up * SPREAD_LEGS.1;
                    ground.reached(trunk.hips[side], out, leg, leg * GROUND_REACH)
                }),
                each(&|side| {
                    let out = trunk.sides[side] * SPREAD_ARMS.0 + up * SPREAD_ARMS.1;
                    ground.reached(trunk.shoulders[side], out, arm, arm * GROUND_REACH)
                }),
            ),
            (Posture::Kneeling, Some(ground)) => {
                let ahead = ground.along(up).or_else(|| ground.along(front));
                let ahead = ahead.unwrap_or_else(|| trunk.frame.heading);
                let kneeling = (
                    each(&|side| ground.under(trunk.hips[side]) - ahead * rig.lower_leg_length),
                    each(&|side| {
                        ground.reached(trunk.shoulders[side], ahead, arm, arm * GROUND_REACH)
                    }),
                );
                self.getting_up(kneeling, trunk, &ground, rig)
            }
            (Posture::SatBack, Some(ground)) => {
                let headward = ground.along(up).unwrap_or_else(|| -trunk.frame.heading);
                let sat = (
                    each(&|side| {
                        ground.reached(
                            trunk.hips[side],
                            -headward,
                            SAT_FEET_AHEAD,
                            leg * GROUND_REACH,
                        )
                    }),
                    each(&|side| {
                        let out = headward + trunk.sides[side] * SAT_HANDS_OUT;
                        ground.reached(
                            trunk.shoulders[side],
                            out,
                            SAT_HANDS_BEHIND,
                            arm * GROUND_REACH,
                        )
                    }),
                );
                self.getting_up(sat, trunk, &ground, rig)
            }
            (Posture::Buckling | Posture::Spread | Posture::Kneeling | Posture::SatBack, None) => {
                sprawled()
            }
        }
    }

    /// Feet and hands as the body pitches up off `gathered`, set from its
    /// hips and shoulders as they are: the feet drawn in under the hips, and
    /// the hands, pushing off the ground at first, let hang at the sides.
    /// Lying along the ground, `gathered` as it is.
    fn getting_up(
        &self,
        gathered: ([Point3<f32>; 2], [Point3<f32>; 2]),
        trunk: &Trunk,
        ground: &GroundPlane,
        rig: &CharacterRigConfig,
    ) -> ([Point3<f32>; 2], [Point3<f32>; 2]) {
        let arm = rig.arm_length();
        let drawn_in = smoothstep(FEET_IN.0, FEET_IN.1, self.risen);
        let let_go = smoothstep(HANDS_OFF.0, HANDS_OFF.1, self.risen);
        let (feet, hands) = gathered;
        let feet = [0, 1].map(|side| {
            let under_hip = ground.under(trunk.hips[side]);
            feet[side] + (under_hip - feet[side]) * drawn_in
        });
        let hands = [0, 1].map(|side| {
            let hanging = trunk.shoulders[side] - trunk.frame.up * arm * STRETCH
                + trunk.sides[side] * SPRAWL_ARM_SPLAY;
            hands[side] + (hanging - hands[side]) * let_go
        });
        (feet, hands)
    }
}

/// How a body is turning.
#[derive(Debug, Clone, Copy, Default)]
struct Turning {
    /// Its spin about its own length, rad/s, either way.
    about_length: f32,
    /// Its spin end over end, rad/s.
    end_over_end: f32,
    /// How far it has turned end over end since it began tumbling, rad
    /// ([`DownFigure::tumbled`]).
    tumbled: f32,
}

/// The posture a body is in, from the one it was in (`None` on its first
/// frame down), its phase, how it lies, the ground it rests on if any, and
/// how it is turning. In the air, spinning end over end is tumbling; on the
/// ground it is only once the body has gone over, not as it falls flat.
fn posture_of(
    last: Option<Posture>,
    phase: &DownPhase,
    frame: &BodyFrame,
    ground: Option<&GroundPlane>,
    turning: &Turning,
) -> Posture {
    let gathered_as = |face_up: bool| {
        if face_up {
            Posture::SatBack
        } else {
            Posture::Kneeling
        }
    };
    match phase {
        DownPhase::Gathered { face_up, .. } => gathered_as(*face_up),
        DownPhase::Rising { .. } => match last {
            Some(posture @ (Posture::Kneeling | Posture::SatBack)) => posture,
            _ => gathered_as(frame.front.y > 0.0),
        },
        DownPhase::Limp { .. } => {
            let past = |posture: Posture, rate: f32, (on, off): (f32, f32)| {
                rate > if last == Some(posture) { off } else { on }
            };
            let upright = ground.is_some_and(|ground| frame.up.dot(&ground.normal) > UPRIGHT);
            let tumbling = past(Posture::Tucked, turning.end_over_end, TUCK_SPIN)
                && (ground.is_none() || turning.tumbled > TUMBLE_TURN);
            if upright {
                Posture::Buckling
            } else if tumbling {
                Posture::Tucked
            } else if ground.is_none() {
                Posture::Sprawled
            } else if past(Posture::LogRoll, turning.about_length.abs(), ROLL_SPIN) {
                Posture::LogRoll
            } else {
                Posture::Spread
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 60.0;

    fn limp() -> DownPhase {
        DownPhase::Limp { remaining: 1.0 }
    }

    fn flat() -> GroundPlane {
        GroundPlane {
            point: Point3::origin(),
            normal: Vector3::y(),
        }
    }

    /// The player's body lying along +z on flat ground, on its front or its
    /// back, its axis a capsule's radius up.
    fn lying_on(face_up: bool) -> (Lying, Point3<f32>) {
        let pitch = if face_up { -FRAC_PI_2 } else { FRAC_PI_2 };
        let rotation = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), pitch);
        let lying = Lying {
            rotation,
            ground: Some(flat()),
        };
        let pelvis = Point3::new(0.0, 0.25, 0.0) - (rotation * Vector3::y()) * 0.075;
        (lying, pelvis)
    }

    /// The figure after `frames` frames lying as `lying` does, in `phase`.
    fn settled(lying: &Lying, phase: &DownPhase, frames: usize) -> DownFigure {
        let mut figure = DownFigure::advance(None, lying, phase, DT);
        for _ in 0..frames {
            figure = DownFigure::advance(Some(&figure), lying, phase, DT);
        }
        figure
    }

    /// The ground a capsule lies on is a radius under its axis lying flat,
    /// and its half height under its centre standing.
    #[test]
    fn a_capsule_rests_on_the_ground_at_its_lowest_point() {
        let centre = Point3::new(1.0, 2.0, 3.0);
        let flat = GroundPlane::under_capsule(centre, Vector3::z(), Vector3::y(), 0.5, 0.25);
        assert!((flat.point.y - 1.75).abs() < 1e-6);
        let standing = GroundPlane::under_capsule(centre, Vector3::y(), Vector3::y(), 0.5, 0.25);
        assert!((standing.point.y - 1.5).abs() < 1e-6);
    }

    /// A limb set on the ground ends on it, out as far as asked where it
    /// reaches that far, and as far as it reaches where not; one too high
    /// hangs toward the ground at full stretch.
    #[test]
    fn a_limb_set_on_the_ground_ends_on_it_within_its_reach() {
        let ground = flat();
        let root = Point3::new(0.0, 0.3, 0.0);
        let near = ground.reached(root, Vector3::x(), 0.2, 0.5);
        assert!((near - Point3::new(0.2, 0.0, 0.0)).norm() < 1e-6);
        let far = ground.reached(root, Vector3::x(), 2.0, 0.5);
        assert!((far - Point3::new(0.4, 0.0, 0.0)).norm() < 1e-5);
        let high = ground.reached(Point3::new(0.0, 0.8, 0.0), Vector3::x(), 0.2, 0.5);
        assert!((high - Point3::new(0.0, 0.3, 0.0)).norm() < 1e-6);
    }

    /// At rest on its front, gathered, the figure is on its hands and knees:
    /// its feet and hands on the ground, its knees a thigh under its hips.
    #[test]
    fn gathered_on_its_front_it_kneels_on_the_ground() {
        let rig = CharacterRigConfig::default();
        let (lying, pelvis) = lying_on(false);
        let gathered = DownPhase::Gathered {
            remaining: 1.0,
            elapsed: 0.5,
            face_up: false,
        };
        let figure = settled(&lying, &gathered, 60);
        let (feet, hands) = figure.limbs(&rig, pelvis);
        for limb in feet.iter().chain(&hands) {
            assert!(limb.y.abs() < 1e-3, "off the ground: {limb}");
        }
        for (foot, side) in feet.iter().zip([figure.frame.left(), figure.frame.right]) {
            let hip = pelvis + side * rig.hip_width;
            let knee_at = (foot - hip).norm();
            assert!(
                (knee_at - (rig.upper_leg_length.powi(2) + rig.lower_leg_length.powi(2)).sqrt())
                    .abs()
                    < 0.02,
                "foot {knee_at:.3} from its hip"
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
        let gathered = DownPhase::Gathered {
            remaining: 1.0,
            elapsed: 0.5,
            face_up: true,
        };
        let figure = settled(&lying, &gathered, 60);
        let (drop, lean) = figure.trunk(&rig, pelvis);
        assert!(((pelvis + drop).y - rig.pelvis_radius).abs() < 1e-3);
        assert!((lean - SIT_LEAN).abs() < 1e-3);
        let (feet, hands) = figure.limbs(&rig, pelvis);
        let head = figure.frame.up;
        for (foot, hand) in feet.iter().zip(&hands) {
            assert!(foot.y.abs() < 1e-3 && hand.y.abs() < 1e-3);
            assert!((foot - pelvis).dot(&head) < 0.0, "feet toward the feet");
            assert!((hand - pelvis).dot(&head) > rig.torso_height * 0.5);
        }
    }

    /// Tumbling end over end, through the air or, once it has gone over,
    /// along the ground, a body tucks, and lets go once it stops; falling
    /// flat, it does not. Rolling along the ground about its length, it log
    /// rolls; sliding without rolling, it spreads; still upright on the
    /// ground, it buckles.
    #[test]
    fn the_posture_follows_the_spin() {
        let frame = BodyFrame::of(&UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 1.2));
        let end_over_end = |rate: f32, tumbled: f32| Turning {
            end_over_end: rate,
            tumbled,
            ..Turning::default()
        };
        let rolling = |rate: f32| Turning {
            about_length: rate,
            ..Turning::default()
        };
        let air = |last, turning| posture_of(last, &limp(), &frame, None, &turning);
        let ground = |last, turning| posture_of(last, &limp(), &frame, Some(&flat()), &turning);
        assert_eq!(air(None, end_over_end(5.0, 0.0)), Posture::Tucked);
        let tucked = Some(Posture::Tucked);
        assert_eq!(air(tucked, end_over_end(3.0, 1.0)), Posture::Tucked);
        assert_eq!(air(tucked, end_over_end(2.0, 0.0)), Posture::Sprawled);
        assert_eq!(air(None, rolling(4.0)), Posture::Sprawled);
        assert_eq!(ground(None, rolling(4.0)), Posture::LogRoll);
        assert_eq!(
            ground(Some(Posture::LogRoll), rolling(-2.0)),
            Posture::LogRoll
        );
        assert_eq!(
            ground(None, end_over_end(5.0, 1.5)),
            Posture::Spread,
            "falling flat"
        );
        assert_eq!(
            ground(None, end_over_end(5.0, 3.5)),
            Posture::Tucked,
            "gone over"
        );
        assert_eq!(ground(None, Turning::default()), Posture::Spread);
        let upright = BodyFrame::of(&UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.3));
        let buckling = posture_of(None, &limp(), &upright, Some(&flat()), &Turning::default());
        assert_eq!(buckling, Posture::Buckling);
    }

    /// A body bouncing along the ground keeps to it between bounces, and is
    /// in the air once it has been off it for longer than `GROUND_GRACE`.
    #[test]
    fn a_body_bouncing_along_the_ground_keeps_to_it() {
        let (on_ground, _) = lying_on(false);
        let bounced = Lying {
            ground: None,
            ..on_ground
        };
        let mut figure = settled(&on_ground, &limp(), 10);
        assert_eq!(figure.posture, Posture::Spread);
        figure = DownFigure::advance(Some(&figure), &bounced, &limp(), DT);
        assert_eq!(figure.posture, Posture::Spread, "between bounces");
        for _ in 0..(GROUND_GRACE / DT).ceil() as usize {
            figure = DownFigure::advance(Some(&figure), &bounced, &limp(), DT);
        }
        assert_eq!(figure.posture, Posture::Sprawled, "in the air");
    }

    /// A body spinning end over end through the air, its turn read frame to
    /// frame, comes to be drawn tucked, eased in over a few frames.
    #[test]
    fn a_body_tumbling_through_the_air_tucks() {
        let turned = |frame: usize| Lying {
            rotation: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 6.0 * DT * frame as f32),
            ground: None,
        };
        let mut figure = DownFigure::advance(None, &turned(0), &limp(), DT);
        assert_eq!(figure.weights[Posture::Sprawled as usize], 1.0);
        for frame in 1..30 {
            figure = DownFigure::advance(Some(&figure), &turned(frame), &limp(), DT);
        }
        assert_eq!(figure.posture, Posture::Tucked);
        assert!(figure.weights[Posture::Tucked as usize] > 0.95);
        assert!((figure.weights.iter().sum::<f32>() - 1.0).abs() < 1e-5);
    }
}
