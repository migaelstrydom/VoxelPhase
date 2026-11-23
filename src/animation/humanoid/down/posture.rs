//! Which posture a knocked-down body is in, read from its phase, how it lies
//! and how it turns, and how much of each posture is drawn as one gives way
//! to the next.

use std::f32::consts::PI;

use nalgebra::Vector3;

use super::ground::GroundPlane;
use crate::animation::humanoid::body_frame::BodyFrame;
use crate::character::locomotion::{DownConfig, DownPhase};

/// How a knocked-down body holds its limbs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Posture {
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

impl Posture {
    /// Every posture.
    const ALL: [Posture; 7] = [
        Posture::Buckling,
        Posture::Sprawled,
        Posture::Tucked,
        Posture::LogRoll,
        Posture::Spread,
        Posture::Kneeling,
        Posture::SatBack,
    ];

    /// Its name, for reports.
    pub(super) fn tag(self) -> &'static str {
        match self {
            Posture::Buckling => "buckling",
            Posture::Sprawled => "sprawled",
            Posture::Tucked => "tucked",
            Posture::LogRoll => "log_roll",
            Posture::Spread => "spread",
            Posture::Kneeling => "kneeling",
            Posture::SatBack => "sat_back",
        }
    }
}

/// How much of each posture is drawn, summing to one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct PostureWeights([f32; Posture::ALL.len()]);

impl PostureWeights {
    /// All of `posture` and none of the rest.
    pub(super) fn only(posture: Posture) -> Self {
        Self(Posture::ALL.map(|held| if held == posture { 1.0 } else { 0.0 }))
    }

    /// These weights eased `ease` of the way toward all of `posture`.
    pub(super) fn eased_toward(&self, posture: Posture, ease: f32) -> Self {
        let target = Self::only(posture);
        let mut weights = self.0;
        for (weight, target) in weights.iter_mut().zip(target.0) {
            *weight += (target - *weight) * ease;
        }
        Self(weights)
    }

    /// The postures drawn and how much of each, summing to one: any too
    /// slight to see is left out.
    pub(super) fn drawn(&self) -> impl Iterator<Item = (Posture, f32)> + '_ {
        let seen = |weight: f32| weight > UNSEEN;
        let total: f32 = self.0.iter().copied().filter(|&weight| seen(weight)).sum();
        Posture::ALL
            .into_iter()
            .zip(self.0)
            .filter(move |&(_, weight)| seen(weight))
            .map(move |(posture, weight)| (posture, weight / total))
    }
}

/// A posture weight below which it is not drawn at all.
const UNSEEN: f32 = 1e-3;

/// A threshold a rising value crosses at `enter` and a falling one at
/// `exit`, so one hovering about it does not flicker.
#[derive(Debug, Clone, Copy)]
struct Hysteresis {
    enter: f32,
    exit: f32,
}

impl Hysteresis {
    /// Whether `value` is past the threshold, for one that `was` past it.
    fn past(&self, value: f32, was: bool) -> bool {
        value > if was { self.exit } else { self.enter }
    }
}

/// Spin end over end, rad/s, past which a tumbling body tucks.
const TUCK_SPIN: Hysteresis = Hysteresis {
    enter: 4.0,
    exit: 2.5,
};
/// Spin about its own length, rad/s, past which a body on the ground is
/// rolling.
const ROLL_SPIN: Hysteresis = Hysteresis {
    enter: 3.0,
    exit: 1.5,
};
/// How far a body on the ground turns end over end before it is tumbling
/// rather than falling over, rad: half a turn, over its head or its heels.
const TUMBLE_TURN: f32 = PI;
/// How much further than lying down (`DownConfig::lies`) a buckling body
/// leans before it is drawn lying, rad: one falling through the measure
/// jitters about it on its contacts.
const UPRIGHT_SLACK: f32 = 0.09;

/// How a body is turning.
#[derive(Debug, Clone, Copy, Default)]
pub(super) struct Turning {
    /// Its spin about its own length, rad/s, either way.
    about_length: f32,
    /// Its spin end over end, rad/s.
    end_over_end: f32,
    /// How far it has turned end over end, rad, since it began spinning
    /// faster than a tucked body lets go at.
    pub(super) tumbled: f32,
}

impl Turning {
    /// How a body in `frame` turns at `spin`, rad/s, world space, over a
    /// step of `dt`, having `tumbled` so far. On the `ground`, a spin about
    /// its normal turns the body flat on it, not end over end.
    pub(super) fn of(
        spin: Vector3<f32>,
        frame: &BodyFrame,
        ground: Option<&GroundPlane>,
        tumbled: f32,
        dt: f32,
    ) -> Self {
        let about_length = spin.dot(&frame.up);
        let mut across = spin - frame.up * about_length;
        if let Some(ground) = ground {
            across -= ground.normal * across.dot(&ground.normal);
        }
        let end_over_end = across.norm();
        Self {
            about_length,
            end_over_end,
            tumbled: if end_over_end > TUCK_SPIN.exit {
                tumbled + end_over_end * dt
            } else {
                0.0
            },
        }
    }
}

/// The posture a body is in, from the one it was in (`None` on its first
/// frame down), its phase, how it lies, the ground it rests on if any, how
/// it is turning, and how the state judges a body lying down. Gathered and
/// getting up, it is the gathered posture for its face. Limp, it is read
/// from how it lies and turns: in the air, spinning end over end is
/// tumbling; on the ground it is only once the body has gone over, not as
/// it falls flat.
pub(super) fn posture_of(
    last: Option<Posture>,
    phase: &DownPhase,
    frame: &BodyFrame,
    ground: Option<&GroundPlane>,
    turning: &Turning,
    judged: &DownConfig,
) -> Posture {
    match *phase {
        DownPhase::Gathered { face_up, .. } | DownPhase::Rising { face_up, .. } => {
            if face_up {
                Posture::SatBack
            } else {
                Posture::Kneeling
            }
        }
        DownPhase::Limp { .. } => {
            let was = |posture: Posture| last == Some(posture);
            let lean = frame.up.y.clamp(-1.0, 1.0).acos();
            let slack = if was(Posture::Buckling) {
                UPRIGHT_SLACK
            } else {
                0.0
            };
            let upright = ground.is_some() && !judged.lies(lean - slack);
            let tumbling = TUCK_SPIN.past(turning.end_over_end, was(Posture::Tucked))
                && (ground.is_none() || turning.tumbled > TUMBLE_TURN);
            if upright {
                Posture::Buckling
            } else if tumbling {
                Posture::Tucked
            } else if ground.is_none() {
                Posture::Sprawled
            } else if ROLL_SPIN.past(turning.about_length.abs(), was(Posture::LogRoll)) {
                Posture::LogRoll
            } else {
                Posture::Spread
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::FRAC_PI_2;

    use nalgebra::UnitQuaternion;

    use super::*;
    use crate::animation::humanoid::down::ground::level_ground;

    fn limp() -> DownPhase {
        DownPhase::Limp { remaining: 1.0 }
    }

    fn lying_at(lean: f32) -> BodyFrame {
        BodyFrame::of(&UnitQuaternion::from_axis_angle(&Vector3::x_axis(), lean))
    }

    /// Tumbling end over end, through the air or, once it has gone over,
    /// along the ground, a body tucks, and lets go once it stops; falling
    /// flat, it does not. Rolling along the ground about its length, it log
    /// rolls; sliding without rolling, it spreads.
    #[test]
    fn the_posture_follows_the_spin() {
        let judged = DownConfig::default();
        let frame = lying_at(1.2);
        let end_over_end = |rate: f32, tumbled: f32| Turning {
            end_over_end: rate,
            tumbled,
            ..Turning::default()
        };
        let rolling = |rate: f32| Turning {
            about_length: rate,
            ..Turning::default()
        };
        let flat = level_ground();
        let air = |last, turning| posture_of(last, &limp(), &frame, None, &turning, &judged);
        let ground =
            |last, turning| posture_of(last, &limp(), &frame, Some(&flat), &turning, &judged);
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
    }

    /// Still upright on the ground by the state's own measure, a body
    /// buckles, and keeps buckling a little past it as it falls through.
    #[test]
    fn a_body_upright_on_the_ground_buckles() {
        let judged = DownConfig::default();
        let flat = level_ground();
        let posture = |last, lean: f32| {
            posture_of(
                last,
                &limp(),
                &lying_at(lean),
                Some(&flat),
                &Turning::default(),
                &judged,
            )
        };
        let edge = judged.lying_tilt;
        assert_eq!(posture(None, edge - 0.05), Posture::Buckling);
        assert_eq!(posture(None, edge + 0.05), Posture::Spread);
        assert_eq!(
            posture(Some(Posture::Buckling), edge + 0.05),
            Posture::Buckling
        );
        assert_eq!(
            posture(Some(Posture::Buckling), edge + 0.15),
            Posture::Spread
        );
    }

    /// A body spinning flat on the ground is not tumbling, however long it
    /// spins; one turning over is.
    #[test]
    fn a_spin_flat_on_the_ground_is_not_tumbling() {
        let frame = lying_at(FRAC_PI_2);
        let flat = level_ground();
        let spun = |spin: Vector3<f32>| {
            (0..60).fold(Turning::default(), |turning, _| {
                Turning::of(spin, &frame, Some(&flat), turning.tumbled, 1.0 / 60.0)
            })
        };
        let flat_spin = spun(Vector3::y() * 6.0);
        assert!(flat_spin.end_over_end < 1e-3 && flat_spin.tumbled == 0.0);
        let over = spun(frame.right * 6.0);
        assert!((over.end_over_end - 6.0).abs() < 1e-4 && over.tumbled > TUMBLE_TURN);
    }

    /// The postures drawn always sum to one, those too slight to see left
    /// out of the sum as well as the drawing.
    #[test]
    fn the_postures_drawn_sum_to_one() {
        let mut weights = PostureWeights::only(Posture::Spread);
        for _ in 0..50 {
            weights = weights.eased_toward(Posture::Kneeling, 0.1);
        }
        let drawn: Vec<_> = weights.drawn().collect();
        assert_eq!(drawn.len(), 2, "the spread pose is still faintly drawn");
        let total: f32 = drawn.iter().map(|&(_, weight)| weight).sum();
        assert!((total - 1.0).abs() < 1e-6);
        for _ in 0..100 {
            weights = weights.eased_toward(Posture::Kneeling, 0.1);
        }
        assert_eq!(
            weights.drawn().collect::<Vec<_>>(),
            [(Posture::Kneeling, 1.0)]
        );
    }
}
