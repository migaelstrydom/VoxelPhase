//! An overarm throw, laid over whatever the upper body is doing.
//!
//! ```text
//!   click          release (= windup)        follow-through        home
//!     │ cock back ──► whip over the shoulder ──► across to the hip ──► fade
//!     │   shoulder back, lean back   shoulder through, lean in
//!   weight 0 ──► 1 ───────────────────────────────────────► 1 ──► 0
//! ```
//!
//! The body turns to face a throw (gameplay steers it), but a throw to the
//! side or behind leaves before it has come round. The chest makes up the
//! difference, up to [`MAX_CHEST_TURN`], and the throw is made in the chest's
//! frame: keys are along where the chest faces, up, and out to the throwing
//! side, in arm lengths from the shoulder, so they fit any rig and any aim.
//!
//! The cock-back is quick and the whip accelerates into the release, so most
//! of the movement happens right after the click and the wind-up reads as
//! the throw starting, not as a delay. The follow-through is long and loose:
//! it is what makes the throw feel thrown.

use std::f32::consts::{PI, TAU};

use nalgebra::{Point3, Vector3};

use super::pose_state::smoothstep;
use crate::animation::pose::PoseFragment;
use crate::animation::state::ThrowGesture;
use crate::character::ThrowingHand;

/// Seconds from the release to the end of the follow-through.
const FOLLOW_THROUGH: f32 = 0.14;
/// Seconds from the end of the follow-through back to the arms' own pose.
const RECOVER: f32 = 0.3;
/// Share of the wind-up spent drawing the arm back; the rest is the whip.
const COCK_SHARE: f32 = 0.45;
/// Furthest the chest turns from the hips toward a throw, in radians.
const MAX_CHEST_TURN: f32 = 1.0;
/// Furthest the shoulders twist in all, chest turn and wind-up together.
const MAX_TWIST: f32 = 1.3;

/// One moment of the throw.
#[derive(Clone, Copy)]
struct Key {
    /// Throwing hand from its shoulder: (along aim, up, out to its side), in
    /// arm lengths.
    hand: Vector3<f32>,
    /// The other hand from its shoulder, the same way, with "out" toward its
    /// own side.
    off_hand: Vector3<f32>,
    /// Shoulder twist toward the throwing side's shoulder going *back*, in
    /// radians.
    twist: f32,
    /// Torso lean forward, in radians, on top of the gait's own.
    lean: f32,
}

/// Arm drawn back over the shoulder, elbow up; the other arm points the way.
const COCKED: Key = Key {
    hand: Vector3::new(-0.5, 0.5, 0.35),
    off_hand: Vector3::new(0.75, 0.3, 0.05),
    twist: 0.5,
    lean: -0.12,
};
/// Letting go: high over the head, the hand come in over the crown, so the
/// grenade leaves from the middle of the body where its arc starts.
const RELEASE: Key = Key {
    hand: Vector3::new(0.0, 0.75, -0.45),
    off_hand: Vector3::new(0.4, -0.3, 0.2),
    twist: -0.2,
    lean: 0.08,
};
/// Carried through, across the body to the far hip.
const FOLLOWED_THROUGH: Key = Key {
    hand: Vector3::new(0.5, -0.65, -0.45),
    off_hand: Vector3::new(-0.25, -0.7, 0.35),
    twist: -0.6,
    lean: 0.28,
};

impl ThrowGesture {
    /// Seconds from the click until the arms are their own again.
    pub fn duration(&self) -> f32 {
        self.windup + FOLLOW_THROUGH + RECOVER
    }

    /// Whether the throw has played out.
    pub fn finished(&self) -> bool {
        self.elapsed >= self.duration()
    }

    /// How much of the pose is the throw's rather than the arms' own, in
    /// [0, 1].
    fn weight(&self) -> f32 {
        let t = self.elapsed;
        let rise = if self.windup > 0.0 {
            // Off the mark fast: ease out, so the first frames move most.
            let r = (t / (self.windup * COCK_SHARE)).clamp(0.0, 1.0);
            1.0 - (1.0 - r) * (1.0 - r)
        } else {
            1.0
        };
        let fade_from = self.windup + FOLLOW_THROUGH;
        rise * (1.0 - smoothstep(fade_from, fade_from + RECOVER, t))
    }

    /// How far the chest turns from `facing` toward the aim, in radians
    /// about +y (positive turns left).
    fn chest_turn(&self, facing: Vector3<f32>) -> f32 {
        let off = yaw_of(self.aim) - yaw_of(facing);
        let off = (off + PI).rem_euclid(TAU) - PI;
        off.clamp(-MAX_CHEST_TURN, MAX_CHEST_TURN)
    }

    /// The pose of the throw this moment, before weighting.
    fn key(&self) -> Key {
        let t = self.elapsed;
        let cocked_at = self.windup * COCK_SHARE;
        if t >= self.windup {
            // Through: fast off the release, settling at the hip.
            let s = ((t - self.windup) / FOLLOW_THROUGH).min(1.0);
            lerp_key(&RELEASE, &FOLLOWED_THROUGH, 1.0 - (1.0 - s) * (1.0 - s))
        } else if t <= cocked_at {
            COCKED
        } else {
            // The whip: accelerating into the release.
            let s = (t - cocked_at) / (self.windup - cocked_at);
            lerp_key(&COCKED, &RELEASE, s * s)
        }
    }
}

/// Heading of a horizontal direction, in radians about +y from +z.
fn yaw_of(direction: Vector3<f32>) -> f32 {
    direction.x.atan2(direction.z)
}

fn lerp_key(a: &Key, b: &Key, t: f32) -> Key {
    Key {
        hand: a.hand.lerp(&b.hand, t),
        off_hand: a.off_hand.lerp(&b.off_hand, t),
        twist: a.twist + (b.twist - a.twist) * t,
        lean: a.lean + (b.lean - a.lean) * t,
    }
}

/// What a throw needs of the torso it is laid over.
pub struct ThrowRig {
    /// Horizontal direction the hips face.
    pub facing: Vector3<f32>,
    /// Arm length, shoulder to hand.
    pub reach: f32,
    /// The gait's own torso lean, in radians.
    pub torso_pitch: f32,
}

/// The overlay's say on the twist and lean, which decide where the
/// shoulders are; the caller places them and hands them back to
/// [`apply`].
pub struct TorsoTurn {
    pub twist: f32,
    pub torso_pitch: f32,
}

/// How the throws in progress turn the torso, starting from `twist`.
pub fn torso_turn(throws: &[ThrowGesture], twist: f32, rig: &ThrowRig) -> TorsoTurn {
    let mut turn = TorsoTurn {
        twist,
        torso_pitch: rig.torso_pitch,
    };
    for gesture in throws {
        let (key, weight) = (gesture.key(), gesture.weight());
        // Positive twist brings the left shoulder forward, so drawing the
        // right shoulder back is positive.
        let sign = match gesture.hand {
            ThrowingHand::Right => 1.0,
            ThrowingHand::Left => -1.0,
        };
        // Turning the chest left brings the right shoulder forward: negative.
        let twist =
            (sign * key.twist - gesture.chest_turn(rig.facing)).clamp(-MAX_TWIST, MAX_TWIST);
        turn.twist += (twist - turn.twist) * weight;
        turn.torso_pitch += key.lean * weight;
    }
    turn
}

/// Lay the throws in progress over `fragment`: its hands, with the shoulders
/// already where [`torso_turn`] put them.
pub fn apply(
    throws: &[ThrowGesture],
    fragment: &mut PoseFragment,
    shoulders: [Point3<f32>; 2],
    turn: &TorsoTurn,
    rig: &ThrowRig,
) {
    let Some(hands) = fragment.hands.as_mut() else {
        return;
    };
    let busy = |hand: ThrowingHand| throws.iter().any(|g| g.hand == hand);
    for gesture in throws {
        let (key, weight) = (gesture.key(), gesture.weight());
        let up = Vector3::y();
        let turned = yaw_of(rig.facing) + gesture.chest_turn(rig.facing);
        let aim = Vector3::new(turned.sin(), 0.0, turned.cos());
        // Out to the throwing side, and to the other hand's side.
        let right = aim.cross(&up);
        let (side, off_side) = match gesture.hand {
            ThrowingHand::Right => (right, -right),
            ThrowingHand::Left => (-right, right),
        };
        let place = |shoulder: Point3<f32>, k: Vector3<f32>, out: Vector3<f32>| {
            let offset = (aim * k.x + up * k.y + out * k.z) * rig.reach;
            shoulder + offset.cap_magnitude(rig.reach)
        };
        let (throwing, other) = match gesture.hand {
            ThrowingHand::Right => (&mut hands.right, &mut hands.left),
            ThrowingHand::Left => (&mut hands.left, &mut hands.right),
        };
        let (throwing_shoulder, other_shoulder) = match gesture.hand {
            ThrowingHand::Right => (shoulders[1], shoulders[0]),
            ThrowingHand::Left => (shoulders[0], shoulders[1]),
        };
        let target = place(throwing_shoulder, key.hand, side);
        *throwing = Point3::from(throwing.coords.lerp(&target.coords, weight));
        // The other arm counters, unless it is busy throwing too.
        if !busy(gesture.hand.other()) {
            let target = place(other_shoulder, key.off_hand, off_side);
            *other = Point3::from(other.coords.lerp(&target.coords, weight));
        }
    }
    fragment.shoulder_twist = Some(turn.twist);
    fragment.torso_pitch = Some(turn.torso_pitch);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gesture(elapsed: f32) -> ThrowGesture {
        ThrowGesture {
            stroke: 0,
            hand: ThrowingHand::Right,
            elapsed,
            windup: 0.12,
            aim: Vector3::z(),
        }
    }

    /// The click is answered at once: a frame in, the arm is already most
    /// of the way to drawn back.
    #[test]
    fn the_arm_moves_on_the_first_frame() {
        assert!(gesture(1.0 / 60.0).weight() > 0.5);
    }

    /// Overarm: the hand lets go high, come in over the head from its own
    /// side, not out beside the shoulder.
    #[test]
    fn the_hand_is_high_and_over_the_head_at_the_release() {
        let key = gesture(0.12).key();
        assert!(key.hand.y > 0.5 && key.hand.z < 0.0);
    }

    /// A throw behind turns the chest as far as it goes toward it, and the
    /// way round that is shorter; one ahead does not turn it at all.
    #[test]
    fn the_chest_turns_toward_a_throw_the_body_has_not_faced() {
        let facing = Vector3::z();
        let ahead = gesture(0.0);
        let left = ThrowGesture {
            aim: Vector3::x(),
            ..ahead
        };
        let behind_right = ThrowGesture {
            aim: Vector3::new(-0.1, 0.0, -1.0).normalize(),
            ..ahead
        };
        assert!(ahead.chest_turn(facing).abs() < 1e-6);
        assert!((left.chest_turn(facing) - MAX_CHEST_TURN).abs() < 1e-6);
        assert!((behind_right.chest_turn(facing) + MAX_CHEST_TURN).abs() < 1e-6);
    }

    #[test]
    fn the_arms_are_their_own_again_once_it_has_played_out() {
        let g = gesture(0.12 + FOLLOW_THROUGH + RECOVER);
        assert!(g.finished());
        assert!(g.weight() < 1e-4);
    }

    /// No wind-up throws from the release pose on the click and carries
    /// straight through.
    #[test]
    fn a_throw_with_no_wind_up_starts_at_the_release() {
        let g = ThrowGesture {
            windup: 0.0,
            ..gesture(0.0)
        };
        assert_eq!(g.weight(), 1.0);
        assert!((g.key().hand - RELEASE.hand).magnitude() < 1e-6);
    }
}
