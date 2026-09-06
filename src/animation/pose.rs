//! Pose fragments and blend primitives.
//!
//! A `PoseFragment` is a sparse, partial pose: each channel is `Some` only if
//! the emitting state wants to drive it. Composition is overlay semantics —
//! channels set by a higher-priority fragment win over lower-priority ones.
//!
//! `Crossfade` + `BlendPolicy` provide a small, open/closed blending primitive
//! so non-linear curves or per-channel policies can be added later without
//! touching the driver.

use nalgebra::{Point3, Vector2, Vector3};

/// Whether a pose's feet belong to the world or to the body.
///
/// A blend holds the pose a state was in while the next one fades up, and the
/// body keeps moving underneath it. What should happen to the held feet
/// depends entirely on where the state put them:
///
/// ```text
///   Hips: feet hang off the pelvis (airborne, launching)
///         body moves -> feet move with it, or the legs tear off
///
///   World: feet were placed on the ground (standing, walking, landing)
///         body moves -> feet stay, which is what standing on a floor means
/// ```
///
/// Translating the second kind is how a *planted* foot gets dragged across the
/// floor for the length of a blend: the further the body travels during it, the
/// further the foot slides.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FootAnchor {
    /// Carried by the pelvis; translate with it.
    Hips,
    /// Placed on the ground; leave where it is.
    World,
}

/// Target positions for both feet.
#[derive(Debug, Clone)]
pub struct FeetPose {
    pub left: Point3<f32>,
    pub right: Point3<f32>,
    /// What the positions are attached to. See [`FootAnchor`].
    pub anchor: FootAnchor,
}

/// Target positions for both hands.
#[derive(Debug, Clone)]
pub struct HandsPose {
    pub left: Point3<f32>,
    pub right: Point3<f32>,
}

/// A sparse pose contribution from a single state.
///
/// `None` on a channel means "this state does not drive that channel";
/// composition and blending leave such channels to whichever other fragment
/// does drive them.
#[derive(Debug, Clone, Default)]
pub struct PoseFragment {
    pub feet: Option<FeetPose>,
    pub hands: Option<HandsPose>,
    pub pelvis_offset: Option<Vector3<f32>>,
    pub shoulder_twist: Option<f32>,
    pub head_tilt: Option<Vector2<f32>>,
    pub head_bob: Option<f32>,
    /// Forward lean of the torso around the lateral axis, in radians.
    /// Positive = leaning forward (crouched). The skeleton rotates the
    /// chest/head mesh by this angle; `UpperState` reads the same value
    /// so shoulders (and thus hand targets) tilt consistently.
    pub torso_pitch: Option<f32>,
}

impl PoseFragment {
    /// Return a copy with body-carried spatial channels translated by `delta`.
    /// Hands ride the shoulders and always move; feet move only when their
    /// [`FootAnchor`] says they hang off the hips. `pelvis_offset` is already
    /// an offset-from-pelvis and scalar channels are frame-invariant.
    pub fn translated(&self, delta: Vector3<f32>) -> Self {
        Self {
            feet: self.feet.as_ref().map(|f| match f.anchor {
                FootAnchor::World => f.clone(),
                FootAnchor::Hips => FeetPose {
                    left: f.left + delta,
                    right: f.right + delta,
                    anchor: f.anchor,
                },
            }),
            hands: self.hands.as_ref().map(|h| HandsPose {
                left: h.left + delta,
                right: h.right + delta,
            }),
            pelvis_offset: self.pelvis_offset,
            shoulder_twist: self.shoulder_twist,
            head_tilt: self.head_tilt,
            head_bob: self.head_bob,
            torso_pitch: self.torso_pitch,
        }
    }

    /// Return a copy with world-anchored feet pulled in to within `reach` of
    /// `pelvis`, along the line to it. Hip-carried feet are already within
    /// reach by construction, and are left alone.
    ///
    /// This is what ends a push-off. The alternative is asking for a foot the
    /// leg cannot get to, which the skeleton then clamps on its own terms —
    /// dragging the foot in along the floor rather than lifting it off.
    pub fn with_feet_within(&self, reach: f32, pelvis: Point3<f32>) -> Self {
        let Some(feet) = &self.feet else {
            return self.clone();
        };
        if feet.anchor != FootAnchor::World {
            return self.clone();
        }
        Self {
            feet: Some(FeetPose {
                left: pull_within(feet.left, pelvis, reach),
                right: pull_within(feet.right, pelvis, reach),
                anchor: feet.anchor,
            }),
            ..self.clone()
        }
    }

    /// Overlay `overlay` on top of `self`. For each channel, `overlay`
    /// wins if it sets it; otherwise `self` passes through.
    pub fn compose(&self, overlay: &Self) -> Self {
        Self {
            feet: overlay.feet.clone().or_else(|| self.feet.clone()),
            hands: overlay.hands.clone().or_else(|| self.hands.clone()),
            pelvis_offset: overlay.pelvis_offset.or(self.pelvis_offset),
            shoulder_twist: overlay.shoulder_twist.or(self.shoulder_twist),
            head_tilt: overlay.head_tilt.or(self.head_tilt),
            head_bob: overlay.head_bob.or(self.head_bob),
            torso_pitch: overlay.torso_pitch.or(self.torso_pitch),
        }
    }

    /// Per-channel blend toward `other` by `t` in `[0, 1]`.
    ///
    /// Both sides `Some` → lerp. Only one side `Some` → that side unchanged
    /// (the channel is still driven). Both `None` → `None`.
    pub fn lerp(&self, other: &Self, t: f32) -> Self {
        Self {
            feet: lerp_option(&self.feet, &other.feet, t, lerp_feet),
            hands: lerp_option(&self.hands, &other.hands, t, lerp_hands),
            pelvis_offset: lerp_option(&self.pelvis_offset, &other.pelvis_offset, t, |a, b, t| {
                a + (b - a) * t
            }),
            shoulder_twist: lerp_option(
                &self.shoulder_twist,
                &other.shoulder_twist,
                t,
                |a, b, t| a + (b - a) * t,
            ),
            head_tilt: lerp_option(&self.head_tilt, &other.head_tilt, t, |a, b, t| {
                a + (b - a) * t
            }),
            head_bob: lerp_option(&self.head_bob, &other.head_bob, t, |a, b, t| {
                a + (b - a) * t
            }),
            torso_pitch: lerp_option(&self.torso_pitch, &other.torso_pitch, t, |a, b, t| {
                a + (b - a) * t
            }),
        }
    }
}

fn lerp_option<T: Clone, F: Fn(&T, &T, f32) -> T>(
    a: &Option<T>,
    b: &Option<T>,
    t: f32,
    f: F,
) -> Option<T> {
    match (a, b) {
        (Some(a), Some(b)) => Some(f(a, b, t)),
        (Some(a), None) => Some(a.clone()),
        (None, Some(b)) => Some(b.clone()),
        (None, None) => None,
    }
}

/// `point` moved along the line to `anchor` until it is no further than
/// `reach` away. Points already inside are returned unchanged.
fn pull_within(point: Point3<f32>, anchor: Point3<f32>, reach: f32) -> Point3<f32> {
    let offset = point - anchor;
    let distance = offset.magnitude();
    if distance <= reach || distance < 1e-6 {
        return point;
    }
    anchor + offset * (reach / distance)
}

fn lerp_point(a: &Point3<f32>, b: &Point3<f32>, t: f32) -> Point3<f32> {
    a + (b - a) * t
}

/// Positions are world-space on both sides, so they lerp directly — except
/// when the ground has just taken the feet over.
///
/// Lerping from hip-hung feet to placed ones walks the foot from under the
/// pelvis to its plant in a straight line at floor level, over the whole
/// blend, while the body keeps moving: a landing at speed draws a foot
/// skating a quarter of a metre into position. There is nothing to
/// interpolate there. The placer knows where the foot goes the moment the
/// body is grounded, and the correction is worth one frame of pop instead of
/// a tenth of a second of skate.
///
/// The reverse (ground to air) still blends: feet leaving the floor are
/// leaving it gradually, and that is a lift, not a skate.
///
/// The result takes the target's anchor: the blend is turning `a` into `b`,
/// so if it is itself snapshotted for a further blend it should behave as
/// what it is becoming.
fn lerp_feet(a: &FeetPose, b: &FeetPose, t: f32) -> FeetPose {
    if a.anchor == FootAnchor::Hips && b.anchor == FootAnchor::World {
        return b.clone();
    }
    FeetPose {
        left: lerp_point(&a.left, &b.left, t),
        right: lerp_point(&a.right, &b.right, t),
        anchor: b.anchor,
    }
}

fn lerp_hands(a: &HandsPose, b: &HandsPose, t: f32) -> HandsPose {
    HandsPose {
        left: lerp_point(&a.left, &b.left, t),
        right: lerp_point(&a.right, &b.right, t),
    }
}

/// Kind of cycle a state exposes for upper-body synchronisation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CycleKind {
    Stride,
    Stroke,
}

/// Generic cycle handshake between lower-body and upper-body FSMs.
#[derive(Debug, Clone, Copy)]
pub struct Cycle {
    pub phase: f32,
    pub kind: CycleKind,
}

/// Maps elapsed blend time to a weight in `[0, 1]`.
pub trait BlendPolicy {
    fn weight(&self, elapsed: f32, duration: f32) -> f32;
}

/// Linear blend policy. The default.
#[derive(Debug, Clone, Copy, Default)]
pub struct Linear;

impl BlendPolicy for Linear {
    fn weight(&self, elapsed: f32, duration: f32) -> f32 {
        if duration <= 0.0 {
            return 1.0;
        }
        (elapsed / duration).clamp(0.0, 1.0)
    }
}

/// A running crossfade from a snapshotted `from` fragment toward whatever
/// the FSM is currently emitting.
///
/// The snapshot is anchored to the pelvis position at capture time. When
/// sampled, the `from` fragment's body-carried channels are translated by
/// `current_pelvis - from_pelvis` so hands, and feet that hang off the hips,
/// ride along with body motion during the blend (e.g. a rising pelvis during a
/// jump). Feet the state placed on the ground stay where they were put — see
/// [`FootAnchor`].
#[derive(Debug, Clone)]
pub struct Crossfade<P: BlendPolicy = Linear> {
    pub from: PoseFragment,
    pub from_pelvis: Point3<f32>,
    /// How far a held world-anchored foot may sit from the pelvis before it
    /// has to come along. A foot left on the floor while the body leaves it
    /// is a push-off, and a push-off ends when the leg runs out.
    pub reach: f32,
    pub to_duration: f32,
    pub elapsed: f32,
    pub policy: P,
}

impl<P: BlendPolicy> Crossfade<P> {
    /// Whether the crossfade is still in progress.
    pub fn is_active(&self) -> bool {
        self.elapsed < self.to_duration
    }

    /// Blend `self.from` (translated into the current pelvis frame) toward
    /// `current` using the policy weight.
    pub fn sample(&self, current: &PoseFragment, current_pelvis: Point3<f32>) -> PoseFragment {
        let t = self.policy.weight(self.elapsed, self.to_duration);
        let delta = current_pelvis - self.from_pelvis;
        self.from
            .translated(delta)
            .with_feet_within(self.reach, current_pelvis)
            .lerp(current, t)
    }

    /// Advance the crossfade timer.
    pub fn tick(&mut self, dt: f32) {
        self.elapsed += dt;
    }
}
