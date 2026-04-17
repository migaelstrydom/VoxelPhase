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

/// Target positions for both feet.
#[derive(Debug, Clone)]
pub struct FeetPose {
    pub left: Point3<f32>,
    pub right: Point3<f32>,
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
}

impl PoseFragment {
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

fn lerp_point(a: &Point3<f32>, b: &Point3<f32>, t: f32) -> Point3<f32> {
    a + (b - a) * t
}

fn lerp_feet(a: &FeetPose, b: &FeetPose, t: f32) -> FeetPose {
    FeetPose {
        left: lerp_point(&a.left, &b.left, t),
        right: lerp_point(&a.right, &b.right, t),
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
#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct Crossfade<P: BlendPolicy = Linear> {
    pub from: PoseFragment,
    pub to_duration: f32,
    pub elapsed: f32,
    pub policy: P,
}

#[allow(dead_code)]
impl<P: BlendPolicy> Crossfade<P> {
    /// Whether the crossfade is still in progress.
    pub fn is_active(&self) -> bool {
        self.elapsed < self.to_duration
    }

    /// Blend `self.from` toward `current` using the policy weight.
    pub fn sample(&self, current: &PoseFragment) -> PoseFragment {
        let t = self.policy.weight(self.elapsed, self.to_duration);
        self.from.lerp(current, t)
    }

    /// Advance the crossfade timer.
    pub fn tick(&mut self, dt: f32) {
        self.elapsed += dt;
    }
}
