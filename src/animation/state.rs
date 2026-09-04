//! Runtime state for character animation.
//!
//! All mutable animation state lives here - no state scattered across components.

use nalgebra::{Point3, Vector2, Vector3};

/// State of a single foot.
#[derive(Debug, Clone)]
pub struct FootState {
    /// Current foot position in world space. `y` is the intended
    /// ground-contact height; the rendered capsule sits above it by
    /// `foot_radius` so its bottom tangent is on the ground.
    pub position: Point3<f32>,
    /// World-space position when planted.
    pub planted_position: Point3<f32>,
    /// Ground surface normal at contact point.
    pub normal: Vector3<f32>,
    /// Horizontal forward direction of the foot (toe points here). The
    /// driver mirrors the body's facing into this each frame; per-foot
    /// swing yaw is future polish.
    pub forward: Vector3<f32>,
    /// Foot up-axis used by rendering. Mirrored from the foot placer's
    /// slerped orientation each frame. Matches world Y for a flat stance
    /// on level ground.
    pub up: Vector3<f32>,

    /// Most recent ground contact point from probe (if any).
    pub ground_contact: Option<Point3<f32>>,
    /// Ground normal from probe.
    pub ground_normal: Option<Vector3<f32>>,
}

impl FootState {
    /// Create a new foot state at the given position.
    pub fn new(position: Point3<f32>) -> Self {
        Self {
            position,
            planted_position: position,
            normal: Vector3::y(),
            forward: Vector3::new(0.0, 0.0, 1.0),
            up: Vector3::y(),
            ground_contact: None,
            ground_normal: None,
        }
    }
}

/// State of a single hand (simpler than FootState - no ground contact).
#[derive(Debug, Clone)]
pub struct HandState {
    /// Current hand position in world space.
    pub position: Point3<f32>,
}

impl HandState {
    /// Create a new hand state at the given position.
    pub fn new(position: Point3<f32>) -> Self {
        Self { position }
    }
}

/// Complete character animation state.
///
/// This is the single source of truth for all animation state.
/// No other components should store animation-related state.
#[derive(Debug, Clone)]
pub struct AnimationState {
    /// Stride phase in [0, TAU). Derived from the foot placer each
    /// frame; parameterises arm swing, shoulder twist, head bob.
    pub stride_phase: f32,
    /// Stride activity in [0, 1]. Snaps to 1 whenever either foot is
    /// stepping and decays exponentially when both are planted. Scales
    /// arm swing amplitude, shoulder twist, and head bob so the upper
    /// body tracks real motion rather than the gait FSM — passive
    /// motion (slope slide) no longer flickers between idle and walking
    /// poses as speed oscillates across the idle threshold.
    pub stride_activity: f32,

    /// Left foot state.
    pub left: FootState,
    /// Right foot state.
    pub right: FootState,

    /// Left hand state.
    pub left_hand: HandState,
    /// Right hand state.
    pub right_hand: HandState,

    /// Current shoulder twist angle (radians, positive = left shoulder forward).
    pub shoulder_twist: f32,
    /// Head tilt angles (x = forward/back pitch, y = left/right roll).
    pub head_tilt: Vector2<f32>,
    /// Head vertical bob offset.
    pub head_bob: f32,

    /// Current pelvis position.
    pub pelvis_position: Point3<f32>,
    /// Current facing direction (horizontal, normalized).
    pub facing: Vector3<f32>,

    /// Whether something is holding the character up, taken from the
    /// `Grounding` component — the Support Set's answer, not the probes'.
    /// Animation reads support; it no longer decides it.
    pub is_grounded: bool,
}

impl AnimationState {
    /// Create initial state at the given pelvis position.
    pub fn new(pelvis_position: Point3<f32>, leg_length: f32) -> Self {
        let foot_y = pelvis_position.y - leg_length * 0.85;
        let left_foot = Point3::new(pelvis_position.x + 0.12, foot_y, pelvis_position.z);
        let right_foot = Point3::new(pelvis_position.x - 0.12, foot_y, pelvis_position.z);

        // Hands start at rest position (hanging by sides)
        let hand_y = pelvis_position.y + 0.1; // Roughly at hip height initially
        let left_hand = Point3::new(pelvis_position.x + 0.2, hand_y, pelvis_position.z);
        let right_hand = Point3::new(pelvis_position.x - 0.2, hand_y, pelvis_position.z);

        Self {
            stride_phase: 0.0,
            stride_activity: 0.0,

            left: FootState::new(left_foot),
            right: FootState::new(right_foot),

            left_hand: HandState::new(left_hand),
            right_hand: HandState::new(right_hand),

            shoulder_twist: 0.0,
            head_tilt: Vector2::new(0.0, 0.0),
            head_bob: 0.0,

            pelvis_position,
            facing: Vector3::new(0.0, 0.0, 1.0),

            is_grounded: false,
        }
    }
}
