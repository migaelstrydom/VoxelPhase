//! Runtime state for biped animation.
//!
//! All mutable animation state lives here - no state scattered across components.

use nalgebra::{Point3, Vector2, Vector3};

/// High-level locomotion mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LocomotionMode {
    #[default]
    Idle,
    Walking,
    Falling,
}

/// State of a single foot.
#[derive(Debug, Clone)]
pub struct FootState {
    /// Current foot position in world space.
    pub position: Point3<f32>,
    /// World-space position when planted.
    pub planted_position: Point3<f32>,
    /// Ground surface normal at contact point.
    pub normal: Vector3<f32>,

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

/// Complete biped animation state.
///
/// This is the single source of truth for all animation state.
/// No other components should store animation-related state.
#[derive(Debug, Clone)]
pub struct BipedState {
    /// Current locomotion mode.
    pub mode: LocomotionMode,
    /// Previous locomotion mode (for transition detection).
    pub prev_mode: LocomotionMode,
    /// Time spent in current mode.
    pub mode_time: f32,

    /// Stride wheel rotation [0, TAU).
    pub wheel_angle: f32,

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

    /// Whether the character has any ground contact.
    pub is_grounded: bool,
}

impl BipedState {
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
            mode: LocomotionMode::Idle,
            prev_mode: LocomotionMode::Idle,
            mode_time: 0.0,

            wheel_angle: 0.0,

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

    /// Update locomotion mode and track transitions.
    pub fn set_mode(&mut self, new_mode: LocomotionMode, dt: f32) {
        if new_mode != self.mode {
            self.prev_mode = self.mode;
            self.mode = new_mode;
            self.mode_time = 0.0;
        } else {
            self.mode_time += dt;
        }
    }
}
