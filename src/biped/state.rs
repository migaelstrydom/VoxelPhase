//! Runtime state for biped animation.
//!
//! All mutable animation state lives here - no state scattered across components.

use nalgebra::{Point3, Vector3};

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

        Self {
            mode: LocomotionMode::Idle,
            prev_mode: LocomotionMode::Idle,
            mode_time: 0.0,

            wheel_angle: 0.0,

            left: FootState::new(left_foot),
            right: FootState::new(right_foot),

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
