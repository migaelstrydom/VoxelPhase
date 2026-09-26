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
    /// Ground surface normal under the foot, mirrored from the probe
    /// reading legged locomotion resolved this frame.
    pub normal: Vector3<f32>,
    /// Horizontal forward direction of the foot (toe points here). The
    /// driver mirrors the body's facing into this each frame; per-foot
    /// swing yaw is future polish.
    pub forward: Vector3<f32>,
    /// Foot up-axis used by rendering. Mirrored from the foot placer's
    /// slerped orientation each frame. Matches world Y for a flat stance
    /// on level ground.
    pub up: Vector3<f32>,
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
        }
    }
}

/// A fall into water, from the splash until the swimmer has recovered.
///
/// Arms thrown up as the body goes under, swept down to haul it back up,
/// then treading again: a short choreography the samplers play off `progress`.
#[derive(Debug, Clone, Copy)]
pub struct WaterEntry {
    /// Seconds since the body went in.
    pub elapsed: f32,
    /// How hard it went in, in [0, 1], from the speed it hit the water at.
    pub strength: f32,
}

impl WaterEntry {
    /// How long the recovery takes, in seconds.
    pub const DURATION: f32 = 0.9;
    /// Impact speed at which the reaction is at its fullest, in m/s: about a
    /// two-metre drop.
    const FULL_SPEED: f32 = 5.0;
    /// Impact speed below which there is no reaction at all: stepping in.
    const LEAST_SPEED: f32 = 1.5;

    /// An entry at `impact_speed`, or `None` if that was too gentle to react
    /// to.
    pub fn at(impact_speed: f32) -> Option<Self> {
        let strength = ((impact_speed - Self::LEAST_SPEED)
            / (Self::FULL_SPEED - Self::LEAST_SPEED))
            .clamp(0.0, 1.0);
        (strength > 0.0).then_some(Self {
            elapsed: 0.0,
            strength,
        })
    }

    /// Progress through the recovery, in [0, 1].
    pub fn progress(&self) -> f32 {
        (self.elapsed / Self::DURATION).clamp(0.0, 1.0)
    }

    /// Advance by `dt`; `None` once the recovery is over.
    pub fn advanced(self, dt: f32) -> Option<Self> {
        let next = Self {
            elapsed: self.elapsed + dt,
            ..self
        };
        (next.elapsed < Self::DURATION).then_some(next)
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

    /// How far the physics body lies over from upright toward its facing, in
    /// radians, as measured — not as asked for. The rig is built in this
    /// frame (see `BodyFrame`), so it lies down and stands up with the body.
    pub body_pitch: f32,
    /// The body's velocity relative to whatever carries it.
    pub velocity: Vector3<f32>,
    /// The water's surface over the body, as drawn, if it is in or over any.
    pub water_surface: Option<f32>,
    /// Swim stroke phase in [0, TAU): one full cycle of both arms. Advances
    /// with speed through the water; parameterises arms, kick and roll.
    pub stroke_phase: f32,
    /// How deep the character is wading, in [0, 1]: 0 dry, 1 with the water
    /// at its chest. Eased, so a wave does not jerk the arms.
    pub wade: f32,
    /// A fall into the water still being recovered from.
    pub water_entry: Option<WaterEntry>,
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

            body_pitch: 0.0,
            velocity: Vector3::zeros(),
            water_surface: None,
            stroke_phase: 0.0,
            wade: 0.0,
            water_entry: None,
        }
    }
}
