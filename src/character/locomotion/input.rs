use nalgebra::{Point3, Vector3};

use super::requests::{PlannedLanding, PlannedTakeoff};
use crate::character::config::LocomotionConfig;
use crate::character::immersion::Immersion;
use crate::character::ledge::SensedEdge;

/// Per-frame inputs to `LocomotionState::tick` and `enter`.
pub struct LocomotionInput<'a> {
    pub dt: f32,
    pub facts: Facts,
    pub requests: Requests,
    pub config: &'a LocomotionConfig,
}

/// What is true of the body this frame. `y` is up.
#[derive(Debug, Clone, Copy)]
pub struct Facts {
    /// Whether the body is supported, its contact chatter forgiven
    /// (`Conditioning::support`).
    pub grounded: bool,
    /// The water at the body.
    pub immersion: Immersion,
    /// The body's centre.
    pub position: Point3<f32>,
    /// The body's velocity, in m/s.
    pub velocity: Vector3<f32>,
    /// Seconds since the contacts last held the body up, forgiven or not;
    /// zero in contact.
    pub since_contact: f32,
}

impl Facts {
    /// Speed across the ground, in m/s.
    pub fn horizontal_speed(&self) -> f32 {
        self.velocity.x.hypot(self.velocity.z)
    }

    /// Whether the body is moving up.
    pub fn rising(&self) -> bool {
        self.velocity.y > 0.0
    }
}

/// What is asked of the state this frame, by the player or a brain through
/// the conditioning stage, and by the traversal plan.
#[derive(Debug, Clone, Copy, Default)]
pub struct Requests {
    /// Movement intent direction (planar, at most unit length).
    pub move_dir: Vector3<f32>,
    /// A jump press, buffered.
    pub jump: bool,
    /// Whether the jump key is held this frame.
    pub jump_held: bool,
    /// Crouch was pressed within the buffer window: a jump now is a long
    /// jump.
    pub long_jump_armed: bool,
    /// A takeoff the plan asks for this frame. A jump pressed the same frame
    /// wins.
    pub takeoff: Option<PlannedTakeoff>,
    /// Where the plan now lands a Hop under way.
    pub landing: Option<PlannedLanding>,
    /// The edge at the hands: one they may catch, or the one they hold,
    /// found again. `None` while holding is an edge lost.
    pub edge: Option<SensedEdge>,
    /// Crouch pressed this frame: a drop, hanging.
    pub drop: bool,
}
