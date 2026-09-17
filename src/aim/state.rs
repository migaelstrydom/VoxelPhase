//! The frame's answer to "where would it land".

use super::source::AimKind;
use super::trajectory::Impact;

/// A predicted landing, and what throw it belongs to.
#[derive(Debug, Clone, Copy)]
pub struct AimSolution {
    /// Which throw was predicted.
    pub kind: AimKind,
    /// Where it would come down.
    pub impact: Impact,
}

/// Where the player's throw would land this frame, or `None` when they are not
/// aiming or the throw reaches nothing.
///
/// An ECS resource, rewritten every frame by [`super::AimPredictionSystem`].
/// It carries a world-space point and nothing about how it is displayed, so
/// the same solution can drive a cursor, a debug overlay or a trajectory arc
/// without any of them knowing about each other.
#[derive(Debug, Default, Clone, Copy)]
pub struct AimState {
    pub solution: Option<AimSolution>,
}

impl AimState {
    /// True when there is a landing to draw.
    pub fn is_aiming(&self) -> bool {
        self.solution.is_some()
    }
}
