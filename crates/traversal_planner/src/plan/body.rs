use serde::{Deserialize, Serialize};

/// The body's state in path coordinates. The body is on the path, so it has no
/// `d`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct BodyState {
    /// Distance along the path of the capsule's bottom-sphere centre.
    pub s: f32,
    /// Height of the capsule's bottom-sphere centre.
    pub z: f32,
    /// Speed along the path, in m/s.
    pub speed: f32,
    /// Vertical speed, in m/s.
    pub climb: f32,
}
