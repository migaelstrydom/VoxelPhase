use specs::{Component, DenseVecStorage};

/// The hands' sense for edges. A body carrying one finds the edge at its
/// hands each frame (`LedgeSensorSystem`) and hands it to the locomotion
/// state as a request; a body without one never grabs a ledge.
#[derive(Component, Debug, Clone, Default)]
#[storage(DenseVecStorage)]
pub struct LedgeSensor {
    /// Seconds left before the hands catch again, after letting go.
    pub regrab_lock: f32,
    /// Whether the body was on a ledge last frame.
    pub was_holding: bool,
}
