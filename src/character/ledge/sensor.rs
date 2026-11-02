use specs::{Component, DenseVecStorage};

use super::config::LedgeConfig;
use super::edge::Edge;

/// The hands' sense for edges. A body carrying one finds the edge at its
/// hands each frame (`LedgeSensorSystem`); a body without one never grabs a
/// ledge.
#[derive(Component, Debug, Clone, Default)]
#[storage(DenseVecStorage)]
pub struct LedgeSensor {
    /// What counts as an edge, and how one is held.
    pub config: LedgeConfig,
    /// The edge at the hands this frame, if there is one.
    pub edge: Option<Edge>,
}
