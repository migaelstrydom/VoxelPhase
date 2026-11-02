use nalgebra::Point3;
use specs::{Component, DenseVecStorage};

/// The hands' sense for edges. A body carrying one finds the edge at its
/// hands each frame (`LedgeSensorSystem`) and hands it to the locomotion
/// state as a request; a body without one never grabs a ledge. What else it
/// finds is published here for the animation.
#[derive(Component, Debug, Clone, Default)]
#[storage(DenseVecStorage)]
pub struct LedgeSensor {
    /// Seconds left before the hands catch again, after letting go.
    pub regrab_lock: f32,
    /// Whether the body was on a ledge last frame.
    pub was_holding: bool,
    /// While hanging: whether a face stands below the edge where the feet
    /// hang, for them to brace on.
    pub wall_below: bool,
    /// In a jump or a fall: the edge the hands meet further along the arc,
    /// if they meet one soon.
    pub ahead: Option<EdgeAhead>,
}

/// An edge the hands will meet along the body's arc.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EdgeAhead {
    /// Where each hand will hold, as the edge stands now.
    pub hands: [Point3<f32>; 2],
    /// How soon, in seconds.
    pub in_seconds: f32,
}
