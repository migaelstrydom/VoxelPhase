//! The piper's own frame: its origin is the middle of the ball it collides
//! as, +Z the way its head faces, and the ground a radius below.

/// Radius of the ball it collides as, m.
pub(super) const RADIUS: f32 = 0.42;
/// How high its tallest drone's mouth stands above its middle, m: where a
/// bomb leaves it.
pub(super) const MUZZLE: f32 = 1.25;
