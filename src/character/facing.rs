//! Which way a character is pointing.

use nalgebra::Vector3;

/// Convert a Y-axis rotation angle to a facing direction vector (unit, XZ plane).
///
/// Shared by everything that needs the direction a character would throw,
/// reach or step in, so the convention is stated once.
pub fn facing_from_rotation(rotation_y: f32) -> Vector3<f32> {
    Vector3::new(rotation_y.sin(), 0.0, rotation_y.cos())
}
