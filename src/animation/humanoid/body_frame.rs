//! The torso's frame, laid over by the body's pitch.

use nalgebra::Vector3;

use crate::animation::rig::right_vector;

/// Directions a humanoid's upper body and legs are built along.
///
/// Upright, `up` is world up and `front` the facing. A swimmer lies over
/// about `right`, and everything a pose says in terms of "up the spine" and
/// "out of the chest" goes with it:
///
/// ```text
///   pitch 0:   up ▲  front ▶            pitch 90°:   up ▶  (toward the head)
///                                                    front ▼ (the belly)
/// ```
///
/// Hips are laid along `right` whatever the pitch, so the lateral axis never
/// changes; only `up` and `front` turn.
#[derive(Debug, Clone, Copy)]
pub struct BodyFrame {
    /// Horizontal facing: where the character is heading.
    pub heading: Vector3<f32>,
    /// Out to the character's right.
    pub right: Vector3<f32>,
    /// Up the spine.
    pub up: Vector3<f32>,
    /// Out of the chest.
    pub front: Vector3<f32>,
}

impl BodyFrame {
    /// The frame of a body heading along `facing` and laid over by `pitch`
    /// radians toward it.
    pub fn new(facing: Vector3<f32>, pitch: f32) -> Self {
        let (sin, cos) = pitch.sin_cos();
        Self {
            heading: facing,
            right: right_vector(facing),
            up: Vector3::y() * cos + facing * sin,
            front: facing * cos - Vector3::y() * sin,
        }
    }

    /// The same frame leaned a further `pitch` radians forward — a torso
    /// hunched over hips that are not.
    pub fn leaned(&self, pitch: f32) -> Self {
        let (sin, cos) = pitch.sin_cos();
        Self {
            up: self.up * cos + self.front * sin,
            front: self.front * cos - self.up * sin,
            ..*self
        }
    }

    pub fn left(&self) -> Vector3<f32> {
        -self.right
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    #[test]
    fn upright_is_the_world() {
        let f = BodyFrame::new(Vector3::z(), 0.0);
        assert!((f.up - Vector3::y()).magnitude() < 1e-6);
        assert!((f.front - Vector3::z()).magnitude() < 1e-6);
    }

    #[test]
    fn a_swimmer_lies_face_down_along_its_heading() {
        let f = BodyFrame::new(Vector3::x(), FRAC_PI_2);
        assert!((f.up - Vector3::x()).magnitude() < 1e-6, "head leads");
        assert!((f.front + Vector3::y()).magnitude() < 1e-6, "belly down");
        assert!(f.right.dot(&f.up).abs() < 1e-6);
    }

    #[test]
    fn leaning_composes_with_pitch() {
        let a = BodyFrame::new(Vector3::z(), 0.3).leaned(0.4);
        let b = BodyFrame::new(Vector3::z(), 0.7);
        assert!((a.up - b.up).magnitude() < 1e-5);
        assert!((a.front - b.front).magnitude() < 1e-5);
    }
}
