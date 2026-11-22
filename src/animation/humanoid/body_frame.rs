//! The torso's frame, laid over by the body's pitch.

use nalgebra::{UnitQuaternion, Vector3};

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

    /// The frame of a body turned by `rotation`, however it lies: its own
    /// up (local +Y), front (+Z) and right (−X), heading the way its front
    /// points across the ground or, lying face up or down, its head. A body
    /// held at a heading and pitch has the frame [`new`](Self::new) gives.
    pub fn of(rotation: &UnitQuaternion<f32>) -> Self {
        let up = rotation * Vector3::y();
        let front = rotation * Vector3::z();
        let across = |v: Vector3<f32>| Vector3::new(v.x, 0.0, v.z).try_normalize(0.3);
        Self {
            heading: across(front)
                .or_else(|| across(up))
                .unwrap_or_else(Vector3::z),
            right: rotation * -Vector3::x(),
            up,
            front,
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

    /// A body held at a heading and pitch is drawn as `new` draws it; one
    /// knocked onto its side lies with its right side up.
    #[test]
    fn a_body_is_drawn_as_it_lies() {
        let held = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.6)
            * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.4);
        let a = BodyFrame::of(&held);
        let b = BodyFrame::new(Vector3::new(0.6f32.sin(), 0.0, 0.6f32.cos()), 0.4);
        for (x, y) in [
            (a.up, b.up),
            (a.front, b.front),
            (a.right, b.right),
            (a.heading, b.heading),
        ] {
            assert!((x - y).magnitude() < 1e-5, "{x} against {y}");
        }
        let on_its_side = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -FRAC_PI_2);
        let lying = BodyFrame::of(&on_its_side);
        assert!(
            (lying.right - Vector3::y()).magnitude() < 1e-5,
            "{}",
            lying.right
        );
        assert!((lying.heading - Vector3::z()).magnitude() < 1e-5);
    }

    #[test]
    fn leaning_composes_with_pitch() {
        let a = BodyFrame::new(Vector3::z(), 0.3).leaned(0.4);
        let b = BodyFrame::new(Vector3::z(), 0.7);
        assert!((a.up - b.up).magnitude() < 1e-5);
        assert!((a.front - b.front).magnitude() < 1e-5);
    }
}
