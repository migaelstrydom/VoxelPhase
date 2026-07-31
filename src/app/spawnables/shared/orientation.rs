//! Horizontal orientation for spawnables that have a meaningful axis.

use nalgebra::{Point3, UnitQuaternion, Vector3};

/// A yaw rotation about `+Y`, applied to a spawnable's own axes.
///
/// A spawnable with a meaningful horizontal axis — a wall, a beam, a temple —
/// builds its parts as offsets from an origin in its own coordinates. Turning
/// it is then two operations and only two: rotate each part's offset, and give
/// each body the same rotation. `Yaw` is those two operations, which is what
/// lets a spawnable gain orientation without gaining a coordinate system.
///
/// ```text
///        yaw 0                     yaw 90
///     ┌──────────┐                    ┌───┐
///     │  ▶ +X    │       ────▶         │   │
///     └──────────┘                     │ ▼ │  local +X maps onto world −Z
///                                      └───┘
/// ```
///
/// Units are **degrees**, matching `AnchorDef::yaw` and `Placement`'s yaw. A
/// segment placed at a quarter turn adds its own yaw to every object it holds,
/// in `LevelObject::place_in`, so an author writes an object's orientation
/// relative to the area it sits in and never in world terms.
#[derive(Clone, Copy, Default)]
pub struct Yaw {
    degrees: f32,
}

impl Yaw {
    pub fn degrees(degrees: f32) -> Self {
        Self { degrees }
    }

    /// True when this rotation would change nothing, so callers can keep the
    /// identity path exactly as it was.
    pub fn is_identity(&self) -> bool {
        self.degrees.abs() < 1e-6
    }

    /// As a quaternion, for a body's or a collider's rotation.
    pub fn rotation(&self) -> UnitQuaternion<f32> {
        if self.is_identity() {
            UnitQuaternion::identity()
        } else {
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), self.degrees.to_radians())
        }
    }

    /// Rotate an offset authored in the spawnable's own axes.
    pub fn turn(&self, local: Vector3<f32>) -> Vector3<f32> {
        if self.is_identity() {
            local
        } else {
            self.rotation() * local
        }
    }

    /// World position of a part authored at `local`, relative to `origin`.
    pub fn place(&self, origin: (f32, f32, f32), local: Vector3<f32>) -> Point3<f32> {
        Point3::new(origin.0, origin.1, origin.2) + self.turn(local)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A quarter turn about `+Y` maps local `+X` onto world `−Z`, which is the
    /// same convention `SegmentFrame` uses. If these two ever disagree, an
    /// object in a rotated segment faces backwards.
    #[test]
    fn a_quarter_turn_maps_plus_x_onto_minus_z() {
        let turned = Yaw::degrees(90.0).turn(Vector3::x());
        assert!(
            (turned - Vector3::new(0.0, 0.0, -1.0)).norm() < 1e-5,
            "got {turned:?}"
        );
    }

    #[test]
    fn placement_rotates_about_the_origin() {
        let p = Yaw::degrees(90.0).place((10.0, 2.0, 10.0), Vector3::new(4.0, 1.0, 0.0));
        assert!((p - Point3::new(10.0, 3.0, 6.0)).norm() < 1e-5, "got {p:?}");
    }
}
