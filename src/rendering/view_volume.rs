use nalgebra::{Matrix4, Vector4};

use crate::rendering::reflection::BoundingSphere;

/// The part of the world the camera can see, as far as deciding whether an
/// object is on screen needs to know: the four side planes of its view.
///
/// ```text
///          left   right
///            \     /
///             \   /        a sphere is in view unless it lies wholly
///              \ /         outside one of the four planes
///              eye
/// ```
///
/// No near or far plane. Something too near to see is about to be seen, and
/// the scene's far plane is beyond anything worth culling at. The side planes
/// alone already exclude everything behind the camera: they meet at the eye,
/// and behind it the half-spaces they bound do not overlap.
///
/// Built from the frame's own matrices, like the shadow pass's fit, so it can
/// never describe a different camera from the one being drawn.
#[derive(Clone, Copy, Debug)]
pub struct ViewVolume {
    /// Each plane as (normal, offset), normals pointing inward and unit
    /// length, so a point's signed distance is `dot(plane, (p, 1))`.
    /// `None` until the first view is set: everything is in it.
    planes: Option<[Vector4<f32>; 4]>,
}

impl Default for ViewVolume {
    fn default() -> Self {
        Self::EVERYTHING
    }
}

impl ViewVolume {
    /// A volume containing everything, for before any camera is known.
    pub const EVERYTHING: Self = Self { planes: None };

    /// The volume seen through `view` and `proj`, widened so that the tangent
    /// of each half-angle is `widen` times the camera's.
    ///
    /// Widening answers "will it be on screen shortly" rather than "is it on
    /// screen now", for work that has to start a frame before its result
    /// shows. The planes come out of the rows of the clip matrix
    /// (Gribb–Hartmann): x ≥ −w is the left plane, x ≤ w the right, and so
    /// on.
    pub fn new(view: &Matrix4<f32>, proj: &Matrix4<f32>, widen: f32) -> Self {
        let mut proj = *proj;
        proj[(0, 0)] /= widen;
        proj[(1, 1)] /= widen;
        let clip = proj * view;
        let row = |i: usize| clip.row(i).transpose();
        let (x, y, w) = (row(0), row(1), row(3));
        let normalised = |plane: Vector4<f32>| plane / plane.xyz().norm();
        Self {
            planes: Some([
                normalised(w + x),
                normalised(w - x),
                normalised(w + y),
                normalised(w - y),
            ]),
        }
    }

    /// Whether any of `sphere` is inside the volume.
    pub fn contains(&self, sphere: &BoundingSphere) -> bool {
        let Some(planes) = &self.planes else {
            return true;
        };
        let centre = sphere.centre.push(1.0);
        planes
            .iter()
            .all(|plane| plane.dot(&centre) >= -sphere.radius)
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use super::*;

    /// A camera at the origin looking down −z with a 90° vertical field of
    /// view and a square aspect.
    fn volume(widen: f32) -> ViewVolume {
        let view = Matrix4::look_at_rh(
            &Point3::origin(),
            &Point3::new(0.0, 0.0, -1.0),
            &Vector3::y(),
        );
        let mut proj = Matrix4::new_perspective(1.0, std::f32::consts::FRAC_PI_2, 0.1, 100.0);
        proj[(1, 1)] *= -1.0;
        ViewVolume::new(&view, &proj, widen)
    }

    fn sphere(x: f32, y: f32, z: f32, radius: f32) -> BoundingSphere {
        BoundingSphere {
            centre: Vector3::new(x, y, z),
            radius,
        }
    }

    #[test]
    fn what_is_ahead_is_in_view_and_what_is_behind_is_not() {
        let volume = volume(1.0);
        assert!(volume.contains(&sphere(0.0, 0.0, -10.0, 0.5)));
        assert!(!volume.contains(&sphere(0.0, 0.0, 10.0, 0.5)));
    }

    #[test]
    fn a_sphere_straddling_the_edge_is_in_view() {
        // At 45° half-angle the right edge at distance 10 is x = 10.
        let volume = volume(1.0);
        assert!(volume.contains(&sphere(10.5, 0.0, -10.0, 1.0)));
        assert!(!volume.contains(&sphere(12.0, 0.0, -10.0, 1.0)));
    }

    #[test]
    fn widening_takes_in_what_is_just_off_screen() {
        let off_screen = sphere(12.0, 0.0, -10.0, 0.5);
        assert!(!volume(1.0).contains(&off_screen));
        assert!(volume(1.3).contains(&off_screen));
    }

    #[test]
    fn before_any_camera_everything_is_in_view() {
        assert!(ViewVolume::EVERYTHING.contains(&sphere(0.0, 0.0, 1e6, 0.1)));
    }
}
