//! The shape of the view frustum, and the bounding sphere of a slice of it.
//!
//! The shadow map is fitted to the part of the world the camera can actually
//! see. Doing that with a *sphere* rather than a tight box is deliberate: a
//! sphere's radius depends only on the frustum's shape and the distance it is
//! cut at, never on where the camera is pointed.
//!
//! ```text
//!        near                        distance
//!         |                             |
//!   eye --+-----------------------------+--->  forward
//!         |         .-''''''-.          |
//!          \     .''          ''.       |
//!           \   /       o        \      |     o = centre, forward_offset ahead
//!            \  \                /      |     radius = constant for a given fov
//!             \  ''.          .''       |
//!              \    '-......-'          |
//! ```
//!
//! That constancy is what makes the texel snapping in
//! [`ShadowFraming::light_view_proj`](super::volume::ShadowFraming::light_view_proj)
//! work. A box fitted tightly to the frustum's corners would shrink and grow as
//! the camera turned, changing the size of the snap grid every frame and making
//! every shadow edge crawl — the exact artifact snapping exists to prevent.

use nalgebra::Matrix4;

/// The shape of a perspective view frustum, without reference to where the
/// camera holding it is or which way it faces.
#[derive(Clone, Copy, Debug)]
pub struct ViewFrustum {
    /// `tan(fov_x / 2)`: half-width of the view cone at unit distance.
    pub tan_half_x: f32,

    /// `tan(fov_y / 2)`: half-height of the view cone at unit distance.
    pub tan_half_y: f32,

    /// Distance to the near clip plane. Contributes little to the fit at any
    /// realistic shadow distance, but it is what makes the slice a frustum
    /// rather than a cone.
    pub near: f32,
}

impl Default for ViewFrustum {
    /// The engine's usual camera: 45° vertical fov at 16:9. Used only as a
    /// stand-in before the first real frame has been seen.
    fn default() -> Self {
        let tan_half_y = (45.0f32.to_radians() * 0.5).tan();
        Self {
            tan_half_x: tan_half_y * 16.0 / 9.0,
            tan_half_y,
            near: 0.1,
        }
    }
}

impl ViewFrustum {
    /// Recover the frustum's shape from a perspective projection matrix.
    ///
    /// Assumes the convention every camera in the engine produces: nalgebra's
    /// `new_perspective` with `[(1, 1)]` negated for Vulkan's downward clip-space
    /// Y, hence the absolute value.
    pub fn from_projection(proj: &Matrix4<f32>) -> Self {
        let tan_half_x = safe_reciprocal(proj[(0, 0)].abs());
        let tan_half_y = safe_reciprocal(proj[(1, 1)].abs());

        // For nalgebra's OpenGL-style depth range,
        // `[(2, 2)] = (far + near) / (near - far)` and
        // `[(2, 3)] = 2 * far * near / (near - far)`, whose ratio is the near
        // plane with `far` cancelled out.
        let denominator = proj[(2, 2)] - 1.0;
        let near = if denominator.abs() > 1e-6 {
            (proj[(2, 3)] / denominator).abs()
        } else {
            Self::default().near
        };

        Self {
            tan_half_x,
            tan_half_y,
            near,
        }
    }

    /// The smallest sphere containing everything between the near plane and
    /// `distance` along the view axis.
    pub fn slice_bounding_sphere(&self, distance: f32) -> FrustumSlice {
        let near = self.near.max(1e-3);
        let far = distance.max(near + 1e-3);

        // Squared radius of the view cone at unit distance, which is all the
        // corners contribute to the fit.
        let spread = self.tan_half_x * self.tan_half_x + self.tan_half_y * self.tan_half_y;

        // Equating the distance to a near corner with the distance to a far
        // corner puts the centre here. A wide enough cone pushes it past the
        // far plane, at which point the far circle alone bounds the slice.
        let centre = 0.5 * (far + near) * (1.0 + spread);
        if centre >= far {
            return FrustumSlice {
                forward_offset: far,
                radius: far * spread.sqrt(),
            };
        }

        let radius = (spread * near * near + (near - centre) * (near - centre)).sqrt();
        FrustumSlice {
            forward_offset: centre,
            radius,
        }
    }
}

/// A bounding sphere around part of the view frustum, expressed relative to the
/// camera so that only `forward_offset` has to be re-applied as it moves.
#[derive(Clone, Copy, Debug)]
pub struct FrustumSlice {
    /// How far along the camera's forward axis the sphere's centre sits.
    pub forward_offset: f32,

    /// The sphere's radius. A function of the frustum's shape and the slice
    /// distance only — constant while those are, however the camera turns.
    pub radius: f32,
}

fn safe_reciprocal(value: f32) -> f32 {
    if value > 1e-6 {
        1.0 / value
    } else {
        1.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn projection(fov_y_degrees: f32, aspect: f32, near: f32, far: f32) -> Matrix4<f32> {
        let mut proj = Matrix4::new_perspective(aspect, fov_y_degrees.to_radians(), near, far);
        proj[(1, 1)] *= -1.0;
        proj
    }

    #[test]
    fn recovers_the_shape_it_was_built_from() {
        let fov_y = 45.0f32;
        let aspect = 16.0 / 9.0;
        let frustum = ViewFrustum::from_projection(&projection(fov_y, aspect, 0.1, 500.0));

        let expected_y = (fov_y.to_radians() * 0.5).tan();
        assert!((frustum.tan_half_y - expected_y).abs() < 1e-5);
        assert!((frustum.tan_half_x - expected_y * aspect).abs() < 1e-5);
        assert!((frustum.near - 0.1).abs() < 1e-4, "near = {}", frustum.near);
    }

    #[test]
    fn the_sphere_contains_every_corner_of_the_slice() {
        let frustum = ViewFrustum::from_projection(&projection(60.0, 4.0 / 3.0, 0.2, 500.0));
        let distance = 45.0;
        let slice = frustum.slice_bounding_sphere(distance);

        for depth in [frustum.near, distance] {
            let corner_x = frustum.tan_half_x * depth;
            let corner_y = frustum.tan_half_y * depth;
            let along = depth - slice.forward_offset;
            let d = (corner_x * corner_x + corner_y * corner_y + along * along).sqrt();
            assert!(
                d <= slice.radius + 1e-3,
                "corner at depth {depth} is {d} from the centre, radius {}",
                slice.radius
            );
        }
    }

    #[test]
    fn the_fit_is_tight_at_both_ends() {
        // Both circles should touch the sphere: if one does not, a smaller
        // sphere would have done and texels are being spent on nothing.
        let frustum = ViewFrustum::from_projection(&projection(45.0, 16.0 / 9.0, 0.1, 500.0));
        let slice = frustum.slice_bounding_sphere(40.0);

        let distance_to_corner = |depth: f32| {
            let along = depth - slice.forward_offset;
            (frustum.tan_half_x * frustum.tan_half_x * depth * depth
                + frustum.tan_half_y * frustum.tan_half_y * depth * depth
                + along * along)
                .sqrt()
        };

        assert!((distance_to_corner(frustum.near) - slice.radius).abs() < 1e-3);
        assert!((distance_to_corner(40.0) - slice.radius).abs() < 1e-3);
    }

    #[test]
    fn a_very_wide_cone_falls_back_to_the_far_circle() {
        // Past roughly a 90° diagonal the equal-distance centre lands beyond
        // the far plane, and clamping it there is what keeps the fit valid.
        let frustum = ViewFrustum::from_projection(&projection(140.0, 2.0, 0.1, 500.0));
        let slice = frustum.slice_bounding_sphere(30.0);

        assert!((slice.forward_offset - 30.0).abs() < 1e-4);
        let corner = (frustum.tan_half_x * frustum.tan_half_x
            + frustum.tan_half_y * frustum.tan_half_y)
            .sqrt()
            * 30.0;
        assert!(slice.radius >= corner - 1e-3);
    }
}
