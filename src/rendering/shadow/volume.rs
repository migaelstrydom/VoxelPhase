//! The slab of world the sun shadow map covers, and the matrix that maps it.
//!
//! A directional light has no position, so a shadow map for one is an
//! orthographic box aimed along the light and parked somewhere useful. With a
//! single map (no cascades) "somewhere useful" is the part of the world the
//! camera can see: the box is fitted to the bounding sphere of the view frustum
//! out to [`ShadowVolume::shadow_distance`], so no texels are spent behind the
//! viewer or outside the view cone.
//!
//! ```text
//!            sun direction
//!                  \
//!                   \   +-----------------+  <- back plane, depth 0
//!                    \  |                 |
//!   camera --------> centre (snapped)     |     radius on each side
//!                       |                 |
//!                       +-----------------+  <- front plane, depth = depth_extent
//! ```
//!
//! The split between the two types here matters:
//!
//! - [`ShadowVolume`] is authored configuration. It says how far shadows should
//!   reach and how they should be biased, and knows nothing about any camera.
//! - [`ShadowFraming`] is that configuration resolved against a particular view
//!   frustum. It owns the fitted radius, and therefore everything derived from
//!   it — texel size, normal offset, and the light matrix itself.
//!
//! Radius is a property of the framing rather than of the volume because it is
//! *derived*: fixing it by hand meant guessing at the camera's field of view.

use nalgebra::{Matrix4, Point3, Vector3};

use crate::rendering::shadow::frustum::ViewFrustum;

/// How the sun's shadow map is framed and biased. Authored settings only —
/// resolve it against a camera with [`ShadowVolume::fit`].
#[derive(Clone, Copy, Debug)]
pub struct ShadowVolume {
    /// How far from the camera shadows are kept, in world units.
    ///
    /// This is the dial for reach. The covered box is sized to contain the view
    /// frustum out to here, so raising it extends shadows into the distance at
    /// the cost of coarser texels everywhere — there is one map, and it is
    /// stretched over whatever this asks for.
    pub shadow_distance: f32,

    /// Depth of the box along the light direction. Must clear the tallest
    /// caster above the covered area, or its shadow is clipped away.
    pub depth_extent: f32,

    /// Edge length of the (square) depth map, in texels.
    ///
    /// This is what buys back the definition that `shadow_distance` and
    /// `pcf_radius` between them spend. Both of those widen the PCF kernel's
    /// *world* footprint — one by making texels bigger, the other by taking
    /// more of them — and a caster thinner than that footprint dissolves. More
    /// texels is the only dial that narrows it without giving up reach or
    /// softness. It is also the expensive one: cost is quadratic, in depth
    /// memory and in shadow-pass fill alike.
    pub resolution: u32,

    /// How dark a fully shadowed surface goes, as a fraction of the sun's
    /// contribution. 1 removes the sun entirely and leaves only sky and ambient
    /// fill, which is physically right; lower values are an artistic retreat
    /// that amounts to light leaking through solid occluders.
    pub strength: f32,

    /// Normal-offset distance as a multiple of one shadow texel's world size.
    ///
    /// Expressed in texels rather than metres so that changing `resolution` or
    /// `shadow_distance` does not silently re-tune the bias.
    pub normal_offset_texels: f32,

    /// Half-width of the PCF kernel, in texels: 1 gives 3x3 taps, 2 gives 5x5.
    ///
    /// Uniform rather than a shader constant so a bench sweep can ladder it
    /// against the fill level without a recompile — the two are judged
    /// together, since edge softness and shadow depth trade off by eye.
    /// `SHADOW_PCF_MAX_RADIUS` in shader/shadow.glsl bounds what the shader
    /// will honour.
    pub pcf_radius: u32,
}

impl Default for ShadowVolume {
    fn default() -> Self {
        Self {
            // At the engine's usual 45°/16:9 camera this fits a box roughly 77m
            // across, reaching far enough that mid-distance geometry stays
            // grounded.
            shadow_distance: 45.0,
            depth_extent: 160.0,
            // 1.9cm per texel over that box, which puts the 5x5 kernel's
            // footprint at 9.4cm — narrower than a forearm, so limbs and
            // railings still cast something with shape in it. 2048 was tried
            // first and did not: at 3.8cm the kernel spanned 19cm and every
            // thin caster in the game turned into a smear. 67MB of D32 is the
            // price of that, and it is the reason this is 4096 and not higher.
            resolution: 4096,
            strength: 1.0,
            normal_offset_texels: 1.5,
            pcf_radius: 2,
        }
    }
}

impl ShadowVolume {
    /// Resolve this volume against the camera it will be framed around.
    pub fn fit(&self, frustum: &ViewFrustum) -> ShadowFraming {
        let slice = frustum.slice_bounding_sphere(self.shadow_distance);
        ShadowFraming {
            volume: *self,
            radius: slice.radius,
            forward_offset: slice.forward_offset,
        }
    }
}

/// A [`ShadowVolume`] fitted to a view frustum: everything the shadow pass and
/// the shader lookup need, including the quantities that depend on the fit.
#[derive(Clone, Copy, Debug)]
pub struct ShadowFraming {
    volume: ShadowVolume,

    /// Half-width of the covered square, from the frustum's bounding sphere.
    radius: f32,

    /// How far ahead of the camera that sphere is centred.
    forward_offset: f32,
}

impl ShadowFraming {
    /// The settings this framing resolved.
    pub fn volume(&self) -> &ShadowVolume {
        &self.volume
    }

    /// Half-width of the covered square, in world units.
    pub fn radius(&self) -> f32 {
        self.radius
    }

    /// How far ahead of the camera the covered square is centred.
    pub fn forward_offset(&self) -> f32 {
        self.forward_offset
    }

    /// World size of one shadow map texel.
    pub fn texel_world_size(&self) -> f32 {
        2.0 * self.radius / self.volume.resolution as f32
    }

    /// One texel as a fraction of the map, for PCF tap spacing.
    pub fn texel_uv_size(&self) -> f32 {
        1.0 / self.volume.resolution as f32
    }

    /// Normal-offset distance in world units, for the receiver-side bias.
    pub fn normal_offset(&self) -> f32 {
        self.volume.normal_offset_texels * self.texel_world_size()
    }

    /// World-space to the sun's clip space, for both the shadow pass and the
    /// lookup that reads its result.
    ///
    /// `camera_forward` need not be normalized; `sun_direction` points from a
    /// surface *towards* the sun, matching `SceneLighting::sun_direction`.
    pub fn light_view_proj(
        &self,
        camera_pos: &Vector3<f32>,
        camera_forward: &Vector3<f32>,
        sun_direction: &Vector3<f32>,
    ) -> Matrix4<f32> {
        let sun = normalize_or(sun_direction, &Vector3::y());
        let forward = flatten_forward(camera_forward, &sun);

        let centre_world = camera_pos + forward * self.forward_offset;

        // Orientation alone: an eye at the origin looking along the light, so
        // the centre can be quantised in the light's own frame before the box
        // is placed on it.
        let rotation =
            Matrix4::look_at_rh(&Point3::origin(), &Point3::from(-sun), &stable_up(&sun));
        let centre_in_light = rotation.transform_point(&Point3::from(centre_world)).coords;

        // Texel snapping. Without it the box slides continuously as the camera
        // moves, every shadow edge lands on a different sub-texel position each
        // frame, and the whole frame crawls. Quantising the centre to the texel
        // grid means the map's contents translate in whole texels instead.
        // This is only valid because `radius` — and hence the grid spacing — is
        // constant while the camera turns; see `ViewFrustum`.
        let texel = self.texel_world_size();
        let centre = centre_in_light.map(|c| (c / texel).round() * texel);

        // Put the snapped centre half a box-depth in front of the light.
        let extent = self.volume.depth_extent;
        let offset = Vector3::new(-centre.x, -centre.y, -centre.z - extent * 0.5);
        let view = Matrix4::new_translation(&offset) * rotation;

        orthographic_vk(self.radius, extent) * view
    }
}

/// Normalize, falling back when the input is degenerate.
fn normalize_or(v: &Vector3<f32>, fallback: &Vector3<f32>) -> Vector3<f32> {
    let length = v.norm();
    if length > 1e-6 {
        v / length
    } else {
        *fallback
    }
}

/// The camera's heading, projected off the sun axis and normalized.
///
/// Only the component perpendicular to the light moves the box across the map;
/// the component along it just slides the box within its own depth, wasting
/// coverage. Since the box is square in the light's own frame, dropping that
/// component leaves the covered region of the map identical. Looking straight
/// down the sun leaves nothing to project, so any perpendicular direction will
/// do.
fn flatten_forward(camera_forward: &Vector3<f32>, sun: &Vector3<f32>) -> Vector3<f32> {
    let forward = normalize_or(camera_forward, &-Vector3::z());
    let flattened = forward - sun * forward.dot(sun);
    normalize_or(&flattened, &perpendicular(sun))
}

/// An up vector that is not parallel to the light.
fn stable_up(sun: &Vector3<f32>) -> Vector3<f32> {
    if sun.y.abs() > 0.99 {
        Vector3::z()
    } else {
        Vector3::y()
    }
}

/// Any unit vector perpendicular to `v`.
fn perpendicular(v: &Vector3<f32>) -> Vector3<f32> {
    let axis = if v.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    normalize_or(&v.cross(&axis), &Vector3::x())
}

/// Symmetric orthographic projection with Vulkan's conventions: depth in
/// `[0, 1]` and +Y down in clip space.
///
/// nalgebra's `new_orthographic` targets OpenGL's `[-1, 1]` depth, which would
/// throw away the near half of the box, so the matrix is built here instead.
fn orthographic_vk(radius: f32, depth_extent: f32) -> Matrix4<f32> {
    let inv_radius = 1.0 / radius;
    let mut proj = Matrix4::identity();
    proj[(0, 0)] = inv_radius;
    // Negated for Vulkan's downward clip-space Y, matching `Camera`'s flip.
    proj[(1, 1)] = -inv_radius;
    // Looking down -Z in view space, so depth 0 is at z = 0 and 1 at -extent.
    proj[(2, 2)] = -1.0 / depth_extent;
    proj[(2, 3)] = 0.0;
    proj
}

#[cfg(test)]
mod tests {
    use super::*;

    fn framing() -> ShadowFraming {
        ShadowVolume::default().fit(&ViewFrustum::default())
    }

    /// Project a world point and return its NDC coordinates.
    fn project(m: &Matrix4<f32>, p: Vector3<f32>) -> Vector3<f32> {
        let clip = m * p.push(1.0);
        clip.xyz() / clip.w
    }

    #[test]
    fn the_covered_centre_lands_in_the_middle_of_the_map() {
        let f = framing();
        let camera = Vector3::new(3.0, 5.0, -2.0);
        // Perpendicular to the sun, so `flatten_forward` leaves it alone and
        // the centre is exactly `forward_offset` straight ahead.
        let forward = Vector3::new(0.0, 0.0, -1.0);
        let sun = Vector3::new(0.0, 1.0, 0.0);

        let m = f.light_view_proj(&camera, &forward, &sun);
        let centre = camera + forward * f.forward_offset();
        let ndc = project(&m, centre);

        // Snapping moves the centre by at most half a texel, which is a small
        // fraction of the radius.
        let tolerance = f.texel_world_size() / f.radius();
        assert!(ndc.x.abs() < tolerance, "x = {}", ndc.x);
        assert!(ndc.y.abs() < tolerance, "y = {}", ndc.y);
        // Depth 0 is the far side of the box, so the centre sits at the middle.
        assert!((ndc.z - 0.5).abs() < 0.01, "z = {}", ndc.z);
    }

    #[test]
    fn depth_stays_inside_the_vulkan_range() {
        let f = framing();
        let sun = Vector3::new(0.0, 1.0, 0.0);
        let m = f.light_view_proj(&Vector3::zeros(), &Vector3::new(0.0, 0.0, -1.0), &sun);

        let centre = Vector3::new(0.0, 0.0, -f.forward_offset());
        let extent = f.volume().depth_extent;
        // A point just under the top of the box, and one just above its floor.
        let high = project(&m, centre + Vector3::new(0.0, extent * 0.5 - 1.0, 0.0));
        let low = project(&m, centre - Vector3::new(0.0, extent * 0.5 - 1.0, 0.0));

        assert!(high.z > 0.0 && high.z < 1.0, "high = {}", high.z);
        assert!(low.z > 0.0 && low.z < 1.0, "low = {}", low.z);
        // Nearer the sun means nearer the light's near plane, which is depth 0.
        assert!(high.z < low.z);
    }

    #[test]
    fn the_centre_ignores_camera_pitch_along_the_light() {
        // Tilting the camera up and down under an overhead sun moves the box
        // only in the plane the shadow map covers — pitch would otherwise spend
        // coverage sliding the box through its own depth.
        let f = framing();
        let sun = Vector3::new(0.0, 1.0, 0.0);
        let level = f.light_view_proj(&Vector3::zeros(), &Vector3::new(0.0, 0.0, -1.0), &sun);
        let pitched = f.light_view_proj(
            &Vector3::zeros(),
            &Vector3::new(0.0, -1.0, -1.0).normalize(),
            &sun,
        );

        let probe = Vector3::new(2.0, 0.0, -8.0);
        let delta = project(&level, probe).xy() - project(&pitched, probe).xy();
        assert!(delta.norm() < 1e-4, "centre moved by {}", delta.norm());
    }

    #[test]
    fn covers_the_full_radius_and_no_more() {
        let f = framing();
        let sun = Vector3::new(0.0, 1.0, 0.0);
        let camera = Vector3::zeros();
        let forward = Vector3::new(0.0, 0.0, -1.0);
        let m = f.light_view_proj(&camera, &forward, &sun);
        let centre = camera + forward * f.forward_offset();

        let inside = project(&m, centre + Vector3::new(f.radius() * 0.9, 0.0, 0.0));
        assert!(inside.x.abs() < 1.0, "x = {}", inside.x);

        let outside = project(&m, centre + Vector3::new(f.radius() * 1.1, 0.0, 0.0));
        assert!(outside.x.abs() > 1.0, "x = {}", outside.x);
    }

    #[test]
    fn the_whole_visible_frustum_is_inside_the_map() {
        // The point of fitting to the frustum: everything the camera can see
        // out to `shadow_distance` receives, whichever way the camera faces.
        let volume = ShadowVolume::default();
        let frustum = ViewFrustum::default();
        let f = volume.fit(&frustum);
        let sun = Vector3::new(0.35, 0.8, -0.2).normalize();
        let camera = Vector3::new(-4.0, 2.0, 7.0);

        for yaw in [0.0f32, 0.7, 1.9, 3.0, 4.4, 5.8] {
            let forward = Vector3::new(yaw.sin(), -0.25, -yaw.cos()).normalize();
            let right = forward.cross(&Vector3::y()).normalize();
            let up = right.cross(&forward);
            let m = f.light_view_proj(&camera, &forward, &sun);

            for depth in [frustum.near, volume.shadow_distance] {
                for sx in [-1.0f32, 1.0] {
                    for sy in [-1.0f32, 1.0] {
                        let corner = camera
                            + forward * depth
                            + right * (sx * frustum.tan_half_x * depth)
                            + up * (sy * frustum.tan_half_y * depth);
                        let ndc = project(&m, corner);
                        assert!(
                            ndc.x.abs() <= 1.0 && ndc.y.abs() <= 1.0,
                            "corner at depth {depth} fell outside the map: {ndc}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn the_fitted_radius_does_not_change_as_the_camera_rotates() {
        // The snap grid's spacing is the radius divided by the resolution. If
        // the radius breathed with camera orientation the grid would resize
        // every frame and every shadow edge would crawl, which is the artifact
        // the snapping exists to prevent. Checked through the matrix rather
        // than on the field, so a fit that reached the projection some other
        // way would still be caught.
        let f = framing();
        let sun = Vector3::new(0.3, 0.9, 0.3).normalize();

        // The world-to-light scale is the length of the projection's first row.
        let scale = |forward: Vector3<f32>| {
            let m = f.light_view_proj(&Vector3::new(1.0, 2.0, 3.0), &forward, &sun);
            Vector3::new(m[(0, 0)], m[(0, 1)], m[(0, 2)]).norm()
        };

        let reference = scale(Vector3::new(0.0, 0.0, -1.0));
        assert!((reference - 1.0 / f.radius()).abs() < 1e-5);

        for yaw in [0.0f32, 0.9, 2.1, 3.4, 5.2] {
            for pitch in [-1.1f32, -0.3, 0.0, 0.6, 1.2] {
                let forward = Vector3::new(
                    yaw.sin() * pitch.cos(),
                    pitch.sin(),
                    -yaw.cos() * pitch.cos(),
                );
                let s = scale(forward);
                assert!(
                    (s - reference).abs() < 1e-5,
                    "scale changed to {s} from {reference} at yaw {yaw}, pitch {pitch}"
                );
            }
        }
    }

    #[test]
    fn sub_texel_camera_motion_does_not_move_the_map() {
        let f = framing();
        let sun = Vector3::new(0.3, 0.9, 0.3).normalize();
        let forward = Vector3::new(0.0, 0.0, -1.0);

        // A nudge well under one texel must leave the projection untouched:
        // that is what stops shadow edges crawling as the camera drifts.
        let a = f.light_view_proj(&Vector3::zeros(), &forward, &sun);
        let nudge = Vector3::new(f.texel_world_size() * 0.05, 0.0, 0.0);
        let b = f.light_view_proj(&nudge, &forward, &sun);

        let probe = Vector3::new(1.0, 2.0, -10.0);
        let delta = project(&a, probe) - project(&b, probe);
        assert!(delta.norm() < 1e-4, "map shifted by {}", delta.norm());
    }

    #[test]
    fn a_sun_straight_overhead_still_produces_a_usable_frame() {
        // The degenerate case: the light is parallel to the default up vector,
        // and a naive look-at would produce NaNs.
        let m = framing().light_view_proj(
            &Vector3::zeros(),
            &Vector3::new(0.0, 0.0, -1.0),
            &Vector3::new(0.0, 1.0, 0.0),
        );
        assert!(m.iter().all(|c| c.is_finite()));
    }

    #[test]
    fn a_camera_looking_along_the_sun_still_produces_a_usable_frame() {
        let sun = Vector3::new(0.0, 1.0, 0.0);
        let m = framing().light_view_proj(&Vector3::zeros(), &-sun, &sun);
        assert!(m.iter().all(|c| c.is_finite()));
    }
}
