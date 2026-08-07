//! The slab of world the sun shadow map covers, and the matrix that maps it.
//!
//! A directional light has no position, so a shadow map for one is an
//! orthographic box aimed along the light and parked somewhere useful. With a
//! single map (no cascades) "somewhere useful" is a fixed-size box centred a
//! little ahead of the camera: close enough that the shadows a player reads
//! from are inside it, small enough that its texels stay meaningful.
//!
//! ```text
//!            sun direction
//!                  \
//!                   \   +-----------------+  <- back plane, depth 0
//!                    \  |                 |
//!   camera --------> focus (snapped)      |     radius on each side
//!                       |                 |
//!                       +-----------------+  <- front plane, depth = depth_extent
//! ```

use nalgebra::{Matrix4, Point3, Vector3};

/// How the sun's shadow map is framed around the viewer.
#[derive(Clone, Copy, Debug)]
pub struct ShadowVolume {
    /// Half-width of the covered square, in world units. Everything within this
    /// distance of the focus point casts and receives.
    pub radius: f32,

    /// How far ahead of the camera the covered square is centred. Biasing
    /// forward spends the budget on what the player is looking at rather than
    /// on what is behind them.
    pub focus_distance: f32,

    /// Depth of the box along the light direction. Must clear the tallest
    /// caster above the focus point, or its shadow is clipped away.
    pub depth_extent: f32,

    /// Edge length of the (square) depth map, in texels.
    pub resolution: u32,

    /// How dark a fully shadowed surface goes, as a fraction of the sun's
    /// contribution. 1 removes the sun entirely and leaves only sky and ambient
    /// fill, which is physically right; lower values are an artistic retreat.
    pub strength: f32,

    /// Normal-offset distance as a multiple of one shadow texel's world size.
    ///
    /// Expressed in texels rather than metres so that changing `resolution` or
    /// `radius` does not silently re-tune the bias.
    pub normal_offset_texels: f32,
}

impl Default for ShadowVolume {
    fn default() -> Self {
        Self {
            // 48m across the focus point. At 2048 texels that is ~4.7cm per
            // texel — fine enough that a character's limbs cast a readable
            // shadow, wide enough to cover the ground a jump is judged against.
            radius: 24.0,
            focus_distance: 12.0,
            depth_extent: 160.0,
            resolution: 2048,
            strength: 1.0,
            normal_offset_texels: 1.5,
        }
    }
}

impl ShadowVolume {
    /// World size of one shadow map texel.
    pub fn texel_world_size(&self) -> f32 {
        2.0 * self.radius / self.resolution as f32
    }

    /// One texel as a fraction of the map, for PCF tap spacing.
    pub fn texel_uv_size(&self) -> f32 {
        1.0 / self.resolution as f32
    }

    /// Normal-offset distance in world units, for the receiver-side bias.
    pub fn normal_offset(&self) -> f32 {
        self.normal_offset_texels * self.texel_world_size()
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

        let focus = camera_pos + forward * self.focus_distance;

        // Orientation alone: an eye at the origin looking along the light, so
        // the focus point can be quantised in the light's own frame before the
        // box is centred on it.
        let rotation =
            Matrix4::look_at_rh(&Point3::origin(), &Point3::from(-sun), &stable_up(&sun));
        let focus_in_light = rotation.transform_point(&Point3::from(focus)).coords;

        // Texel snapping. Without it the box slides continuously as the camera
        // moves, every shadow edge lands on a different sub-texel position each
        // frame, and the whole frame crawls. Quantising the centre to the texel
        // grid means the map's contents translate in whole texels instead.
        let texel = self.texel_world_size();
        let centre = focus_in_light.map(|c| (c / texel).round() * texel);

        // Put the snapped centre half a box-depth in front of the light.
        let offset = Vector3::new(-centre.x, -centre.y, -centre.z - self.depth_extent * 0.5);
        let view = Matrix4::new_translation(&offset) * rotation;

        orthographic_vk(self.radius, self.depth_extent) * view
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
/// coverage. Looking straight down the sun leaves nothing to project, so any
/// perpendicular direction will do.
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

    fn volume() -> ShadowVolume {
        ShadowVolume::default()
    }

    /// Project a world point and return its NDC coordinates.
    fn project(m: &Matrix4<f32>, p: Vector3<f32>) -> Vector3<f32> {
        let clip = m * p.push(1.0);
        clip.xyz() / clip.w
    }

    #[test]
    fn focus_point_lands_in_the_middle_of_the_map() {
        let v = volume();
        let camera = Vector3::new(3.0, 5.0, -2.0);
        // Perpendicular to the sun, so `flatten_forward` leaves it alone and
        // the focus point is exactly `focus_distance` straight ahead.
        let forward = Vector3::new(0.0, 0.0, -1.0);
        let sun = Vector3::new(0.0, 1.0, 0.0);

        let m = v.light_view_proj(&camera, &forward, &sun);
        let focus = camera + forward * v.focus_distance;
        let ndc = project(&m, focus);

        // Snapping moves the focus by at most half a texel, which is a small
        // fraction of the radius.
        let tolerance = v.texel_world_size() / v.radius;
        assert!(ndc.x.abs() < tolerance, "x = {}", ndc.x);
        assert!(ndc.y.abs() < tolerance, "y = {}", ndc.y);
        // Depth 0 is the far side of the box, so the centre sits at the middle.
        assert!((ndc.z - 0.5).abs() < 0.01, "z = {}", ndc.z);
    }

    #[test]
    fn depth_stays_inside_the_vulkan_range() {
        let v = volume();
        let sun = Vector3::new(0.0, 1.0, 0.0);
        let m = v.light_view_proj(&Vector3::zeros(), &Vector3::new(0.0, 0.0, -1.0), &sun);

        let focus = Vector3::new(0.0, 0.0, -v.focus_distance);
        // A point just under the top of the box, and one just above its floor.
        let high = project(
            &m,
            focus + Vector3::new(0.0, v.depth_extent * 0.5 - 1.0, 0.0),
        );
        let low = project(
            &m,
            focus - Vector3::new(0.0, v.depth_extent * 0.5 - 1.0, 0.0),
        );

        assert!(high.z > 0.0 && high.z < 1.0, "high = {}", high.z);
        assert!(low.z > 0.0 && low.z < 1.0, "low = {}", low.z);
        // Nearer the sun means nearer the light's near plane, which is depth 0.
        assert!(high.z < low.z);
    }

    #[test]
    fn the_focus_point_ignores_camera_pitch_along_the_light() {
        // Tilting the camera up and down under an overhead sun moves the focus
        // point only in the plane the shadow map covers — pitch would otherwise
        // spend coverage sliding the box through its own depth.
        let v = volume();
        let sun = Vector3::new(0.0, 1.0, 0.0);
        let level = v.light_view_proj(&Vector3::zeros(), &Vector3::new(0.0, 0.0, -1.0), &sun);
        let pitched = v.light_view_proj(
            &Vector3::zeros(),
            &Vector3::new(0.0, -1.0, -1.0).normalize(),
            &sun,
        );

        let probe = Vector3::new(2.0, 0.0, -8.0);
        let delta = project(&level, probe).xy() - project(&pitched, probe).xy();
        assert!(delta.norm() < 1e-4, "focus moved by {}", delta.norm());
    }

    #[test]
    fn covers_the_full_radius_and_no_more() {
        let v = volume();
        let sun = Vector3::new(0.0, 1.0, 0.0);
        let camera = Vector3::zeros();
        let forward = Vector3::new(0.0, 0.0, -1.0);
        let m = v.light_view_proj(&camera, &forward, &sun);
        let focus = camera + forward * v.focus_distance;

        let inside = project(&m, focus + Vector3::new(v.radius * 0.9, 0.0, 0.0));
        assert!(inside.x.abs() < 1.0, "x = {}", inside.x);

        let outside = project(&m, focus + Vector3::new(v.radius * 1.1, 0.0, 0.0));
        assert!(outside.x.abs() > 1.0, "x = {}", outside.x);
    }

    #[test]
    fn sub_texel_camera_motion_does_not_move_the_map() {
        let v = volume();
        let sun = Vector3::new(0.3, 0.9, 0.3).normalize();
        let forward = Vector3::new(0.0, 0.0, -1.0);

        // A nudge well under one texel must leave the projection untouched:
        // that is what stops shadow edges crawling as the camera drifts.
        let a = v.light_view_proj(&Vector3::zeros(), &forward, &sun);
        let nudge = Vector3::new(v.texel_world_size() * 0.05, 0.0, 0.0);
        let b = v.light_view_proj(&nudge, &forward, &sun);

        let probe = Vector3::new(1.0, 2.0, -10.0);
        let delta = project(&a, probe) - project(&b, probe);
        assert!(delta.norm() < 1e-4, "map shifted by {}", delta.norm());
    }

    #[test]
    fn a_sun_straight_overhead_still_produces_a_usable_frame() {
        // The degenerate case: the light is parallel to the default up vector,
        // and a naive look-at would produce NaNs.
        let v = volume();
        let m = v.light_view_proj(
            &Vector3::zeros(),
            &Vector3::new(0.0, 0.0, -1.0),
            &Vector3::new(0.0, 1.0, 0.0),
        );
        assert!(m.iter().all(|c| c.is_finite()));
    }

    #[test]
    fn a_camera_looking_along_the_sun_still_produces_a_usable_frame() {
        let v = volume();
        let sun = Vector3::new(0.0, 1.0, 0.0);
        let m = v.light_view_proj(&Vector3::zeros(), &-sun, &sun);
        assert!(m.iter().all(|c| c.is_finite()));
    }
}
