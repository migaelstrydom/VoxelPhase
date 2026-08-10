//! World-space triplanar texture projection.
//!
//! A mesh that carries authored texture coordinates is textured by them. Terrain
//! cannot be: marching cubes emits an isosurface with no UV parameterisation,
//! and destruction re-meshes it, so any UV assignment would have to be recomputed
//! per crater. Terrain is therefore addressed by world position instead, sampled
//! once per axis plane and blended by how much the surface faces each.
//!
//! The CPU side of that decision is this component; the sampling itself is
//! `shader/triplanar.glsl`.
//!
//! This component describes *how* to project and holds no opinion about what
//! any particular surface should look like. The values terrain projects at live
//! with terrain, in `src/terrain/surface.rs`, alongside the rest of its
//! appearance.

/// How a mesh's texture is addressed: by its own vertex texture coordinates, or
/// by world position projected along the three axis planes.
///
/// Disabled is the default, because everything except terrain has UVs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TriplanarProjection {
    /// Texture repeats per world unit. Zero disables the projection entirely,
    /// and the mesh's vertex texture coordinates are used instead.
    pub scale: f32,

    /// Exponent applied to the normal's axis components to weight the three
    /// projections. Meaningless when `scale` is zero.
    pub sharpness: f32,
}

impl TriplanarProjection {
    /// Texture by the mesh's own vertex texture coordinates.
    pub const DISABLED: Self = Self {
        scale: 0.0,
        sharpness: 0.0,
    };

    /// Project at `scale` texture repeats per world unit.
    ///
    /// `sharpness` decides how quickly a surface commits to the axis plane it
    /// most nearly faces: too low and all three projections ghost through each
    /// other on a slope, which reads as a smeared double image; too high and
    /// the transition narrows until the change of projection shows as a seam on
    /// 45-degree faces.
    pub const fn new(scale: f32, sharpness: f32) -> Self {
        Self { scale, sharpness }
    }

    /// Whether this projection replaces the mesh's texture coordinates.
    pub fn is_enabled(&self) -> bool {
        self.scale > 0.0
    }

    /// Pack for the fragment push constant. Must match the `projection` field
    /// of the `MaterialPushConstants` block in `shader/material.glsl`.
    pub fn packed(&self) -> [f32; 2] {
        [self.scale, self.sharpness]
    }
}

impl Default for TriplanarProjection {
    fn default() -> Self {
        Self::DISABLED
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_default_projection_leaves_uv_textured_meshes_alone() {
        assert!(!TriplanarProjection::default().is_enabled());
        assert_eq!(TriplanarProjection::default().packed()[0], 0.0);
    }

    #[test]
    fn any_positive_scale_replaces_the_meshs_texture_coordinates() {
        assert!(TriplanarProjection::new(0.1, 4.0).is_enabled());
        assert_eq!(TriplanarProjection::new(0.1, 4.0).packed(), [0.1, 4.0]);
    }
}
