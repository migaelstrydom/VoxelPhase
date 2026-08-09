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

/// How sharply a surface commits to the axis plane it most nearly faces.
///
/// The blend weight of each plane is its normal component raised to this power.
/// Too low and all three projections ghost through each other on any slope,
/// which reads as a smeared double image; too high and the transition narrows
/// until the change of projection is visible as a seam on 45-degree faces.
const DEFAULT_SHARPNESS: f32 = 4.0;

/// Texture repeats per world unit for terrain.
///
/// Deliberately equal to the scale of the top-down UV that terrain vertices
/// already carried (`pos.xz * 0.1`), so that switching to the projection leaves
/// flat ground looking exactly as it did and changes only the steep faces the
/// old projection was smearing.
const TERRAIN_SCALE: f32 = 0.1;

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

    /// The projection terrain is textured by.
    pub const TERRAIN: Self = Self {
        scale: TERRAIN_SCALE,
        sharpness: DEFAULT_SHARPNESS,
    };

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
    fn terrain_projects_at_the_scale_of_the_uvs_it_replaces() {
        // Terrain vertices carried `tex_coords = pos.xz * 0.1`, which is the
        // Y-plane of this projection. Matching it is what keeps flat ground
        // unchanged, so a drift here is a silent change to every level's floor.
        assert_eq!(TriplanarProjection::TERRAIN.scale, 0.1);
        assert!(TriplanarProjection::TERRAIN.is_enabled());
    }
}
