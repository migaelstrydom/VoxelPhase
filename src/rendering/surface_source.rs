//! Where a fragment's shading inputs come from.
//!
//! One bit used to decide all of this. `materialTriplanarScale() > 0` meant, at
//! once: sample the albedo by world position, read per-vertex surface character
//! out of the texture-coordinate channel, take roughness from that character
//! rather than from the material, and scale the detail normal by it. That is
//! four independent decisions riding one flag, and it is true for exactly one
//! thing in the game — terrain.
//!
//! The consequence was that no other surface could have any part of it. A prop
//! textured by its own UVs could not have a detail normal, because asking for
//! one meant asking for world-projected albedo and terrain's roughness model as
//! well. Splitting the bits is what lets a menhir take the grain and keep its
//! texture.
//!
//! ```text
//!   terrain    ALBEDO_TRIPLANAR | CHARACTER_IN_TEXCOORD
//!              | ROUGHNESS_FROM_CHARACTER | RELIEF_FROM_CHARACTER
//!   stone prop GRAIN_OBJECT_SPACE
//!   plank      GRAIN_BY_UV
//!   beach ball (none)
//! ```

/// Bit flags selecting a surface's shading inputs.
///
/// Hand-rolled rather than pulled from `bitflags` because the set is tiny, the
/// values are a contract with `shader/surface_source.glsl`, and the two files
/// must be read side by side to be checked.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct SurfaceSource(pub u32);

impl SurfaceSource {
    /// Nothing special: albedo from the mesh's own texture coordinates,
    /// roughness from the material, no detail normal.
    pub const PLAIN: Self = Self(0);

    /// Sample the albedo by world position across the three axis planes,
    /// instead of by the mesh's texture coordinates.
    ///
    /// For surfaces with no usable UV parameterisation — marching-cubes
    /// terrain, whose isosurface is re-meshed on every crater.
    pub const ALBEDO_TRIPLANAR: Self = Self(1 << 0);

    /// The location-2 vertex channel carries surface character (hardness),
    /// not texture coordinates.
    ///
    /// Only meaningful on a mesh that does not need its UVs, which in practice
    /// means one that also sets [`ALBEDO_TRIPLANAR`](Self::ALBEDO_TRIPLANAR).
    pub const CHARACTER_IN_TEXCOORD: Self = Self(1 << 1);

    /// Derive roughness per fragment from surface character rather than taking
    /// the material's single authored value.
    ///
    /// A terrain chunk carries many materials but a draw carries one finish, so
    /// terrain cannot express itself through the material's roughness.
    pub const ROUGHNESS_FROM_CHARACTER: Self = Self(1 << 2);

    /// Scale the detail normal by surface character rather than by the
    /// material's authored grain strength. Chalk ends up softer-featured than
    /// rock without a second texture.
    pub const RELIEF_FROM_CHARACTER: Self = Self(1 << 3);

    /// Perturb the normal with the shared grain atlas, projected from the
    /// mesh's *model-space* position across the three axis planes.
    ///
    /// Model space, not world space, because a prop moves. A world-projected
    /// grain would swim across the surface of a toppling menhir; terrain gets
    /// away with world space only because it never moves.
    pub const GRAIN_OBJECT_SPACE: Self = Self(1 << 4);

    /// Perturb the normal with the shared grain atlas, addressed by the mesh's
    /// own texture coordinates.
    ///
    /// For grain that has a direction the mesh already knows — wood fibre
    /// running along a plank, which a triplanar projection cannot express.
    pub const GRAIN_BY_UV: Self = Self(1 << 5);

    /// Everything terrain asks for. The four bits that used to be one.
    pub const TERRAIN: Self = Self(
        Self::ALBEDO_TRIPLANAR.0
            | Self::CHARACTER_IN_TEXCOORD.0
            | Self::ROUGHNESS_FROM_CHARACTER.0
            | Self::RELIEF_FROM_CHARACTER.0,
    );

    /// Combine two sets of flags.
    pub const fn with(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    /// Whether every bit of `other` is set here.
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    /// Whether any grain projection is selected.
    pub const fn has_grain(self) -> bool {
        self.0 & (Self::GRAIN_OBJECT_SPACE.0 | Self::GRAIN_BY_UV.0) != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_plain_surface_asks_for_nothing() {
        assert_eq!(SurfaceSource::PLAIN.0, 0);
        assert!(!SurfaceSource::PLAIN.has_grain());
        assert!(!SurfaceSource::PLAIN.contains(SurfaceSource::ALBEDO_TRIPLANAR));
    }

    /// The bits are a contract with the shader. Renumbering one silently
    /// reassigns behaviour, so the values are pinned here rather than left to
    /// whatever order the constants happen to be declared in.
    #[test]
    fn the_bit_values_are_the_ones_the_shader_declares() {
        assert_eq!(SurfaceSource::ALBEDO_TRIPLANAR.0, 1);
        assert_eq!(SurfaceSource::CHARACTER_IN_TEXCOORD.0, 2);
        assert_eq!(SurfaceSource::ROUGHNESS_FROM_CHARACTER.0, 4);
        assert_eq!(SurfaceSource::RELIEF_FROM_CHARACTER.0, 8);
        assert_eq!(SurfaceSource::GRAIN_OBJECT_SPACE.0, 16);
        assert_eq!(SurfaceSource::GRAIN_BY_UV.0, 32);
    }

    /// The whole point of the split: a prop can ask for grain without
    /// inheriting terrain's albedo projection or its roughness model.
    #[test]
    fn grain_is_independent_of_the_terrain_flags() {
        let prop = SurfaceSource::GRAIN_OBJECT_SPACE;

        assert!(prop.has_grain());
        assert!(!prop.contains(SurfaceSource::ALBEDO_TRIPLANAR));
        assert!(!prop.contains(SurfaceSource::ROUGHNESS_FROM_CHARACTER));
        assert!(!prop.contains(SurfaceSource::CHARACTER_IN_TEXCOORD));
    }

    #[test]
    fn terrain_asks_for_all_four_of_the_bits_it_used_to_get_from_one() {
        let terrain = SurfaceSource::TERRAIN;

        assert!(terrain.contains(SurfaceSource::ALBEDO_TRIPLANAR));
        assert!(terrain.contains(SurfaceSource::CHARACTER_IN_TEXCOORD));
        assert!(terrain.contains(SurfaceSource::ROUGHNESS_FROM_CHARACTER));
        assert!(terrain.contains(SurfaceSource::RELIEF_FROM_CHARACTER));

        // Terrain's detail rides its own packed field, not the grain atlas.
        assert!(!terrain.has_grain());
    }

    #[test]
    fn flags_combine() {
        let both = SurfaceSource::GRAIN_BY_UV.with(SurfaceSource::ROUGHNESS_FROM_CHARACTER);

        assert!(both.contains(SurfaceSource::GRAIN_BY_UV));
        assert!(both.contains(SurfaceSource::ROUGHNESS_FROM_CHARACTER));
        assert!(!both.contains(SurfaceSource::GRAIN_OBJECT_SPACE));
    }
}
