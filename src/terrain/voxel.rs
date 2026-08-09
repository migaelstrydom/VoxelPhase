//! Voxel types and materials.
//!
//! # Durability
//!
//! What a voxel is made of is the only thing that decides how hard it is to
//! remove. There is no per-voxel health: [`VoxelMaterial::toughness`] is a pure
//! function, evaluated when a blast asks, so nothing can go stale against the
//! geometry the player has since carved. Depth plays no part — terrain that
//! should resist is authored out of material that resists, in the level's
//! `material_layers`.
//!
//! An earlier scheme stored a health byte per voxel and raised it with depth
//! below the surface. It was baked at generation and never re-derived, so it
//! drifted from the geometry as soon as anything was destroyed, and it made two
//! voxels of visibly identical rock behave differently for reasons the player
//! could not see. Toughness-by-material is both legible — the colour is the
//! durability — and immune to that class of bug.

/// Material type for a voxel, determining its properties and appearance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(u8)]
pub enum VoxelMaterial {
    #[default]
    Air = 0,
    Rock = 1,
    Grass = 2,
    Dirt = 3,
    /// Pale grey-yellow cave surface crust from mineral deposits.
    Ite = 4,
    /// Pale grey cave wall rock — the main bulk of cave interiors.
    Limestone = 5,
    /// Dark layered deep-cave rock.
    Slate = 6,
    /// Loose pale grain — beaches, and the readable surface for a route laid
    /// over ground of a different colour.
    Sand = 7,
    /// The floor of the world. Indestructible, and the only material that is —
    /// it exists to stop the player digging out of the level, which is a
    /// property of the world's extent rather than of any rock.
    Bedrock = 8,
}

impl VoxelMaterial {
    /// Get a color for this material (for simple vertex coloring).
    pub fn color(&self) -> [f32; 4] {
        match self {
            VoxelMaterial::Air => [0.0, 0.0, 0.0, 0.0],
            VoxelMaterial::Rock => [0.5, 0.5, 0.5, 1.0],
            // Deep enough to sit below the props standing on it. A light green
            // reads as a bright slab under a sunlit sky and flattens the frame,
            // because terrain fills more of it than anything else does.
            // A yellow-green rather than a pure green, at the luminance and
            // saturation the pure green had — only the hue moves, by about 15
            // degrees towards yellow. Vegetation lit by a warm sun reads yellow
            // long before it reads green, and a green sitting on the primary is
            // the one hue that cannot warm up: the sun's tint has no red
            // headroom left to give it.
            VoxelMaterial::Grass => [0.24, 0.39, 0.12, 1.0],
            VoxelMaterial::Dirt => [0.5, 0.3, 0.1, 1.0],
            VoxelMaterial::Ite => [0.55, 0.50, 0.35, 1.0],
            VoxelMaterial::Limestone => [0.75, 0.73, 0.68, 1.0],
            VoxelMaterial::Slate => [0.30, 0.32, 0.35, 1.0],
            VoxelMaterial::Sand => [0.84, 0.76, 0.55, 1.0],
            // Near-black, and deliberately unlike every other rock: a player who
            // reaches it should be able to see that this one is different before
            // spending a charge on it.
            VoxelMaterial::Bedrock => [0.10, 0.10, 0.12, 1.0],
        }
    }

    /// Budget a blast must spend to remove one voxel of this material.
    ///
    /// `None` means indestructible: no charge removes it, however large. Only
    /// [`Bedrock`](VoxelMaterial::Bedrock) is, and it is the world's floor
    /// rather than a durability tier — every other material yields to a big
    /// enough charge, and to a small enough charge repeated.
    ///
    /// Air costs nothing because there is nothing there to take.
    pub fn toughness(&self) -> Option<f32> {
        match self {
            VoxelMaterial::Air => Some(0.0),
            VoxelMaterial::Grass => Some(1.0),
            VoxelMaterial::Sand => Some(1.0),
            VoxelMaterial::Dirt => Some(2.0),
            VoxelMaterial::Ite => Some(3.0),
            VoxelMaterial::Limestone => Some(4.0),
            VoxelMaterial::Rock => Some(5.0),
            VoxelMaterial::Slate => Some(8.0),
            VoxelMaterial::Bedrock => None,
        }
    }

    /// Whether no charge can remove this material.
    pub fn is_indestructible(&self) -> bool {
        self.toughness().is_none()
    }

    /// Check if this material is solid (should be collided with).
    pub fn is_solid(&self) -> bool {
        !matches!(self, VoxelMaterial::Air)
    }
}

/// A single voxel with density and material.
/// Density is used for smooth surface extraction (Marching Cubes).
/// - density > 0.0: inside solid
/// - density < 0.0: outside solid (air)
/// - density = 0.0: exactly on the surface
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Voxel {
    /// Signed distance from surface. Positive = inside, negative = outside.
    pub density: f32,
    /// Material type of this voxel.
    pub material: VoxelMaterial,
}

impl Voxel {
    /// Create an air voxel: empty space, a full voxel away from any surface.
    ///
    /// The saturated `-1.0` is right for *empty* space and wrong for *freshly
    /// cut* space, and the difference is not cosmetic. Density is a signed
    /// distance to the surface in voxels ([`Voxel::density`]), so a sample
    /// beside a fresh cut carries how far the cut is from it — a fraction —
    /// while this carries "nothing near". Stamping this into a carved region
    /// puts a step in the field where the geometry is smooth, and marching
    /// cubes reads the gradient of that step as a slope.
    ///
    /// So this constructor is not the thing to change. Callers that remove
    /// solid must compute the distance to the surface they are cutting with —
    /// `csg::carve_density` — and pass it in, as `Chunk::carve_sphere` and
    /// `csg::carve_with_sdf` both do. Air that was always air is what this is
    /// for, and for that the value is correct.
    pub fn air() -> Self {
        Self {
            density: -1.0,
            material: VoxelMaterial::Air,
        }
    }

    /// Create a solid voxel of the given material.
    pub fn solid(material: VoxelMaterial) -> Self {
        Self {
            density: 1.0,
            material,
        }
    }

    /// Whether this voxel is inside the surface and made of something.
    pub fn is_solid(&self) -> bool {
        self.density > 0.0 && self.material.is_solid()
    }
}

impl Default for Voxel {
    fn default() -> Self {
        Self::air()
    }
}
