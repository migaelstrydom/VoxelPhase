//! Voxel types and materials.

/// Material type for a voxel, determining its properties and appearance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(u8)]
#[allow(dead_code)] // Some variants (Sand, Water) are part of API but not yet used in generation
pub enum VoxelMaterial {
    #[default]
    Air = 0,
    Rock = 1,
    Grass = 2,
    Dirt = 3,
    Sand = 4,
    Water = 5,
}

impl VoxelMaterial {
    /// Check if this material is solid (should be collided with).
    pub fn is_solid(&self) -> bool {
        !matches!(self, VoxelMaterial::Air | VoxelMaterial::Water)
    }

    /// Check if this material is visible (should be rendered).
    pub fn is_visible(&self) -> bool {
        !matches!(self, VoxelMaterial::Air)
    }

    /// Get a color for this material (for simple vertex coloring).
    pub fn color(&self) -> [f32; 4] {
        match self {
            VoxelMaterial::Air => [0.0, 0.0, 0.0, 0.0],
            VoxelMaterial::Rock => [0.5, 0.5, 0.5, 1.0],
            VoxelMaterial::Grass => [0.3, 0.7, 0.2, 1.0],
            VoxelMaterial::Dirt => [0.5, 0.3, 0.1, 1.0],
            VoxelMaterial::Sand => [0.9, 0.8, 0.5, 1.0],
            VoxelMaterial::Water => [0.2, 0.4, 0.8, 0.7],
        }
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
    /// Create a new voxel with the given density and material.
    pub fn new(density: f32, material: VoxelMaterial) -> Self {
        Self { density, material }
    }

    /// Create an air voxel.
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
}

impl Default for Voxel {
    fn default() -> Self {
        Self::air()
    }
}

#[cfg(test)]
impl Voxel {
    /// Check if this voxel is considered solid (inside the surface).
    pub(crate) fn is_solid(&self) -> bool {
        self.density > 0.0 && self.material.is_solid()
    }

    /// Check if this voxel should be rendered.
    fn is_visible(&self) -> bool {
        self.density > 0.0 && self.material.is_visible()
    }
}
