//! Voxel types and materials.

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
}

impl VoxelMaterial {
    /// Get a color for this material (for simple vertex coloring).
    pub fn color(&self) -> [f32; 4] {
        match self {
            VoxelMaterial::Air => [0.0, 0.0, 0.0, 0.0],
            VoxelMaterial::Rock => [0.5, 0.5, 0.5, 1.0],
            VoxelMaterial::Grass => [0.3, 0.7, 0.2, 1.0],
            VoxelMaterial::Dirt => [0.5, 0.3, 0.1, 1.0],
            VoxelMaterial::Ite => [0.55, 0.50, 0.35, 1.0],
            VoxelMaterial::Limestone => [0.75, 0.73, 0.68, 1.0],
            VoxelMaterial::Slate => [0.30, 0.32, 0.35, 1.0],
            VoxelMaterial::Sand => [0.84, 0.76, 0.55, 1.0],
        }
    }

    /// Base health for this material before depth modifiers.
    pub fn base_health(&self) -> u8 {
        match self {
            VoxelMaterial::Air => 0,
            VoxelMaterial::Grass => 1,
            VoxelMaterial::Dirt => 2,
            VoxelMaterial::Ite => 3,
            VoxelMaterial::Rock => 5,
            VoxelMaterial::Limestone => 4,
            VoxelMaterial::Slate => 8,
            VoxelMaterial::Sand => 1,
        }
    }
}

/// Health value indicating this voxel cannot be destroyed.
pub const INDESTRUCTIBLE: u8 = u8::MAX;

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
    /// Remaining hit points. 0 = already destroyed (air), 255 = indestructible.
    pub health: u8,
}

impl Voxel {
    /// Create an air voxel.
    pub fn air() -> Self {
        Self {
            density: -1.0,
            material: VoxelMaterial::Air,
            health: 0,
        }
    }

    /// Create a solid voxel with explicit health.
    pub fn solid(material: VoxelMaterial, health: u8) -> Self {
        Self {
            density: 1.0,
            material,
            health,
        }
    }

    /// Apply damage, returning the updated voxel. Indestructible voxels are unaffected.
    /// If health reaches zero the voxel becomes air.
    pub fn apply_damage(mut self, damage: u8) -> Self {
        if self.health == INDESTRUCTIBLE || self.health == 0 {
            return self;
        }
        if damage >= self.health {
            return Self::air();
        }
        self.health -= damage;
        self
    }
}

impl Default for Voxel {
    fn default() -> Self {
        Self::air()
    }
}

/// Controls how voxel health scales with depth below the terrain surface.
pub struct DurabilityConfig {
    /// Health added per unit of depth below the surface.
    /// Surface voxels always start at 1 HP, so at depth `d` the health is
    /// `1 + (d * health_per_depth) as u8`, capped at 254.
    pub health_per_depth: f32,

    /// Thickness (in world units) of the indestructible bedrock layer at the
    /// bottom of the terrain bounds.
    pub bedrock_thickness: f32,
}

impl Default for DurabilityConfig {
    fn default() -> Self {
        Self {
            health_per_depth: 3.0,
            bedrock_thickness: 2.0,
        }
    }
}

impl DurabilityConfig {
    /// Compute health for a voxel at the given y coordinate.
    ///
    /// `surface_y` is the terrain surface height at this column.
    /// `floor_y` is the bottom of the world bounds.
    pub fn health_at(&self, y: f32, surface_y: f32, floor_y: f32) -> u8 {
        if y <= floor_y + self.bedrock_thickness {
            return INDESTRUCTIBLE;
        }
        let depth = (surface_y - y).max(0.0);
        let hp = 1.0 + depth * self.health_per_depth;
        (hp as u8).min(INDESTRUCTIBLE - 1)
    }
}

#[cfg(test)]
impl Voxel {
    /// Check if this voxel is considered solid (inside the surface).
    pub(crate) fn is_solid(&self) -> bool {
        self.density > 0.0 && self.material.is_solid()
    }
}

#[cfg(test)]
impl VoxelMaterial {
    /// Check if this material is solid (should be collided with).
    pub fn is_solid(&self) -> bool {
        !matches!(self, VoxelMaterial::Air)
    }
}
