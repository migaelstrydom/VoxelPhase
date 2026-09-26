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

use crate::collision::SurfaceId;
use crate::physics::StaticSurface;

/// Toughness that maps to half hardness. Sits just above the mid-range
/// materials so that the destructible ladder spends most of its span on the
/// soft half, where the visible difference between chalk and stone is, rather
/// than crowding against the indestructible ceiling.
const HARDNESS_MIDPOINT: f32 = 4.0;

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

    /// How hard this material is to break, on a 0-to-1 scale: 0 is the softest
    /// thing that can exist and 1 is unbreakable.
    ///
    /// This is what the renderer shades from, so that a surface's finish is a
    /// readout of the number destruction actually spends against — soft
    /// materials read chalky, hard ones read dense and glinty, and a player can
    /// tell before firing whether a wall will go.
    ///
    /// `t / (t + HARDNESS_MIDPOINT)` rather than a linear rescale, for two
    /// reasons. It is bounded, so a level that authors a toughness far outside
    /// the current range still lands somewhere sensible instead of clipping.
    /// And [`Bedrock`](VoxelMaterial::Bedrock) is the curve's limit as toughness
    /// grows without bound rather than a special case bolted on: infinitely
    /// tough is exactly what indestructible means.
    ///
    /// Deliberately compressive at the top. Toughness is a gameplay-tuned
    /// number rather than a measured one, so the mapping stays coarse and
    /// monotonic: only large toughness differences produce visible ones, and
    /// retuning a material's difficulty cannot quietly restyle the level.
    pub fn hardness(&self) -> f32 {
        match self.toughness() {
            Some(toughness) => toughness / (toughness + HARDNESS_MIDPOINT),
            None => 1.0,
        }
    }

    /// Check if this material is solid (should be collided with).
    pub fn is_solid(&self) -> bool {
        !matches!(self, VoxelMaterial::Air)
    }

    /// What a body touching this material meets: its grip, its bounce, and
    /// whether it gives way (see `physics::static_surface`).
    ///
    /// Loose ground yields, so its own friction is the contact's whatever
    /// lies on it — a block of ice sits on grass about as firmly as a crate
    /// does. Rock is hard and meets a collider as another collider would.
    ///
    /// Starting values, tuned by feel rather than measured: published
    /// coefficients for soil and turf vary with moisture more than between
    /// materials.
    pub fn surface(&self) -> Option<StaticSurface> {
        match self {
            VoxelMaterial::Air => None,
            VoxelMaterial::Grass => Some(StaticSurface::yielding(0.45, 0.2)),
            VoxelMaterial::Dirt => Some(StaticSurface::yielding(0.55, 0.15)),
            VoxelMaterial::Sand => Some(StaticSurface::yielding(0.5, 0.1)),
            VoxelMaterial::Ite => Some(StaticSurface::rigid(0.6, 0.3)),
            VoxelMaterial::Limestone => Some(StaticSurface::rigid(0.6, 0.3)),
            VoxelMaterial::Rock => Some(StaticSurface::rigid(0.7, 0.3)),
            VoxelMaterial::Slate => Some(StaticSurface::rigid(0.5, 0.3)),
            VoxelMaterial::Bedrock => Some(StaticSurface::rigid(0.7, 0.3)),
        }
    }

    /// The id a triangle of this material carries into the physics engine.
    /// Air is [`SurfaceId::UNSPECIFIED`], which no surface triangle is made of.
    pub fn surface_id(&self) -> SurfaceId {
        SurfaceId(*self as u8)
    }

    /// The material a [`surface_id`](Self::surface_id) came from, or `None`
    /// for an id terrain never issued.
    pub fn from_surface_id(id: SurfaceId) -> Option<Self> {
        match id.0 {
            0 => Some(VoxelMaterial::Air),
            1 => Some(VoxelMaterial::Rock),
            2 => Some(VoxelMaterial::Grass),
            3 => Some(VoxelMaterial::Dirt),
            4 => Some(VoxelMaterial::Ite),
            5 => Some(VoxelMaterial::Limestone),
            6 => Some(VoxelMaterial::Slate),
            7 => Some(VoxelMaterial::Sand),
            8 => Some(VoxelMaterial::Bedrock),
            _ => None,
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

#[cfg(test)]
mod tests {
    use super::*;

    const DESTRUCTIBLE: [VoxelMaterial; 7] = [
        VoxelMaterial::Grass,
        VoxelMaterial::Sand,
        VoxelMaterial::Dirt,
        VoxelMaterial::Ite,
        VoxelMaterial::Limestone,
        VoxelMaterial::Rock,
        VoxelMaterial::Slate,
    ];

    /// Every surface a triangle can carry must come back as the material it
    /// was cut from, or the physics would read one rock as another.
    #[test]
    fn surface_ids_round_trip() {
        for material in DESTRUCTIBLE
            .into_iter()
            .chain([VoxelMaterial::Air, VoxelMaterial::Bedrock])
        {
            assert_eq!(
                VoxelMaterial::from_surface_id(material.surface_id()),
                Some(material)
            );
        }
        assert_eq!(VoxelMaterial::Air.surface_id(), SurfaceId::UNSPECIFIED);
        assert!(VoxelMaterial::Air.surface().is_none());
    }

    /// The whole point of shading from hardness is that a harder surface never
    /// looks softer than an easier one. Anything non-monotonic would make the
    /// finish an unreliable readout, which is worse than not having it.
    #[test]
    fn hardness_never_falls_as_toughness_rises() {
        let mut ranked = DESTRUCTIBLE;
        ranked.sort_by(|a, b| {
            a.toughness()
                .unwrap()
                .partial_cmp(&b.toughness().unwrap())
                .unwrap()
        });

        for pair in ranked.windows(2) {
            assert!(
                pair[0].hardness() <= pair[1].hardness(),
                "{:?} ({}) reads harder than {:?} ({})",
                pair[0],
                pair[0].hardness(),
                pair[1],
                pair[1].hardness()
            );
        }
    }

    /// Hardness is consumed as a 0-to-1 shading parameter, and a value outside
    /// that range would drive roughness past its own limits.
    #[test]
    fn hardness_stays_within_the_unit_range() {
        for material in DESTRUCTIBLE {
            let hardness = material.hardness();
            assert!(
                (0.0..=1.0).contains(&hardness),
                "{material:?} has hardness {hardness}"
            );
        }
        assert_eq!(VoxelMaterial::Air.hardness(), 0.0);
    }

    /// Indestructible is the limit of the same curve, not a separate branch in
    /// disguise: bedrock must sit above every material that can be broken.
    #[test]
    fn nothing_destructible_reads_as_hard_as_bedrock() {
        let hardest = DESTRUCTIBLE
            .iter()
            .map(|m| m.hardness())
            .fold(0.0f32, f32::max);

        assert!(
            hardest < VoxelMaterial::Bedrock.hardness(),
            "the toughest destructible material reads {hardest}, at the bedrock ceiling"
        );
    }

    /// Coarse and monotonic is the requirement, but a ladder whose rungs are
    /// indistinguishable would be a readout the player cannot read. Grass and
    /// rock are three tiers apart and must look clearly different.
    #[test]
    fn the_ends_of_the_destructible_ladder_are_far_apart() {
        let spread = VoxelMaterial::Rock.hardness() - VoxelMaterial::Grass.hardness();
        assert!(spread > 0.3, "grass to rock spans only {spread}");
    }
}
