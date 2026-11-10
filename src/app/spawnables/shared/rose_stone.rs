//! The rose stone a garden's fountains are carved from, and the gilt on it.

use crate::app::sculpt::{ContactShade, Mottle};
use crate::rendering::colour::Colour;
use crate::rendering::grain::GrainSpec;
use crate::rendering::material::{Material, MaterialId, SurfaceFinish};
use crate::rendering::physical_finish::PhysicalSurface;

/// Carved stone: heavy, rough, dead to a knock.
pub const STONE: PhysicalSurface = PhysicalSurface {
    restitution: 0.1,
    friction: 0.7,
    density: 2200.0,
};

/// The stone's finish: matte, with a fine tooth.
const CARVED: SurfaceFinish = SurfaceFinish {
    roughness: 0.8,
    metallic: 0.0,
};
const CARVED_GRAIN: GrainSpec = GrainSpec::STONE.with_scale(1.2).with_strength(0.3);

/// Gilding. Half metallic reads as gold; fully metallic goes olive against
/// a blue sky.
const GILT: SurfaceFinish = SurfaceFinish {
    roughness: 0.35,
    metallic: 0.5,
};

/// How many materials [`materials`] makes.
pub const MATERIAL_COUNT: usize = 2;

/// The stone's materials, in the order [`Looks::of`] reads them: the carved
/// stone and the gilt.
pub fn materials() -> Vec<Material> {
    let plain = || Material::coloured(Colour::WHITE);
    vec![
        plain().with_finish(CARVED).with_grain(CARVED_GRAIN),
        plain().with_finish(GILT),
    ]
}

/// What a sculpted part of a fountain is painted with.
#[derive(Clone, Copy)]
pub struct Looks {
    stone: MaterialId,
    gilt: MaterialId,
}

impl Looks {
    /// From the ids [`materials`] were registered under, in order.
    pub fn of(materials: &[MaterialId]) -> Self {
        Self {
            stone: materials[0],
            gilt: materials[1],
        }
    }

    /// The stone itself: a dusty rose.
    pub fn rose(self) -> (Colour, MaterialId) {
        (Colour::new(0.46, 0.14, 0.19, 1.0), self.stone)
    }

    /// A paler rose, for a part to stand out from its neighbours.
    pub fn blush(self) -> (Colour, MaterialId) {
        (Colour::new(0.60, 0.27, 0.30, 1.0), self.stone)
    }

    /// A blue glaze over the stone.
    pub fn lapis(self) -> (Colour, MaterialId) {
        (Colour::new(0.03, 0.09, 0.36, 1.0), self.stone)
    }

    pub fn gold(self) -> (Colour, MaterialId) {
        (Colour::new(1.0, 0.62, 0.18, 1.0), self.gilt)
    }
}

/// The unevenness of the stone's colour on a piece `size` metres across.
pub fn mottle(size: f32) -> Mottle {
    Mottle {
        frequency: 1.4 / size,
        value: 0.35,
        warmth: 0.18,
        seed: 23,
    }
}

/// The shade where one carved part meets another, on a piece `size` metres
/// across.
pub fn contact_shade(size: f32) -> ContactShade {
    ContactShade {
        reach: 0.12 * size,
        depth: 0.6,
        floor: 0.35,
    }
}
