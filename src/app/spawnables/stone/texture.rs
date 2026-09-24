//! What old stone's texture is baked from, and how big it is laid on.

use crate::app::spawnables::shared::models::SurfaceUvs;
use crate::app::spawnables::MaterialCtx;
use crate::core::error::EngineResult;
use crate::rendering::material::MaterialId;
use crate::rendering::pattern::{self, Pattern, Spread};
use crate::rendering::substance::Substance;

/// How much surface one tile of texture covers on a stone of
/// [`REFERENCE_THICKNESS`], in metres.
///
/// A thicker stone gets bigger tiles, but only as the square root of its size:
/// cracks and pits are drawn larger on a bigger block, as a toy-scale world
/// wants, but not so much larger that a five-metre block shows no more of
/// them than a half-metre one — which is what gives the scale away.
const REFERENCE_TILE: f32 = 0.8;
const REFERENCE_THICKNESS: f32 = 0.5;

/// What a weathered stone texture is baked from.
///
/// Public so the visual bench can bake exactly what a level does.
pub struct StoneTexture {
    pub pattern: &'static Pattern,
    /// Stone from one quarry should look like stone from one quarry: one seed
    /// for every block of a structure.
    pub seed: u32,
    /// Edge length of one tile, in texels.
    pub tile_size: u32,
    /// How many tiles one baked texture holds across. With the tile scaled
    /// to the stone (see [`REFERENCE_TILE`]), two across is several
    /// blocks of surface before anything comes round again — and every block
    /// reads its own part of it, since each is textured from where it stands.
    pub tiles: f32,
}

impl StoneTexture {
    pub const WEATHERED: Self = Self {
        pattern: &pattern::WEATHERED_STONE,
        seed: 77,
        tile_size: 256,
        tiles: 2.0,
    };

    /// The same texture cut from another part of the quarry.
    pub const fn with_seed(self, seed: u32) -> Self {
        Self { seed, ..self }
    }

    pub fn spread(&self) -> Spread {
        Spread::covering(self.tiles)
    }

    /// A material of `substance` wearing this texture.
    pub fn material(
        &self,
        ctx: &mut MaterialCtx,
        substance: &Substance,
    ) -> EngineResult<MaterialId> {
        ctx.patterned_spread(
            substance,
            self.pattern,
            self.seed,
            self.tile_size,
            self.spread(),
        )
    }

    /// How the texture is laid on a stone `thickness` metres thick.
    pub fn uvs(&self, thickness: f32) -> SurfaceUvs {
        let tile = REFERENCE_TILE * (thickness / REFERENCE_THICKNESS).sqrt();
        SurfaceUvs::PerMetre(1.0 / (tile * self.tiles))
    }
}
