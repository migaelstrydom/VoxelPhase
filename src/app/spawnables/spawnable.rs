//! Core trait and context types for the spawnable object library.

use specs::{Entity, World};

use crate::core::error::EngineResult;
use crate::rendering::material::{MaterialId, MaterialManagerBuilder};
use crate::rendering::pattern::{Pattern, Spread, TextureCache};
use crate::rendering::substance::Substance;
use crate::resources::textures::TextureManager;

/// Resources available during the material-creation phase.
///
/// Passed to [`Spawnable::create_materials`] so that each spawnable can
/// register its materials without needing to know about the full init flow.
pub struct MaterialCtx<'a> {
    pub textures: &'a TextureManager,
    pub materials: &'a mut MaterialManagerBuilder,

    /// Baked pattern textures, shared across every object in the level load.
    ///
    /// Lives here rather than per spawnable because the sharing that matters is
    /// *between* objects: a level's twenty menhirs asked for the same stone
    /// twenty times, and nothing was in a position to notice.
    pub textures_cache: &'a mut TextureCache,
}

impl MaterialCtx<'_> {
    /// Register a material whose texture is a pattern baked over a substance's
    /// palette, reusing an identical texture if one has already been made.
    ///
    /// The path a spawnable should take unless it needs something the pattern
    /// library cannot express. It ties the three libraries together in one
    /// call: the substance decides the palette, finish and grain, the pattern
    /// decides the markings, and the cache decides whether any work is needed.
    pub fn patterned(
        &mut self,
        substance: &Substance,
        pattern: &Pattern,
        seed: u32,
        size: u32,
    ) -> EngineResult<MaterialId> {
        self.patterned_spread(substance, pattern, seed, size, Spread::ONE)
    }

    /// The same, for a surface large enough that one tile of the pattern would
    /// visibly repeat across it.
    ///
    /// The caller is the only one who knows how many metres of surface a tile
    /// has to cover, so it is the caller that asks for the [`Spread`]. See
    /// [`Spread`] for what it costs.
    pub fn patterned_spread(
        &mut self,
        substance: &Substance,
        pattern: &Pattern,
        seed: u32,
        tile_size: u32,
        spread: Spread,
    ) -> EngineResult<MaterialId> {
        let texture = self.textures_cache.get_or_bake(
            self.textures,
            pattern,
            &substance.palette,
            seed,
            tile_size,
            spread,
        )?;

        Ok(self.materials.register(substance.material(texture)))
    }
}

/// A level object that can be spawned into the ECS world.
///
/// Each spawnable owns its serialized data and knows how to:
/// 1. Declare how many materials it needs (for pre-allocation).
/// 2. Create those materials during init.
/// 3. Spawn itself into the world using the pre-created materials.
pub trait Spawnable {
    /// How many materials this instance needs.
    fn material_count(&self) -> usize;

    /// Create materials during init (before `MaterialManagerBuilder::build()`).
    ///
    /// Must return exactly [`material_count`](Self::material_count) entries.
    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>>;

    /// Spawn the object into the ECS world.
    ///
    /// `materials` contains exactly the IDs returned by [`create_materials`](Self::create_materials).
    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity>;
}
