//! Core trait and context types for the spawnable object library.

use specs::{Entity, World};

use crate::core::error::EngineResult;
use crate::rendering::material::{MaterialId, MaterialManagerBuilder};
use crate::resources::textures::TextureManager;

/// Resources available during the material-creation phase.
///
/// Passed to [`Spawnable::create_materials`] so that each spawnable can
/// register its materials without needing to know about the full init flow.
pub struct MaterialCtx<'a> {
    pub textures: &'a TextureManager,
    pub materials: &'a mut MaterialManagerBuilder,
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
