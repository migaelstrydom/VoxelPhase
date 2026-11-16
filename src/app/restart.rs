//! Putting the room back as it began.
//!
//! ```text
//!   Level ──────────────────────────────────────────────────┐
//!   old World ── Renderer, ResourceManager, TextureManager ─┼─▶ GameWorld::assemble ─▶ new World
//!            └── InputState (held keys, mouse capture) ─────┘
//! ```
//!
//! The caller reads the level from its file again, so a room edited on disk
//! is the room the player gets back. Everything the level spawned is dropped
//! with the old world; the GPU context outlives it.

use specs::{Dispatcher, World, WorldExt};

use crate::core::error::{EngineError, EngineResult};
use crate::input::InputState;
use crate::level::Level;
use crate::rendering::renderer::Renderer;
use crate::resources::manager::ResourceManager;
use crate::resources::textures::TextureManager;
use crate::time::Time;

use super::game_world::GameWorld;
use super::spawners::spawn_camera;

/// Replace `world` and `dispatcher` with `level` as it begins, keeping the
/// renderer and the player's input.
///
/// An error leaves `world` without a scene, so it cannot be run on.
pub fn restart_level<'a, 'b>(
    world: &mut World,
    dispatcher: &mut Dispatcher<'a, 'b>,
    level: &Level,
    viewport: (u32, u32),
) -> EngineResult<()> {
    // Wait until the GPU has finished every frame in flight: they read
    // buffers and textures that dropping the old world lets go of.
    {
        let renderer = world.read_resource::<Renderer>();
        unsafe { renderer.vulkan_context.device().device_wait_idle() }
            .map_err(|e| EngineError::InvalidState(format!("Failed to wait for idle: {}", e)))?;
    }

    let mut old = std::mem::replace(world, World::new());
    let (mut renderer, resource_manager, texture_manager) = take_gpu_context(&mut old)?;
    let input = old.remove::<InputState>();
    drop(old);
    renderer.forget_scene();

    let GameWorld {
        world: mut fresh,
        dispatcher: fresh_dispatcher,
        player,
    } = GameWorld::assemble(renderer, resource_manager, texture_manager, level)?;
    spawn_camera(&mut fresh, player, viewport.0, viewport.1);
    if let Some(input) = input {
        fresh.insert(input);
    }
    // Loading took time the first frame should not be handed.
    fresh.write_resource::<Time>().restart();

    *world = fresh;
    *dispatcher = fresh_dispatcher;
    Ok(())
}

/// The GPU context out of a world that is about to be dropped.
fn take_gpu_context(
    world: &mut World,
) -> EngineResult<(Renderer, ResourceManager, TextureManager)> {
    let missing = |what: &str| EngineError::InvalidState(format!("World has no {}", what));
    Ok((
        world
            .remove::<Renderer>()
            .ok_or_else(|| missing("Renderer"))?,
        world
            .remove::<ResourceManager>()
            .ok_or_else(|| missing("ResourceManager"))?,
        world
            .remove::<TextureManager>()
            .ok_or_else(|| missing("TextureManager"))?,
    ))
}
