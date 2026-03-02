mod app;
mod biped;
mod camera;
mod collision;
mod components;
mod core;
mod debug;
mod explosion;
mod geometry;
mod input;
mod level;
mod model;
mod particles;
mod physics;
mod player;
mod projectile;
mod rendering;
mod resources;
mod sensing;
mod skeleton;
mod systems;
mod terrain;
mod time;
mod utils;
mod world;

use std::path::Path;

use crate::app::App;
use crate::core::error::EngineResult;

const DEFAULT_LEVEL: &str = "levels/test_arena.level.ron";

fn setup_logging() {
    env_logger::Builder::new()
        .filter_level(log::LevelFilter::Info)
        .filter_module("ash", log::LevelFilter::Info) // Reduce Vulkan noise
        .init();
}

fn main() -> EngineResult<()> {
    setup_logging();

    let level_path = std::env::args().nth(1).unwrap_or_else(|| DEFAULT_LEVEL.to_string());
    log::info!("Starting RustDude — loading {}", level_path);

    match App::new(1200, 800, "RustDude Engine", Path::new(&level_path)) {
        Ok(mut app) => {
            if let Err(e) = app.run() {
                eprintln!("Application error: {}", e);
            }
        }
        Err(e) => {
            eprintln!("Error creating App: {}", e);
        }
    }

    Ok(())
}
