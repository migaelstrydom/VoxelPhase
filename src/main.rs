mod app;
mod camera;
mod collision;
mod components;
mod core;
mod debug;
mod explosion;
mod geometry;
mod input;
mod model;
mod particles;
mod player;
mod projectile;
mod rendering;
mod resources;
mod skeleton;
mod systems;
mod terrain;
mod time;
mod world;

use crate::app::App;
use crate::core::error::EngineResult;

fn setup_logging() {
    env_logger::Builder::new()
        .filter_level(log::LevelFilter::Debug)
        .filter_module("ash", log::LevelFilter::Info) // Reduce Vulkan noise
        .init();
}

fn main() -> EngineResult<()> {
    setup_logging();
    log::info!("Starting RustDude");

    match App::new(1200, 800, "RustDude Engine") {
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
