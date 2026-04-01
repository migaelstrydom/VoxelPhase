use std::path::Path;

use voxel_phase::app::App;
use voxel_phase::core::error::EngineResult;

const DEFAULT_LEVEL: &str = "levels/test_arena.level.ron";

fn setup_logging() {
    env_logger::Builder::new()
        .filter_level(log::LevelFilter::Info)
        .filter_module("ash", log::LevelFilter::Info) // Reduce Vulkan noise
        .init();
}

fn main() -> EngineResult<()> {
    setup_logging();

    let level_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| DEFAULT_LEVEL.to_string());
    log::info!("Starting Voxel Phase — loading {}", level_path);

    match App::new(1200, 800, "Voxel Phase", Path::new(&level_path)) {
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
