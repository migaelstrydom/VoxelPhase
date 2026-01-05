mod app;
mod components;
mod core;
mod rendering;
mod systems;
mod world;
mod resources;

use crate::app::App;
use crate::core::error::EngineResult;

fn setup_logging() {
    env_logger::Builder::new()
        .filter_level(log::LevelFilter::Info)
        .init();
}

fn main() -> EngineResult<()> {
    setup_logging();
    log::info!("Starting RustDude");

    match App::new(800, 600, "RustDude Engine") {
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
