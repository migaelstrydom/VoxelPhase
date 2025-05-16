use std::error::Error;

mod app;
mod components;
mod core;
mod rendering;
mod systems;

use crate::app::App;

fn setup_logging() {
    env_logger::Builder::new()
        .filter_level(log::LevelFilter::Info)
        .init();
}

fn main() -> Result<(), Box<dyn Error>> {
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
