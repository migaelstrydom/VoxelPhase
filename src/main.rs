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

    let mut app = App::new(1920, 1080, "RustDude App")?;
    app.run(|renderer, world| {
        if let Err(e) = renderer.render_frame(world) {
            log::error!("Error during render_frame: {}", e);
            // Potentially handle exit here or let the app continue
        }
    })?;

    Ok(())
}
