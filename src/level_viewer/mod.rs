//! Headless render of a level as authored, through the real pipeline, to a PNG contact sheet.

mod shots;
mod viewer;

pub use shots::{custom_shot, standard_shots, ViewerShot};
pub use viewer::LevelViewer;
