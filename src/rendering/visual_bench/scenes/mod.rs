//! The scene library, and the registry the CLI looks names up in.

pub mod material_grid;
pub mod palette;
pub mod props;
pub mod registry;
pub mod sun_sweep;

pub use material_grid::MaterialGrid;
pub use palette::Palette;
pub use props::Props;
pub use registry::{all_scenes, find_scene};
pub use sun_sweep::SunSweep;
