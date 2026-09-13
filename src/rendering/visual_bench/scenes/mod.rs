//! The scene library, and the registry the CLI looks names up in.

pub mod craters;
pub mod critter;
pub mod critter_walk;
pub mod grade_sweep;
pub mod level_props;
pub mod material_grid;
pub mod palette;
pub mod physics_finish;
pub mod prop_grain;
pub mod props;
pub mod registry;
pub mod shadow_tuning;
pub mod shadows;
pub mod sun_sweep;
pub mod terrain_ao;
pub mod terrain_detail;
pub mod terrain_finish;
pub mod terrain_forms;
pub mod voxel_terrain;

pub use craters::Craters;
pub use critter::Critter;
pub use critter_walk::CritterWalk;
pub use grade_sweep::GradeSweep;
pub use material_grid::MaterialGrid;
pub use palette::Palette;
pub use physics_finish::PhysicsFinish;
pub use prop_grain::PropGrain;
pub use props::Props;
pub use registry::{all_scenes, find_scene};
pub use shadow_tuning::ShadowTuning;
pub use shadows::Shadows;
pub use sun_sweep::SunSweep;
pub use terrain_ao::TerrainAo;
pub use terrain_detail::TerrainDetail;
pub use terrain_finish::TerrainFinish;
pub use terrain_forms::TerrainForms;
