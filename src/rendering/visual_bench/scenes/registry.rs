//! The list of scenes the CLI can run.
//!
//! One place to register a new scene, mirroring how the physics bench lists its
//! scenarios.

use crate::rendering::visual_bench::scene::VisualScene;
use crate::rendering::visual_bench::scenes::{
    Craters, GradeSweep, MaterialGrid, Palette, PhysicsFinish, Props, ShadowTuning, Shadows,
    SunSweep, TerrainAo, TerrainForms,
};

/// Every scene, in the order `--list` prints them.
pub fn all_scenes() -> Vec<Box<dyn VisualScene>> {
    vec![
        Box::new(Craters),
        Box::new(GradeSweep),
        Box::new(MaterialGrid),
        Box::new(Palette),
        Box::new(PhysicsFinish),
        Box::new(Props),
        Box::new(ShadowTuning),
        Box::new(Shadows),
        Box::new(SunSweep),
        Box::new(TerrainAo),
        Box::new(TerrainForms),
    ]
}

/// Look a scene up by name.
pub fn find_scene(name: &str) -> Option<Box<dyn VisualScene>> {
    all_scenes().into_iter().find(|scene| scene.name() == name)
}
