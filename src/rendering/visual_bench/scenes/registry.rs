//! The list of scenes the CLI can run.
//!
//! One place to register a new scene, mirroring how the physics bench lists its
//! scenarios.

use crate::rendering::visual_bench::scene::VisualScene;
use crate::rendering::visual_bench::scenes::{MaterialGrid, Palette, Props, SunSweep};

/// Every scene, in the order `--list` prints them.
pub fn all_scenes() -> Vec<Box<dyn VisualScene>> {
    vec![
        Box::new(MaterialGrid),
        Box::new(Palette),
        Box::new(Props),
        Box::new(SunSweep),
    ]
}

/// Look a scene up by name.
pub fn find_scene(name: &str) -> Option<Box<dyn VisualScene>> {
    all_scenes().into_iter().find(|scene| scene.name() == name)
}
