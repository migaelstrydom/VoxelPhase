//! The list of scenes the CLI can run.
//!
//! One place to register a new scene, mirroring how the physics bench lists its
//! scenarios.

use crate::rendering::visual_bench::scene::VisualScene;
use crate::rendering::visual_bench::scenes::{
    Craters, Critter, CritterWalk, GradeSweep, Ice, IceFracture, MaterialGrid, Palette, Peeper,
    PhysicsFinish, PropGrain, Props, ShadowTuning, Shadows, SunSweep, TerrainAo, TerrainDetail,
    TerrainFinish, TerrainForms,
};

/// Every scene, in the order `--list` prints them.
pub fn all_scenes() -> Vec<Box<dyn VisualScene>> {
    vec![
        Box::new(Craters),
        Box::new(Critter),
        Box::new(CritterWalk),
        Box::new(GradeSweep),
        Box::new(Ice),
        Box::new(IceFracture),
        Box::new(MaterialGrid),
        Box::new(Palette),
        Box::new(Peeper),
        Box::new(PhysicsFinish),
        Box::new(Props),
        Box::new(ShadowTuning),
        Box::new(Shadows),
        Box::new(SunSweep),
        Box::new(TerrainAo),
        Box::new(PropGrain),
        Box::new(TerrainDetail),
        Box::new(TerrainFinish),
        Box::new(TerrainForms),
    ]
}

/// Look a scene up by name.
pub fn find_scene(name: &str) -> Option<Box<dyn VisualScene>> {
    all_scenes().into_iter().find(|scene| scene.name() == name)
}
