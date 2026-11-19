//! The list of scenes the CLI can run.
//!
//! One place to register a new scene, mirroring how the physics bench lists its
//! scenarios.

use crate::rendering::visual_bench::scene::VisualScene;
use crate::rendering::visual_bench::scenes::{
    Craters, Creature, Critter, CritterWalk, Fountain, Fruit, GradeSweep, HoverSled, Ice,
    IceFracture, Marble, MaterialGrid, Objective, Palette, PhysicsFinish, Plants, PropGrain, Props,
    ShadowTuning, Shadows, SunSweep, TerrainAo, TerrainDetail, TerrainFinish, TerrainForms, Turtle,
    WeatheredStone,
};

/// Every scene, in the order `--list` prints them.
pub fn all_scenes() -> Vec<Box<dyn VisualScene>> {
    vec![
        Box::new(Craters),
        Box::new(Critter),
        Box::new(CritterWalk),
        Box::new(Fruit),
        Box::new(Fountain),
        Box::new(Creature::Bestiary),
        Box::new(Creature::Peeper),
        Box::new(Creature::Stalker),
        Box::new(Creature::EggWalker),
        Box::new(Creature::FishWalker),
        Box::new(Creature::BellCarrier),
        Box::new(Creature::Roller),
        Box::new(Creature::Piper),
        Box::new(Creature::FlyingFish),
        Box::new(Creature::Mussel),
        Box::new(Marble),
        Box::new(GradeSweep),
        Box::new(HoverSled),
        Box::new(Ice),
        Box::new(IceFracture),
        Box::new(MaterialGrid),
        Box::new(Objective),
        Box::new(Palette),
        Box::new(PhysicsFinish),
        Box::new(Plants),
        Box::new(Props),
        Box::new(ShadowTuning),
        Box::new(Shadows),
        Box::new(SunSweep),
        Box::new(TerrainAo),
        Box::new(PropGrain),
        Box::new(TerrainDetail),
        Box::new(TerrainFinish),
        Box::new(TerrainForms),
        Box::new(Turtle),
        Box::new(WeatheredStone),
    ]
}

/// Look a scene up by name.
pub fn find_scene(name: &str) -> Option<Box<dyn VisualScene>> {
    all_scenes().into_iter().find(|scene| scene.name() == name)
}
