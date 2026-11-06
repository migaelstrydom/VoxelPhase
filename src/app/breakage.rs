//! The break pipeline on its own, for tools and tests that step a world by
//! hand: the systems that crack, cleave, split and draw breakable bodies, in
//! the order the dispatcher runs them.
//!
//! ```text
//!   GlassCrackSystem ─┐
//!   SolidCleaveSystem ┴▶ FractureSystem ──▶ SheetShareSystem ─┐
//!                                           SolidShareSystem  │
//!                                           AnchorShareSystem ├▶ PartDrawSystem
//!                                           PlantShareSystem ─┘
//! ```

use specs::{RunNow, World, WorldExt};

use crate::cleave::{BrittleSolid, SolidCleaveSystem, SolidShareSystem};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, TerrainAnchored, Velocity,
};
use crate::debug::DebugLog;
use crate::foliage::Foliage;
use crate::fracture::{CompoundFracture, Debris, FractureSystem, PieceSplits};
use crate::glass::{BrittleSheet, GlassCrackSystem, SheetShareSystem};
use crate::model::PartLooks;
use crate::physics::PhysicsImpulseQueue;
use crate::plants::{PlantPiece, PlantShareSystem};
use crate::systems::{AnchorShareSystem, PartDrawSystem};

/// Register what the break pipeline reads and writes, for a world built by
/// hand rather than by `WorldBuilder`.
pub fn register(world: &mut World) {
    world.register::<Position>();
    world.register::<Velocity>();
    world.register::<Orientation>();
    world.register::<RigidBodyComponent>();
    world.register::<ModelInstance>();
    world.register::<Renderable>();
    world.register::<PartLooks>();
    world.register::<CompoundFracture>();
    world.register::<BrittleSheet>();
    world.register::<BrittleSolid>();
    world.register::<Debris>();
    world.register::<TerrainAnchored>();
    world.register::<PlantPiece>();
    world.register::<Foliage>();
    world.insert(PieceSplits::default());
    world.insert(PhysicsImpulseQueue::default());
    world.insert(DebugLog::default());
}

/// One frame of the break pipeline, after the physics step.
pub fn run(world: &mut World) {
    GlassCrackSystem.run_now(world);
    SolidCleaveSystem.run_now(world);
    FractureSystem.run_now(world);
    SheetShareSystem.run_now(world);
    SolidShareSystem.run_now(world);
    AnchorShareSystem.run_now(world);
    PlantShareSystem.run_now(world);
    PartDrawSystem.run_now(world);
    world.maintain();
}
