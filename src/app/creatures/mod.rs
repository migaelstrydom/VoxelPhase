//! Creature definitions — the bestiary.
//!
//! Creatures implement [`Spawnable`](crate::app::spawnables::Spawnable), the
//! same trait props use. There is no separate `CreatureDef`: the contract a
//! creature needs is exactly the contract a spawnable already has — declare
//! materials, create them during init, build entities — and a second trait with
//! an identical shape would be duplication rather than abstraction.
//!
//! What separates a creature from a prop is not its trait but what it attaches:
//! `Brain`, `Perception`, `Health`, and a locomotion component. Everything the
//! spawnables README documents about geometry, textures and physics applies
//! here unchanged.
//!
//! A def must contain no behaviour — no `update`, no per-creature system. It is
//! a prefab; what the creature *does* comes from the generic systems in
//! [`crate::creature`] that read the components it attached.

mod body_parts;
mod fish;
mod heart_critter;
mod in_water;
mod peeper;
mod roller;
mod turtle;

pub use fish::FishDef;
pub use heart_critter::HeartCritterDef;
pub use peeper::PeeperDef;
pub use roller::RollerDef;
pub use turtle::TurtleDef;
