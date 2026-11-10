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
//!
//! ```text
//!   creature       moves by              its part in a level
//!   ─────────────  ────────────────────  ─────────────────────────────────────
//!   Roller         rolling               a thorned pod nothing can kill: outrun
//!                                        it or strand it in a crater
//!   Peeper         stilts, eye           sees far, pecks, dies easily
//!   Stalker        stilts, costume       slow, tall, hard to kill, long beak
//!   EggWalker      stilts, costume       quick and weak; comes in clutches
//!   FishWalker     stilts, costume       walks on into water after its quarry
//!   BellCarrier    stilts, costume       cannot be hurt; heavy, so it is walked
//!                                        off ledges and into deep water
//!   Piper          does not move         lobs real bombs at whoever it sees
//!   FlyingFish     flying                holds a height, stoops on the open
//!   Mussel         does not move         snaps shut on whoever steps in
//!   Fish, Turtle   swimming              harmless; they keep clear
//!   HeartCritter   hopping               a collectable that runs away
//! ```
//!
//! The four in costume share one rig, body and spawn with the peeper
//! (`stilt_walker`, `animation::stilts`): each is a set of proportions and a
//! sculpted costume worn in place of the peeper's eye. `visual_bench
//! bestiary` draws every creature, and `levels/bestiary.level.ron` stands
//! them all on one lawn.

mod bell_carrier;
mod egg_walker;
mod fish;
mod fish_walker;
mod flying_fish;
mod heart_critter;
mod in_water;
mod mussel;
mod peeper;
mod piper;
mod roller;
mod sculpted;
mod stalker;
mod stilt_walker;
mod turtle;

pub use bell_carrier::BellCarrierDef;
pub use egg_walker::EggWalkerDef;
pub use fish::FishDef;
pub use fish_walker::FishWalkerDef;
pub use flying_fish::FlyingFishDef;
pub use heart_critter::HeartCritterDef;
pub use mussel::MusselDef;
pub use peeper::PeeperDef;
pub use piper::PiperDef;
pub use roller::RollerDef;
pub use sculpted::sculpt_material;
pub use stalker::StalkerDef;
pub use turtle::TurtleDef;
