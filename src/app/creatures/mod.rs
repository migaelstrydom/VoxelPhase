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
//!   Peeper         stilts                sees far, pecks, dies easily
//!   Stalker        stilts                slow, tall, hard to kill, long beak
//!   EggWalker      stilts                quick and weak; comes in clutches
//!   FishWalker     stilts                walks on into water after its quarry
//!   BellCarrier    stilts                cannot be hurt; heavy, so it is walked
//!                                        off ledges and into deep water
//!   Piper          does not move         lobs real bombs at whoever it sees
//!   FlyingFish     flying                holds a height, stoops on the open
//!   Mussel         does not move         snaps shut on whoever steps in
//!   Fish, Turtle   swimming              harmless; they keep clear
//!   HeartCritter   hopping               a collectable that runs away
//! ```
//!
//! The five on stilts share one rig, body and spawn (`stilt_walker`,
//! `animation::stilts`): each is a set of proportions and a dress, the
//! sculpted pieces the rig's joints carry. Every creature is sculpted in
//! clay (`app::sculpt`) and painted in the level's one bestiary palette
//! (`palette`), its likeness made once a run (`likeness`); the ones that are
//! not on stilts are a `PosedModel` their behaviour poses. `visual_bench
//! bestiary` draws every creature, and each level in `levels/bestiary/`
//! stands one of them on a lawn.

mod anatomy;
mod bell_carrier;
mod bird_anatomy;
mod egg_walker;
mod fish;
mod fish_anatomy;
mod fish_walker;
mod flying_fish;
mod heart_critter;
mod in_water;
mod likeness;
mod mussel;
mod palette;
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
pub use palette::materials as creature_materials;
pub use peeper::PeeperDef;
pub use piper::PiperDef;
pub use roller::RollerDef;
pub use sculpted::Exhibit;
pub use stalker::StalkerDef;
pub use turtle::TurtleDef;
