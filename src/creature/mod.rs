//! Creature AI runtime.
//!
//! A creature is a character whose intent comes from a brain instead of a
//! keyboard. Everything below the intent — the locomotion FSM, coyote time,
//! grounding, animation — is [`crate::character`], shared with the player.
//!
//! ```text
//!   PerceptionSystem ─► Perception ─► BrainSystem ─► CharacterIntent
//!         │                              │                 │
//!    sight cone,                     Behaviour FSM     CharacterControlSystem
//!    hearing, LOS,                   + steering              (shared)
//!    memory                          primitives
//! ```
//!
//! Perception answers "what is out there", the brain answers "what to do about
//! it", and steering answers "which way is that". Keeping the three apart is
//! what lets a new creature reuse two of them and rewrite only the third.
//!
//! Not every creature walks. A roller, a swimmer and a flyer each turn the
//! same intent into their own way of moving, and two creatures do not move
//! at all: a lobber throws bombs at what it perceives, and a snap trap bites
//! what steps into it.
//!
//! A melee blow lands as a strike (`PhysicsImpulse::Strike`) on the first
//! body its swing touches (`Swing`), which harm reads as a shove; a roller
//! strikes with its own body, surging at its target once it is close
//! (`Lunge`). Neither strikes a character that is down.
//!
//! A creature that wears a shell (`Shell`) loses it in shards to the blow
//! that kills it.
//!
//! The definitions of individual creatures — geometry, materials, physics —
//! live in `crate::app::creatures`, mirroring the spawnables library.

mod alert_telegraph;
mod brain;
mod collectable;
mod flyer;
mod lobber;
mod melee;
mod perception;
mod roller;
mod shell;
mod snap_trap;
pub mod steering;
mod swimmer;
mod swing;

pub use alert_telegraph::{AlertTelegraph, AlertTelegraphSystem};
pub use brain::{Behaviour, Brain, BrainSystem, Temperament};
pub use collectable::{Collectable, CollectionSystem, Reward};
pub use flyer::{Flyer, FlyerLocomotionSystem, Wing, WingBeat};
pub use lobber::{
    Cheeks, Gaze, Lobber, LobberBody, LobberSystem, Puffing, Rider, Showing, Strut, Swell, Throw,
};
pub use melee::{MeleeAttack, MeleeAttackSystem, StrikePhase};
pub use perception::{PerceivedTarget, Perception, PerceptionSystem};
pub use roller::{Drive, Lunge, LungePhase, Quarry, Roller, RollerLocomotionSystem};
pub use shell::{Shell, ShellSystem};
pub use snap_trap::{Caught, Cycle, Hinge, Moment, SnapTrap, SnapTrapSystem, TrapPart, TrapRig};
pub use swimmer::{Swimmer, SwimmerLocomotionSystem};
pub use swing::Swing;
