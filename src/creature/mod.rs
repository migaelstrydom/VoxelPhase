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
//! Every creature strikes with its own body: a melee blow, its wind-up
//! over, is a rush along the way the creature faces (`Rush`), and a roller
//! surges at its target once it is close (`Lunge`). The blow is the two
//! bodies meeting, which harm judges as it does any hit. None strikes a
//! character that is down.
//!
//! A creature that wears a shell (`Shell`) loses it in pieces to the blow
//! that kills it; a roller, all shell, splits in two.
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
mod rush;
mod shell;
mod snap_trap;
pub mod steering;
mod swimmer;

pub use alert_telegraph::{AlertTelegraph, AlertTelegraphSystem};
pub use brain::{Behaviour, Brain, BrainSystem, Temperament};
pub use collectable::{Collectable, CollectionSystem, Reward};
pub use flyer::{FlightPose, Flyer, FlyerLocomotionSystem, Wing, WingBeat};
pub use lobber::{
    Cheeks, Gaze, Lobber, LobberBody, LobberSystem, Puffing, Rider, Showing, Strut, Swell, Throw,
};
pub use melee::{MeleeAttack, MeleeAttackSystem, StrikePhase};
pub use perception::{PerceivedTarget, Perception, PerceptionSystem};
pub use roller::{Drive, Lunge, LungePhase, Quarry, Roller, RollerLocomotionSystem};
pub use rush::{Rush, RushAim};
pub use shell::{Cracks, Shell, ShellSystem};
pub use snap_trap::{Caught, Cycle, Hinge, Moment, SnapTrap, SnapTrapSystem, TrapPart, TrapRig};
pub use swimmer::{Swimmer, SwimmerLocomotionSystem};
