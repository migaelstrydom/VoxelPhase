//! Harm: how hard a character was struck, read from the physics' hits and
//! shoves and judged in three bands: light (shoved, nothing more), heavy
//! (knocked down) and deadly (knocked down, and the room restarts a beat
//! later). There is no health. Design: `docs/GAME_DESIGN.md` *Harm*; the
//! meter it reads: `docs/IMPACT_METER.md`.
//!
//! ```text
//!   PhysicsWorld::hits_on, shoves_on ─► Blow ─► Band ─► Harm
//!                                                        ├─ knock ─► CharacterControlSystem ─► LocomotionState::Down
//!                                                        └─ dying runs out ─► LevelRestart
//! ```
//!
//! A blow is judged by how sudden a stop it was: its change of speed over
//! what it was spread across (`Blow::sudden`). A body braced for it, moving
//! into it while in control, spreads it over its limbs.

mod blow;
mod component;
mod config;
mod system;

#[cfg(test)]
mod tests;

pub use blow::{Band, Blow};
pub use component::Harm;
pub use config::{HarmConfig, FLESH_GIVE};
pub use system::{worst_blow, HarmSystem};
