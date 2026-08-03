//! Health, injury, and death.
//!
//! The engine already had explosions, fire and a physics solver but nothing
//! that read any of them as *harm*. This module is the missing consumer.
//!
//! ```text
//!   BlastDamageSystem  ─┐
//!   BurnDamageSystem   ─┼─► DamageQueue ─► DamageApplySystem ─► Health
//!   ImpactDamageSystem ─┘                        │
//!                                                └─► Dead ─► DeathSystem
//!                                                              ├─ release constraints
//!                                                              ├─ clear intent
//!                                                              └─ despawn corpse
//! ```
//!
//! Sources never touch `Health` — they only push events. Adding a new one
//! (acid, drowning, falling into lava) means writing a system that pushes to
//! the queue and changing nothing else.

mod apply;
mod death;
mod event;
mod health;
mod sources;

pub use apply::DamageApplySystem;
pub use death::{DeathSystem, Ragdoll};
pub use event::{DamageEvent, DamageKind, DamageQueue};
pub use health::{Dead, Health};
pub use sources::{BlastDamageSystem, BurnDamageSystem, ImpactDamageSystem, LastVelocity};
