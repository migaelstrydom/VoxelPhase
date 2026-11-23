//! Hits: what a contact that closed fast did, in quantities the solver cannot
//! get wrong, and what a blast gave a body. Readers judge them (harm, glass,
//! fracture); nothing here knows who reads. Design: `docs/IMPACT_METER.md`.
//!
//! ```text
//!   solver ── approach_speed on each contact ──▶ HitRecorder, after every solve
//!                                                  │ per pair: its fastest substep,
//!                                                  │ the patch within its give,
//!                                                  │ its effective mass there
//!                                                  ▼
//!                                     PhysicsWorld::hits() ──▶ severity (Hit's methods,
//!                                                              and one side's: HitSide)
//!   blasts, as they are applied ──▶ Shove ──▶ PhysicsWorld::shoves()
//! ```

mod event;
mod patch;
mod recorder;
mod severity;
mod side;
#[cfg(test)]
mod tests;

pub use event::{Hit, Shove};
pub use patch::Patch;
pub use recorder::HitConfig;
pub(crate) use recorder::HitRecorder;
pub use side::HitSide;
