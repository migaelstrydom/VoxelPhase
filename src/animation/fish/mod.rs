//! A fish's rig: fins a wave runs down, wings that beat, and eyes that rove
//! on their own, each quicker the faster the body carrying them goes.
//!
//! ```text
//!   Velocity ──speed──► FishRig::advance ──► pose ──► PosedModel (wings, eyes)
//!   Player ──watched──►  (wave, beat,    └──► shape ─► ProceduralMesh (fins)
//!                         glances)
//! ```
//!
//! The fins are `model::Ripple`s, each vertex swung about the root it grows
//! from; the wings beat as a flyer's do (`creature::WingBeat`). The rig only
//! draws: the body is whatever moves it.

mod rig;
mod system;

pub use rig::{FinWave, FishRig, RippledFin, RovingEye};
pub use system::FishRigSystem;
