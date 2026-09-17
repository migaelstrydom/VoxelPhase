//! Where a throw would land.
//!
//! ```text
//!   CharacterState (arm) ─┐
//!   CharacterIntent       ├─► AimPredictionSystem ──► AimState ──► HUD
//!   FollowTarget (camera) ┘        │        ▲
//!                                  │        │
//!                      AimSource ──┘        └── TrajectoryPredictor
//!                   (Launch, from the           (ballistic flight vs
//!                    same numbers the            a ProbeSet of terrain
//!                    throw itself uses)          and bodies)
//! ```
//!
//! The load-bearing idea is the [`Launch`]: a throw stated as an origin, a
//! velocity and the gravity it flies under. The code that *performs* a throw
//! builds one and the predictor reads it, so a cursor cannot drift away from
//! the throw it is drawing — changing the grenade's arc moves both at once.
//!
//! Nothing here knows about screens or pixels. The prediction is a world-space
//! point; turning that into something on the display is the HUD's business.

mod gate;
mod probe;
mod source;
mod state;
mod system;
mod trajectory;

pub mod launch;

pub use gate::AimGate;
pub use launch::Launch;
pub use probe::BodiesExcept;
pub use source::{AimContext, AimKind, AimSource, GrenadeAim, HeldObjectAim};
pub use state::{AimSolution, AimState};
pub use system::AimPredictionSystem;
pub use trajectory::{Impact, TrajectoryPredictor};
