//! Keyboard and mouse state, and the gameplay actions read from it.

mod actions;
mod state;

pub use actions::{GameplayActions, InputActionSystem};
pub use state::InputState;
