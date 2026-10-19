//! Hover sleds: a hull held up on suspensions, carrying whoever stands on its deck and gliding the way they lean.

pub mod components;
pub mod glide;
pub mod system;

pub use components::HoverSled;
pub use glide::Glide;
pub use system::HoverSledSystem;
