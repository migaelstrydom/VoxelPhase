//! The player: a marker on the one character the camera follows and the
//! keyboard drives. All of the locomotion machinery lives in
//! [`crate::character`], which the player shares with AI creatures.

mod components;

pub use components::Player;
