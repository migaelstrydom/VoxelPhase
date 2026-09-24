//! The typeface the game draws text with, embedded at compile time.
//!
//! One copy, shared by everything that sets type: the debug overlay and HUD
//! rasterise it into an atlas, and engravings cut it into a surface.

/// JetBrains Mono Regular, as TrueType.
pub const JETBRAINS_MONO: &[u8] = include_bytes!("../../data/JetBrainsMono-Regular.ttf");
