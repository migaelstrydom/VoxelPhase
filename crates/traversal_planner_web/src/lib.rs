//! The traversal planner compiled to wasm for the web tool: scenes in, plans
//! and their drawings out, as JSON across a C-style boundary.
//!
//! ```text
//!   page (JS) ──JSON──► ffi::call ──► api ──► traversal_planner ──► svg + plan
//!                                      └──► scrub: replan frame by frame,
//!                                           loose stones giving way
//! ```

mod api;
mod builtins;
mod ffi;
mod scrub;

pub use api::{handle, Op};
