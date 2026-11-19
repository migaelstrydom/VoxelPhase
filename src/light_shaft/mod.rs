//! Shafts of light: a column of light standing up from a point into the sky, drawn as light rather than as a surface.
//!
//! ```text
//!   owner (a brazier) ──aim: ShaftLook──▶ LightShaft ◀── LightShaftSystem
//!                                          │ look eases toward aim;
//!                                          │ streaks drift up
//!                                          ▼
//!   RenderSystem ──ShaftMesh::of──▶ renderer.draw_light_shaft ──▶ light_shaft shader
//!                                   (sorted blended pass, premultiplied, adds light)
//! ```
//!
//! A shaft marks a place from across a level: it stands straight up from
//! its entity whichever way the entity is turned, brightest through its
//! middle and at its foot, fading toward its edges and the sky.

mod mesh;
mod shaft;

pub use mesh::ShaftMesh;
pub use shaft::{LightShaft, LightShaftSystem, ShaftLook};
