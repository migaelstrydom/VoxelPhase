//! The game's heads-up display.
//!
//! ```text
//!   AimState ─┐
//!   camera    ├─► HudContext ─► Hud ─► HudElement::update / draw
//!   screen    ┘                          │
//!                                        ▼
//!                                   HudPainter ─► OverlayGeometry ─► OverlayRenderer
//! ```
//!
//! Three separations carry the whole module:
//!
//! * An element **animates and draws**; it never talks to Vulkan. All it gets
//!   is a [`HudContext`] (what the world says this frame) and a [`HudPainter`]
//!   (somewhere to put rectangles).
//! * The painter **produces geometry**, not draw calls. The frame's text and
//!   HUD end up in one batch, which is what the overlay's single shared vertex
//!   buffer requires.
//! * [`Hud`] **owns the element list** and nothing else, so a new element is a
//!   new file and one line in [`Hud::game`].

mod element;
mod hud;
mod painter;
mod reticle;

pub use element::{HudContext, HudElement};
pub use hud::Hud;
pub use painter::HudPainter;
pub use reticle::{AimReticle, ReticleStyle};
