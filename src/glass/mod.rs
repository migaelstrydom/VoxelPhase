//! Brittle sheets: glass that cracks where it is hit.
//!
//! ```text
//!   ImpactLedger / blasts / held load
//!        │
//!        ▼
//!   GlassCrackSystem ── polygon_of(child) ──▶ CrazeRule ──▶ web of cells
//!        │                                                    │
//!        │        detach child, attach one prism per cell ◀───┘
//!        │        join touching cells, sever the ones under the hit
//!        ▼
//!   CompoundFracture ──▶ FractureSystem splits the severed shards off
//! ```

mod components;
mod crazing;
mod fatigue;
mod pane;
mod polygon;
pub mod systems;
mod voronoi;
mod web;

pub use components::BrittleSheet;
pub use crazing::CrazeRule;
pub use fatigue::{FatigueRule, FatigueTracker};
pub use pane::SheetFrame;
pub use polygon::ConvexPolygon;
pub use systems::GlassCrackSystem;
pub use web::CrackWeb;
