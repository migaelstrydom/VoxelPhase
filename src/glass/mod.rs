//! Brittle sheets: glass that cracks where it is hit.
//!
//! ```text
//!   ImpactLedger / blasts / held load
//!        │
//!        ▼
//!   GlassCrackSystem ── polygon_of(child) ──▶ CrazeRule ──▶ web of cells
//!        │                                                    │
//!        │   replace_part: one prism per cell, made and  ◀────┘
//!        │   drawn by the spawnable's FaceMesher; bond touching cells,
//!        │   weld cells along held edges, sever the ones under the hit
//!        ▼
//!   CompoundFracture ──▶ FractureSystem splits the severed shards off
//!        │
//!        ▼ PieceSplit
//!   SheetShareSystem ──▶ a split-off cluster's share of the BrittleSheet
//! ```

mod components;
mod crazing;
mod fatigue;
mod held;
mod jolt;
mod pane;
mod polygon;
mod remnant;
pub mod systems;
mod voronoi;
mod web;

pub use components::BrittleSheet;
pub use crazing::CrazeRule;
pub use fatigue::{FatigueRule, FatigueTracker};
pub use held::{weld_along, HeldEdge, HeldEdges};
pub use jolt::{BodyJolt, JoltTracker, Motion};
pub use pane::SheetFrame;
pub use polygon::ConvexPolygon;
pub use remnant::{Remnant, RemnantRule};
pub use systems::{GlassCrackSystem, SheetShareSystem};
pub use web::CrackWeb;
