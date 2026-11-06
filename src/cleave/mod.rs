//! Brittle solids: blocks that break into a few wedges where they are struck.
//!
//! The counterpart to [`glass`](crate::glass), which breaks *sheets*. A pane
//! is a plane and crazes into a web of cells across its face; a block is a
//! solid and cleaves along a few surfaces that run right through it.
//!
//! ```text
//!   contact spike / blast
//!        │
//!        ▼
//!   SolidCleaveSystem ── CleaveRule ──▶ tilted cut planes ──▶ split_hull
//!        │                                                        │
//!        │   fracture::replace_part (wedges made and drawn by the ◀┘
//!        │   spawnable's FaceMesher), bond the wedges, sever the struck one
//!        ▼
//!   CompoundFracture ──▶ FractureSystem splits the loose wedges off
//!        │
//!        ▼ PieceSplit
//!   SolidShareSystem ──▶ a split-off piece's share of the BrittleSolid
//! ```

mod components;
mod plan;
pub mod systems;

pub use components::{BlowMeasure, BrittleSolid};
pub use plan::{CleavePiece, CleaveRule};
pub use systems::{SolidCleaveSystem, SolidShareSystem};
