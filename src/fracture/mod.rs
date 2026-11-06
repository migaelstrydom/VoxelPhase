//! Compound body fracture: the parts a body is made of, the bonds that hold
//! them together, and splitting off whatever has come apart.
//!
//! ```text
//!   CompoundFracture { PartGraph, loads }      on the compound's entity
//!        │
//!        ▼  FractureSystem: loads at each bond's anchor ──▶ broken bonds
//!   groups ──▶ kept: the tied one, else the heaviest
//!              every other: split_off whole, with its ties ──▶ new entity
//!                           (its share of the compound and the looks)
//!                           ──▶ PieceSplit for the owners
//!
//!   cutters (glass, cleave) ──▶ replace_part, through the spawnable's
//!                               FaceMesher
//! ```
//!
//! Fracture never draws: how a part looks is kept by collider handle in
//! `model::PartLooks` and drawn by `PartDrawSystem`. Design:
//! `docs/COMPOUND_PARTS.md`.

mod components;
mod contact_load;
mod cutting;
mod debris;
mod depth;
mod load;
mod parts;
mod piece_split;
pub mod systems;

pub use components::{CompoundFracture, PieceTraits};
pub use contact_load::{ContactLoadTracker, ContactSpike, Deadband};
pub use cutting::{replace_part, CutPiece, Cuttable, FaceMesher, MadePart};
pub use debris::{Debris, DebrisBudget, DebrisCullSystem};
pub use depth::BreakDepths;
pub use load::BondLoads;
pub use parts::{Bond, NewPart, Part, PartGraph, PartId, Tie};
pub use piece_split::{PieceSplit, PieceSplits};
pub use systems::FractureSystem;
