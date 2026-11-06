//! Plants: trees grown from a spec, a woody frame of limbs holding up leaf masses (`foliage`).
//!
//! ```text
//!   LollipopSpec ─┐
//!   CypressSpec ──┴─grow(seed)─▶ Plant ──┬──▶ Limbs: a trunk, branches and roots, each a capsule
//!                                        └──▶ LeafMass × n: a FoliageShape, its pose, its limb
//!
//!   Joinery ──build──▶ BuiltPlant: one compound body, every limb and
//!                      every crown's core; a part for the trunk with its
//!                      roots and crown, one per branch with its crown,
//!                      bonded at the fork; the trunk welded into the ground
//!                      (a tie, and TerrainAnchored), its roots buried until
//!                      it is let go
//!   PlantDrawing ─────▶ PartLooks: bark on each limb's capsule, leaves on
//!                      one core piece per crown
//!
//!   fracture: a branch's bond broken at its fork ──▶ PieceSplit
//!   PlantShareSystem ──▶ the piece's PlantPiece and Foliage, its bark
//!                        snapped; the tree's Foliage rebuilt
//! ```
//!
//! A plant is a frame and the leaves it holds. Each crown is foliage like
//! any shrub's, dragging and drawn the same, riding on its piece's body.
//! The tree stands until the ground under it is blown away, then falls and
//! lands on its crowns. A body's colliders never meet each other, so a
//! whole tree's limbs do not; a branch broken off meets what it left like
//! any other body. `docs/PLANTS_DESIGN.md` has the design and its steps.

mod bark;
mod bodies;
mod components;
mod cypress;
mod drawing;
mod limbs;
mod lollipop;
mod plant;
mod roots;
mod systems;

#[cfg(all(test, feature = "foliage_scenarios"))]
mod scenarios;

pub use bark::{BarkSpec, LimbBase};
pub use bodies::{BuiltPlant, Joinery, PlantColliders};
pub use components::{GrownPlant, PlantPiece};
pub use cypress::CypressSpec;
pub use drawing::{PlantDrawing, PlantLook};
pub use limbs::{Limb, LimbKind, Limbs};
pub use lollipop::LollipopSpec;
pub use plant::{LeafMass, Plant};
pub use roots::RootSpec;
pub use systems::PlantShareSystem;
