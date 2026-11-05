//! Plants: trees grown from a spec, a woody frame of limbs holding up leaf masses (`foliage`).
//!
//! ```text
//!   LollipopSpec ─┐
//!   CypressSpec ──┴─grow(seed)─▶ Plant ──┬──▶ Limbs: a trunk and branches, each a capsule
//!                                        └──▶ LeafMass × n: a FoliageShape, its pose, its limb
//!
//!   Joinery ──build──▶ PlantBodies: a body per limb, carrying its crown's core;
//!                      branches welded to what they grow from (breakable),
//!                      the trunk welded into the ground (TerrainAnchored)
//! ```
//!
//! A plant is a frame and the leaves it holds. Each crown is foliage like
//! any shrub's, dragging and drawn the same, riding on its limb's body. The
//! tree stands until the ground under it is blown away, then falls and
//! lands on its crowns; a blast beside a branch breaks it off, and one
//! closer to a crown breaks the crown off its limb (`CrownBreakSystem`),
//! loose as a shrub torn from the ground is.
//! `docs/PLANTS_DESIGN.md` has the design and its steps.

mod bodies;
mod components;
mod cypress;
mod limbs;
mod lollipop;
mod plant;
mod systems;

#[cfg(all(test, feature = "foliage_scenarios"))]
mod scenarios;

pub use bodies::{Joinery, PlantBodies};
pub use components::HeldCrown;
pub use cypress::CypressSpec;
pub use limbs::{Limb, Limbs};
pub use lollipop::LollipopSpec;
pub use plant::{LeafMass, Plant};
pub use systems::CrownBreakSystem;
