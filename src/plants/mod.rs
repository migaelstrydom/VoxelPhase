//! Plants: trees grown from a spec, a woody frame of limbs holding up leaf masses (`foliage`).
//!
//! ```text
//!   LollipopSpec ──grow(seed)──▶ Plant ──┬──▶ Limbs: a trunk and branches
//!                                        │      ├─ colliders: a capsule each, on one static body
//!                                        │      └─ mesh: a tube each
//!                                        └──▶ LeafMass × n: a FoliageShape and its pose
//!                                               └─ spawned as a Foliage clump, OnLimb
//! ```
//!
//! A plant is a frame and the leaves it holds. The leaves are foliage like
//! any shrub's, dragging and drawn the same, but held on a limb: a blast
//! does not tear them loose, since their limb holds them. The trunk is
//! solid. `docs/PLANTS_DESIGN.md` has the design and its steps.

mod limbs;
mod lollipop;
mod plant;

pub use limbs::{Limb, Limbs};
pub use lollipop::LollipopSpec;
pub use plant::{LeafMass, Plant};
