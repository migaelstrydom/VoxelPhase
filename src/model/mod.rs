//! Model hierarchy and mesh primitive types, and the looks of a body's parts.
//!
//! This module provides types for defining 3D models as hierarchies of parts,
//! each with transforms and mesh primitives. The design aligns with industry
//! standard formats like glTF. [`PartLooks`] says how each collider of a
//! breakable body looks, so its pieces are drawn as they were whole,
//! [`slice`] cuts a mesh with a plane, for a piece broken out of a part, a
//! [`Bulge`] is a mesh blown out of a surface and sunk back into it, a
//! [`Ripple`] is a fin swung on hinges along the root it grows from, and a
//! [`Jaw`] is flesh that opens on a hinge, each vertex by its share.

mod assemble;
mod bulge;
mod jaw;
mod look;
mod parts;
mod ripple;
mod slice;
mod transform;

pub use assemble::{assemble_by_material, PlacedMesh};
pub use bulge::Bulge;
pub use jaw::Jaw;
pub use look::{Look, PartLooks};
pub use parts::{MeshPrimitive, Model, ModelPart};
pub use ripple::{Hinge, Ripple};
pub use slice::{cap, slice, RimEdge, Slice};
pub use transform::Transform;
