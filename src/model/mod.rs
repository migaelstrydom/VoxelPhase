//! Model hierarchy and mesh primitive types, and the looks of a body's parts.
//!
//! This module provides types for defining 3D models as hierarchies of parts,
//! each with transforms and mesh primitives. The design aligns with industry
//! standard formats like glTF. [`PartLooks`] says how each collider of a
//! breakable body looks, so its pieces are drawn as they were whole.

mod assemble;
mod look;
mod parts;
mod transform;

pub use assemble::{assemble_by_material, PlacedMesh};
pub use look::{Look, PartLooks};
pub use parts::{MeshPrimitive, Model, ModelPart};
pub use transform::Transform;
