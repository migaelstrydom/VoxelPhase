//! Model hierarchy and mesh primitive types.
//!
//! This module provides types for defining 3D models as hierarchies of parts,
//! each with transforms and mesh primitives. The design aligns with industry
//! standard formats like glTF.

mod parts;
mod transform;

pub use parts::{MeshPrimitive, Model, ModelPart};
pub use transform::Transform;
