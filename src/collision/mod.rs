//! Collision detection and spatial math.
//!
//! This module provides:
//! - Core collision types: `ContactPoint`, `ContactManifold`, `FeatureId`
//! - Shape view for collision dispatch: `ShapeView`
//! - `ConvexSupport` trait for GJK/EPA/CCD
//! - Segment geometry utilities
//! - Contact reduction (area-maximizing)
//! - AABB for spatial queries
//! - Sphere-triangle intersection tests
//! - Continuous collision detection (CCD)
//! - Mesh patch types for terrain queries

mod aabb;
pub mod contact;
pub mod contact_reducer;
pub mod continuous;
pub mod discrete;
pub mod mesh;
mod mesh_patch;
pub mod obb;
pub mod sat;
pub mod segment;
pub mod shapes;
pub mod sphere_triangle;
pub mod support;

pub use aabb::AABB;
pub use mesh_patch::{MeshPatch, PatchTriangle};
pub use sphere_triangle::Triangle;
