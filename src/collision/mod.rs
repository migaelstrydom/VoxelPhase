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
//! - Ray-triangle intersection (Möller–Trumbore)
//! - Continuous collision detection (CCD)
//! - Mesh patch types for terrain queries

mod aabb;
pub mod capsule;
pub mod contact;
pub mod contact_reducer;
pub mod continuous;
pub mod convex_hull;
pub mod discrete;
pub mod dispatch;
pub mod hull_split;
pub mod mesh;
mod mesh_patch;
pub mod obb;
pub mod ray_triangle;
pub mod sat;
pub mod segment;
pub mod shape_view;
pub mod sphere_triangle;
pub mod support;
pub mod triangle;

pub use aabb::AABB;
pub use convex_hull::ConvexHull;
pub use mesh_patch::{MeshPatch, PatchTriangle};
pub use ray_triangle::RayHit;
pub use shape_view::ShapeView;
pub use triangle::Triangle;
