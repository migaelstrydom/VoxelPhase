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
//! - Swept collision detection (CCD)
//! - Mesh patch types for terrain queries

mod aabb;
pub mod contact;
pub mod contact_reducer;
mod contact_legacy;
pub mod discrete;
pub mod mesh;
mod mesh_patch;
pub mod obb;
pub mod sat;
pub mod segment;
pub mod shapes;
mod shapes_legacy;
pub mod sphere_triangle;
pub mod support;
mod swept;

// --- New collision library types ---
pub use contact::{ContactManifold, ContactPoint, FeatureId};
pub use contact_reducer::ContactReducer;
pub use obb::Obb;
pub use segment::{point_segment_distance_sq, segment_segment_closest_points};
pub use shapes::ShapeView;
pub use support::ConvexSupport;

// --- Legacy types (used by existing code during transition) ---
pub use contact_legacy::ContactPoint as LegacyContactPoint;
pub use shapes_legacy::Sphere;

// --- Unchanged re-exports ---
pub use aabb::AABB;
pub use mesh_patch::{MeshPatch, PatchTriangle};
pub use sphere_triangle::{sphere_triangle_collision_with_feature, Triangle};
pub use swept::{swept_sphere_triangle, SweptContact};
