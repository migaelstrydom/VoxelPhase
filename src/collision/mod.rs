//! Collision detection and spatial math.
//!
//! This module provides:
//! - AABB (Axis-Aligned Bounding Box) for spatial queries
//! - Collision shapes (Sphere, future: Capsule)
//! - Sphere-triangle intersection tests
//! - Swept collision detection (CCD) for fast-moving objects
//! - TerrainCollider for efficient terrain collision queries
//! - Contact manifold generation

mod aabb;
mod contact;
mod mesh_patch;
mod shapes;
mod sphere_triangle;
mod swept;

pub use aabb::AABB;
pub use contact::ContactPoint;
pub use mesh_patch::{MeshPatch, PatchTriangle};
pub use shapes::Sphere;
pub use sphere_triangle::{sphere_triangle_collision, Triangle};
pub use swept::{swept_sphere_triangle, SweptContact};
