//! Common geometry generation utilities.
//!
//! This module provides reusable mesh generation functions for
//! procedural model creation.

mod capsule;
mod cube;
mod cylinder;
mod sphere;

pub use capsule::generate_capsule;
pub use cube::{generate_cube_indices, generate_cube_vertices};
pub use cylinder::generate_cylinder;
pub use sphere::{
    generate_magic_sphere_vertices, generate_sphere_indices, generate_sphere_vertices,
    MagicSphereConfig,
};
