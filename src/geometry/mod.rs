//! Common geometry generation utilities.
//!
//! This module provides reusable mesh generation functions for
//! procedural model creation.

mod capsule;
mod cube;
mod cylinder;
mod dome;
mod ellipsoid_rings;
mod fissured_shell;
mod heart;
mod icosphere;
mod sphere;
mod tube;

pub use capsule::generate_capsule;
pub use cube::{generate_cube_indices, generate_cube_vertices};
pub use cylinder::generate_cylinder;
#[cfg(test)]
pub(crate) use cylinder::widest_u_span;
pub use dome::{generate_dome, DomeSpec};
pub use ellipsoid_rings::generate_ellipsoid_rings;
pub use fissured_shell::{
    generate_fissured_shell, FissureBand, FissuredShell, FissuredShellConfig,
};
pub use heart::{generate_heart, HeartSpec};
pub use icosphere::{generate_geodesic, generate_icosphere, Icosphere};
pub use sphere::{
    generate_magic_sphere_vertices, generate_sphere_indices, generate_sphere_vertices,
    MagicSphereConfig,
};
pub use tube::{append_tube, coil_path, generate_tube};
