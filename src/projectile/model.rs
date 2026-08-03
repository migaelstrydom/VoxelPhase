//! Grenade projectile model definition.
//!
//! A grenade is a molten core caged in a cracked volcanic crust:
//!
//! ```text
//!   ┌─ crust ──── dark metal shell, fissured, glints in the sun
//!   │  ┌─ ember ─ hot rock at the lip of each crack, faintly emissive
//!   │  │  ┌─ core  molten sphere, bright enough to bloom through the gaps
//!   ▼  ▼  ▼
//!  (((  ●  )))
//! ```
//!
//! The three groups are separate primitives so each gets its own material. Only
//! the core is bright enough to bloom on its own; the crust reads as dark rock
//! until the grenade heats up, which `GrenadeVisualSystem` drives by scaling
//! every material's emission at once (see `super::visuals`).

use crate::geometry::{
    generate_fissured_shell, generate_sphere_indices, generate_sphere_vertices, FissuredShellConfig,
};
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::rendering::colour::Colour;
use crate::rendering::material::MaterialId;

/// Radius of the molten core as a fraction of the grenade radius.
///
/// Close enough to the shell that the core shows right at the lip of a crack
/// rather than sitting at the bottom of a well, but not touching, so the crust
/// still silhouettes against it.
const CORE_RADIUS_FRACTION: f32 = 0.93;

/// Subdivision of the core. Lower than the shell: it is seen in slivers through
/// the cracks, and reads by its glow rather than by its outline.
const CORE_SEGMENTS: u32 = 20;
const CORE_RINGS: u32 = 14;

/// Material IDs for the grenade model.
pub struct GrenadeMaterials {
    /// Dark, glossy metal of the outer shell.
    pub crust: MaterialId,

    /// Hot rock bordering each fissure.
    pub ember: MaterialId,

    /// The molten core seen through the fissures.
    pub core: MaterialId,
}

/// Build the grenade model.
///
/// # Arguments
/// * `radius` - Outer radius of the grenade, matching its collider
/// * `core_colour` - Vertex colour of the molten core. Its glow comes from the
///   core material's emission; this only tints the lit surface underneath
/// * `materials` - Pre-registered material IDs
pub fn build_grenade_model(
    radius: f32,
    core_colour: Colour,
    materials: &GrenadeMaterials,
) -> Model {
    let shell = generate_fissured_shell(&FissuredShellConfig::volcanic(radius));
    let core_radius = radius * CORE_RADIUS_FRACTION;

    let parts = vec![ModelPart::new(vec![
        MeshPrimitive {
            vertices: shell.vertices.clone(),
            indices: shell.crust_indices,
            material: materials.crust,
        },
        MeshPrimitive {
            vertices: shell.vertices,
            indices: shell.ember_indices,
            material: materials.ember,
        },
        MeshPrimitive {
            vertices: generate_sphere_vertices(core_radius, CORE_SEGMENTS, CORE_RINGS, core_colour),
            indices: generate_sphere_indices(CORE_SEGMENTS, CORE_RINGS),
            material: materials.core,
        },
    ])];

    Model::flat(parts)
}
