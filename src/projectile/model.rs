//! Grenade projectile model definition.
//!
//! This module provides a simple spherical grenade model using
//! the common sphere generation utilities.

use crate::geometry::{generate_sphere_indices, generate_sphere_vertices};
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::rendering::colour::Colour;
use crate::rendering::material::MaterialId;

/// Material IDs for the grenade model.
pub struct GrenadeMaterials {
    pub body: MaterialId,
}

/// Build the grenade model as a simple sphere.
///
/// # Arguments
/// * `radius` - Radius of the grenade sphere
/// * `colour` - Base colour for the grenade
/// * `materials` - Pre-registered material IDs
pub fn build_grenade_model(radius: f32, colour: Colour, materials: &GrenadeMaterials) -> Model {
    // Lower subdivision than player since grenades are small
    let segments = 16;
    let rings = 12;

    let parts = vec![ModelPart::new(vec![MeshPrimitive {
        vertices: generate_sphere_vertices(radius, segments, rings, colour),
        indices: generate_sphere_indices(segments, rings),
        material: materials.body,
    }])];

    Model::flat(parts)
}
