use std::sync::Arc;

use nalgebra::Vector3;
use specs::{Builder, Entity, World, WorldExt};

use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::geometry::{generate_cube_indices, generate_cube_vertices};
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId, MaterialManagerBuilder};
use crate::resources::textures::TextureManager;
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 128;

/// Generates a procedural wooden-crate texture as raw RGBA pixels.
///
/// Produces a tileable texture with:
/// - Wood-grain base using layered FBM noise
/// - Plank borders (horizontal and vertical dividers)
/// - Corner bracket details for the classic crate look
fn generate_crate_pixels() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let base_r = 0.62_f32;
    let base_g = 0.44;
    let base_b = 0.25;

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Wood grain: stretched noise along one axis
            let grain_scale = 8.0;
            let grain = fbm_2d_periodic(
                u * grain_scale,
                v * grain_scale * 3.0,
                4,
                0.55,
                2.0,
                17,
                Some(grain_scale as i32),
            );

            // Broader colour variation
            let variation = fbm_2d_periodic(
                u * 4.0,
                v * 4.0,
                3,
                0.5,
                2.0,
                83,
                Some(4),
            );

            // Combine into wood colour: grain darkens, variation shifts hue slightly
            let wood_factor = 0.75 + grain * 0.25;
            let hue_shift = (variation - 0.5) * 0.08;

            let mut r = (base_r + hue_shift) * wood_factor;
            let mut g = (base_g + hue_shift * 0.5) * wood_factor;
            let mut b = base_b * wood_factor;

            // Plank borders: dark lines at 0.5 (splits face into 2x2 planks)
            let border_width = 0.03;
            let hx = (u - 0.5).abs();
            let hy = (v - 0.5).abs();
            let h_border = smooth_border(hx, border_width);
            let v_border = smooth_border(hy, border_width);
            let border = (h_border + v_border).min(1.0);

            // Darken at borders
            let border_darken = 1.0 - border * 0.45;
            r *= border_darken;
            g *= border_darken;
            b *= border_darken;

            // Corner brackets: L-shaped marks near the four corners
            let bracket_strength = corner_bracket(u, v);
            let bracket_darken = 1.0 - bracket_strength * 0.5;
            r *= bracket_darken;
            g *= bracket_darken;
            b *= bracket_darken;

            // Edge darkening: subtle vignette at the texture boundary
            let edge_u = (u - 0.5).abs() * 2.0; // 0 at centre, 1 at edge
            let edge_v = (v - 0.5).abs() * 2.0;
            let edge = ((edge_u.max(edge_v) - 0.85) / 0.15).clamp(0.0, 1.0);
            let edge_darken = 1.0 - edge * 0.3;
            r *= edge_darken;
            g *= edge_darken;
            b *= edge_darken;

            pixels.push((r.clamp(0.0, 1.0) * 255.0) as u8);
            pixels.push((g.clamp(0.0, 1.0) * 255.0) as u8);
            pixels.push((b.clamp(0.0, 1.0) * 255.0) as u8);
            pixels.push(255);
        }
    }

    pixels
}

/// Smooth border falloff: returns 0.0 when far from the border, 1.0 at the centre.
fn smooth_border(distance: f32, width: f32) -> f32 {
    (1.0 - (distance / width).min(1.0)).powi(2)
}

/// Returns bracket intensity (0.0–1.0) for L-shaped corner details.
///
/// Each corner has a small L-bracket consisting of two perpendicular bars
/// inset slightly from the corner.
fn corner_bracket(u: f32, v: f32) -> f32 {
    let bracket_inset = 0.06;
    let bracket_length = 0.22;
    let bracket_thickness = 0.025;

    // Test all four corners by mirroring coordinates
    let cu = if u < 0.5 { u } else { 1.0 - u };
    let cv = if v < 0.5 { v } else { 1.0 - v };

    // Horizontal bar of the L
    let h_bar = cu < bracket_length
        && (cv - bracket_inset).abs() < bracket_thickness;

    // Vertical bar of the L
    let v_bar = cv < bracket_length
        && (cu - bracket_inset).abs() < bracket_thickness;

    if h_bar || v_bar { 1.0 } else { 0.0 }
}

/// Creates a crate material with a procedural wood texture.
///
/// Call during initialisation while `MaterialManagerBuilder` is still mutable.
pub fn create_crate_material(
    texture_manager: &TextureManager,
    material_builder: &mut MaterialManagerBuilder,
) -> EngineResult<MaterialId> {
    let pixels = generate_crate_pixels();
    let texture = texture_manager.create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
    let material = Material::textured(texture);
    Ok(material_builder.register(material))
}

/// Builds a crate model with the given half-extents.
fn build_model(half_extents: Vector3<f32>, material: MaterialId) -> Arc<Model> {
    let parts = vec![ModelPart::new(vec![MeshPrimitive {
        vertices: generate_cube_vertices(half_extents, Colour::WHITE),
        indices: generate_cube_indices(),
        material,
    }])];

    Arc::new(Model::flat(parts))
}

/// Spawns a box entity with physics.
pub fn spawn_box(
    world: &mut World,
    initial_pos: nalgebra::Point3<f32>,
    half_extents: Vector3<f32>,
    material: MaterialId,
) -> Entity {
    let model = build_model(half_extents, material);

    let body_handle = {
        let mut physics = world.write_resource::<PhysicsResource>();

        let body_desc = RigidBodyDesc::dynamic()
            .position(initial_pos)
            .gravity_scale(1.0)
            .linear_damping(0.01)
            .angular_damping(0.05);

        let body_handle = physics.0.create_body(body_desc);

        let collider_desc = ColliderDesc::box_shape(half_extents)
            .density(5.0)
            .restitution(0.2)
            .friction(0.6);

        physics.0.attach_collider(body_handle, collider_desc);

        body_handle
    };

    world
        .create_entity()
        .with(Position(Vector3::new(
            initial_pos.x,
            initial_pos.y,
            initial_pos.z,
        )))
        .with(Velocity(Vector3::zeros()))
        .with(Orientation::default())
        .with(RigidBodyComponent(body_handle))
        .with(ModelInstance::new(model))
        .with(Renderable)
        .build()
}
