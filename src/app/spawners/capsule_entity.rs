use std::sync::Arc;

use nalgebra::Vector3;
use specs::{Builder, Entity, World, WorldExt};

use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::geometry::generate_capsule;
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId, MaterialManagerBuilder};
use crate::resources::textures::TextureManager;
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 128;
const MESH_SEGMENTS: u32 = 24;
const CAP_RINGS: u32 = 12;

/// Physics parameters for a spawned capsule.
pub struct CapsulePhysics {
    pub density: f32,
    pub restitution: f32,
    pub friction: f32,
}

impl Default for CapsulePhysics {
    fn default() -> Self {
        Self {
            density: 50.0,
            restitution: 0.2,
            friction: 0.6,
        }
    }
}

/// Create a capsule material with a two-tone pill texture.
pub fn create_capsule_material(
    texture_manager: &TextureManager,
    material_builder: &mut MaterialManagerBuilder,
) -> EngineResult<MaterialId> {
    let pixels = generate_pill_texture();
    let texture = texture_manager.create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
    let material = Material::textured(texture);
    Ok(material_builder.register(material))
}

/// Builds a capsule model.
fn build_model(half_height: f32, radius: f32, material: MaterialId) -> Arc<Model> {
    let (vertices, indices) = generate_capsule(half_height, radius, MESH_SEGMENTS, CAP_RINGS, Colour::WHITE);
    let parts = vec![ModelPart::new(vec![MeshPrimitive {
        vertices,
        indices,
        material,
    }])];
    Arc::new(Model::flat(parts))
}

/// Spawns a capsule entity with physics.
pub fn spawn_capsule(
    world: &mut World,
    initial_pos: nalgebra::Point3<f32>,
    half_height: f32,
    radius: f32,
    material: MaterialId,
    phys: &CapsulePhysics,
) -> Entity {
    let model = build_model(half_height, radius, material);

    let body_handle = {
        let mut physics = world.write_resource::<PhysicsResource>();

        let body_desc = RigidBodyDesc::dynamic()
            .position(initial_pos)
            .gravity_scale(1.0)
            .linear_damping(0.01)
            .angular_damping(0.005);

        let body_handle = physics.world.create_body(body_desc);

        let collider_desc = ColliderDesc::capsule(half_height, radius)
            .density(phys.density)
            .restitution(phys.restitution)
            .friction(phys.friction);

        physics.world.attach_collider(body_handle, collider_desc);

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

/// Generate a two-tone pill/medicine capsule texture.
///
/// The top half and bottom half have different colours with a subtle seam
/// line between them, like a real pharmaceutical capsule.
fn generate_pill_texture() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    // Pick two complementary colours for top/bottom halves.
    let hue_a = rand::random::<f32>() * 6.0;
    let hue_b = (hue_a + 3.0) % 6.0;
    let colour_a = hue_to_rgb(hue_a, 0.55, 0.85);
    let colour_b = hue_to_rgb(hue_b, 0.55, 0.85);
    let seed = rand::random::<u32>();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Pick base colour by hemisphere (v < 0.5 = top, v >= 0.5 = bottom)
            let base = if v < 0.48 {
                colour_a
            } else if v > 0.52 {
                colour_b
            } else {
                // Seam line: darker blend
                let t = (v - 0.48) / 0.04;
                let mixed = lerp_rgb(colour_a, colour_b, t);
                scale_rgb(mixed, 0.7)
            };

            // Subtle surface noise
            let noise = fbm_2d_periodic(u * 8.0, v * 8.0, 3, 0.4, 2.0, seed, Some(8));
            let factor = 0.90 + noise * 0.10;
            let c = scale_rgb(base, factor);

            // Specular highlight along the center strip
            let highlight = ((u - 0.3).abs() / 0.15).min(1.0);
            let c = scale_rgb(c, 0.95 + (1.0 - highlight) * 0.08);

            pixels.push((c.0.clamp(0.0, 1.0) * 255.0) as u8);
            pixels.push((c.1.clamp(0.0, 1.0) * 255.0) as u8);
            pixels.push((c.2.clamp(0.0, 1.0) * 255.0) as u8);
            pixels.push(255);
        }
    }
    pixels
}

fn hue_to_rgb(h: f32, s: f32, v: f32) -> (f32, f32, f32) {
    let h = h % 6.0;
    let i = h.floor() as i32;
    let f = h - h.floor();
    let p = v * (1.0 - s);
    let q = v * (1.0 - f * s);
    let t = v * (1.0 - (1.0 - f) * s);
    match i % 6 {
        0 => (v, t, p),
        1 => (q, v, p),
        2 => (p, v, t),
        3 => (p, q, v),
        4 => (t, p, v),
        _ => (v, p, q),
    }
}

fn lerp_rgb(a: (f32, f32, f32), b: (f32, f32, f32), t: f32) -> (f32, f32, f32) {
    (
        a.0 + (b.0 - a.0) * t,
        a.1 + (b.1 - a.1) * t,
        a.2 + (b.2 - a.2) * t,
    )
}

fn scale_rgb(c: (f32, f32, f32), s: f32) -> (f32, f32, f32) {
    (c.0 * s, c.1 * s, c.2 * s)
}
