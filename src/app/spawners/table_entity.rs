//! Table entity spawner — compound body with procedural wood texture.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
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
use crate::rendering::vertex::Vertex;
use crate::resources::textures::TextureManager;
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 128;

/// Physics parameters for a spawned table.
pub struct TablePhysics {
    pub density: f32,
    pub restitution: f32,
    pub friction: f32,
}

impl Default for TablePhysics {
    fn default() -> Self {
        Self {
            density: 600.0,
            restitution: 0.1,
            friction: 0.5,
        }
    }
}

/// Dimensions for a spawned table.
pub struct TableDimensions {
    /// Half-extents of the table top slab.
    pub top_half_extents: Vector3<f32>,
    /// Half-extents of each leg.
    pub leg_half_extents: Vector3<f32>,
}

impl Default for TableDimensions {
    fn default() -> Self {
        Self {
            top_half_extents: Vector3::new(0.5, 0.025, 0.3),
            leg_half_extents: Vector3::new(0.03, 0.175, 0.03),
        }
    }
}

impl TableDimensions {
    /// Total height from leg bottom to table top surface.
    pub fn total_height(&self) -> f32 {
        self.leg_half_extents.y * 2.0 + self.top_half_extents.y * 2.0
    }
}

// ---------------------------------------------------------------------------
// Procedural wood texture
// ---------------------------------------------------------------------------

/// RGB colour used during texture generation.
#[derive(Clone, Copy)]
struct Rgb {
    r: f32,
    g: f32,
    b: f32,
}

impl Rgb {
    fn new(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b }
    }

    fn scale(self, factor: f32) -> Self {
        Self::new(self.r * factor, self.g * factor, self.b * factor)
    }

    fn write_rgba(self, pixels: &mut Vec<u8>) {
        pixels.push((self.r.clamp(0.0, 1.0) * 255.0) as u8);
        pixels.push((self.g.clamp(0.0, 1.0) * 255.0) as u8);
        pixels.push((self.b.clamp(0.0, 1.0) * 255.0) as u8);
        pixels.push(255);
    }
}

fn rand_range(lo: f32, hi: f32) -> f32 {
    lo + rand::random::<f32>() * (hi - lo)
}

fn rand_u32() -> u32 {
    rand::random::<u32>()
}

/// Generate a polished wood texture — smooth grain with subtle ring patterning,
/// no plank borders or brackets (those are for crates, not furniture).
fn generate_table_wood() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    // Randomise the wood tone: warm oak, dark walnut, or honey pine.
    let tone = rand::random::<u32>() % 3;
    let base = match tone {
        0 => Rgb::new(0.55, 0.38, 0.20), // warm oak
        1 => Rgb::new(0.35, 0.22, 0.12), // dark walnut
        _ => Rgb::new(0.68, 0.52, 0.28), // honey pine
    };

    let hue_offset = rand_range(-0.04, 0.04);
    let seed_grain = rand_u32();
    let seed_rings = rand_u32();
    let seed_knots = rand_u32();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Long directional grain — stretched heavily along V for a plank feel.
            let grain =
                fbm_2d_periodic(u * 6.0, v * 28.0, 4, 0.5, 2.0, seed_grain, Some(6));

            // Broad colour variation across the surface.
            let variation =
                fbm_2d_periodic(u * 3.0, v * 3.0, 3, 0.45, 2.0, seed_grain + 1, Some(3));

            // Subtle ring lines — sinusoidal distortion of a radial pattern.
            let ring_distort =
                fbm_2d_periodic(u * 4.0, v * 4.0, 2, 0.4, 2.0, seed_rings, Some(4));
            let ring_u = u - 0.5 + ring_distort * 0.15;
            let ring_v = (v - 0.5) * 3.0;
            let ring_dist = (ring_u * ring_u + ring_v * ring_v).sqrt();
            let ring = ((ring_dist * 30.0).sin() * 0.5 + 0.5).powf(6.0) * 0.08;

            // Occasional dark knot spots.
            let knot_noise =
                fbm_2d_periodic(u * 2.0, v * 2.0, 3, 0.6, 2.0, seed_knots, Some(2));
            let knot = if knot_noise > 0.72 {
                (knot_noise - 0.72) / 0.28 * 0.2
            } else {
                0.0
            };

            let wood_factor = 0.80 + grain * 0.20;
            let hue_shift = (variation - 0.5) * 0.06 + hue_offset;

            let mut c = Rgb::new(
                (base.r + hue_shift) * wood_factor,
                (base.g + hue_shift * 0.5) * wood_factor,
                base.b * wood_factor,
            );

            // Apply ring pattern as a slight darkening.
            c = c.scale(1.0 - ring);

            // Apply knot darkening.
            c = c.scale(1.0 - knot);

            // Subtle edge darkening for depth.
            let eu = (u - 0.5).abs() * 2.0;
            let ev = (v - 0.5).abs() * 2.0;
            let edge = ((eu.max(ev) - 0.90) / 0.10).clamp(0.0, 1.0);
            c = c.scale(1.0 - edge * 0.15);

            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

/// Create a table material with a procedurally generated wood texture.
pub fn create_table_material(
    texture_manager: &TextureManager,
    material_builder: &mut MaterialManagerBuilder,
) -> EngineResult<MaterialId> {
    let pixels = generate_table_wood();
    let texture = texture_manager.create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
    let material = Material::textured(texture);
    Ok(material_builder.register(material))
}

// ---------------------------------------------------------------------------
// Model building
// ---------------------------------------------------------------------------

/// Generate cube vertices offset by a translation.
fn generate_offset_cube_vertices(
    half_extents: Vector3<f32>,
    offset: Vector3<f32>,
    colour: Colour,
) -> Vec<Vertex> {
    let mut verts = generate_cube_vertices(half_extents, colour);
    for v in &mut verts {
        v.pos.x += offset.x;
        v.pos.y += offset.y;
        v.pos.z += offset.z;
    }
    verts
}

/// Build a table model (top + 4 legs) as a single ModelPart with merged geometry.
fn build_table_model(dims: &TableDimensions, material: MaterialId) -> Arc<Model> {
    let top_he = dims.top_half_extents;
    let leg_he = dims.leg_half_extents;

    let table_height = dims.total_height();
    let top_y = table_height * 0.5 - top_he.y;
    let leg_y = -top_he.y;

    let leg_x = top_he.x - leg_he.x;
    let leg_z = top_he.z - leg_he.z;

    let mut all_vertices = Vec::new();
    let mut all_indices = Vec::new();

    // Helper: append a box's geometry at an offset.
    let mut append_box = |he: Vector3<f32>, offset: Vector3<f32>| {
        let base_idx = all_vertices.len() as u32;
        all_vertices.extend(generate_offset_cube_vertices(he, offset, Colour::WHITE));
        let indices = generate_cube_indices();
        all_indices.extend(indices.iter().map(|i| i + base_idx));
    };

    // Table top
    append_box(top_he, Vector3::new(0.0, top_y, 0.0));

    // Four legs
    let leg_positions = [
        Vector3::new(-leg_x, leg_y, -leg_z),
        Vector3::new(leg_x, leg_y, -leg_z),
        Vector3::new(-leg_x, leg_y, leg_z),
        Vector3::new(leg_x, leg_y, leg_z),
    ];
    for &pos in &leg_positions {
        append_box(leg_he, pos);
    }

    let parts = vec![ModelPart::new(vec![MeshPrimitive {
        vertices: all_vertices,
        indices: all_indices,
        material,
    }])];

    Arc::new(Model::flat(parts))
}

// ---------------------------------------------------------------------------
// Spawning
// ---------------------------------------------------------------------------

/// Spawn a table entity — a compound body with 5 box colliders and a merged mesh.
pub fn spawn_table(
    world: &mut World,
    initial_pos: Point3<f32>,
    dims: &TableDimensions,
    material: MaterialId,
    phys: &TablePhysics,
) -> Entity {
    let top_he = dims.top_half_extents;
    let leg_he = dims.leg_half_extents;

    let table_height = dims.total_height();
    let top_y = table_height * 0.5 - top_he.y;
    let leg_y = -top_he.y;

    let leg_x = top_he.x - leg_he.x;
    let leg_z = top_he.z - leg_he.z;

    let model = build_table_model(dims, material);

    let body_handle = {
        let mut physics = world.write_resource::<PhysicsResource>();

        let body_desc = RigidBodyDesc::dynamic()
            .position(initial_pos)
            .gravity_scale(1.0)
            .linear_damping(0.01)
            .angular_damping(0.005);

        let body_handle = physics.world.create_body(body_desc);

        // Table top collider
        physics.world.attach_collider(
            body_handle,
            ColliderDesc::box_shape(top_he)
                .offset_translation(Vector3::new(0.0, top_y, 0.0))
                .density(phys.density)
                .restitution(phys.restitution)
                .friction(phys.friction),
        );

        // Four leg colliders
        let leg_positions = [
            Vector3::new(-leg_x, leg_y, -leg_z),
            Vector3::new(leg_x, leg_y, -leg_z),
            Vector3::new(-leg_x, leg_y, leg_z),
            Vector3::new(leg_x, leg_y, leg_z),
        ];
        for &pos in &leg_positions {
            physics.world.attach_collider(
                body_handle,
                ColliderDesc::box_shape(leg_he)
                    .offset_translation(pos)
                    .density(phys.density)
                    .restitution(phys.restitution)
                    .friction(phys.friction),
            );
        }

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
