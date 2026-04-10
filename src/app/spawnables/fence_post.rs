//! Fence post spawnable — terrain-anchored vertical cylinder.
//!
//! The post is pinned to the terrain surface via a Fixed constraint, with
//! its bottom third buried in terrain. While anchored the
//! physics collider covers only the exposed portion; on release it expands
//! to the full post length so the freed body tumbles with correct collision.
//!
//! Uses two materials: bark (barrel) and cross-section (caps with wood rings).

use std::f32::consts::TAU;
use std::sync::Arc;

use nalgebra::{Point3, Vector2, Vector3, Vector4};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::textures::Rgb;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, TerrainAnchored, Velocity,
};
use crate::core::error::EngineResult;
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{ColliderDesc, ConstraintKind, RigidBodyDesc};
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId};
use crate::rendering::vertex::Vertex;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainManager;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 256;
const MESH_SEGMENTS: u32 = 16;
/// Fraction of the post buried below the terrain surface.
const BURIED_FRACTION: f32 = 1.0 / 3.0;

#[derive(Deserialize)]
pub struct FencePostDef {
    /// Position (x, z). Y is determined by terrain surface height.
    pub pos: (f32, f32),
    #[serde(default = "FencePostDef::default_half_height")]
    pub half_height: f32,
    #[serde(default = "FencePostDef::default_radius")]
    pub radius: f32,
    #[serde(default = "FencePostDef::default_density")]
    pub density: f32,
}

impl FencePostDef {
    pub fn default_half_height() -> f32 {
        0.5
    }
    pub fn default_radius() -> f32 {
        0.08
    }
    pub fn default_density() -> f32 {
        800.0
    }
}

impl Spawnable for FencePostDef {
    fn material_count(&self) -> usize {
        2
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let seed = rand::random::<u32>();

        let bark_pixels = generate_bark_texture(seed);
        let bark_tex =
            ctx.textures
                .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &bark_pixels, true)?;
        let bark_mat = ctx.materials.register(Material::textured(bark_tex));

        let cross_pixels = generate_cross_section_texture(seed);
        let cross_tex =
            ctx.textures
                .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &cross_pixels, true)?;
        let cross_mat = ctx.materials.register(Material::textured(cross_tex));

        Ok(vec![bark_mat, cross_mat])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let bark_material = materials[0];
        let cross_material = materials[1];

        let surface_y = {
            let terrain = world.read_resource::<TerrainManager>();
            terrain.mesh_surface_height_at(self.pos.0, self.pos.1)
        };

        let Some(surface_y) = surface_y else {
            return Vec::new();
        };

        let full_height = self.half_height * 2.0;
        let buried_depth = full_height * BURIED_FRACTION;
        let exposed_height = full_height - buried_depth;

        // Body center is at the geometric center of the full post.
        let center_y = surface_y - buried_depth + self.half_height;
        let initial_pos = Point3::new(self.pos.0, center_y, self.pos.1);

        // Visual mesh: barrel + caps as separate primitives with different materials.
        let barrel = generate_barrel_mesh(self.half_height, self.radius, MESH_SEGMENTS);
        let caps = generate_cap_meshes(self.half_height, self.radius, MESH_SEGMENTS);

        let parts = vec![ModelPart::new(vec![
            MeshPrimitive {
                vertices: barrel.0,
                indices: barrel.1,
                material: bark_material,
            },
            MeshPrimitive {
                vertices: caps.0,
                indices: caps.1,
                material: cross_material,
            },
        ])];
        let model = Arc::new(Model::flat(parts));

        // Physics: while anchored, the collider covers only the exposed portion
        // and is offset upward so it aligns with the top of the mesh.
        let exposed_half_height = exposed_height / 2.0;
        let collider_offset_y = self.half_height - exposed_half_height;

        let anchored_collider = ColliderDesc::capsule(exposed_half_height, self.radius)
            .density(self.density)
            .restitution(0.1)
            .friction(0.7)
            .offset_translation(Vector3::new(0.0, collider_offset_y, 0.0));

        // Full-size collider for when the post is freed.
        let released_collider = ColliderDesc::capsule(self.half_height, self.radius)
            .density(self.density)
            .restitution(0.1)
            .friction(0.7);

        let (body_handle, anchor_handle, upright_handle) = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body_desc = RigidBodyDesc::dynamic()
                .position(initial_pos)
                .gravity_scale(1.0)
                .linear_damping(0.01)
                .angular_damping(0.05);

            let body_handle = physics.world.create_body(body_desc);
            physics
                .world
                .attach_collider(body_handle, anchored_collider);

            // Pin the bottom of the post to its buried position.
            let local_anchor = Vector3::new(0.0, -self.half_height, 0.0);
            let world_anchor = Point3::new(self.pos.0, surface_y - buried_depth, self.pos.1);

            let fixed_handle = physics
                .world
                .create_constraint(ConstraintKind::world_fixed(
                    body_handle,
                    world_anchor,
                    local_anchor,
                    0.0,
                    f32::MAX,
                ));

            (body_handle, fixed_handle, fixed_handle)
        };

        // Check terrain slightly below the surface so the sample is inside solid voxels.
        let anchor_check = Point3::new(self.pos.0, surface_y - 0.1, self.pos.1);

        vec![world
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
            .with(TerrainAnchored {
                anchor_handle,
                upright_handle,
                anchor_world: anchor_check,
                released_collider: Some(released_collider),
                released_model: None,
            })
            .build()]
    }
}

// ---------------------------------------------------------------------------
// Mesh generation
// ---------------------------------------------------------------------------

/// Barrel (cylinder sides) with outward normals. UV wraps texture around.
fn generate_barrel_mesh(half_height: f32, radius: f32, segments: u32) -> (Vec<Vertex>, Vec<u32>) {
    let colour_vec = Colour::WHITE.to_vec4();
    let mut vertices = Vec::with_capacity((segments * 2) as usize);
    let mut indices = Vec::with_capacity((segments * 6) as usize);

    for i in 0..segments {
        let angle = i as f32 * TAU / segments as f32;
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        let normal = Vector3::new(cos_a, 0.0, sin_a).normalize();
        let u = i as f32 / segments as f32;

        vertices.push(Vertex {
            pos: Vector4::new(cos_a * radius, -half_height, sin_a * radius, 1.0),
            color: colour_vec,
            tex_coords: Vector2::new(u, 1.0),
            normal,
        });
        vertices.push(Vertex {
            pos: Vector4::new(cos_a * radius, half_height, sin_a * radius, 1.0),
            color: colour_vec,
            tex_coords: Vector2::new(u, 0.0),
            normal,
        });
    }

    for i in 0..segments {
        let i0 = i * 2;
        let i1 = i * 2 + 1;
        let i2 = ((i + 1) % segments) * 2;
        let i3 = ((i + 1) % segments) * 2 + 1;
        indices.extend_from_slice(&[i0, i1, i2, i2, i1, i3]);
    }

    (vertices, indices)
}

/// Top and bottom disc caps with planar UV projection.
fn generate_cap_meshes(half_height: f32, radius: f32, segments: u32) -> (Vec<Vertex>, Vec<u32>) {
    let colour_vec = Colour::WHITE.to_vec4();
    let cap_verts = (segments + 1) as usize;
    let mut vertices = Vec::with_capacity(cap_verts * 2);
    let mut indices = Vec::with_capacity((segments * 3 * 2) as usize);

    // Top cap
    let top_center = vertices.len() as u32;
    vertices.push(Vertex {
        pos: Vector4::new(0.0, half_height, 0.0, 1.0),
        color: colour_vec,
        tex_coords: Vector2::new(0.5, 0.5),
        normal: Vector3::y(),
    });
    for i in 0..segments {
        let angle = i as f32 * TAU / segments as f32;
        vertices.push(Vertex {
            pos: Vector4::new(angle.cos() * radius, half_height, angle.sin() * radius, 1.0),
            color: colour_vec,
            tex_coords: Vector2::new(0.5 + angle.cos() * 0.5, 0.5 + angle.sin() * 0.5),
            normal: Vector3::y(),
        });
    }
    for i in 0..segments {
        let curr = top_center + 1 + i;
        let next = top_center + 1 + (i + 1) % segments;
        indices.extend_from_slice(&[top_center, next, curr]);
    }

    // Bottom cap
    let bot_center = vertices.len() as u32;
    vertices.push(Vertex {
        pos: Vector4::new(0.0, -half_height, 0.0, 1.0),
        color: colour_vec,
        tex_coords: Vector2::new(0.5, 0.5),
        normal: -Vector3::y(),
    });
    for i in 0..segments {
        let angle = i as f32 * TAU / segments as f32;
        vertices.push(Vertex {
            pos: Vector4::new(
                angle.cos() * radius,
                -half_height,
                angle.sin() * radius,
                1.0,
            ),
            color: colour_vec,
            tex_coords: Vector2::new(0.5 + angle.cos() * 0.5, 0.5 - angle.sin() * 0.5),
            normal: -Vector3::y(),
        });
    }
    for i in 0..segments {
        let curr = bot_center + 1 + i;
        let next = bot_center + 1 + (i + 1) % segments;
        indices.extend_from_slice(&[bot_center, curr, next]);
    }

    (vertices, indices)
}

// ---------------------------------------------------------------------------
// Texture generation
// ---------------------------------------------------------------------------

/// Bark texture for the cylinder barrel.
///
/// Layers vertical fibrous grain with coarse bark ridges, knots, and
/// weathering to produce a rough, organic tree-trunk look.
fn generate_bark_texture(seed: u32) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    // Colour palette: varied browns from pale heartwood to dark bark.
    let light_bark = Rgb::new(0.50, 0.36, 0.22);
    let mid_bark = Rgb::new(0.38, 0.25, 0.14);
    let dark_bark = Rgb::new(0.22, 0.14, 0.08);
    let ridge_colour = Rgb::new(0.18, 0.10, 0.06);

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Fine vertical grain: high horizontal frequency, low vertical.
            let fine_grain = fbm_2d_periodic(u * 20.0, v * 3.0, 3, 0.5, 2.0, seed, Some(20));

            // Coarse bark ridges: wide vertical bands that shift horizontally.
            let ridge_warp = fbm_2d_periodic(
                u * 3.0,
                v * 1.5,
                2,
                0.4,
                2.0,
                seed.wrapping_add(10),
                Some(3),
            );
            let ridge = ((u * 8.0 + ridge_warp * 1.5).sin() * 0.5 + 0.5).powi(3);

            // Large-scale colour variation across the surface.
            let broad_variation = fbm_2d_periodic(
                u * 2.0,
                v * 2.0,
                2,
                0.5,
                2.0,
                seed.wrapping_add(20),
                Some(2),
            );

            // Blend base colour from light to mid based on broad variation.
            let t_base = (broad_variation * 0.5 + 0.5).clamp(0.0, 1.0);
            let base = light_bark.lerp(mid_bark, t_base);

            // Apply fine grain darkening.
            let grain_factor = 0.80 + fine_grain * 0.20;
            let mut colour = base.scale(grain_factor);

            // Blend in dark bark ridges.
            let ridge_t = (ridge * 0.7).clamp(0.0, 1.0);
            colour = colour.lerp(ridge_colour, ridge_t);

            // Occasional dark patches (simulating damage, lichen, etc.).
            let patches = fbm_2d_periodic(
                u * 6.0,
                v * 6.0,
                3,
                0.6,
                2.0,
                seed.wrapping_add(30),
                Some(6),
            );
            if patches > 0.35 {
                let patch_t = ((patches - 0.35) / 0.3).clamp(0.0, 0.4);
                colour = colour.lerp(dark_bark, patch_t);
            }

            // Weathering: slight desaturation and lightening at the top (v≈0).
            let weather_gradient = (1.0 - v).powi(3) * 0.12;
            let weathered = Rgb::new(0.55, 0.48, 0.40);
            colour = colour.lerp(weathered, weather_gradient);

            colour.write_rgba(&mut pixels);
        }
    }
    pixels
}

/// Cross-section texture for the flat caps.
///
/// Concentric growth rings radiating from a slightly off-center pith,
/// with radial cracks and a paler heartwood colour.
fn generate_cross_section_texture(seed: u32) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let heartwood = Rgb::new(0.65, 0.50, 0.32);
    let ring_light = Rgb::new(0.58, 0.44, 0.28);
    let ring_dark = Rgb::new(0.40, 0.28, 0.16);
    let pith_colour = Rgb::new(0.50, 0.38, 0.24);

    // Slightly off-center pith for a natural look.
    let pith_noise = fbm_2d_periodic(0.5, 0.5, 1, 0.5, 2.0, seed.wrapping_add(50), Some(1));
    let pith_x = 0.5 + pith_noise * 0.06;
    let pith_y =
        0.5 + fbm_2d_periodic(0.3, 0.7, 1, 0.5, 2.0, seed.wrapping_add(51), Some(1)) * 0.06;

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let dx = u - pith_x;
            let dy = v - pith_y;
            let dist = (dx * dx + dy * dy).sqrt();

            // Growth rings: sinusoidal bands that warp slightly.
            let angle = dy.atan2(dx);
            let ring_warp = fbm_2d_periodic(
                angle * 2.0 / TAU + 0.5,
                dist * 4.0,
                2,
                0.4,
                2.0,
                seed.wrapping_add(60),
                Some(2),
            ) * 0.02;
            let ring_dist = dist + ring_warp;
            let ring_phase = (ring_dist * 7.0).fract();
            // Sharper transition: thin dark line for each ring boundary.
            let ring_t = if ring_phase < 0.15 {
                (ring_phase / 0.15).powi(2)
            } else {
                1.0 - ((ring_phase - 0.15) / 0.85).powi(2) * 0.3
            };

            // Blend between light and dark ring colours.
            let ring_colour = ring_dark.lerp(ring_light, ring_t);

            // Blend heartwood toward center (paler, less ring contrast).
            let heartwood_t = (1.0 - (dist * 3.0).min(1.0)).powi(2);
            let mut colour = ring_colour.lerp(heartwood, heartwood_t * 0.5);

            // Pith: small dark spot at center.
            if dist < 0.03 {
                let pith_t = 1.0 - (dist / 0.03);
                colour = colour.lerp(pith_colour, pith_t * 0.6);
            }

            // Radial cracks: thin dark lines radiating from center.
            let crack_noise = fbm_2d_periodic(
                angle * 3.0 / TAU + 0.5,
                0.5,
                3,
                0.5,
                2.0,
                seed.wrapping_add(70),
                Some(3),
            );
            let crack_sharpness = (crack_noise - 0.42).abs();
            if crack_sharpness < 0.008 && dist > 0.05 {
                let crack_t = (1.0 - crack_sharpness / 0.008) * 0.5;
                let crack_fade = (dist * 2.5).min(1.0); // cracks widen toward edge
                colour = colour.lerp(ring_dark, crack_t * crack_fade);
            }

            // Mask to circle: darken outside the disc radius.
            let edge_dist = (dist - 0.45).max(0.0) / 0.05;
            if edge_dist > 0.0 {
                colour = colour.scale(1.0 - edge_dist.min(1.0) * 0.4);
            }

            // Subtle noise for natural variation.
            let noise = fbm_2d_periodic(
                u * 8.0,
                v * 8.0,
                2,
                0.3,
                2.0,
                seed.wrapping_add(80),
                Some(8),
            );
            colour = colour.scale(0.94 + noise * 0.06);

            colour.write_rgba(&mut pixels);
        }
    }
    pixels
}
