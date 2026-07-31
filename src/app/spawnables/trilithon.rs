//! Trilithon spawnable — neolithic stone gateway.
//!
//! Two upright megaliths with a horizontal lintel resting across the top,
//! in the style of Stonehenge. Each stone is an irregular convex polyhedron
//! (distorted cuboid) for a rough-hewn look. All three pieces are independent
//! rigid bodies so the structure can be toppled. A single cold grey stone
//! material is shared across all pieces.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::{build_convex_hull, convex_solid_model, SolidFace};
use super::shared::orientation::Yaw;
use super::shared::textures::Rgb;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::{Material, MaterialId};
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 256;

#[derive(Deserialize)]
pub struct TrilithonDef {
    pub pos: (f32, f32, f32),
    /// Rotation about `+Y`, in degrees — which way the doorway faces.
    #[serde(default)]
    pub yaw: f32,
    /// Half-height of each upright stone.
    #[serde(default = "TrilithonDef::default_upright_half_height")]
    pub upright_half_height: f32,
    /// Half-width (X) of each upright stone.
    #[serde(default = "TrilithonDef::default_upright_half_width")]
    pub upright_half_width: f32,
    /// Half-depth (Z) of each upright stone.
    #[serde(default = "TrilithonDef::default_upright_half_depth")]
    pub upright_half_depth: f32,
    /// Distance between the inner faces of the two uprights.
    #[serde(default = "TrilithonDef::default_gap")]
    pub gap: f32,
    /// Half-thickness (Y) of the lintel.
    #[serde(default = "TrilithonDef::default_lintel_half_thickness")]
    pub lintel_half_thickness: f32,
    /// How far the lintel overhangs past each upright (X direction).
    #[serde(default = "TrilithonDef::default_lintel_overhang")]
    pub lintel_overhang: f32,
    /// Stone density (kg/m^3). Default is granite.
    #[serde(default = "TrilithonDef::default_density")]
    pub density: f32,
}

impl TrilithonDef {
    pub fn default_upright_half_height() -> f32 {
        1.2
    }
    pub fn default_upright_half_width() -> f32 {
        0.3
    }
    pub fn default_upright_half_depth() -> f32 {
        0.4
    }
    pub fn default_gap() -> f32 {
        1.0
    }
    pub fn default_lintel_half_thickness() -> f32 {
        0.25
    }
    pub fn default_lintel_overhang() -> f32 {
        1.0
    }
    pub fn default_density() -> f32 {
        2700.0
    }
}

impl Spawnable for TrilithonDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let seed = rand::random::<u32>();
        let pixels = generate_standing_stone_texture(seed);
        let texture = ctx
            .textures
            .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
        Ok(vec![ctx.materials.register(Material::textured(texture))])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let material = materials[0];
        let base = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let seed = rand::random::<u32>();

        let upright_he = Vector3::new(
            self.upright_half_width,
            self.upright_half_height,
            self.upright_half_depth,
        );

        let x_offset = self.gap / 2.0 + self.upright_half_width;
        let upright_y = base.y + self.upright_half_height;

        // The trilithon's opening faces along its own ±Z, so yaw is what puts
        // the doorway where the author meant it.
        let yaw = Yaw::degrees(self.yaw);
        let origin = (base.x, base.y, base.z);
        let left_pos = yaw.place(origin, Vector3::new(-x_offset, upright_y - base.y, 0.0));
        let right_pos = yaw.place(origin, Vector3::new(x_offset, upright_y - base.y, 0.0));

        let lintel_half_x = x_offset + self.upright_half_width + self.lintel_overhang;
        let lintel_he = Vector3::new(
            lintel_half_x,
            self.lintel_half_thickness,
            self.upright_half_depth,
        );
        let lintel_y = base.y + self.upright_half_height * 2.0 + self.lintel_half_thickness;
        let lintel_pos = yaw.place(origin, Vector3::new(0.0, lintel_y - base.y, 0.0));

        // Each stone gets a different seed for unique distortion.
        let faces = cuboid_faces();

        let left_verts = distorted_cuboid(upright_he, seed, true, true);
        let right_verts = distorted_cuboid(upright_he, seed.wrapping_add(1), true, true);
        let lintel_verts = distorted_cuboid(lintel_he, seed.wrapping_add(2), false, true);

        let left_hull = Arc::new(build_convex_hull(&left_verts, &faces));
        let right_hull = Arc::new(build_convex_hull(&right_verts, &faces));
        let lintel_hull = Arc::new(build_convex_hull(&lintel_verts, &faces));

        let left_model = convex_solid_model(&left_verts, &faces, material);
        let right_model = convex_solid_model(&right_verts, &faces, material);
        let lintel_model = convex_solid_model(&lintel_verts, &faces, material);

        let (left_body, right_body, lintel_body) = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let left_body = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(left_pos)
                    .rotation(yaw.rotation())
                    .linear_damping(0.01)
                    .angular_damping(0.005),
            );
            physics.world.attach_collider(
                left_body,
                ColliderDesc::convex_hull(left_hull)
                    .density(self.density)
                    .restitution(0.05)
                    .friction(0.9),
            );

            let right_body = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(right_pos)
                    .rotation(yaw.rotation())
                    .linear_damping(0.01)
                    .angular_damping(0.005),
            );
            physics.world.attach_collider(
                right_body,
                ColliderDesc::convex_hull(right_hull)
                    .density(self.density)
                    .restitution(0.05)
                    .friction(0.9),
            );

            let lintel_body = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(lintel_pos)
                    .rotation(yaw.rotation())
                    .linear_damping(0.01)
                    .angular_damping(0.005),
            );
            physics.world.attach_collider(
                lintel_body,
                ColliderDesc::convex_hull(lintel_hull)
                    .density(self.density)
                    .restitution(0.05)
                    .friction(0.9),
            );

            (left_body, right_body, lintel_body)
        };

        let rotation = yaw.rotation();
        let spawn_entity = move |world: &mut World, pos: Point3<f32>, body, model| -> Entity {
            world
                .create_entity()
                .with(Position(Vector3::new(pos.x, pos.y, pos.z)))
                .with(Velocity(Vector3::zeros()))
                .with(Orientation(rotation))
                .with(RigidBodyComponent(body))
                .with(ModelInstance::new(model))
                .with(Renderable)
                .build()
        };

        vec![
            spawn_entity(world, left_pos, left_body, left_model),
            spawn_entity(world, right_pos, right_body, right_model),
            spawn_entity(world, lintel_pos, lintel_body, lintel_model),
        ]
    }
}

// ---------------------------------------------------------------------------
// Distorted cuboid geometry
// ---------------------------------------------------------------------------

/// Face definitions for a distorted cuboid's 8 vertices. Vertex layout:
///   0: (-x, -y, -z)  1: (+x, -y, -z)  2: (+x, +y, -z)  3: (-x, +y, -z)
///   4: (-x, -y, +z)  5: (+x, -y, +z)  6: (+x, +y, +z)  7: (-x, +y, +z)
fn cuboid_faces() -> Vec<SolidFace> {
    vec![
        // -Z face
        SolidFace {
            vertex_indices: vec![0, 3, 2, 1],
            opposite_vertex: 5,
        },
        // +Z face
        SolidFace {
            vertex_indices: vec![4, 5, 6, 7],
            opposite_vertex: 0,
        },
        // -X face
        SolidFace {
            vertex_indices: vec![0, 4, 7, 3],
            opposite_vertex: 1,
        },
        // +X face
        SolidFace {
            vertex_indices: vec![1, 2, 6, 5],
            opposite_vertex: 0,
        },
        // -Y face
        SolidFace {
            vertex_indices: vec![0, 1, 5, 4],
            opposite_vertex: 2,
        },
        // +Y face
        SolidFace {
            vertex_indices: vec![3, 7, 6, 2],
            opposite_vertex: 0,
        },
    ]
}

/// Generate 8 cuboid vertices with per-vertex random displacement for a
/// rough-hewn stone look. The `distort` fraction is relative to the smallest
/// half-extent so the shape stays convincingly solid.
///
/// `flat_top` / `flat_bottom` suppress Y displacement on the +Y / -Y face
/// vertices respectively, keeping those faces perfectly horizontal for stable
/// stacking surfaces.
fn distorted_cuboid(
    half_extents: Vector3<f32>,
    seed: u32,
    flat_top: bool,
    flat_bottom: bool,
) -> Vec<Vector3<f32>> {
    let he = half_extents;
    let distort = he.x.min(he.y).min(he.z) * 0.5;

    let base_verts = [
        Vector3::new(-he.x, -he.y, -he.z), // 0: -Y
        Vector3::new(he.x, -he.y, -he.z),  // 1: -Y
        Vector3::new(he.x, he.y, -he.z),   // 2: +Y
        Vector3::new(-he.x, he.y, -he.z),  // 3: +Y
        Vector3::new(-he.x, -he.y, he.z),  // 4: -Y
        Vector3::new(he.x, -he.y, he.z),   // 5: -Y
        Vector3::new(he.x, he.y, he.z),    // 6: +Y
        Vector3::new(-he.x, he.y, he.z),   // 7: +Y
    ];

    // Vertex pairs sharing the same column (bottom↔top at each corner).
    // Each pair gets the same X/Z distortion so the stone is irregular in
    // cross-section but vertically straight, keeping the center of mass
    // above the support polygon.
    let column_peer: [usize; 8] = [3, 2, 1, 0, 7, 6, 5, 4];
    let is_top = [false, false, true, true, false, false, true, true];
    let is_bottom = [true, true, false, false, true, true, false, false];

    base_verts
        .iter()
        .enumerate()
        .map(|(i, v)| {
            let canonical = i.min(column_peer[i]);
            let s = seed.wrapping_add(canonical as u32);
            let dx = hash_float(s, 0) * distort;
            let suppress_y = (flat_top && is_top[i]) || (flat_bottom && is_bottom[i]);
            let dy = if suppress_y {
                0.0
            } else {
                hash_float(s, 1) * distort
            };
            let dz = hash_float(s, 2) * distort;
            Vector3::new(v.x + dx, v.y + dy, v.z + dz)
        })
        .collect()
}

/// Deterministic float in -1..1 from a seed and channel.
fn hash_float(seed: u32, channel: u32) -> f32 {
    let mut h = seed
        .wrapping_mul(2654435761)
        .wrapping_add(channel.wrapping_mul(2246822519));
    h ^= h >> 13;
    h = h.wrapping_mul(1597334677);
    h ^= h >> 16;
    (h as f32 / u32::MAX as f32) * 2.0 - 1.0
}

// ---------------------------------------------------------------------------
// Texture generation
// ---------------------------------------------------------------------------

/// Cold grey standing stone texture with subtle grain, weathering stains,
/// and sparse lichen.
fn generate_standing_stone_texture(seed: u32) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let light_stone = Rgb::new(0.64, 0.63, 0.62);
    let mid_stone = Rgb::new(0.50, 0.49, 0.48);
    let dark_stone = Rgb::new(0.34, 0.33, 0.33);
    let quartz_light = Rgb::new(0.72, 0.71, 0.68);
    let dark_grain = Rgb::new(0.30, 0.29, 0.28);
    let lichen = Rgb::new(0.46, 0.50, 0.38);

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let broad = fbm_2d_periodic(u * 2.0, v * 2.0, 2, 0.5, 2.0, seed, Some(2));
            let t_base = (broad * 0.5 + 0.5).clamp(0.0, 1.0);
            let mut colour = light_stone.lerp(mid_stone, t_base);

            let rough = fbm_2d_periodic(
                u * 8.0,
                v * 8.0,
                3,
                0.5,
                2.0,
                seed.wrapping_add(10),
                Some(8),
            );
            colour = colour.scale(0.88 + rough * 0.12);

            let grain = fbm_2d_periodic(
                u * 28.0,
                v * 28.0,
                2,
                0.6,
                2.0,
                seed.wrapping_add(20),
                Some(28),
            );
            if grain > 0.28 {
                let t = ((grain - 0.28) / 0.4).clamp(0.0, 0.4);
                colour = colour.lerp(quartz_light, t);
            } else if grain < -0.2 {
                let t = ((-0.2 - grain) / 0.3).clamp(0.0, 0.35);
                colour = colour.lerp(dark_grain, t);
            }

            let streak_warp = fbm_2d_periodic(
                u * 4.0,
                v * 1.0,
                2,
                0.4,
                2.0,
                seed.wrapping_add(30),
                Some(4),
            );
            let streak = ((u * 10.0 + streak_warp * 1.0).sin() * 0.5 + 0.5).powi(6);
            colour = colour.lerp(dark_stone, streak * v * 0.25);

            let lichen_noise = fbm_2d_periodic(
                u * 5.0,
                v * 5.0,
                3,
                0.55,
                2.0,
                seed.wrapping_add(40),
                Some(5),
            );
            if lichen_noise > 0.32 {
                let t = ((lichen_noise - 0.32) / 0.35).clamp(0.0, 0.45);
                colour = colour.lerp(lichen, t);
            }

            let eu = (0.5 - (u - 0.5).abs()) * 2.0;
            let ev = (0.5 - (v - 0.5).abs()) * 2.0;
            let edge = eu.min(ev).clamp(0.0, 1.0);
            colour = colour.scale(0.93 + edge * 0.07);

            colour.write_rgba(&mut pixels);
        }
    }
    pixels
}
