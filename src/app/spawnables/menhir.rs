//! Menhir spawnable — terrain-anchored standing stone.
//!
//! An egg-shaped monolith pinned to the terrain via AnchorPoint +
//! KeepUpright constraints. The geometry is a UV sphere whose horizontal
//! radius blends linearly from `bottom_radius` (wide base) to `top_radius`
//! (narrow tip) as a function of latitude, producing a natural egg profile.
//!
//! The bottom portion is buried below the surface. When the terrain beneath
//! is destroyed, the anchor releases and the menhir topples as a free body.

use std::f32::consts::{FRAC_PI_2, TAU};
use std::sync::Arc;

use nalgebra::{Point3, UnitVector3, Vector2, Vector3, Vector4};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::{build_convex_hull, SolidFace};
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
/// Longitude segments (around the equator).
const SEGMENTS: usize = 8;
/// Latitude rings between the poles (exclusive of poles themselves).
const RINGS: usize = 6;
/// Fraction of the menhir buried below the terrain surface.
const BURIED_FRACTION: f32 = 0.2;

#[derive(Deserialize)]
pub struct MenhirDef {
    /// Position (x, z). Y is determined by terrain surface height.
    pub pos: (f32, f32),
    /// Half-height of the full stone (including buried portion).
    #[serde(default = "MenhirDef::default_half_height")]
    pub half_height: f32,
    /// Horizontal radius at the widest point of the bottom half.
    #[serde(default = "MenhirDef::default_bottom_radius")]
    pub bottom_radius: f32,
    /// Horizontal radius at the widest point of the top half.
    #[serde(default = "MenhirDef::default_top_radius")]
    pub top_radius: f32,
    /// Stone density (kg/m^3). Default is granite.
    #[serde(default = "MenhirDef::default_density")]
    pub density: f32,
}

impl MenhirDef {
    pub fn default_half_height() -> f32 {
        1.0
    }
    pub fn default_bottom_radius() -> f32 {
        0.4
    }
    pub fn default_top_radius() -> f32 {
        0.25
    }
    pub fn default_density() -> f32 {
        2700.0
    }
}

impl Spawnable for MenhirDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let seed = rand::random::<u32>();
        let pixels = generate_stone_texture(seed);
        let texture =
            ctx.textures
                .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
        Ok(vec![ctx.materials.register(Material::textured(texture))])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let material = materials[0];

        let surface_y = {
            let terrain = world.read_resource::<TerrainManager>();
            terrain.mesh_surface_height_at(self.pos.0, self.pos.1)
        };

        let Some(surface_y) = surface_y else {
            return Vec::new();
        };

        let full_height = self.half_height * 2.0;
        let buried_depth = full_height * BURIED_FRACTION;

        let center_y = surface_y - buried_depth + self.half_height;
        let initial_pos = Point3::new(self.pos.0, center_y, self.pos.1);

        let egg = EggParams {
            half_height: self.half_height,
            bottom_radius: self.bottom_radius,
            top_radius: self.top_radius,
        };

        let model = build_egg_mesh(&egg, material);
        let hull = Arc::new(build_egg_hull(&egg));

        let exposed_half_height = (full_height - buried_depth) / 2.0;
        let collider_offset_y = self.half_height - exposed_half_height;

        let anchored_collider = ColliderDesc::convex_hull(hull.clone())
            .density(self.density)
            .restitution(0.1)
            .friction(0.8)
            .offset_translation(Vector3::new(0.0, collider_offset_y, 0.0));

        let released_collider = ColliderDesc::convex_hull(hull)
            .density(self.density)
            .restitution(0.1)
            .friction(0.8);

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

            let local_anchor = Vector3::new(0.0, -self.half_height, 0.0);
            let world_anchor = Point3::new(self.pos.0, surface_y - buried_depth, self.pos.1);

            let anchor_handle = physics
                .world
                .create_constraint(ConstraintKind::AnchorPoint {
                    body: body_handle,
                    local_anchor,
                    world_anchor,
                    compliance: 0.0,
                    max_impulse: f32::MAX,
                    lock_yaw: true,
                    lock_roll: false,
                });

            let upright_handle = physics
                .world
                .create_constraint(ConstraintKind::KeepUpright {
                    body: body_handle,
                    target_up: UnitVector3::new_normalize(Vector3::y()),
                    compliance: 0.0,
                });

            (body_handle, anchor_handle, upright_handle)
        };

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
            })
            .build()]
    }
}

// ---------------------------------------------------------------------------
// Geometry — UV sphere scaled into an ellipsoid
// ---------------------------------------------------------------------------

/// Parameters defining the egg profile.
struct EggParams {
    half_height: f32,
    bottom_radius: f32,
    top_radius: f32,
}

impl EggParams {
    /// Horizontal radius at latitude `phi` (−π/2 = south pole, +π/2 = north).
    /// Linearly blends between `bottom_radius` and `top_radius` based on
    /// height, multiplied by the ellipse envelope `cos(phi)`.
    fn radius_at(&self, phi: f32) -> f32 {
        let t = (phi.sin() + 1.0) * 0.5; // 0 at south, 1 at north
        let r = self.bottom_radius + (self.top_radius - self.bottom_radius) * t;
        r * phi.cos()
    }

    /// Egg surface point at latitude `phi` and longitude `theta`.
    fn point(&self, phi: f32, theta: f32) -> Vector3<f32> {
        let r = self.radius_at(phi);
        Vector3::new(r * theta.cos(), self.half_height * phi.sin(), r * theta.sin())
    }
}

/// Build a visual mesh: UV sphere with `SEGMENTS` longitude and `RINGS`
/// latitude divisions, shaped into an egg by `EggParams`. Cylindrical UV
/// mapping with seam duplication.
fn build_egg_mesh(egg: &EggParams, material: MaterialId) -> Arc<Model> {
    let n = SEGMENTS;
    let r = RINGS;
    let colour_vec = Colour::WHITE.to_vec4();

    let verts_per_ring = n + 1; // seam duplication
    let mut vertices = Vec::with_capacity(r * verts_per_ring + 2);
    let mut indices = Vec::new();

    // South pole (bottom).
    let south = 0u32;
    vertices.push(Vertex {
        pos: Vector4::new(0.0, -egg.half_height, 0.0, 1.0),
        color: colour_vec,
        tex_coords: Vector2::new(0.5, 1.0),
        normal: -Vector3::y(),
    });

    // Latitude rings from south to north.
    let ring_base = 1u32;
    for ri in 0..r {
        let phi = -FRAC_PI_2 + (ri as f32 + 1.0) / (r as f32 + 1.0) * std::f32::consts::PI;
        let v_coord = 1.0 - (ri as f32 + 1.0) / (r as f32 + 1.0);

        for si in 0..=n {
            let theta = si as f32 * TAU / n as f32;
            let pos = egg.point(phi, theta);

            // Approximate normal via finite-difference on the egg surface.
            let eps = 1e-3;
            let dp_dphi = egg.point(phi + eps, theta) - egg.point(phi - eps, theta);
            let dp_dtheta = egg.point(phi, theta + eps) - egg.point(phi, theta - eps);
            let normal = dp_dphi.cross(&dp_dtheta).normalize();

            let u_coord = si as f32 / n as f32;

            vertices.push(Vertex {
                pos: Vector4::new(pos.x, pos.y, pos.z, 1.0),
                color: colour_vec,
                tex_coords: Vector2::new(u_coord, v_coord),
                normal,
            });
        }
    }

    // North pole (top).
    let north = vertices.len() as u32;
    vertices.push(Vertex {
        pos: Vector4::new(0.0, egg.half_height, 0.0, 1.0),
        color: colour_vec,
        tex_coords: Vector2::new(0.5, 0.0),
        normal: Vector3::y(),
    });

    let rv = |ri: usize, si: usize| -> u32 { ring_base + (ri * verts_per_ring + si) as u32 };

    // South fan.
    for si in 0..n {
        indices.extend_from_slice(&[south, rv(0, si), rv(0, si + 1)]);
    }

    // Quads between adjacent rings.
    for ri in 0..(r - 1) {
        for si in 0..n {
            let b0 = rv(ri, si);
            let b1 = rv(ri, si + 1);
            let t0 = rv(ri + 1, si);
            let t1 = rv(ri + 1, si + 1);
            indices.extend_from_slice(&[b0, t1, b1, b0, t0, t1]);
        }
    }

    // North fan.
    let last = r - 1;
    for si in 0..n {
        indices.extend_from_slice(&[rv(last, si), north, rv(last, si + 1)]);
    }

    let parts = vec![ModelPart::new(vec![MeshPrimitive {
        vertices,
        indices,
        material,
    }])];
    Arc::new(Model::flat(parts))
}

/// Build a convex hull from the same egg vertices (no seam duplication
/// needed for physics).
fn build_egg_hull(egg: &EggParams) -> crate::collision::ConvexHull {
    let n = SEGMENTS;
    let r = RINGS;
    let mut vertices = Vec::with_capacity(r * n + 2);

    // South pole.
    vertices.push(Vector3::new(0.0, -egg.half_height, 0.0));

    for ri in 0..r {
        let phi = -FRAC_PI_2 + (ri as f32 + 1.0) / (r as f32 + 1.0) * std::f32::consts::PI;
        for si in 0..n {
            let theta = si as f32 * TAU / n as f32;
            vertices.push(egg.point(phi, theta));
        }
    }

    // North pole.
    let north = vertices.len();
    vertices.push(Vector3::new(0.0, egg.half_height, 0.0));

    let mut faces = Vec::new();

    // Vertex index helper: 0 = south pole, 1..=r*n = ring verts, north = last.
    let rv = |ri: usize, si: usize| -> usize { 1 + ri * n + si };

    // South fan triangles.
    for si in 0..n {
        let next = (si + 1) % n;
        faces.push(SolidFace {
            vertex_indices: vec![0, rv(0, next), rv(0, si)],
            opposite_vertex: north,
        });
    }

    // Side quads.
    for ri in 0..(r - 1) {
        for si in 0..n {
            let next = (si + 1) % n;
            faces.push(SolidFace {
                vertex_indices: vec![rv(ri, si), rv(ri, next), rv(ri + 1, next), rv(ri + 1, si)],
                opposite_vertex: rv(ri, (si + n / 2) % n),
            });
        }
    }

    // North fan triangles.
    let last = r - 1;
    for si in 0..n {
        let next = (si + 1) % n;
        faces.push(SolidFace {
            vertex_indices: vec![rv(last, si), rv(last, next), north],
            opposite_vertex: 0,
        });
    }

    build_convex_hull(&vertices, &faces)
}

// ---------------------------------------------------------------------------
// Texture generation
// ---------------------------------------------------------------------------

/// Procedural grey stone texture with visible grain, veins, and patches.
fn generate_stone_texture(seed: u32) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let base_grey = Rgb::new(0.58, 0.56, 0.54);
    let light_grey = Rgb::new(0.72, 0.70, 0.68);
    let dark_grey = Rgb::new(0.38, 0.36, 0.34);
    let vein_colour = Rgb::new(0.32, 0.30, 0.29);

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Broad colour variation — large blotchy regions.
            let broad = fbm_2d_periodic(u * 3.0, v * 3.0, 3, 0.5, 2.0, seed, Some(3));
            let t = (broad * 0.5 + 0.5).clamp(0.0, 1.0);
            let mut colour = light_grey.lerp(base_grey, t);

            // Medium undulation — gives a bumpy, weathered look.
            let mid = fbm_2d_periodic(
                u * 8.0,
                v * 8.0,
                3,
                0.5,
                2.0,
                seed.wrapping_add(10),
                Some(8),
            );
            colour = colour.scale(0.85 + mid * 0.15);

            // Dark patches — lichen or mineral deposits.
            let patch = fbm_2d_periodic(
                u * 4.0,
                v * 4.0,
                2,
                0.5,
                2.0,
                seed.wrapping_add(20),
                Some(4),
            );
            if patch > 0.15 {
                let pt = ((patch - 0.15) / 0.4).clamp(0.0, 0.5);
                colour = colour.lerp(dark_grey, pt);
            }

            // Thin veins / cracks — high-frequency ridgeline pattern.
            let vein_raw = fbm_2d_periodic(
                u * 14.0,
                v * 14.0,
                3,
                0.6,
                2.0,
                seed.wrapping_add(40),
                Some(14),
            );
            let vein_strength = (1.0 - (vein_raw * 4.0).abs()).max(0.0);
            colour = colour.lerp(vein_colour, vein_strength * 0.6);

            // Fine grain — mineral speckle.
            let fine = fbm_2d_periodic(
                u * 24.0,
                v * 24.0,
                2,
                0.5,
                2.0,
                seed.wrapping_add(30),
                Some(24),
            );
            colour = colour.scale(0.90 + fine * 0.10);

            colour.write_rgba(&mut pixels);
        }
    }
    pixels
}
