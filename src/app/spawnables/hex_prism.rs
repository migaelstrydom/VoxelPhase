//! Hexagonal prism spawnable — honeycomb cell with ConvexHull collider.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::finish::{ColliderSurface, MaterialSurface};
use super::shared::models::{build_convex_hull, convex_solid_model, SolidFace};
use super::shared::orientation::Yaw;
use super::shared::textures::*;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::{Material, MaterialId, MaterialManagerBuilder};
use crate::rendering::physical_finish::PhysicalSurface;
use crate::resources::textures::TextureManager;
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 128;

#[derive(Deserialize)]
pub struct HexPrismDef {
    pub pos: (f32, f32, f32),
    /// Circumradius of the hexagonal cross-section (center to vertex).
    #[serde(default = "HexPrismDef::default_radius")]
    pub radius: f32,
    /// Half-height of the prism along the Y axis.
    #[serde(default = "HexPrismDef::default_half_height")]
    pub half_height: f32,
    #[serde(default = "HexPrismDef::default_density")]
    pub density: f32,
    #[serde(default = "HexPrismDef::default_restitution")]
    pub restitution: f32,
    #[serde(default = "HexPrismDef::default_friction")]
    pub friction: f32,
}

impl HexPrismDef {
    pub fn default_radius() -> f32 {
        0.5
    }
    pub fn default_half_height() -> f32 {
        0.3
    }
    pub fn default_density() -> f32 {
        500.0
    }
    pub fn default_restitution() -> f32 {
        0.2
    }
    pub fn default_friction() -> f32 {
        0.6
    }

    /// The one declaration of this prism's physics, shading included.
    fn surface(&self) -> PhysicalSurface {
        PhysicalSurface {
            restitution: self.restitution,
            friction: self.friction,
            density: self.density,
        }
    }
}

impl Spawnable for HexPrismDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let mat = create_honeycomb_material(self.surface(), ctx.textures, ctx.materials)?;
        Ok(vec![mat])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let initial_pos = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let material = materials[0];

        let (vertices, faces) = hex_prism_geometry(self.radius, self.half_height);
        let hull = Arc::new(build_convex_hull(&vertices, &faces));
        let model = convex_solid_model(&vertices, &faces, material);

        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body_desc = RigidBodyDesc::dynamic()
                .position(initial_pos)
                .gravity_scale(1.0)
                .linear_damping(0.01)
                .angular_damping(0.005);

            let body_handle = physics.world.create_body(body_desc);

            let collider_desc =
                ColliderDesc::convex_hull(hull).with_physical_surface(self.surface());

            physics.world.attach_collider(body_handle, collider_desc);

            body_handle
        };

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
            .build()]
    }
}

// ---------------------------------------------------------------------------
// HoneycombWall — a tiled hex-prism wall
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
pub struct HoneycombWallDef {
    /// World position of the bottom-center of the wall.
    pub base: (f32, f32, f32),
    /// Number of columns along the wall's own `+X`.
    #[serde(default = "HoneycombWallDef::default_columns")]
    pub columns: u32,
    /// Rotation about `+Y`, in degrees — which way the wall faces.
    #[serde(default)]
    pub yaw: f32,
    /// Number of rows.
    #[serde(default = "HoneycombWallDef::default_rows")]
    pub rows: u32,
    /// Circumradius of each hexagonal cell.
    #[serde(default = "HexPrismDef::default_radius")]
    pub radius: f32,
    /// Half-height (depth) of each cell.
    #[serde(default = "HexPrismDef::default_half_height")]
    pub half_height: f32,
    #[serde(default = "HexPrismDef::default_density")]
    pub density: f32,
}

impl HoneycombWallDef {
    pub fn default_columns() -> u32 {
        5
    }
    pub fn default_rows() -> u32 {
        4
    }

    fn total_cells(&self) -> usize {
        (self.columns * self.rows) as usize
    }

    /// Cells in a wall sit a little grippier and deader than a loose prism, so
    /// the wall holds its course instead of shedding cells.
    fn surface(&self) -> PhysicalSurface {
        PhysicalSurface {
            restitution: 0.15,
            friction: 0.7,
            density: self.density,
        }
    }
}

impl Spawnable for HoneycombWallDef {
    fn material_count(&self) -> usize {
        self.total_cells()
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let count = self.total_cells();
        let mut mats = Vec::with_capacity(count);
        for _ in 0..count {
            mats.push(create_honeycomb_material(
                self.surface(),
                ctx.textures,
                ctx.materials,
            )?);
        }
        Ok(mats)
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        // Wall geometry: prism axis along Z so hex faces point toward/away
        // from the viewer, and the tiling grid lies in the XY plane.
        let (vertices, faces) = hex_prism_geometry_z(self.radius, self.half_height);
        let hull = Arc::new(build_convex_hull(&vertices, &faces));

        // Pointy-topped hex tiling in XY: flat edges at top/bottom of each cell.
        // Horizontal spacing: sqrt(3) * radius (center-to-center along X).
        // Vertical spacing: 1.5 * radius (center-to-center along Y).
        // Odd rows are offset right by half the horizontal spacing.
        let col_spacing = self.radius * 3.0f32.sqrt();
        let row_spacing = self.radius * 1.5;

        let total_width = (self.columns - 1) as f32 * col_spacing;
        let x_start = self.base.0 - total_width * 0.5;
        let yaw = Yaw::degrees(self.yaw);

        let mut entities = Vec::with_capacity(self.total_cells());
        let mut mat_idx = 0;

        for row in 0..self.rows {
            let y = self.base.1 + self.radius + row as f32 * row_spacing;
            let x_offset = if row % 2 == 1 { col_spacing * 0.5 } else { 0.0 };

            for col in 0..self.columns {
                let x = x_start + col as f32 * col_spacing + x_offset;
                let pos = yaw.place(
                    self.base,
                    Vector3::new(x - self.base.0, y - self.base.1, 0.0),
                );

                let model = convex_solid_model(&vertices, &faces, materials[mat_idx]);

                let body_handle = {
                    let mut physics = world.write_resource::<PhysicsResource>();
                    let body_desc = RigidBodyDesc::dynamic()
                        .position(pos)
                        .rotation(yaw.rotation())
                        .gravity_scale(1.0)
                        .linear_damping(0.01)
                        .angular_damping(0.005);
                    let body_handle = physics.world.create_body(body_desc);
                    physics.world.attach_collider(
                        body_handle,
                        ColliderDesc::convex_hull(hull.clone())
                            .with_physical_surface(self.surface()),
                    );
                    body_handle
                };

                entities.push(
                    world
                        .create_entity()
                        .with(Position(Vector3::new(pos.x, pos.y, pos.z)))
                        .with(Velocity(Vector3::zeros()))
                        .with(Orientation(yaw.rotation()))
                        .with(RigidBodyComponent(body_handle))
                        .with(ModelInstance::new(model))
                        .with(Renderable)
                        .build(),
                );

                mat_idx += 1;
            }
        }

        entities
    }
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

/// Vertices and faces of a hexagonal prism.
///
/// 12 vertices: 6 on the top hex, 6 on the bottom hex.
/// 8 faces: top hex, bottom hex, 6 rectangular sides.
///
/// The hex is flat-topped (a vertex at +X, edges parallel to Z).
pub fn hex_prism_geometry(radius: f32, half_height: f32) -> (Vec<Vector3<f32>>, Vec<SolidFace>) {
    let mut vertices = Vec::with_capacity(12);

    // Top and bottom hex rings. Vertex 0 at +X, winding CCW from above.
    for i in 0..6 {
        let angle = std::f32::consts::FRAC_PI_3 * i as f32;
        let x = radius * angle.cos();
        let z = radius * angle.sin();
        vertices.push(Vector3::new(x, half_height, z));
    }
    for i in 0..6 {
        let angle = std::f32::consts::FRAC_PI_3 * i as f32;
        let x = radius * angle.cos();
        let z = radius * angle.sin();
        vertices.push(Vector3::new(x, -half_height, z));
    }

    let mut faces = Vec::with_capacity(8);

    // Top face (indices 0–5). Opposite vertex is any bottom vertex.
    faces.push(SolidFace {
        vertex_indices: vec![0, 1, 2, 3, 4, 5],
        opposite_vertex: 6,
    });

    // Bottom face (indices 6–11, reversed winding). Opposite is any top vertex.
    faces.push(SolidFace {
        vertex_indices: vec![11, 10, 9, 8, 7, 6],
        opposite_vertex: 0,
    });

    // 6 rectangular side faces. Each connects top edge (i, i+1) to bottom edge.
    // Opposite vertex is the top vertex directly across the hex.
    for i in 0..6usize {
        let next = (i + 1) % 6;
        let opposite = (i + 3) % 6;
        faces.push(SolidFace {
            vertex_indices: vec![i, i + 6, next + 6, next],
            opposite_vertex: opposite,
        });
    }

    (vertices, faces)
}

/// Hex prism with the prism axis along Z (hex faces point ±Z).
///
/// Used by the honeycomb wall so cells tile in the XY plane with the flat
/// hex faces pointing toward/away from the viewer. The hex ring is
/// pointy-topped (a vertex at +Y) so flat edges sit horizontally — this
/// gives the classic honeycomb look when viewed from the front.
fn hex_prism_geometry_z(radius: f32, half_depth: f32) -> (Vec<Vector3<f32>>, Vec<SolidFace>) {
    let mut vertices = Vec::with_capacity(12);

    // Front and back hex rings in the XY plane.
    // Pointy-topped: vertex 0 at +Y, winding CCW from the front (+Z side).
    for i in 0..6 {
        let angle = std::f32::consts::FRAC_PI_3 * i as f32 + std::f32::consts::FRAC_PI_6;
        let x = radius * angle.cos();
        let y = radius * angle.sin();
        vertices.push(Vector3::new(x, y, half_depth)); // front: indices 0–5
    }
    for i in 0..6 {
        let angle = std::f32::consts::FRAC_PI_3 * i as f32 + std::f32::consts::FRAC_PI_6;
        let x = radius * angle.cos();
        let y = radius * angle.sin();
        vertices.push(Vector3::new(x, y, -half_depth)); // back: indices 6–11
    }

    let mut faces = Vec::with_capacity(8);

    // Front face (indices 0–5). Opposite is any back vertex.
    faces.push(SolidFace {
        vertex_indices: vec![0, 1, 2, 3, 4, 5],
        opposite_vertex: 6,
    });

    // Back face (indices 6–11, reversed winding). Opposite is any front vertex.
    faces.push(SolidFace {
        vertex_indices: vec![11, 10, 9, 8, 7, 6],
        opposite_vertex: 0,
    });

    // 6 rectangular side faces.
    for i in 0..6usize {
        let next = (i + 1) % 6;
        let opposite = (i + 3) % 6;
        faces.push(SolidFace {
            vertex_indices: vec![i, i + 6, next + 6, next],
            opposite_vertex: opposite,
        });
    }

    (vertices, faces)
}

// ---------------------------------------------------------------------------
// Honeycomb texture generation
// ---------------------------------------------------------------------------

fn create_honeycomb_material(
    surface: PhysicalSurface,
    texture_manager: &TextureManager,
    material_builder: &mut MaterialManagerBuilder,
) -> EngineResult<MaterialId> {
    let pixels = generate_honeycomb_texture();
    let texture = texture_manager.create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
    let material = Material::textured(texture).with_derived_finish(surface);
    Ok(material_builder.register(material))
}

/// Procedural honeycomb texture: golden amber with glistening honey sheen,
/// inspired by Banjo-Kazooie's collectible honeycombs.
fn generate_honeycomb_texture() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    // Golden amber base with per-cell warmth variation.
    let warmth = rand_range(-0.04, 0.04);
    let base = Rgb::new(0.92 + warmth, 0.72 + warmth * 0.5, 0.15 + warmth * 0.2);
    let dark = Rgb::new(0.70, 0.50, 0.08);
    let highlight = Rgb::new(1.0, 0.92, 0.50);
    let glisten = Rgb::new(1.0, 0.98, 0.85);

    let seed_wax = rand_u32();
    let seed_cell = rand_u32();
    let seed_glisten = rand_u32();

    // Randomize the main specular highlight position per cell.
    let spec_cx = rand_range(0.25, 0.45);
    let spec_cy = rand_range(0.25, 0.45);

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Waxy surface variation — broad, smooth undulation.
            let wax = fbm_2d_periodic(u * 3.0, v * 3.0, 2, 0.5, 2.0, seed_wax, Some(3));
            let wax_factor = 0.88 + wax * 0.12;

            // Cellular noise for honey-in-the-comb look.
            let cell = fbm_2d_periodic(u * 8.0, v * 8.0, 3, 0.4, 2.0, seed_cell, Some(8));

            // Dark amber in the "wells", bright on the "ridges".
            let t = (cell * 0.5 + 0.5).clamp(0.0, 1.0);
            let mut c = dark.lerp(base, t);
            c = c.scale(wax_factor);

            // Primary specular highlight — broad glossy sheen.
            let du = u - spec_cx;
            let dv = v - spec_cy;
            let spec = (1.0 - ((du * du + dv * dv) * 6.0).min(1.0)).powi(4);
            c = c.lerp(highlight, spec * 0.50);

            // Glistening sheen — smooth bright patches from noise peaks,
            // like light catching a wet honey surface.
            let g = fbm_2d_periodic(u * 6.0, v * 6.0, 2, 0.3, 2.0, seed_glisten, Some(6));
            let sparkle = ((g - 0.45) / 0.55).clamp(0.0, 1.0).powi(2);
            c = c.lerp(highlight, sparkle * 0.25);

            // Secondary smaller highlight — offset from the main one for a
            // wet, rounded surface look.
            let du2 = u - (spec_cx + 0.25);
            let dv2 = v - (spec_cy + 0.20);
            let spec2 = (1.0 - ((du2 * du2 + dv2 * dv2) * 18.0).min(1.0)).powi(5);
            c = c.lerp(glisten, spec2 * 0.30);

            // Darken edges for hex cell border definition.
            let edge = border_band(u, v, 0.06);
            c = c.lerp(dark, edge * 0.5);

            c = c.scale(edge_vignette(u, v));
            c.write_rgba(&mut pixels);
        }
    }
    pixels
}
