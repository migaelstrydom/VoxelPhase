//! Playground spinning wheel — terrain-anchored disc that spins freely.
//!
//! A flat disc pinned to the terrain via BallJoint + KeepUpright. Players can
//! spin it by pushing against it. When the terrain is destroyed, it breaks free
//! as a normal dynamic body.
//!
//! Uses three materials: painted top, painted bottom, and metallic rim.

use std::f32::consts::TAU;
use std::sync::Arc;

use nalgebra::{Point3, UnitVector3, Vector2, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::finish::{ColliderSurface, MaterialSurface};
use super::shared::models::{build_convex_hull, SolidFace};
use super::shared::textures::seed_from_ground;
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
use crate::rendering::physical_finish::PhysicalSurface;
use crate::rendering::vertex::Vertex;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 256;
const MESH_SEGMENTS: u32 = 32;
const NUM_COLOUR_SEGMENTS: u32 = 6;

/// Restitution of the wheel's collider. Low bounce — it's meant to spin
/// freely, not bounce off the ground.
const RESTITUTION: f32 = 0.2;
/// Friction of the wheel's collider.
const FRICTION: f32 = 0.5;

#[derive(Deserialize)]
pub struct PlayWheelDef {
    /// Position (x, z). Y is determined by terrain surface height.
    pub pos: (f32, f32),
    #[serde(default = "PlayWheelDef::default_radius")]
    pub radius: f32,
    #[serde(default = "PlayWheelDef::default_thickness")]
    pub thickness: f32,
    #[serde(default = "PlayWheelDef::default_density")]
    pub density: f32,
    /// Height of the disc center above the terrain surface.
    #[serde(default = "PlayWheelDef::default_hub_height")]
    pub hub_height: f32,
}

impl PlayWheelDef {
    pub fn default_radius() -> f32 {
        1.0
    }
    pub fn default_thickness() -> f32 {
        0.06
    }
    pub fn default_density() -> f32 {
        2400.0
    }
    pub fn default_hub_height() -> f32 {
        0.4
    }

    /// The one declaration of this wheel's physics. The collider takes the
    /// coefficients and the material takes the finish they imply, so the two
    /// cannot drift apart.
    fn surface(&self) -> PhysicalSurface {
        PhysicalSurface {
            restitution: RESTITUTION,
            friction: FRICTION,
            density: self.density,
        }
    }
}

impl Spawnable for PlayWheelDef {
    fn material_count(&self) -> usize {
        3
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let seed = seed_from_ground(self.pos, 0);

        let top_pixels = generate_wheel_face_texture(seed, false);
        let top_tex =
            ctx.textures
                .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &top_pixels, true)?;
        let top_mat = ctx
            .materials
            .register(Material::textured(top_tex).with_derived_finish(self.surface()));

        let bot_pixels = generate_wheel_face_texture(seed, true);
        let bot_tex =
            ctx.textures
                .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &bot_pixels, true)?;
        let bot_mat = ctx
            .materials
            .register(Material::textured(bot_tex).with_derived_finish(self.surface()));

        let rim_pixels = generate_rim_texture(seed);
        let rim_tex =
            ctx.textures
                .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &rim_pixels, true)?;
        let rim_mat = ctx
            .materials
            .register(Material::textured(rim_tex).with_derived_finish(self.surface()));

        Ok(vec![top_mat, bot_mat, rim_mat])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let top_material = materials[0];
        let bot_material = materials[1];
        let rim_material = materials[2];

        let surface_y = {
            let terrain = world.read_resource::<TerrainWorld>();
            terrain.mesh_surface_height_at(self.pos.0, self.pos.1)
        };

        let Some(surface_y) = surface_y else {
            return Vec::new();
        };

        let half_thickness = self.thickness / 2.0;
        let center_y = surface_y + self.hub_height;
        let initial_pos = Point3::new(self.pos.0, center_y, self.pos.1);

        // Visual mesh: three primitives (top cap, bottom cap, rim barrel).
        let top_cap = generate_disc_mesh(half_thickness, self.radius, MESH_SEGMENTS, true);
        let bot_cap = generate_disc_mesh(half_thickness, self.radius, MESH_SEGMENTS, false);
        let rim = generate_rim_mesh(half_thickness, self.radius, MESH_SEGMENTS);

        let parts = vec![ModelPart::new(vec![
            MeshPrimitive {
                vertices: top_cap.0,
                indices: top_cap.1,
                material: top_material,
            },
            MeshPrimitive {
                vertices: bot_cap.0,
                indices: bot_cap.1,
                material: bot_material,
            },
            MeshPrimitive {
                vertices: rim.0,
                indices: rim.1,
                material: rim_material,
            },
        ])];
        let model = Arc::new(Model::flat(parts));

        let hull = build_disc_hull(self.radius, half_thickness);
        let collider =
            ColliderDesc::convex_hull(Arc::new(hull)).with_physical_surface(self.surface());

        let (body_handle, anchor_handle, upright_handle) = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body_desc = RigidBodyDesc::dynamic()
                .position(initial_pos)
                .gravity_scale(1.0)
                .linear_damping(0.01)
                .angular_damping(0.5);

            let body_handle = physics.world.create_body(body_desc);
            physics.world.attach_collider(body_handle, collider);

            let anchor_handle = physics
                .world
                .create_constraint(ConstraintKind::world_ball_joint(
                    body_handle,
                    initial_pos,
                    Vector3::zeros(),
                    0.0,
                    f32::MAX,
                ));

            let upright_handle = physics
                .world
                .create_constraint(ConstraintKind::KeepUpright {
                    body: body_handle,
                    target_up: UnitVector3::new_normalize(Vector3::y()),
                    compliance: 0.0,
                    max_impulse: f32::INFINITY,
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
                anchor_points: vec![anchor_check],
                released_model: None,
            })
            .build()]
    }
}

// ---------------------------------------------------------------------------
// Collider
// ---------------------------------------------------------------------------

/// Number of sides for the disc convex hull (top + bottom ring = 2× this).
const HULL_SIDES: usize = 16;

/// Build a regular polygon prism approximating a disc.
fn build_disc_hull(radius: f32, half_thickness: f32) -> crate::collision::ConvexHull {
    let mut vertices = Vec::with_capacity(HULL_SIDES * 2);

    // Bottom ring, then top ring.
    for ring in 0..2 {
        let y = if ring == 0 {
            -half_thickness
        } else {
            half_thickness
        };
        for i in 0..HULL_SIDES {
            let angle = i as f32 * TAU / HULL_SIDES as f32;
            vertices.push(Vector3::new(angle.cos() * radius, y, angle.sin() * radius));
        }
    }

    let n = HULL_SIDES;
    let mut faces = Vec::new();

    // Top face: indices n..2n, opposite vertex is any bottom vertex (e.g. 0).
    faces.push(SolidFace {
        vertex_indices: (n..2 * n).collect(),
        opposite_vertex: 0,
    });

    // Bottom face: indices 0..n (reversed winding), opposite vertex is any top (e.g. n).
    faces.push(SolidFace {
        vertex_indices: (0..n).rev().collect(),
        opposite_vertex: n,
    });

    // Side quads: each connects bottom[i], bottom[i+1], top[i+1], top[i].
    // Opposite vertex is the center of the opposite side — use the vertex
    // diametrically opposite on the same ring.
    for i in 0..n {
        let i_next = (i + 1) % n;
        let opposite = (i + n / 2) % n; // opposite side, bottom ring
        faces.push(SolidFace {
            vertex_indices: vec![i, i_next, i_next + n, i + n],
            opposite_vertex: opposite,
        });
    }

    build_convex_hull(&vertices, &faces)
}

// ---------------------------------------------------------------------------
// Mesh generation
// ---------------------------------------------------------------------------

/// Single disc cap (top or bottom).
fn generate_disc_mesh(
    half_thickness: f32,
    radius: f32,
    segments: u32,
    top: bool,
) -> (Vec<Vertex>, Vec<u32>) {
    let colour_vec = Colour::WHITE.to_vec4();
    let y = if top { half_thickness } else { -half_thickness };
    let normal = if top { Vector3::y() } else { -Vector3::y() };

    let mut vertices = Vec::with_capacity((segments + 1) as usize);
    let mut indices = Vec::with_capacity((segments * 3) as usize);

    // Center vertex
    vertices.push(Vertex {
        pos: Vector3::new(0.0, y, 0.0),
        color: colour_vec,
        tex_coords: Vector2::new(0.5, 0.5),
        normal,
        ao: 1.0,
    });

    for i in 0..segments {
        let angle = i as f32 * TAU / segments as f32;
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        let uv_y = if top {
            0.5 + sin_a * 0.5
        } else {
            0.5 - sin_a * 0.5
        };
        vertices.push(Vertex {
            pos: Vector3::new(cos_a * radius, y, sin_a * radius),
            color: colour_vec,
            tex_coords: Vector2::new(0.5 + cos_a * 0.5, uv_y),
            normal,
            ao: 1.0,
        });
    }

    for i in 0..segments {
        let curr = 1 + i;
        let next = 1 + (i + 1) % segments;
        if top {
            indices.extend_from_slice(&[0, next, curr]);
        } else {
            indices.extend_from_slice(&[0, curr, next]);
        }
    }

    (vertices, indices)
}

/// Rim (barrel sides) of the disc.
fn generate_rim_mesh(half_thickness: f32, radius: f32, segments: u32) -> (Vec<Vertex>, Vec<u32>) {
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
            pos: Vector3::new(cos_a * radius, -half_thickness, sin_a * radius),
            color: colour_vec,
            tex_coords: Vector2::new(u, 1.0),
            normal,
            ao: 1.0,
        });
        vertices.push(Vertex {
            pos: Vector3::new(cos_a * radius, half_thickness, sin_a * radius),
            color: colour_vec,
            tex_coords: Vector2::new(u, 0.0),
            normal,
            ao: 1.0,
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

// ---------------------------------------------------------------------------
// Texture generation
// ---------------------------------------------------------------------------

/// Colour palette for the wheel segments.
const SEGMENT_COLOURS: [Rgb; 6] = [
    Rgb {
        r: 0.85,
        g: 0.20,
        b: 0.18,
    }, // red
    Rgb {
        r: 0.95,
        g: 0.75,
        b: 0.15,
    }, // yellow
    Rgb {
        r: 0.20,
        g: 0.60,
        b: 0.85,
    }, // blue
    Rgb {
        r: 0.25,
        g: 0.75,
        b: 0.30,
    }, // green
    Rgb {
        r: 0.90,
        g: 0.50,
        b: 0.15,
    }, // orange
    Rgb {
        r: 0.65,
        g: 0.25,
        b: 0.75,
    }, // purple
];

/// Painted wheel face with coloured pie segments and a central hub.
fn generate_wheel_face_texture(seed: u32, flip: bool) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let hub_colour = Rgb::new(0.45, 0.45, 0.50);
    let divider_colour = Rgb::new(0.35, 0.35, 0.38);
    let bolt_colour = Rgb::new(0.60, 0.60, 0.65);

    let hub_radius = 0.12;
    let bolt_ring_radius = 0.08;
    let bolt_dot_radius = 0.015;
    let num_bolts = 6u32;

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let dx = u - 0.5;
            let dy = v - 0.5;
            let dist = (dx * dx + dy * dy).sqrt();

            let raw_angle = if flip { (-dy).atan2(dx) } else { dy.atan2(dx) };
            let angle = (raw_angle + TAU) % TAU;

            let colour = if dist < hub_radius {
                // Central hub: metallic grey with bolt dots.
                let mut c = hub_colour;

                // Bolt pattern
                for b in 0..num_bolts {
                    let bolt_angle = b as f32 * TAU / num_bolts as f32;
                    let bx = 0.5 + bolt_ring_radius * bolt_angle.cos();
                    let by = 0.5 + bolt_ring_radius * bolt_angle.sin();
                    let bd = ((u - bx).powi(2) + (v - by).powi(2)).sqrt();
                    if bd < bolt_dot_radius {
                        let t = 1.0 - (bd / bolt_dot_radius);
                        c = c.lerp(bolt_colour, t * 0.8);
                    }
                }

                // Hub edge ring
                if dist > hub_radius - 0.015 {
                    let t = ((dist - (hub_radius - 0.015)) / 0.015).clamp(0.0, 1.0);
                    c = c.lerp(divider_colour, t * 0.5);
                }

                c
            } else if dist > 0.48 {
                // Outer edge darkening.
                let edge_t = ((dist - 0.48) / 0.02).clamp(0.0, 1.0);
                let seg_idx = (angle / TAU * NUM_COLOUR_SEGMENTS as f32) as usize
                    % NUM_COLOUR_SEGMENTS as usize;
                SEGMENT_COLOURS[seg_idx].scale(1.0 - edge_t * 0.35)
            } else {
                // Coloured pie segments.
                let segment_angle = TAU / NUM_COLOUR_SEGMENTS as f32;
                let seg_idx = (angle / TAU * NUM_COLOUR_SEGMENTS as f32) as usize
                    % NUM_COLOUR_SEGMENTS as usize;
                let seg_phase = (angle % segment_angle) / segment_angle;

                let mut c = SEGMENT_COLOURS[seg_idx];

                // Divider lines between segments.
                let divider_width = 0.03;
                if seg_phase < divider_width || seg_phase > (1.0 - divider_width) {
                    let edge_dist = seg_phase.min(1.0 - seg_phase) / divider_width;
                    let t = (1.0 - edge_dist).clamp(0.0, 1.0).powi(2);
                    c = c.lerp(divider_colour, t * 0.7);
                }

                // Subtle paint wear noise.
                let wear = fbm_2d_periodic(
                    u * 6.0,
                    v * 6.0,
                    2,
                    0.4,
                    2.0,
                    seed.wrapping_add(seg_idx as u32),
                    Some(6),
                );
                c = c.scale(0.88 + wear * 0.12);

                // Radial shading: slight darkening toward edge.
                let radial_shade = 1.0 - (dist - 0.15) * 0.15;
                c = c.scale(radial_shade.clamp(0.85, 1.0));

                c
            };

            colour.write_rgba(&mut pixels);
        }
    }
    pixels
}

/// Metallic rim texture with scuffs and wear.
fn generate_rim_texture(seed: u32) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let base_metal = Rgb::new(0.50, 0.50, 0.55);
    let dark_metal = Rgb::new(0.32, 0.32, 0.36);

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Brushed metal grain (horizontal).
            let grain = fbm_2d_periodic(
                u * 30.0,
                v * 3.0,
                2,
                0.4,
                2.0,
                seed.wrapping_add(100),
                Some(30),
            );
            let mut colour = base_metal.lerp(dark_metal, (grain * 0.5 + 0.5).clamp(0.0, 1.0));

            // Scuff marks.
            let scuffs = fbm_2d_periodic(
                u * 8.0,
                v * 8.0,
                3,
                0.6,
                2.0,
                seed.wrapping_add(110),
                Some(8),
            );
            if scuffs > 0.3 {
                let t = ((scuffs - 0.3) / 0.4).clamp(0.0, 0.3);
                colour = colour.scale(1.0 - t);
            }

            // Edge bevels (top and bottom of rim).
            let bevel = (v.min(1.0 - v) / 0.15).clamp(0.0, 1.0);
            colour = colour.scale(0.7 + bevel * 0.3);

            colour.write_rgba(&mut pixels);
        }
    }
    pixels
}
