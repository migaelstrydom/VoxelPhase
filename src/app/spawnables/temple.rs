//! Greek temple spawnable — classical Doric peristyle temple with columns,
//! entablature, pediments, and pitched roof. Golden-ratio proportions
//! throughout. Every piece is an independent rigid body.

use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector2, Vector3, Vector4};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::{
    build_convex_hull, compound_cuboid_model, convex_solid_model, cuboid_model, SolidFace,
};
use super::shared::textures::*;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::fracture::{CompoundFracture, FractureJoint};
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::{Material, MaterialId};
use crate::rendering::vertex::Vertex;
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 128;
const PHI: f32 = 1.618034;

/// Number of sides on the Doric column polygon — matches the 20 canonical
/// flutes so each polygon face aligns with exactly one flute channel.
const COLUMN_SIDES: u32 = 20;

const MAT_STONE: usize = 0;
const MAT_MARBLE: usize = 1;
const MAT_FLUTED: usize = 2;
const MATERIAL_COUNT: usize = 3;

#[derive(Deserialize)]
pub struct TempleDef {
    /// World position of the ground-level center of the temple.
    pub pos: (f32, f32, f32),
    /// Height of the columns (everything else scales from this).
    #[serde(default = "TempleDef::default_column_height")]
    pub column_height: f32,
    /// Number of columns along the front (short) side.
    #[serde(default = "TempleDef::default_front_columns")]
    pub front_columns: u32,
    /// Number of columns along the side (long) side, including corners.
    #[serde(default = "TempleDef::default_side_columns")]
    pub side_columns: u32,
}

impl TempleDef {
    pub fn default_column_height() -> f32 {
        8.0
    }
    pub fn default_front_columns() -> u32 {
        6
    }
    pub fn default_side_columns() -> u32 {
        9
    }
}

/// All measurements derived from column_height using classical Doric ratios
/// and the golden ratio.
struct TempleLayout {
    col_base_r: f32,
    col_top_r: f32,
    col_height: f32,
    spacing: f32,

    half_w: f32,
    half_l: f32,

    num_steps: u32,
    step_h: f32,
    step_margin: f32,
    stylobate_top: f32,

    entab_h: f32,
    entab_overhang: f32,
    entab_base_y: f32,

    pediment_h: f32,
    pediment_base_y: f32,
    roof_t: f32,
}

impl TempleLayout {
    fn from_def(def: &TempleDef) -> Self {
        let ch = def.column_height;

        let col_base_r = ch / 12.0;
        let col_top_r = col_base_r * 0.82;
        let spacing = ch / PHI.powi(2);

        let half_w = (def.front_columns - 1) as f32 * spacing / 2.0;
        let half_l = (def.side_columns - 1) as f32 * spacing / 2.0;

        let num_steps = 3_u32;
        let step_h = ch / 24.0;
        let step_margin = spacing * 0.12;
        let stylobate_top = def.pos.1 + num_steps as f32 * step_h;

        let entab_h = ch / PHI.powi(2);
        let entab_overhang = col_base_r * 0.6;
        let entab_base_y = stylobate_top + ch;

        let pediment_h = half_w / PHI;
        let pediment_base_y = entab_base_y + entab_h;
        let roof_t = ch / 40.0;

        Self {
            col_base_r,
            col_top_r,
            col_height: ch,
            spacing,
            half_w,
            half_l,
            num_steps,
            step_h,
            step_margin,
            stylobate_top,
            entab_h,
            entab_overhang,
            entab_base_y,
            pediment_h,
            pediment_base_y,
            roof_t,
        }
    }
}

impl Spawnable for TempleDef {
    fn material_count(&self) -> usize {
        MATERIAL_COUNT
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(vec![
            create_temple_stone_material(ctx.textures, ctx.materials)?,
            create_marble_material(ctx.textures, ctx.materials)?,
            create_fluted_marble_material(ctx.textures, ctx.materials)?,
        ])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let px = self.pos.0;
        let py = self.pos.1;
        let pz = self.pos.2;
        let stone = materials[MAT_STONE];
        let marble = materials[MAT_MARBLE];
        let fluted = materials[MAT_FLUTED];

        let lay = TempleLayout::from_def(self);
        let density = 2400.0;
        let friction = 1.5;

        let mut entities = Vec::new();

        // -----------------------------------------------------------------
        // Spawn helpers
        // -----------------------------------------------------------------

        let spawn_box = |world: &mut World,
                         entities: &mut Vec<Entity>,
                         pos: Point3<f32>,
                         he: Vector3<f32>,
                         mat: MaterialId| {
            let model = cuboid_model(he, mat);
            let body_handle = {
                let mut physics = world.write_resource::<PhysicsResource>();
                let bh = physics.world.create_body(
                    RigidBodyDesc::dynamic()
                        .position(pos)
                        .linear_damping(0.01)
                        .angular_damping(0.005),
                );
                physics.world.attach_collider(
                    bh,
                    ColliderDesc::box_shape(he)
                        .density(density)
                        .restitution(0.05)
                        .friction(friction),
                );
                bh
            };
            entities.push(
                world
                    .create_entity()
                    .with(Position(Vector3::new(pos.x, pos.y, pos.z)))
                    .with(Velocity(Vector3::zeros()))
                    .with(Orientation::default())
                    .with(RigidBodyComponent(body_handle))
                    .with(ModelInstance::new(model))
                    .with(Renderable)
                    .build(),
            );
        };

        let spawn_rotated_box = |world: &mut World,
                                 entities: &mut Vec<Entity>,
                                 pos: Point3<f32>,
                                 he: Vector3<f32>,
                                 rot: UnitQuaternion<f32>,
                                 mat: MaterialId| {
            let model = cuboid_model(he, mat);
            let body_handle = {
                let mut physics = world.write_resource::<PhysicsResource>();
                let bh = physics.world.create_body(
                    RigidBodyDesc::dynamic()
                        .position(pos)
                        .rotation(rot)
                        .linear_damping(0.01)
                        .angular_damping(0.005),
                );
                physics.world.attach_collider(
                    bh,
                    ColliderDesc::box_shape(he)
                        .density(density)
                        .restitution(0.05)
                        .friction(friction),
                );
                bh
            };
            entities.push(
                world
                    .create_entity()
                    .with(Position(Vector3::new(pos.x, pos.y, pos.z)))
                    .with(Velocity(Vector3::zeros()))
                    .with(Orientation(rot))
                    .with(RigidBodyComponent(body_handle))
                    .with(ModelInstance::new(model))
                    .with(Renderable)
                    .build(),
            );
        };

        let spawn_hull = |world: &mut World,
                          entities: &mut Vec<Entity>,
                          world_verts: &[Vector3<f32>],
                          faces: &[SolidFace],
                          mat: MaterialId| {
            let centroid =
                world_verts.iter().copied().sum::<Vector3<f32>>() / world_verts.len() as f32;
            let local: Vec<_> = world_verts.iter().map(|v| v - centroid).collect();
            let pos = Point3::new(centroid.x, centroid.y, centroid.z);
            let hull = Arc::new(build_convex_hull(&local, faces));
            let model = convex_solid_model(&local, faces, mat);
            let body_handle = {
                let mut physics = world.write_resource::<PhysicsResource>();
                let bh = physics.world.create_body(
                    RigidBodyDesc::dynamic()
                        .position(pos)
                        .linear_damping(0.01)
                        .angular_damping(0.005),
                );
                physics.world.attach_collider(
                    bh,
                    ColliderDesc::convex_hull(hull)
                        .density(density)
                        .restitution(0.05)
                        .friction(friction),
                );
                bh
            };
            entities.push(
                world
                    .create_entity()
                    .with(Position(Vector3::new(pos.x, pos.y, pos.z)))
                    .with(Velocity(Vector3::zeros()))
                    .with(Orientation::default())
                    .with(RigidBodyComponent(body_handle))
                    .with(ModelInstance::new(model))
                    .with(Renderable)
                    .build(),
            );
        };

        // =================================================================
        // Stylobate — three stepped platforms as a compound body
        // =================================================================

        let top_step_hw = lay.half_w + lay.col_base_r + lay.step_margin;
        let top_step_hl = lay.half_l + lay.col_base_r + lay.step_margin;

        let stylobate_center_y = py + lay.num_steps as f32 * lay.step_h / 2.0;
        let stylobate_pos = Point3::new(px, stylobate_center_y, pz);

        let mut step_boxes = Vec::new();
        for i in 0..lay.num_steps {
            let grow = (lay.num_steps - 1 - i) as f32 * lay.step_margin;
            let hw = top_step_hw + grow;
            let hl = top_step_hl + grow;
            let hh = lay.step_h / 2.0;
            let offset_y = i as f32 * lay.step_h + hh - lay.num_steps as f32 * lay.step_h / 2.0;

            step_boxes.push((Vector3::new(hw, hh, hl), Vector3::new(0.0, offset_y, 0.0)));
        }

        let stylobate_model = compound_cuboid_model(&step_boxes, stone);

        let stylobate_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();
            let bh = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(stylobate_pos)
                    .linear_damping(0.01)
                    .angular_damping(0.005),
            );
            for &(ref he, ref offset) in &step_boxes {
                physics.world.attach_collider(
                    bh,
                    ColliderDesc::box_shape(*he)
                        .offset_translation(*offset)
                        .density(density)
                        .restitution(0.05)
                        .friction(friction),
                );
            }
            bh
        };

        let stylobate_fracture = CompoundFracture {
            joints: vec![
                FractureJoint {
                    child_a: 0,
                    child_b: 1,
                    threshold: 50000.0,
                },
                FractureJoint {
                    child_a: 1,
                    child_b: 2,
                    threshold: 50000.0,
                },
            ],
            child_count: 3,
            material: stone,
        };

        entities.push(
            world
                .create_entity()
                .with(Position(Vector3::new(
                    stylobate_pos.x,
                    stylobate_pos.y,
                    stylobate_pos.z,
                )))
                .with(Velocity(Vector3::zeros()))
                .with(Orientation::default())
                .with(RigidBodyComponent(stylobate_handle))
                .with(ModelInstance::new(stylobate_model))
                .with(Renderable)
                .with(stylobate_fracture)
                .build(),
        );

        // =================================================================
        // Columns — fluted tapered prisms around the perimeter
        // =================================================================

        let col_y_base = lay.stylobate_top;

        let mut col_positions = Vec::new();

        // Front and back rows.
        for i in 0..self.front_columns {
            let x = px - lay.half_w + i as f32 * lay.spacing;
            col_positions.push((x, pz + lay.half_l));
            col_positions.push((x, pz - lay.half_l));
        }
        // Side rows (excluding corners already placed).
        for j in 1..(self.side_columns - 1) {
            let z = pz - lay.half_l + j as f32 * lay.spacing;
            col_positions.push((px - lay.half_w, z));
            col_positions.push((px + lay.half_w, z));
        }

        for &(cx, cz) in &col_positions {
            // Physics hull.
            let (hull_verts, hull_faces) =
                column_geometry(lay.col_base_r, lay.col_top_r, lay.col_height, COLUMN_SIDES);
            let world_hull: Vec<_> = hull_verts
                .iter()
                .map(|v| Vector3::new(v.x + cx, v.y + col_y_base, v.z + cz))
                .collect();
            let centroid =
                world_hull.iter().copied().sum::<Vector3<f32>>() / world_hull.len() as f32;
            let local_hull: Vec<_> = world_hull.iter().map(|v| v - centroid).collect();

            let pos = Point3::new(centroid.x, centroid.y, centroid.z);
            let hull = Arc::new(build_convex_hull(&local_hull, &hull_faces));

            // Rendering model with cylindrical UVs.
            let model = fluted_column_model(
                lay.col_base_r,
                lay.col_top_r,
                lay.col_height,
                COLUMN_SIDES,
                fluted,
            );

            let body_handle = {
                let mut physics = world.write_resource::<PhysicsResource>();
                let bh = physics.world.create_body(
                    RigidBodyDesc::dynamic()
                        .position(pos)
                        .linear_damping(0.01)
                        .angular_damping(0.005),
                );
                physics.world.attach_collider(
                    bh,
                    ColliderDesc::convex_hull(hull)
                        .density(density)
                        .restitution(0.05)
                        .friction(friction),
                );
                bh
            };

            entities.push(
                world
                    .create_entity()
                    .with(Position(Vector3::new(pos.x, pos.y, pos.z)))
                    .with(Velocity(Vector3::zeros()))
                    .with(Orientation::default())
                    .with(RigidBodyComponent(body_handle))
                    .with(ModelInstance::new(model))
                    .with(Renderable)
                    .build(),
            );
        }

        // =================================================================
        // Entablature — four beams (architrave + frieze + cornice)
        // =================================================================

        let entab_hh = lay.entab_h / 2.0;
        let entab_cy = lay.entab_base_y + entab_hh;
        let beam_depth = lay.col_base_r + lay.entab_overhang;

        let fb_hw = lay.half_w + beam_depth;
        for &z_sign in &[1.0_f32, -1.0] {
            spawn_box(
                world,
                &mut entities,
                Point3::new(px, entab_cy, pz + z_sign * lay.half_l),
                Vector3::new(fb_hw, entab_hh, beam_depth),
                marble,
            );
        }

        let side_hl = lay.half_l - beam_depth;
        for &x_sign in &[1.0_f32, -1.0] {
            spawn_box(
                world,
                &mut entities,
                Point3::new(px + x_sign * lay.half_w, entab_cy, pz),
                Vector3::new(beam_depth, entab_hh, side_hl),
                marble,
            );
        }

        // =================================================================
        // Pediments — triangular gable ends (front and back)
        // =================================================================

        let ped_base = lay.pediment_base_y;
        let ped_peak = ped_base + lay.pediment_h;
        let ped_hw = lay.half_w + beam_depth;

        for &z_sign in &[1.0_f32, -1.0] {
            let z_outer = pz + z_sign * (lay.half_l + beam_depth);
            let z_inner = pz + z_sign * (lay.half_l - beam_depth);
            let (verts, faces) = gable_geometry(px, ped_base, ped_peak, ped_hw, z_outer, z_inner);
            spawn_hull(world, &mut entities, &verts, &faces, marble);
        }

        // =================================================================
        // Roof — two sloped panels meeting at the ridge
        // =================================================================

        let roof_overhang = lay.step_margin * 2.0;
        let eave_dist = ped_hw;
        let peak_h = lay.pediment_h;
        let slope_len = (eave_dist * eave_dist + peak_h * peak_h).sqrt();
        let roof_angle = peak_h.atan2(eave_dist);
        let roof_half_depth = lay.half_l + beam_depth + roof_overhang;
        let roof_he = Vector3::new(slope_len / 2.0, lay.roof_t / 2.0, roof_half_depth);

        for &side in &[-1.0_f32, 1.0] {
            let mid_x = px + side * eave_dist / 2.0;
            let mid_y = ped_base + peak_h / 2.0;
            let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -side * roof_angle);

            spawn_rotated_box(
                world,
                &mut entities,
                Point3::new(mid_x, mid_y, pz),
                roof_he,
                rot,
                marble,
            );
        }

        entities
    }
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

/// Tapered polygonal prism for a Doric column (used for physics hull).
///
/// Base sits at y=0, top at y=`height`. Returns vertices + face definitions.
fn column_geometry(
    base_radius: f32,
    top_radius: f32,
    height: f32,
    num_sides: u32,
) -> (Vec<Vector3<f32>>, Vec<SolidFace>) {
    let n = num_sides as usize;
    let angle_step = std::f32::consts::TAU / num_sides as f32;

    let mut vertices = Vec::with_capacity(2 * n);

    for i in 0..n {
        let angle = i as f32 * angle_step;
        vertices.push(Vector3::new(
            base_radius * angle.cos(),
            0.0,
            base_radius * angle.sin(),
        ));
    }
    for i in 0..n {
        let angle = i as f32 * angle_step;
        vertices.push(Vector3::new(
            top_radius * angle.cos(),
            height,
            top_radius * angle.sin(),
        ));
    }

    let mut faces = Vec::with_capacity(n + 2);

    faces.push(SolidFace {
        vertex_indices: (0..n).rev().collect(),
        opposite_vertex: n,
    });
    faces.push(SolidFace {
        vertex_indices: (n..2 * n).collect(),
        opposite_vertex: 0,
    });
    for i in 0..n {
        let i_next = (i + 1) % n;
        faces.push(SolidFace {
            vertex_indices: vec![i, i_next, n + i_next, n + i],
            opposite_vertex: (i + n / 2) % n,
        });
    }

    (vertices, faces)
}

/// Build a column rendering model with cylindrical UV mapping.
///
/// U wraps around the circumference (0..1 maps to one full revolution) so the
/// fluted texture tiles seamlessly. V runs from 0 (base) to 1 (capital).
/// Side vertices get smooth radial normals; caps get flat up/down normals.
/// Geometry is centred at the origin to match the physics hull's local space.
fn fluted_column_model(
    base_radius: f32,
    top_radius: f32,
    height: f32,
    num_sides: u32,
    material: MaterialId,
) -> Arc<Model> {
    let n = num_sides as usize;
    let angle_step = std::f32::consts::TAU / num_sides as f32;
    let white = Vector4::new(1.0, 1.0, 1.0, 1.0);

    // The hull centroid (in raw local space) sits at y = height/2 with x=z=0
    // (symmetric polygon). Shift the model down by the same amount so the
    // rendering mesh and physics hull share the same origin.
    let oy = -height / 2.0;

    let mut verts: Vec<Vertex> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();

    // --- Side faces (cylindrical UVs, smooth radial normals) ---
    for i in 0..n {
        let base_idx = verts.len() as u32;

        let angle_a = i as f32 * angle_step;
        let angle_b = (i + 1) as f32 * angle_step;

        // UV: each face spans one flute. U goes from i/n to (i+1)/n.
        let u_a = i as f32 / n as f32;
        let u_b = (i + 1) as f32 / n as f32;

        let (cos_a, sin_a) = (angle_a.cos(), angle_a.sin());
        let (cos_b, sin_b) = (angle_b.cos(), angle_b.sin());

        // Smooth outward-pointing normals.
        let na = Vector3::new(cos_a, 0.0, sin_a);
        let nb = Vector3::new(cos_b, 0.0, sin_b);

        // Bottom-left (base, angle_a).
        verts.push(Vertex {
            pos: Vector4::new(base_radius * cos_a, oy, base_radius * sin_a, 1.0),
            color: white,
            tex_coords: Vector2::new(u_a, 0.0),
            normal: na,
        });
        // Bottom-right (base, angle_b).
        verts.push(Vertex {
            pos: Vector4::new(base_radius * cos_b, oy, base_radius * sin_b, 1.0),
            color: white,
            tex_coords: Vector2::new(u_b, 0.0),
            normal: nb,
        });
        // Top-right (capital, angle_b).
        verts.push(Vertex {
            pos: Vector4::new(top_radius * cos_b, height + oy, top_radius * sin_b, 1.0),
            color: white,
            tex_coords: Vector2::new(u_b, 1.0),
            normal: nb,
        });
        // Top-left (capital, angle_a).
        verts.push(Vertex {
            pos: Vector4::new(top_radius * cos_a, height + oy, top_radius * sin_a, 1.0),
            color: white,
            tex_coords: Vector2::new(u_a, 1.0),
            normal: na,
        });

        indices.extend_from_slice(&[
            base_idx,
            base_idx + 2,
            base_idx + 1,
            base_idx,
            base_idx + 3,
            base_idx + 2,
        ]);
    }

    // --- Bottom cap (flat downward normal, radial UVs) ---
    let cap_base = verts.len() as u32;
    let down = Vector3::new(0.0, -1.0, 0.0);
    for i in 0..n {
        let angle = i as f32 * angle_step;
        let (c, s) = (angle.cos(), angle.sin());
        verts.push(Vertex {
            pos: Vector4::new(base_radius * c, oy, base_radius * s, 1.0),
            color: white,
            tex_coords: Vector2::new(0.5 + 0.5 * c, 0.5 + 0.5 * s),
            normal: down,
        });
    }
    for i in 1..(n as u32 - 1) {
        indices.extend_from_slice(&[cap_base, cap_base + i, cap_base + i + 1]);
    }

    // --- Top cap (flat upward normal, radial UVs) ---
    let top_base = verts.len() as u32;
    let up = Vector3::new(0.0, 1.0, 0.0);
    for i in 0..n {
        let angle = i as f32 * angle_step;
        let (c, s) = (angle.cos(), angle.sin());
        verts.push(Vertex {
            pos: Vector4::new(top_radius * c, height + oy, top_radius * s, 1.0),
            color: white,
            tex_coords: Vector2::new(0.5 + 0.5 * c, 0.5 + 0.5 * s),
            normal: up,
        });
    }
    for i in 1..(n as u32 - 1) {
        indices.extend_from_slice(&[top_base, top_base + i + 1, top_base + i]);
    }

    let parts = vec![ModelPart::new(vec![MeshPrimitive {
        vertices: verts,
        indices,
        material,
    }])];
    Arc::new(Model::flat(parts))
}

/// Triangular gable prism.
fn gable_geometry(
    cx: f32,
    base_y: f32,
    peak_y: f32,
    half_w: f32,
    z_a: f32,
    z_b: f32,
) -> (Vec<Vector3<f32>>, Vec<SolidFace>) {
    let vertices = vec![
        Vector3::new(cx - half_w, base_y, z_a), // 0
        Vector3::new(cx + half_w, base_y, z_a), // 1
        Vector3::new(cx, peak_y, z_a),          // 2
        Vector3::new(cx - half_w, base_y, z_b), // 3
        Vector3::new(cx + half_w, base_y, z_b), // 4
        Vector3::new(cx, peak_y, z_b),          // 5
    ];

    let faces = vec![
        SolidFace {
            vertex_indices: vec![0, 1, 2],
            opposite_vertex: 3,
        },
        SolidFace {
            vertex_indices: vec![5, 4, 3],
            opposite_vertex: 0,
        },
        SolidFace {
            vertex_indices: vec![0, 3, 4, 1],
            opposite_vertex: 2,
        },
        SolidFace {
            vertex_indices: vec![0, 2, 5, 3],
            opposite_vertex: 1,
        },
        SolidFace {
            vertex_indices: vec![1, 4, 5, 2],
            opposite_vertex: 0,
        },
    ];

    (vertices, faces)
}

// ---------------------------------------------------------------------------
// Procedural textures
// ---------------------------------------------------------------------------

fn create_temple_stone_material(
    textures: &crate::resources::textures::TextureManager,
    materials: &mut crate::rendering::material::MaterialManagerBuilder,
) -> EngineResult<MaterialId> {
    let pixels = generate_temple_stone_texture();
    let texture = textures.create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
    Ok(materials.register(Material::textured(texture)))
}

fn create_marble_material(
    textures: &crate::resources::textures::TextureManager,
    materials: &mut crate::rendering::material::MaterialManagerBuilder,
) -> EngineResult<MaterialId> {
    let pixels = generate_marble_texture();
    let texture = textures.create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
    Ok(materials.register(Material::textured(texture)))
}

fn create_fluted_marble_material(
    textures: &crate::resources::textures::TextureManager,
    materials: &mut crate::rendering::material::MaterialManagerBuilder,
) -> EngineResult<MaterialId> {
    let pixels = generate_fluted_marble_texture();
    let texture = textures.create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
    Ok(materials.register(Material::textured(texture)))
}

/// Light grey stone for the stylobate platform.
fn generate_temple_stone_texture() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let base = Rgb::new(0.80, 0.78, 0.74);
    let seed_grain = rand_u32();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let grain = fbm_2d_periodic(u * 16.0, v * 16.0, 3, 0.5, 2.0, seed_grain, Some(16));
            let mut c = base.scale(0.92 + grain * 0.08);

            let edge = border_band(u, v, 0.04);
            c = c.scale(1.0 - edge * 0.18);
            c = c.scale(edge_vignette(u, v));

            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

/// Warm Pentelic marble — cream-white with subtle grey-blue veining.
fn generate_marble_texture() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let base = Rgb::new(0.94, 0.92, 0.88);
    let vein_color = Rgb::new(0.72, 0.74, 0.78);

    let seed_grain = rand_u32();
    let seed_vein1 = rand_u32();
    let seed_vein2 = rand_u32();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let grain = fbm_2d_periodic(u * 20.0, v * 20.0, 3, 0.5, 2.0, seed_grain, Some(20));
            let mut c = base.scale(0.96 + grain * 0.04);

            let vein1 = fbm_2d_periodic(
                u * 3.0 + v * 2.0,
                v * 5.0 - u * 1.5,
                4,
                0.6,
                2.0,
                seed_vein1,
                Some(5),
            );
            let vein_band1 = ((vein1 - 0.48).abs() < 0.025) as u8 as f32;
            c = c.lerp(vein_color, vein_band1 * 0.30);

            let vein2 = fbm_2d_periodic(
                u * 6.0 - v * 3.0,
                v * 8.0 + u * 2.0,
                3,
                0.5,
                2.0,
                seed_vein2,
                Some(8),
            );
            let vein_band2 = ((vein2 - 0.50).abs() < 0.015) as u8 as f32;
            c = c.lerp(vein_color, vein_band2 * 0.18);

            let warmth = fbm_2d_periodic(u * 2.0, v * 2.0, 2, 0.4, 2.0, seed_grain + 7, Some(2));
            c = Rgb::new(
                c.r + (warmth - 0.5) * 0.03,
                c.g + (warmth - 0.5) * 0.02,
                c.b,
            );

            c = c.scale(edge_vignette(u, v));
            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

/// Fluted Doric marble — 20 vertical concave channels with sharp arrises.
///
/// U wraps around the column (one full revolution = one texture width).
/// Each flute occupies 1/20 of the U range. A cosine profile darkens the
/// groove centres and highlights the sharp ridges between flutes.
/// The marble base colour and subtle veining show through underneath.
fn generate_fluted_marble_texture() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let base = Rgb::new(0.94, 0.92, 0.88);
    let vein_color = Rgb::new(0.72, 0.74, 0.78);
    let flute_shadow = Rgb::new(0.78, 0.76, 0.72);

    let num_flutes = COLUMN_SIDES as f32;
    let seed_grain = rand_u32();
    let seed_vein = rand_u32();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Marble base with grain.
            let grain = fbm_2d_periodic(u * 20.0, v * 20.0, 3, 0.5, 2.0, seed_grain, Some(20));
            let mut c = base.scale(0.96 + grain * 0.04);

            // Subtle veining (lighter than flat marble — fluting is the star).
            let vein = fbm_2d_periodic(
                u * 4.0 + v * 2.0,
                v * 6.0 - u * 1.5,
                3,
                0.6,
                2.0,
                seed_vein,
                Some(6),
            );
            let vein_band = ((vein - 0.48).abs() < 0.02) as u8 as f32;
            c = c.lerp(vein_color, vein_band * 0.15);

            // Flute concavity: cosine profile per flute channel.
            // frac is 0 at left arris, 0.5 at groove centre, 1 at right arris.
            let flute_u = u * num_flutes;
            let frac = flute_u.fract();
            // depth: 0 at arrises, 1 at groove centre.
            let depth = (1.0 - (std::f32::consts::TAU * frac).cos()) * 0.5;
            c = c.lerp(flute_shadow, depth * 0.35);

            // Arris highlight — bright line at the ridge between flutes.
            let arris_dist = frac.min(1.0 - frac);
            if arris_dist < 0.06 {
                let t = 1.0 - arris_dist / 0.06;
                c = c.scale(1.0 + t * 0.08);
            }

            c.write_rgba(&mut pixels);
        }
    }
    pixels
}
