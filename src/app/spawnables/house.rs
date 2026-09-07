//! House spawnable — brick house with pitched gable roof, chimney, door,
//! and window openings. Built from individual OBB and convex-hull pieces
//! so every part can be knocked loose independently.

use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::{build_convex_hull, convex_solid_model, cuboid_model, SolidFace};
use super::shared::textures::*;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::fire::components::Flammable;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::MaterialId;
use crate::rendering::substance::{self, ColliderSubstance, Substance};
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 128;

/// Number of distinct materials: stone, brick, slate.
const MATERIAL_COUNT: usize = 3;
const MAT_STONE: usize = 0;
const MAT_BRICK: usize = 1;
const MAT_SLATE: usize = 2;

/// What the house is built of. Stone walls, brick detail, a slate roof.
///
/// The three used to share one set of coefficients *and* one derived finish,
/// so a slate roof and a rubble wall came out of the renderer identically. They
/// still behave close enough to each other that the colliders are all built
/// from the wall's stone; what they no longer share is how they look.
const WALL: Substance = substance::GRANITE;
const DETAIL: Substance = substance::BRICK;
const ROOF: Substance = substance::SLATE;

#[derive(Deserialize)]
pub struct HouseDef {
    pub pos: (f32, f32, f32),
    pub half_extents: (f32, f32, f32),
}

impl Spawnable for HouseDef {
    fn material_count(&self) -> usize {
        MATERIAL_COUNT
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(vec![
            create_stone_material(ctx.textures, ctx.materials)?,
            create_brick_material(ctx.textures, ctx.materials)?,
            create_slate_material(ctx.textures, ctx.materials)?,
        ])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let px = self.pos.0;
        let py = self.pos.1;
        let pz = self.pos.2;
        let wx = self.half_extents.0;
        let hy = self.half_extents.1;
        let wz = self.half_extents.2;

        let stone = materials[MAT_STONE];
        let brick = materials[MAT_BRICK];
        let slate = materials[MAT_SLATE];

        // --- Proportions ---
        let wt = (hy * 0.12).max(0.06);
        let fh = hy * 0.12;
        let wall_h = hy * 1.6;
        let peak_h = hy * 0.75;
        let rt = hy * 0.10;
        let overhang = hy * 0.15;

        let wall_base = py + 2.0 * fh;
        let wall_top = wall_base + wall_h;

        // Door opening.
        let door_half_w = wx * 0.25;
        let door_h = wall_h * 0.65;

        // Window opening.
        let side_wall_hz = wz - wt;
        let win_hz = side_wall_hz * 0.22;
        let win_hy = wall_h * 0.18;
        let win_center_y = wall_base + wall_h * 0.55;

        let mut entities = Vec::new();

        // Spawn helpers ---------------------------------------------------

        let spawn_box = |world: &mut World,
                         entities: &mut Vec<Entity>,
                         pos: Point3<f32>,
                         he: Vector3<f32>,
                         mat: MaterialId,
                         flammable: bool| {
            let model = cuboid_model(he, mat);
            let body_handle = {
                let mut physics = world.write_resource::<PhysicsResource>();
                let body_desc = RigidBodyDesc::dynamic()
                    .position(pos)
                    .gravity_scale(1.0)
                    .linear_damping(0.01)
                    .angular_damping(0.005);
                let body_handle = physics.world.create_body(body_desc);
                physics
                    .world
                    .attach_collider(body_handle, ColliderDesc::box_shape(he).of(&WALL));
                body_handle
            };
            let mut builder = world
                .create_entity()
                .with(Position(Vector3::new(pos.x, pos.y, pos.z)))
                .with(Velocity(Vector3::zeros()))
                .with(Orientation::default())
                .with(RigidBodyComponent(body_handle))
                .with(ModelInstance::new(model))
                .with(Renderable);
            if flammable {
                builder = builder.with(Flammable::wood());
            }
            entities.push(builder.build());
        };

        let spawn_rotated_box = |world: &mut World,
                                 entities: &mut Vec<Entity>,
                                 pos: Point3<f32>,
                                 he: Vector3<f32>,
                                 rotation: UnitQuaternion<f32>,
                                 mat: MaterialId,
                                 flammable: bool| {
            let model = cuboid_model(he, mat);
            let body_handle = {
                let mut physics = world.write_resource::<PhysicsResource>();
                let body_desc = RigidBodyDesc::dynamic()
                    .position(pos)
                    .rotation(rotation)
                    .gravity_scale(1.0)
                    .linear_damping(0.01)
                    .angular_damping(0.005);
                let body_handle = physics.world.create_body(body_desc);
                physics
                    .world
                    .attach_collider(body_handle, ColliderDesc::box_shape(he).of(&WALL));
                body_handle
            };
            let mut builder = world
                .create_entity()
                .with(Position(Vector3::new(pos.x, pos.y, pos.z)))
                .with(Velocity(Vector3::zeros()))
                .with(Orientation(rotation))
                .with(RigidBodyComponent(body_handle))
                .with(ModelInstance::new(model))
                .with(Renderable);
            if flammable {
                builder = builder.with(Flammable::wood());
            }
            entities.push(builder.build());
        };

        let spawn_hull = |world: &mut World,
                          entities: &mut Vec<Entity>,
                          world_verts: &[Vector3<f32>],
                          faces: &[SolidFace],
                          mat: MaterialId| {
            let centroid =
                world_verts.iter().copied().sum::<Vector3<f32>>() / world_verts.len() as f32;
            let local_verts: Vec<_> = world_verts.iter().map(|v| v - centroid).collect();

            let pos = Point3::new(centroid.x, centroid.y, centroid.z);
            let hull = Arc::new(build_convex_hull(&local_verts, faces));
            let model = convex_solid_model(&local_verts, faces, mat);

            let body_handle = {
                let mut physics = world.write_resource::<PhysicsResource>();
                let body_desc = RigidBodyDesc::dynamic()
                    .position(pos)
                    .gravity_scale(1.0)
                    .linear_damping(0.01)
                    .angular_damping(0.005);
                let body_handle = physics.world.create_body(body_desc);
                physics
                    .world
                    .attach_collider(body_handle, ColliderDesc::convex_hull(hull).of(&WALL));
                body_handle
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

        // === Foundation ===
        spawn_box(
            world,
            &mut entities,
            Point3::new(px, py + fh, pz),
            Vector3::new(wx + wt + 0.02, fh, wz + wt + 0.02),
            stone,
            false,
        );

        // === Back wall (solid, at z = pz - wz) ===
        let wall_half_h = wall_h / 2.0;
        spawn_box(
            world,
            &mut entities,
            Point3::new(px, wall_base + wall_half_h, pz - wz),
            Vector3::new(wx + wt, wall_half_h, wt),
            brick,
            false,
        );

        // === Front wall (at z = pz + wz) — split for door ===
        let front_side_hw = (wx + wt - door_half_w) / 2.0;
        let door_half_h = door_h / 2.0;

        // Left of door.
        spawn_box(
            world,
            &mut entities,
            Point3::new(
                px - door_half_w - front_side_hw,
                wall_base + door_half_h,
                pz + wz,
            ),
            Vector3::new(front_side_hw, door_half_h, wt),
            brick,
            false,
        );
        // Right of door.
        spawn_box(
            world,
            &mut entities,
            Point3::new(
                px + door_half_w + front_side_hw,
                wall_base + door_half_h,
                pz + wz,
            ),
            Vector3::new(front_side_hw, door_half_h, wt),
            brick,
            false,
        );
        // Above door (lintel to wall top).
        let above_door_half_h = (wall_h - door_h) / 2.0;
        spawn_box(
            world,
            &mut entities,
            Point3::new(px, wall_top - above_door_half_h, pz + wz),
            Vector3::new(wx + wt, above_door_half_h, wt),
            brick,
            false,
        );

        // === Side walls with window openings ===
        let col_hz = (side_wall_hz - win_hz) / 2.0;
        let win_bottom = win_center_y - win_hy;
        let win_top = win_center_y + win_hy;
        let lower_half_h = (win_bottom - wall_base) / 2.0;
        let upper_half_h = (wall_top - win_top) / 2.0;

        for sign in [-1.0_f32, 1.0] {
            let cx = px + sign * wx;

            // Lower wall (below window), full depth.
            spawn_box(
                world,
                &mut entities,
                Point3::new(cx, wall_base + lower_half_h, pz),
                Vector3::new(wt, lower_half_h, side_wall_hz),
                brick,
                false,
            );
            // Upper wall (above window), full depth.
            spawn_box(
                world,
                &mut entities,
                Point3::new(cx, wall_top - upper_half_h, pz),
                Vector3::new(wt, upper_half_h, side_wall_hz),
                brick,
                false,
            );
            // Column toward back (between back wall and window).
            spawn_box(
                world,
                &mut entities,
                Point3::new(cx, win_center_y, pz - win_hz - col_hz),
                Vector3::new(wt, win_hy, col_hz),
                brick,
                false,
            );
            // Column toward front (between window and front wall).
            spawn_box(
                world,
                &mut entities,
                Point3::new(cx, win_center_y, pz + win_hz + col_hz),
                Vector3::new(wt, win_hy, col_hz),
                brick,
                false,
            );
        }

        // === Gable triangles (front and back) ===
        // Triangular infill above the rectangular walls, at ±Z faces.
        // Ridge runs along Z, gable faces are in the XY plane.
        for &z_sign in &[1.0_f32, -1.0] {
            let z_outer = pz + z_sign * (wz + wt);
            let z_inner = pz + z_sign * (wz - wt);

            let (verts, faces) =
                gable_geometry(px, wall_top, wall_top + peak_h, wx + wt, z_outer, z_inner);
            spawn_hull(world, &mut entities, &verts, &faces, brick);
        }

        // === Pitched roof (two rotated OBB slabs) ===
        let eave_dist = wx + overhang;
        let slope_len = (eave_dist * eave_dist + peak_h * peak_h).sqrt();
        let roof_angle = peak_h.atan2(eave_dist);
        let roof_half_depth = wz + overhang;
        let roof_he = Vector3::new(slope_len / 2.0, rt / 2.0, roof_half_depth);

        for &side in &[-1.0_f32, 1.0] {
            let mid_x = px + side * eave_dist / 2.0;
            let mid_y = wall_top + peak_h / 2.0;
            let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -side * roof_angle);

            spawn_rotated_box(
                world,
                &mut entities,
                Point3::new(mid_x, mid_y, pz),
                roof_he,
                rotation,
                slate,
                true,
            );
        }

        // === Chimney ===
        let chimney_hw = wt * 1.2;
        let chimney_hh = peak_h * 0.55;
        let chimney_x = px + wx * 0.45;
        let chimney_z = pz - wz * 0.35;
        let chimney_y = wall_top + chimney_hh;

        spawn_box(
            world,
            &mut entities,
            Point3::new(chimney_x, chimney_y, chimney_z),
            Vector3::new(chimney_hw, chimney_hh, chimney_hw),
            brick,
            false,
        );

        entities
    }
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

/// Vertices and faces for a gable triangle (triangular prism, thin in Z).
///
/// The triangle spans from `(cx - half_w, base_y)` to `(cx + half_w, base_y)`
/// with a peak at `(cx, peak_y)`, extruded between `z_a` and `z_b`.
fn gable_geometry(
    cx: f32,
    base_y: f32,
    peak_y: f32,
    half_w: f32,
    z_a: f32,
    z_b: f32,
) -> (Vec<Vector3<f32>>, Vec<SolidFace>) {
    let vertices = vec![
        // Front triangle (z_a)
        Vector3::new(cx - half_w, base_y, z_a), // 0: bottom-left
        Vector3::new(cx + half_w, base_y, z_a), // 1: bottom-right
        Vector3::new(cx, peak_y, z_a),          // 2: peak
        // Back triangle (z_b)
        Vector3::new(cx - half_w, base_y, z_b), // 3: bottom-left
        Vector3::new(cx + half_w, base_y, z_b), // 4: bottom-right
        Vector3::new(cx, peak_y, z_b),          // 5: peak
    ];

    let faces = vec![
        // Front face (z_a side).
        SolidFace {
            vertex_indices: vec![0, 1, 2],
            opposite_vertex: 3,
        },
        // Back face (z_b side).
        SolidFace {
            vertex_indices: vec![5, 4, 3],
            opposite_vertex: 0,
        },
        // Bottom face.
        SolidFace {
            vertex_indices: vec![0, 3, 4, 1],
            opposite_vertex: 2,
        },
        // Left slope face.
        SolidFace {
            vertex_indices: vec![0, 2, 5, 3],
            opposite_vertex: 1,
        },
        // Right slope face.
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

fn create_stone_material(
    textures: &crate::resources::textures::TextureManager,
    materials: &mut crate::rendering::material::MaterialManagerBuilder,
) -> EngineResult<MaterialId> {
    let pixels = generate_stone_texture();
    let texture = textures.create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
    Ok(materials.register(WALL.material(texture)))
}

fn create_brick_material(
    textures: &crate::resources::textures::TextureManager,
    materials: &mut crate::rendering::material::MaterialManagerBuilder,
) -> EngineResult<MaterialId> {
    let pixels = generate_brick_texture();
    let texture = textures.create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
    Ok(materials.register(DETAIL.material(texture)))
}

fn create_slate_material(
    textures: &crate::resources::textures::TextureManager,
    materials: &mut crate::rendering::material::MaterialManagerBuilder,
) -> EngineResult<MaterialId> {
    let pixels = generate_slate_texture();
    let texture = textures.create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
    Ok(materials.register(ROOF.material(texture)))
}

/// Warm grey stone with fine grain noise and chisel-edge darkening.
fn generate_stone_texture() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let base = Rgb::new(0.76, 0.73, 0.66);
    let dark = Rgb::new(0.56, 0.53, 0.46);

    let seed_grain = rand_u32();
    let seed_vein = rand_u32();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let grain = fbm_2d_periodic(u * 16.0, v * 16.0, 3, 0.5, 2.0, seed_grain, Some(16));
            let mut c = base.scale(0.90 + grain * 0.10);

            let vein = fbm_2d_periodic(u * 4.0, v * 8.0, 3, 0.6, 2.0, seed_vein, Some(8));
            let vein_band = ((vein - 0.45).abs() < 0.03) as u8 as f32;
            c = c.lerp(dark, vein_band * 0.18);

            let edge = border_band(u, v, 0.06);
            c = c.scale(1.0 - edge * 0.22);
            c = c.scale(edge_vignette(u, v));

            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

/// Red-brown brick with running bond mortar pattern.
fn generate_brick_texture() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let mortar = Rgb::new(0.72, 0.70, 0.65);
    let seed_grain = rand_u32();

    let brick_rows = 6.0_f32;
    let bricks_per_row = 3.0_f32;
    let mortar_w = 0.035_f32;

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let row_v = v * brick_rows;
            let row_idx = row_v.floor() as i32;
            let frac_v = row_v.fract();

            // Running bond: offset every other row.
            let offset_u = if row_idx % 2 == 0 {
                u
            } else {
                u + 0.5 / bricks_per_row
            };
            let brick_u = offset_u * bricks_per_row;
            let col_idx = brick_u.floor() as i32;
            let frac_u = brick_u.fract();

            // Is this pixel in a mortar joint?
            let h_mortar = frac_v < mortar_w || frac_v > (1.0 - mortar_w);
            let v_mortar = frac_u < mortar_w * bricks_per_row / brick_rows
                || frac_u > (1.0 - mortar_w * bricks_per_row / brick_rows);

            if h_mortar || v_mortar {
                let grain = fbm_2d_periodic(u * 20.0, v * 20.0, 2, 0.4, 2.0, seed_grain, Some(20));
                let c = mortar.scale(0.95 + grain * 0.05);
                c.write_rgba(&mut pixels);
            } else {
                // Per-brick colour variation from hash.
                let h = hash_pair(row_idx, col_idx);
                let hue_shift = ((h & 0xFF) as f32 / 255.0 - 0.5) * 0.06;
                let val_shift = (((h >> 8) & 0xFF) as f32 / 255.0 - 0.5) * 0.08;

                let base = Rgb::new(0.62 + hue_shift, 0.32 + hue_shift * 0.4, 0.22);
                let grain = fbm_2d_periodic(u * 24.0, v * 24.0, 3, 0.5, 2.0, seed_grain, Some(24));
                let c = base.scale(0.88 + grain * 0.12 + val_shift);

                c.write_rgba(&mut pixels);
            }
        }
    }
    pixels
}

/// Dark grey-blue slate with horizontal layering.
fn generate_slate_texture() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let base = Rgb::new(0.32, 0.34, 0.38);
    let highlight = Rgb::new(0.42, 0.44, 0.48);

    let seed_layer = rand_u32();
    let seed_grain = rand_u32();

    let num_layers = 10.0_f32;

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Horizontal slate layers.
            let layer_v = v * num_layers;
            let frac_v = layer_v.fract();
            let layer_idx = layer_v.floor() as i32;

            // Slight variation per layer.
            let layer_hash = hash_pair(layer_idx, 0);
            let layer_shift = ((layer_hash & 0xFF) as f32 / 255.0 - 0.5) * 0.06;

            let mut c = Rgb::new(
                base.r + layer_shift,
                base.g + layer_shift,
                base.b + layer_shift * 0.5,
            );

            // Fine grain.
            let grain = fbm_2d_periodic(u * 20.0, v * 20.0, 3, 0.5, 2.0, seed_grain, Some(20));
            c = c.scale(0.92 + grain * 0.08);

            // Layer edge lines — subtle dark lines at boundaries.
            let edge_dist = frac_v.min(1.0 - frac_v);
            if edge_dist < 0.06 {
                let t = 1.0 - (edge_dist / 0.06);
                c = c.scale(1.0 - t * 0.15);
            }

            // Occasional light streak.
            let streak = fbm_2d_periodic(u * 3.0, v * 1.0, 2, 0.5, 2.0, seed_layer, Some(3));
            if streak > 0.65 {
                let t = (streak - 0.65) / 0.35;
                c = c.lerp(highlight, t * 0.25);
            }

            c = c.scale(edge_vignette(u, v));
            c.write_rgba(&mut pixels);
        }
    }
    pixels
}
