//! House spawnable — brick house with pitched gable roof, chimney, door,
//! and window openings. Built from individual OBB and convex-hull pieces
//! so every part can be knocked loose independently.

use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::{build_convex_hull, convex_solid_model, cuboid_model, SolidFace};
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::fire::components::Flammable;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::MaterialId;
use crate::rendering::pattern;
use crate::rendering::substance::{self, ColliderSubstance, Substance};
use crate::systems::PhysicsResource;

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

/// Fixed, so every house is built of the same stone and one texture serves the
/// whole street.
const WALL_SEED: u32 = 11;
const DETAIL_SEED: u32 = 22;
const ROOF_SEED: u32 = 33;

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
            ctx.patterned(&WALL, &pattern::STONE, WALL_SEED, TEXTURE_SIZE)?,
            ctx.patterned(&DETAIL, &pattern::DRESSED_STONE, DETAIL_SEED, TEXTURE_SIZE)?,
            ctx.patterned(&ROOF, &pattern::SLATE, ROOF_SEED, TEXTURE_SIZE)?,
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
