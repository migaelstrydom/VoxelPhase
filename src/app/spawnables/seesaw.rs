//! Seesaw spawnable — terrain-anchored tilting beam with seats and fulcrum.
//!
//! A compound body (beam + two seats) pinned at its center via a Hinge
//! constraint (Z axis free), so it tilts freely when weight is placed on
//! either end. A separate fulcrum entity sits underneath as a fully-pinned
//! terrain-anchored support. Both break free when the terrain is destroyed.

use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, UnitVector3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::finish::{ColliderSurface, MaterialSurface};
use super::shared::models::{build_convex_hull, convex_solid_model, SolidFace};
use super::shared::textures::seed_from_ground;
use super::shared::textures::Rgb;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, TerrainAnchored, Velocity,
};
use crate::core::error::EngineResult;
use crate::physics::{ColliderDesc, ConstraintKind, RigidBodyDesc};
use crate::rendering::material::{Material, MaterialId};
use crate::rendering::physical_finish::PhysicalSurface;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 128;

#[derive(Deserialize)]
pub struct SeesawDef {
    /// Position (x, z). Y is determined by terrain surface height.
    pub pos: (f32, f32),
    /// Half-length of the beam along X.
    #[serde(default = "SeesawDef::default_beam_half_length")]
    pub beam_half_length: f32,
    /// Half-width of the beam along Z.
    #[serde(default = "SeesawDef::default_beam_half_width")]
    pub beam_half_width: f32,
    /// Half-thickness of the beam along Y.
    #[serde(default = "SeesawDef::default_beam_half_thickness")]
    pub beam_half_thickness: f32,
    /// Height of the fulcrum (pivot above ground).
    #[serde(default = "SeesawDef::default_fulcrum_height")]
    pub fulcrum_height: f32,
    #[serde(default = "SeesawDef::default_density")]
    pub density: f32,
}

impl SeesawDef {
    pub fn default_beam_half_length() -> f32 {
        2.0
    }
    pub fn default_beam_half_width() -> f32 {
        0.2
    }
    pub fn default_beam_half_thickness() -> f32 {
        0.06
    }
    pub fn default_fulcrum_height() -> f32 {
        0.5
    }
    pub fn default_density() -> f32 {
        700.0
    }
}

impl SeesawDef {
    /// Half-extents of each seat box.
    fn seat_half_extents(&self) -> Vector3<f32> {
        Vector3::new(self.beam_half_width, 0.05, self.beam_half_width)
    }

    /// Half-extents of each seat backrest.
    fn backrest_half_extents(&self) -> Vector3<f32> {
        Vector3::new(0.025, 0.12, self.beam_half_width)
    }

    /// How far from center the seats are placed (fraction of beam length).
    fn seat_x_offset(&self) -> f32 {
        self.beam_half_length - self.beam_half_width - 0.05
    }

    /// The beam and its seats/backrests. Declared once so the collider and
    /// the wood it's rendered with cannot disagree about how it behaves.
    fn beam_surface(&self) -> PhysicalSurface {
        PhysicalSurface {
            restitution: 0.1,
            friction: 0.6,
            density: self.density,
        }
    }

    /// The fulcrum: denser than the beam so it stays put underneath it.
    fn fulcrum_surface(&self) -> PhysicalSurface {
        PhysicalSurface {
            restitution: 0.1,
            friction: 0.6,
            density: self.density * 1.5,
        }
    }
}

impl Spawnable for SeesawDef {
    fn material_count(&self) -> usize {
        3
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let seed = seed_from_ground(self.pos, 0);

        let beam_pixels = generate_painted_wood(seed, Rgb::new(0.85, 0.65, 0.15));
        let beam_tex =
            ctx.textures
                .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &beam_pixels, true)?;
        let beam_mat = ctx
            .materials
            .register(Material::textured(beam_tex).with_derived_finish(self.beam_surface()));

        let seat_pixels = generate_painted_wood(seed.wrapping_add(1), Rgb::new(0.80, 0.20, 0.20));
        let seat_tex =
            ctx.textures
                .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &seat_pixels, true)?;
        let seat_mat = ctx
            .materials
            .register(Material::textured(seat_tex).with_derived_finish(self.beam_surface()));

        let fulcrum_pixels =
            generate_painted_wood(seed.wrapping_add(2), Rgb::new(0.25, 0.55, 0.80));
        let fulcrum_tex =
            ctx.textures
                .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &fulcrum_pixels, true)?;
        let fulcrum_mat = ctx
            .materials
            .register(Material::textured(fulcrum_tex).with_derived_finish(self.fulcrum_surface()));

        Ok(vec![beam_mat, seat_mat, fulcrum_mat])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let beam_mat = materials[0];
        let seat_mat = materials[1];
        let fulcrum_mat = materials[2];

        let surface_y = {
            let terrain = world.read_resource::<TerrainWorld>();
            terrain.mesh_surface_height_at(self.pos.0, self.pos.1)
        };

        let Some(surface_y) = surface_y else {
            return Vec::new();
        };

        let pivot_y = surface_y + self.fulcrum_height;
        let beam_center_y = pivot_y + self.beam_half_thickness;
        let beam_pos = Point3::new(self.pos.0, beam_center_y, self.pos.1);

        let seat_he = self.seat_half_extents();
        let back_he = self.backrest_half_extents();
        let seat_x = self.seat_x_offset();
        let seat_y = self.beam_half_thickness + seat_he.y;
        let back_y = seat_y + seat_he.y + back_he.y;

        // Beam visual: compound cuboid with beam plank + 2 seats + 2 backrests.
        let beam_he = Vector3::new(
            self.beam_half_length,
            self.beam_half_thickness,
            self.beam_half_width,
        );

        let model_boxes = vec![
            // Beam plank
            (beam_he, Vector3::zeros(), beam_mat),
            // Left seat
            (seat_he, Vector3::new(-seat_x, seat_y, 0.0), seat_mat),
            // Right seat
            (seat_he, Vector3::new(seat_x, seat_y, 0.0), seat_mat),
            // Left backrest (outer edge)
            (
                back_he,
                Vector3::new(-seat_x - seat_he.x + back_he.x, back_y, 0.0),
                seat_mat,
            ),
            // Right backrest (outer edge)
            (
                back_he,
                Vector3::new(seat_x + seat_he.x - back_he.x, back_y, 0.0),
                seat_mat,
            ),
        ];

        let beam_model = {
            use super::shared::models::multi_material_compound_cuboid_model;
            multi_material_compound_cuboid_model(&model_boxes)
        };

        // Beam physics: compound collider (beam + seats + backrests).
        let (beam_handle, beam_anchor, beam_upright) = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(beam_pos)
                    .linear_damping(0.01)
                    .angular_damping(0.01),
            );

            let mat = |desc: ColliderDesc| desc.with_physical_surface(self.beam_surface());

            // Beam plank
            physics
                .world
                .attach_collider(body, mat(ColliderDesc::box_shape(beam_he)));

            // Left seat + backrest
            physics.world.attach_collider(
                body,
                mat(ColliderDesc::box_shape(seat_he)
                    .offset_translation(Vector3::new(-seat_x, seat_y, 0.0))),
            );
            physics.world.attach_collider(
                body,
                mat(
                    ColliderDesc::box_shape(back_he).offset_translation(Vector3::new(
                        -seat_x - seat_he.x + back_he.x,
                        back_y,
                        0.0,
                    )),
                ),
            );

            // Right seat + backrest
            physics.world.attach_collider(
                body,
                mat(ColliderDesc::box_shape(seat_he)
                    .offset_translation(Vector3::new(seat_x, seat_y, 0.0))),
            );
            physics.world.attach_collider(
                body,
                mat(
                    ColliderDesc::box_shape(back_he).offset_translation(Vector3::new(
                        seat_x + seat_he.x - back_he.x,
                        back_y,
                        0.0,
                    )),
                ),
            );

            // Hinge at beam center — tilts freely around Z only.
            let anchor = physics.world.create_constraint(ConstraintKind::world_hinge(
                body,
                beam_pos,
                Vector3::zeros(),
                UnitVector3::new_normalize(Vector3::z()),
                &UnitQuaternion::identity(),
                0.0,
                f32::MAX,
            ));

            // Dummy KeepUpright handle — we need one for TerrainAnchored but the
            // seesaw must tilt freely, so we don't create one. Use the anchor
            // handle as a placeholder; TerrainAnchored will try to remove it
            // twice on release, which is harmless (remove_constraint returns false).
            (body, anchor, anchor)
        };

        let anchor_check = Point3::new(self.pos.0, surface_y - 0.1, self.pos.1);

        let beam_entity = world
            .create_entity()
            .with(Position(Vector3::new(beam_pos.x, beam_pos.y, beam_pos.z)))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(beam_handle))
            .with(ModelInstance::new(beam_model))
            .with(Renderable)
            .with(TerrainAnchored {
                anchor_handle: beam_anchor,
                upright_handle: beam_upright,
                anchor_world: anchor_check,
                released_collider: None,
                released_model: None,
            })
            .build();

        // --- Fulcrum (separate body, fully pinned) ---
        // Slightly shorter than the pivot height so it doesn't collide with the beam.
        let fulcrum_gap = 0.04;
        let fulcrum_actual_height = self.fulcrum_height - fulcrum_gap;
        let fulcrum_base_half = fulcrum_actual_height * 0.6;
        let fulcrum_depth = self.beam_half_width * 1.2;
        let fulcrum_pos = Point3::new(
            self.pos.0,
            surface_y + fulcrum_actual_height * 0.5,
            self.pos.1,
        );

        let (fulcrum_verts, fulcrum_faces) =
            build_fulcrum_geometry(fulcrum_actual_height, fulcrum_base_half, fulcrum_depth);

        let fulcrum_model = convex_solid_model(&fulcrum_verts, &fulcrum_faces, fulcrum_mat);
        let fulcrum_hull = build_convex_hull(&fulcrum_verts, &fulcrum_faces);

        let (fulcrum_handle, fulcrum_fixed_h) = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(fulcrum_pos)
                    .linear_damping(0.01)
                    .angular_damping(0.01),
            );

            physics.world.attach_collider(
                body,
                ColliderDesc::convex_hull(Arc::new(fulcrum_hull))
                    .with_physical_surface(self.fulcrum_surface()),
            );

            // Single Fixed constraint replaces AnchorPoint + KeepUpright.
            // Locks all 6 DOF; tilt rows get HardProjection (compliance=0).
            let fixed = physics.world.create_constraint(ConstraintKind::world_fixed(
                body,
                fulcrum_pos,
                Vector3::zeros(),
                0.0,
                f32::MAX,
            ));

            (body, fixed)
        };

        let fulcrum_entity = world
            .create_entity()
            .with(Position(Vector3::new(
                fulcrum_pos.x,
                fulcrum_pos.y,
                fulcrum_pos.z,
            )))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(fulcrum_handle))
            .with(ModelInstance::new(fulcrum_model))
            .with(Renderable)
            .with(TerrainAnchored {
                anchor_handle: fulcrum_fixed_h,
                upright_handle: fulcrum_fixed_h,
                anchor_world: anchor_check,
                released_collider: None,
                released_model: None,
            })
            .build();

        vec![beam_entity, fulcrum_entity]
    }
}

// ---------------------------------------------------------------------------
// Fulcrum geometry — triangular prism (ridge at top, flat base)
// ---------------------------------------------------------------------------

/// Build the vertices and faces for a triangular prism fulcrum.
///
/// The prism has a ridge along Z at the top (y = +half_height) and a flat
/// base at y = -half_height. Origin is at the geometric center.
fn build_fulcrum_geometry(
    height: f32,
    base_half_x: f32,
    depth: f32,
) -> (Vec<Vector3<f32>>, Vec<SolidFace>) {
    let half_h = height * 0.5;
    let half_d = depth * 0.5;

    //   0---1       top ridge (y = +half_h)
    //  /|   |\
    // 2-+---+-3     bottom (y = -half_h)
    // Front face shown; back face is 4,5,6,7 at z = +half_d
    // Actually simpler: 6 vertices for a triangular prism.

    // Front triangle (z = -half_d): top, bottom-left, bottom-right
    // Back triangle (z = +half_d): top, bottom-left, bottom-right
    let vertices = vec![
        Vector3::new(0.0, half_h, -half_d),           // 0: front top
        Vector3::new(-base_half_x, -half_h, -half_d), // 1: front bottom-left
        Vector3::new(base_half_x, -half_h, -half_d),  // 2: front bottom-right
        Vector3::new(0.0, half_h, half_d),            // 3: back top
        Vector3::new(-base_half_x, -half_h, half_d),  // 4: back bottom-left
        Vector3::new(base_half_x, -half_h, half_d),   // 5: back bottom-right
    ];

    let faces = vec![
        // Front triangle
        SolidFace {
            vertex_indices: vec![0, 2, 1],
            opposite_vertex: 3,
        },
        // Back triangle
        SolidFace {
            vertex_indices: vec![3, 4, 5],
            opposite_vertex: 0,
        },
        // Left slope
        SolidFace {
            vertex_indices: vec![0, 1, 4, 3],
            opposite_vertex: 2,
        },
        // Right slope
        SolidFace {
            vertex_indices: vec![0, 3, 5, 2],
            opposite_vertex: 1,
        },
        // Bottom
        SolidFace {
            vertex_indices: vec![1, 2, 5, 4],
            opposite_vertex: 0,
        },
    ];

    (vertices, faces)
}

// ---------------------------------------------------------------------------
// Texture generation
// ---------------------------------------------------------------------------

/// Painted wood: a base colour with visible wood grain underneath and
/// paint wear revealing the natural wood.
fn generate_painted_wood(seed: u32, paint_colour: Rgb) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let wood_under = Rgb::new(0.55, 0.40, 0.22);

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Wood grain showing through paint.
            let grain = fbm_2d_periodic(u * 16.0, v * 3.0, 3, 0.5, 2.0, seed, Some(16));
            let grain_factor = 0.90 + grain * 0.10;
            let mut colour = paint_colour.scale(grain_factor);

            // Paint wear patches: reveal wood underneath.
            let wear = fbm_2d_periodic(
                u * 5.0,
                v * 5.0,
                3,
                0.6,
                2.0,
                seed.wrapping_add(10),
                Some(5),
            );
            if wear > 0.40 {
                let t = ((wear - 0.40) / 0.35).clamp(0.0, 0.5);
                colour = colour.lerp(wood_under, t);
            }

            // Subtle colour variation.
            let variation = fbm_2d_periodic(
                u * 3.0,
                v * 3.0,
                2,
                0.4,
                2.0,
                seed.wrapping_add(20),
                Some(3),
            );
            colour = colour.scale(0.92 + variation * 0.08);

            // Edge darkening.
            let eu = (u - 0.5).abs() * 2.0;
            let ev = (v - 0.5).abs() * 2.0;
            let edge = ((eu.max(ev) - 0.85) / 0.15).clamp(0.0, 1.0);
            colour = colour.scale(1.0 - edge * 0.2);

            colour.write_rgba(&mut pixels);
        }
    }
    pixels
}
