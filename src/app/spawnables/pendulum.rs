//! Pendulum spawnable — terrain-anchored frame with a swinging ball.
//!
//! An inverted-L frame (vertical post + horizontal arm) pinned to the terrain
//! via a Fixed constraint. A colourful sphere hangs from the arm tip via a
//! world-anchored BallJoint, free to swing in any direction. A thin rope
//! cylinder connects the ball to the anchor point. Both constraints are
//! released when the terrain beneath is destroyed.

use std::f32::consts::PI;
use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::finish::{ColliderSurface, MaterialSurface};
use super::shared::models::multi_material_compound_cuboid_model;
use super::shared::textures::seed_from_ground;
use super::shared::textures::Rgb;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, TerrainAnchored, Velocity,
};
use crate::core::error::EngineResult;
use crate::geometry::{generate_cylinder, generate_sphere_indices, generate_sphere_vertices};
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{ColliderDesc, ConstraintKind, RigidBodyDesc};
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId};
use crate::rendering::physical_finish::PhysicalSurface;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;
use crate::utils::noise::fbm_2d_periodic;

/// Steel, in kg/m3. The frame is the anchor the whole toy swings from.
const FRAME_DENSITY: f32 = 7800.0;

const TEXTURE_SIZE: u32 = 128;
const SPHERE_SEGMENTS: u32 = 24;
const SPHERE_RINGS: u32 = 16;
const ROPE_RADIUS: f32 = 0.015;
const ROPE_SEGMENTS: u32 = 6;

#[derive(Deserialize)]
pub struct PendulumDef {
    /// Position (x, z). Y is determined by terrain surface height.
    pub pos: (f32, f32),
    /// Height of the vertical post.
    #[serde(default = "PendulumDef::default_frame_height")]
    pub frame_height: f32,
    /// Length of the horizontal arm extending from the post top.
    #[serde(default = "PendulumDef::default_arm_length")]
    pub arm_length: f32,
    /// Distance from the arm tip down to the ball center.
    #[serde(default = "PendulumDef::default_rope_length")]
    pub rope_length: f32,
    /// Radius of the pendulum ball.
    #[serde(default = "PendulumDef::default_ball_radius")]
    pub ball_radius: f32,
    /// Density of the ball (kg/m^3).
    #[serde(default = "PendulumDef::default_ball_density")]
    pub ball_density: f32,
}

impl PendulumDef {
    pub fn default_frame_height() -> f32 {
        3.0
    }
    pub fn default_arm_length() -> f32 {
        1.5
    }
    pub fn default_rope_length() -> f32 {
        1.5
    }
    pub fn default_ball_radius() -> f32 {
        0.35
    }
    pub fn default_ball_density() -> f32 {
        2400.0
    }
}

impl PendulumDef {
    /// The steel frame: dense enough to stay put while the bob swings, and
    /// dense enough that the derived finish reads as metal.
    fn frame_surface(&self) -> PhysicalSurface {
        PhysicalSurface {
            restitution: 0.1,
            friction: 0.5,
            density: FRAME_DENSITY,
        }
    }

    /// The bob. Its density is a level's choice, so how metallic it looks is
    /// too — a heavier bob arrives looking heavier.
    fn ball_surface(&self) -> PhysicalSurface {
        PhysicalSurface {
            restitution: 0.4,
            friction: 0.6,
            density: self.ball_density,
        }
    }

    /// Half-extents of the vertical post.
    fn post_half_extents(&self) -> Vector3<f32> {
        Vector3::new(0.08, self.frame_height / 2.0, 0.08)
    }

    /// Half-extents of the horizontal arm.
    fn arm_half_extents(&self) -> Vector3<f32> {
        Vector3::new(self.arm_length / 2.0, 0.06, 0.06)
    }

    /// Post center position relative to the terrain surface (frame base).
    fn post_center_from_base(&self) -> Vector3<f32> {
        Vector3::new(0.0, self.frame_height / 2.0, 0.0)
    }

    /// Arm center position relative to the terrain surface (frame base).
    fn arm_center_from_base(&self) -> Vector3<f32> {
        Vector3::new(self.arm_length / 2.0, self.frame_height - 0.06, 0.0)
    }

    /// Volume-weighted center of mass of the L-shape, relative to frame base.
    /// Both parts share the same density so volume ratios = mass ratios.
    fn frame_com_from_base(&self) -> Vector3<f32> {
        let post_he = self.post_half_extents();
        let arm_he = self.arm_half_extents();
        let post_vol = post_he.x * post_he.y * post_he.z * 8.0;
        let arm_vol = arm_he.x * arm_he.y * arm_he.z * 8.0;
        let total_vol = post_vol + arm_vol;
        (self.post_center_from_base() * post_vol + self.arm_center_from_base() * arm_vol)
            / total_vol
    }

    /// World-space position of the arm tip (where the ball hangs from).
    fn arm_tip_world(&self, frame_base: &Point3<f32>) -> Point3<f32> {
        Point3::new(
            frame_base.x + self.arm_length,
            frame_base.y + self.frame_height - 0.06,
            frame_base.z,
        )
    }
}

impl Spawnable for PendulumDef {
    fn material_count(&self) -> usize {
        3
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let seed = seed_from_ground(self.pos, 0);

        let frame_pixels = generate_painted_wood(seed, Rgb::new(0.20, 0.55, 0.85));
        let frame_tex =
            ctx.textures
                .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &frame_pixels, true)?;
        let frame_mat = ctx
            .materials
            .register(Material::textured(frame_tex).with_derived_finish(self.frame_surface()));

        let ball_pixels = generate_beach_ball_texture(seed.wrapping_add(10));
        let ball_tex =
            ctx.textures
                .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &ball_pixels, true)?;
        let ball_mat = ctx
            .materials
            .register(Material::textured(ball_tex).with_derived_finish(self.ball_surface()));

        let rope_pixels = generate_painted_wood(seed.wrapping_add(20), Rgb::new(0.55, 0.40, 0.25));
        let rope_tex =
            ctx.textures
                .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &rope_pixels, true)?;
        let rope_mat = ctx
            .materials
            .register(Material::textured(rope_tex).with_derived_finish(self.frame_surface()));

        Ok(vec![frame_mat, ball_mat, rope_mat])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let frame_mat = materials[0];
        let ball_mat = materials[1];
        let rope_mat = materials[2];

        let surface_y = {
            let terrain = world.read_resource::<TerrainWorld>();
            terrain.mesh_surface_height_at(self.pos.0, self.pos.1)
        };

        let Some(surface_y) = surface_y else {
            return Vec::new();
        };

        let frame_base = Point3::new(self.pos.0, surface_y, self.pos.1);
        let arm_tip = self.arm_tip_world(&frame_base);
        let ball_center = Point3::new(arm_tip.x, arm_tip.y - self.rope_length, arm_tip.z);

        // --- Frame entity ---
        //
        // The body origin is at the L-shape's center of mass so the frame
        // tumbles naturally when the constraint is released. Collider offsets
        // and model vertices are relative to this CoM.

        let com = self.frame_com_from_base();
        let frame_world_pos = frame_base + com;

        let frame_model = multi_material_compound_cuboid_model(&[
            (
                self.post_half_extents(),
                self.post_center_from_base() - com,
                frame_mat,
            ),
            (
                self.arm_half_extents(),
                self.arm_center_from_base() - com,
                frame_mat,
            ),
        ]);

        let (frame_handle, frame_constraint) = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(frame_world_pos)
                    .linear_damping(0.01)
                    .angular_damping(0.01),
            );

            physics.world.attach_collider(
                body,
                ColliderDesc::box_shape(self.post_half_extents())
                    .with_physical_surface(self.frame_surface())
                    .offset_translation(self.post_center_from_base() - com),
            );
            physics.world.attach_collider(
                body,
                ColliderDesc::box_shape(self.arm_half_extents())
                    .with_physical_surface(self.frame_surface())
                    .offset_translation(self.arm_center_from_base() - com),
            );

            let constraint = physics.world.create_constraint(ConstraintKind::world_fixed(
                body,
                frame_world_pos,
                Vector3::zeros(),
                &UnitQuaternion::identity(),
                0.0,
                f32::MAX,
            ));

            (body, constraint)
        };

        let anchor_check = Point3::new(self.pos.0, surface_y - 0.1, self.pos.1);

        let frame_entity = world
            .create_entity()
            .with(Position(frame_world_pos.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(frame_handle))
            .with(ModelInstance::new(frame_model))
            .with(Renderable)
            .with(TerrainAnchored {
                anchor_handle: frame_constraint,
                upright_handle: frame_constraint,
                anchor_points: vec![anchor_check],
                released_collider: None,
                released_model: None,
            })
            .build();

        // --- Ball entity ---
        //
        // The model includes both the sphere and a thin rope cylinder
        // extending upward from the ball center to the anchor point.
        // The rope is a static visual — it only looks correct when the
        // ball hangs directly below the anchor. At small swing angles
        // it's close enough; at large angles the rope visually stretches
        // but the physics is still correct.

        let ball_model =
            build_ball_with_rope_model(self.ball_radius, self.rope_length, ball_mat, rope_mat);

        let (ball_handle, ball_constraint) = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(ball_center)
                    .linear_damping(0.02)
                    .angular_damping(0.01),
            );

            physics.world.attach_collider(
                body,
                ColliderDesc::sphere(self.ball_radius).with_physical_surface(self.ball_surface()),
            );

            let constraint = physics
                .world
                .create_constraint(ConstraintKind::world_ball_joint(
                    body,
                    arm_tip,
                    Vector3::new(0.0, self.rope_length, 0.0),
                    0.0,
                    f32::MAX,
                ));

            (body, constraint)
        };

        let ball_entity = world
            .create_entity()
            .with(Position(ball_center.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(ball_handle))
            .with(ModelInstance::new(ball_model))
            .with(Renderable)
            .with(TerrainAnchored {
                anchor_handle: ball_constraint,
                upright_handle: ball_constraint,
                anchor_points: vec![anchor_check],
                released_collider: None,
                released_model: Some(build_ball_model(self.ball_radius, ball_mat)),
            })
            .build();

        vec![frame_entity, ball_entity]
    }
}

// ---------------------------------------------------------------------------
// Ball + rope model
// ---------------------------------------------------------------------------

/// Build a textured sphere model (no rope).
fn build_ball_model(radius: f32, material: MaterialId) -> Arc<Model> {
    let vertices = generate_sphere_vertices(radius, SPHERE_SEGMENTS, SPHERE_RINGS, Colour::WHITE);
    let indices = generate_sphere_indices(SPHERE_SEGMENTS, SPHERE_RINGS);

    let parts = vec![ModelPart::new(vec![MeshPrimitive {
        vertices,
        indices,
        material,
    }])];

    Arc::new(Model::flat(parts))
}

/// Build a textured sphere with a thin rope cylinder extending upward.
fn build_ball_with_rope_model(
    radius: f32,
    rope_length: f32,
    ball_material: MaterialId,
    rope_material: MaterialId,
) -> Arc<Model> {
    let ball_verts = generate_sphere_vertices(radius, SPHERE_SEGMENTS, SPHERE_RINGS, Colour::WHITE);
    let ball_indices = generate_sphere_indices(SPHERE_SEGMENTS, SPHERE_RINGS);

    let rope_start = Point3::new(0.0, radius, 0.0);
    let rope_end = Point3::new(0.0, rope_length, 0.0);
    let (rope_verts, rope_indices) = generate_cylinder(
        rope_start,
        rope_end,
        ROPE_RADIUS,
        ROPE_SEGMENTS,
        Colour::WHITE,
    );

    let parts = vec![ModelPart::new(vec![
        MeshPrimitive {
            vertices: ball_verts,
            indices: ball_indices,
            material: ball_material,
        },
        MeshPrimitive {
            vertices: rope_verts,
            indices: rope_indices,
            material: rope_material,
        },
    ])];

    Arc::new(Model::flat(parts))
}

// ---------------------------------------------------------------------------
// Texture generation
// ---------------------------------------------------------------------------

/// Painted wood with visible grain — same style as the seesaw.
fn generate_painted_wood(seed: u32, base_colour: Rgb) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);
    let dark = base_colour.scale(0.6);

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let grain = fbm_2d_periodic(u * 12.0, v * 3.0, 2, 0.4, 2.0, seed, Some(12));
            let colour = base_colour.lerp(dark, (grain * 0.5 + 0.5).clamp(0.0, 1.0) * 0.3);
            colour.write_rgba(&mut pixels);
        }
    }
    pixels
}

/// Cartoony beach-ball style stripes: bold alternating colour segments
/// with a white equator band.
fn generate_beach_ball_texture(seed: u32) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let colours = [
        Rgb::new(0.95, 0.20, 0.20), // red
        Rgb::new(1.00, 1.00, 1.00), // white
        Rgb::new(0.20, 0.60, 0.95), // blue
        Rgb::new(1.00, 1.00, 1.00), // white
        Rgb::new(0.95, 0.85, 0.10), // yellow
        Rgb::new(1.00, 1.00, 1.00), // white
    ];
    let num_segments = colours.len() as f32;

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let seg_idx = (u * num_segments) as usize % colours.len();
            let seg_phase = (u * num_segments).fract();

            let mut colour = colours[seg_idx];

            // Soft transition between segments.
            let edge_width = 0.08;
            if seg_phase < edge_width {
                let prev_idx = (seg_idx + colours.len() - 1) % colours.len();
                let t = seg_phase / edge_width;
                colour = colours[prev_idx].lerp(colour, t);
            } else if seg_phase > 1.0 - edge_width {
                let next_idx = (seg_idx + 1) % colours.len();
                let t = (seg_phase - (1.0 - edge_width)) / edge_width;
                colour = colour.lerp(colours[next_idx], t);
            }

            // Subtle shading: darken near poles.
            let lat_factor = (v * PI).sin();
            colour = colour.scale(0.7 + lat_factor * 0.3);

            // Faint surface noise for a painted look.
            let noise = fbm_2d_periodic(u * 8.0, v * 8.0, 2, 0.3, 2.0, seed, Some(8));
            colour = colour.scale(0.92 + noise * 0.08);

            colour.write_rgba(&mut pixels);
        }
    }
    pixels
}
