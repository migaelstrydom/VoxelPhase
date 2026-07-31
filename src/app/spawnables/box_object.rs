//! Box-family spawnables: Box, Crate, HeavyCrate, Plank.
//!
//! All share the cuboid model and box-style material system.

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Entity, World};

use super::shared::models::cuboid_model;
use super::shared::orientation::Yaw;
use super::shared::textures::*;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::fire::components::Flammable;
use crate::level::BoxStyle;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::{Material, MaterialId, MaterialManagerBuilder};
use crate::resources::textures::TextureManager;
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

use specs::{Builder, WorldExt};

const TEXTURE_SIZE: u32 = 128;

// ---------------------------------------------------------------------------
// Box styles — procedural texture generation
// ---------------------------------------------------------------------------

/// Generate texture pixels for a specific box style.
fn generate_pixels_for_style(style: BoxStyle) -> Vec<u8> {
    match style {
        BoxStyle::WoodenCrate => generate_wooden_crate(),
        BoxStyle::Cardboard => generate_cardboard_box(),
        BoxStyle::Metal => generate_metal_container(),
        BoxStyle::Gift => generate_gift_box(),
        BoxStyle::Stone => generate_stone_block(),
        BoxStyle::Brick => generate_brick_block(),
        BoxStyle::Warning => generate_warning_box(),
        BoxStyle::Random => {
            let pick = rand::random::<u32>() % 7;
            match pick {
                0 => generate_wooden_crate(),
                1 => generate_cardboard_box(),
                2 => generate_metal_container(),
                3 => generate_gift_box(),
                4 => generate_stone_block(),
                5 => generate_brick_block(),
                _ => generate_warning_box(),
            }
        }
    }
}

/// Creates a single box material for the given style.
pub fn create_box_material_for_style(
    style: BoxStyle,
    texture_manager: &TextureManager,
    material_builder: &mut MaterialManagerBuilder,
) -> EngineResult<MaterialId> {
    let pixels = generate_pixels_for_style(style);
    let texture = texture_manager.create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
    let material = Material::textured(texture);
    Ok(material_builder.register(material))
}

// ---------------------------------------------------------------------------
// Shared spawn helper
// ---------------------------------------------------------------------------

/// Spawn a box entity with physics and optional flammability.
#[allow(clippy::too_many_arguments)]
fn spawn_box_entity(
    world: &mut World,
    pos: Point3<f32>,
    half_extents: Vector3<f32>,
    yaw: Yaw,
    material: MaterialId,
    density: f32,
    restitution: f32,
    friction: f32,
    flammable: bool,
) -> Entity {
    let model = cuboid_model(half_extents, material);

    let body_handle = {
        let mut physics = world.write_resource::<PhysicsResource>();

        let body_desc = RigidBodyDesc::dynamic()
            .position(pos)
            .rotation(yaw.rotation())
            .gravity_scale(1.0)
            .linear_damping(0.01)
            .angular_damping(0.005);

        let body_handle = physics.world.create_body(body_desc);

        let collider_desc = ColliderDesc::box_shape(half_extents)
            .density(density)
            .restitution(restitution)
            .friction(friction);

        physics.world.attach_collider(body_handle, collider_desc);

        body_handle
    };

    let mut builder = world
        .create_entity()
        .with(Position(Vector3::new(pos.x, pos.y, pos.z)))
        .with(Velocity(Vector3::zeros()))
        .with(Orientation(yaw.rotation()))
        .with(RigidBodyComponent(body_handle))
        .with(ModelInstance::new(model))
        .with(Renderable);

    if flammable {
        builder = builder.with(Flammable::wood());
    }

    builder.build()
}

// ---------------------------------------------------------------------------
// BoxDef
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
pub struct BoxDef {
    pub pos: (f32, f32, f32),
    pub half_extents: (f32, f32, f32),
    /// Rotation about `+Y`, in degrees. Meaningful for any box that is not a
    /// cube; a rotated segment adds its own yaw to this.
    #[serde(default)]
    pub yaw: f32,
    #[serde(default)]
    pub style: BoxStyle,
    #[serde(default = "BoxDef::default_density")]
    pub density: f32,
    #[serde(default = "BoxDef::default_restitution")]
    pub restitution: f32,
    #[serde(default = "BoxDef::default_friction")]
    pub friction: f32,
}

impl BoxDef {
    pub fn default_density() -> f32 {
        50.0
    }
    pub fn default_restitution() -> f32 {
        0.2
    }
    pub fn default_friction() -> f32 {
        0.6
    }
}

impl Spawnable for BoxDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(vec![create_box_material_for_style(
            self.style,
            ctx.textures,
            ctx.materials,
        )?])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let he = Vector3::new(
            self.half_extents.0,
            self.half_extents.1,
            self.half_extents.2,
        );
        vec![spawn_box_entity(
            world,
            Point3::new(self.pos.0, self.pos.1, self.pos.2),
            he,
            Yaw::degrees(self.yaw),
            materials[0],
            self.density,
            self.restitution,
            self.friction,
            true,
        )]
    }
}

// ---------------------------------------------------------------------------
// CrateDef
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
pub struct CrateDef {
    pub pos: (f32, f32, f32),
    /// Cube half-extent.
    pub size: f32,
}

impl Spawnable for CrateDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(vec![create_box_material_for_style(
            BoxStyle::WoodenCrate,
            ctx.textures,
            ctx.materials,
        )?])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let he = Vector3::new(self.size, self.size, self.size);
        vec![spawn_box_entity(
            world,
            Point3::new(self.pos.0, self.pos.1, self.pos.2),
            he,
            Yaw::default(),
            materials[0],
            50.0,
            0.2,
            0.6,
            true,
        )]
    }
}

// ---------------------------------------------------------------------------
// HeavyCrateDef
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
pub struct HeavyCrateDef {
    pub pos: (f32, f32, f32),
    pub size: f32,
}

impl Spawnable for HeavyCrateDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(vec![create_box_material_for_style(
            BoxStyle::Metal,
            ctx.textures,
            ctx.materials,
        )?])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let he = Vector3::new(self.size, self.size, self.size);
        vec![spawn_box_entity(
            world,
            Point3::new(self.pos.0, self.pos.1, self.pos.2),
            he,
            Yaw::default(),
            materials[0],
            150.0,
            0.2,
            0.6,
            true,
        )]
    }
}

// ---------------------------------------------------------------------------
// PlankDef
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
pub struct PlankDef {
    pub pos: (f32, f32, f32),
    /// Extent along the plank's own `+X` before yaw.
    pub length: f32,
    /// Extent along its own `+Z`.
    pub width: f32,
    /// Rotation about `+Y`, in degrees.
    #[serde(default)]
    pub yaw: f32,
}

impl Spawnable for PlankDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(vec![create_box_material_for_style(
            BoxStyle::WoodenCrate,
            ctx.textures,
            ctx.materials,
        )?])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let he = Vector3::new(self.length * 0.5, 0.3, self.width * 0.5);
        vec![spawn_box_entity(
            world,
            Point3::new(self.pos.0, self.pos.1, self.pos.2),
            he,
            Yaw::degrees(self.yaw),
            materials[0],
            500.0,
            0.2,
            0.6,
            true,
        )]
    }
}

// ---------------------------------------------------------------------------
// Texture generators (moved from old box_entity.rs)
// ---------------------------------------------------------------------------

fn generate_wooden_crate() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let hue = rand_range(-0.06, 0.06);
    let base = Rgb::new(0.62 + hue, 0.44 + hue * 0.5, 0.25);
    let seed_a = rand_u32();
    let seed_b = rand_u32();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let grain = fbm_2d_periodic(u * 8.0, v * 24.0, 4, 0.55, 2.0, seed_a, Some(8));
            let variation = fbm_2d_periodic(u * 4.0, v * 4.0, 3, 0.5, 2.0, seed_b, Some(4));

            let wood_factor = 0.75 + grain * 0.25;
            let hue_shift = (variation - 0.5) * 0.08;

            let mut c = Rgb::new(
                (base.r + hue_shift) * wood_factor,
                (base.g + hue_shift * 0.5) * wood_factor,
                base.b * wood_factor,
            );

            c = c.scale(1.0 - plank_border(u, v, 0.03) * 0.45);
            c = c.scale(1.0 - corner_bracket(u, v) * 0.5);
            c = c.scale(edge_vignette(u, v));

            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

fn generate_cardboard_box() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let warmth = rand_range(-0.04, 0.04);
    let base = Rgb::new(0.76 + warmth, 0.65 + warmth, 0.48);
    let tape = Rgb::new(0.72, 0.62, 0.42);
    let seed = rand_u32();

    let tape_horizontal = rand::random::<bool>();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let fibre = fbm_2d_periodic(u * 16.0, v * 16.0, 3, 0.4, 2.5, seed, Some(16));
            let c_factor = 0.88 + fibre * 0.12;
            let mut c = base.scale(c_factor);

            let tape_coord = if tape_horizontal { v } else { u };
            let tape_dist = (tape_coord - 0.5).abs();
            if tape_dist < 0.06 {
                let tape_blend = 1.0 - (tape_dist / 0.06);
                c = c.lerp(tape, tape_blend * 0.7);
            }

            let ridge = ((v * size as f32 * 0.5).sin() * 0.5 + 0.5) * 0.05;
            c = c.scale(1.0 - ridge);

            let fold_v = fold_line(v, 0.18, 0.015);
            let fold_v2 = fold_line(v, 0.82, 0.015);
            c = c.scale(1.0 - (fold_v + fold_v2) * 0.25);

            c = c.scale(edge_vignette(u, v));
            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

fn generate_metal_container() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let tint = rand::random::<u32>() % 3;
    let base = match tint {
        0 => Rgb::new(0.52, 0.55, 0.62),
        1 => Rgb::new(0.50, 0.58, 0.52),
        _ => Rgb::new(0.58, 0.55, 0.50),
    };
    let seed = rand_u32();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let brush = fbm_2d_periodic(u * 6.0, v * 30.0, 3, 0.5, 2.0, seed, Some(6));
            let scratches = fbm_2d_periodic(u * 20.0, v * 20.0, 2, 0.3, 2.0, seed + 7, Some(20));
            let c_factor = 0.82 + brush * 0.12 + scratches * 0.06;
            let mut c = base.scale(c_factor);

            let seam = plank_border(u, v, 0.02);
            c = c.scale(1.0 - seam * 0.35);

            let rivet = rivet_pattern(u, v, size);
            c = c.scale(1.0 - rivet * 0.3);

            c = c.scale(edge_vignette(u, v));
            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

fn generate_gift_box() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let hue = rand_range(0.0, 6.0);
    let base = hue_to_rgb(hue, 0.65, 0.85);
    let ribbon = Rgb::new(0.95, 0.92, 0.55);
    let seed = rand_u32();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let pattern = fbm_2d_periodic(u * 12.0, v * 12.0, 2, 0.4, 2.0, seed, Some(12));
            let c_factor = 0.92 + pattern * 0.08;
            let mut c = base.scale(c_factor);

            let ribbon_h = (v - 0.5).abs() < 0.045;
            let ribbon_v = (u - 0.5).abs() < 0.045;
            if ribbon_h || ribbon_v {
                c = c.lerp(ribbon, 0.85);
            }

            let cx = (u - 0.5) * 2.0;
            let cy = (v - 0.5) * 2.0;
            let dist_sq = cx * cx + cy * cy;
            if dist_sq < 0.04 {
                let bow_blend = 1.0 - (dist_sq / 0.04).sqrt();
                c = c.lerp(ribbon.scale(1.1), bow_blend * 0.9);
            }

            c = c.scale(edge_vignette(u, v));
            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

fn generate_stone_block() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let warmth = rand_range(-0.05, 0.05);
    let base = Rgb::new(0.58 + warmth, 0.56 + warmth, 0.54 + warmth);
    let seed = rand_u32();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let rock = fbm_2d_periodic(u * 8.0, v * 8.0, 5, 0.6, 2.0, seed, Some(8));
            let detail = fbm_2d_periodic(u * 16.0, v * 16.0, 3, 0.4, 2.0, seed + 3, Some(16));

            let c_factor = 0.70 + rock * 0.20 + detail * 0.10;
            let mut c = base.scale(c_factor);

            let crack = crack_pattern(u, v, seed + 11);
            c = c.scale(1.0 - crack * 0.4);

            let edge = border_band(u, v, 0.06);
            c = c.scale(1.0 - edge * 0.25);

            c = c.scale(edge_vignette(u, v));
            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

fn generate_brick_block() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let hue = rand_range(-0.04, 0.04);
    let brick_colour = Rgb::new(0.72 + hue, 0.38 + hue * 0.3, 0.28);
    let mortar = Rgb::new(0.78, 0.75, 0.68);
    let seed = rand_u32();

    let rows = 4;
    let cols = 3;

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let row = (v * rows as f32).floor() as i32;
            let offset = if row % 2 == 0 { 0.0 } else { 0.5 / cols as f32 };
            let brick_u = ((u + offset) * cols as f32).fract();
            let brick_v = (v * rows as f32).fract();

            let mortar_width = 0.06;
            let is_mortar = brick_u < mortar_width
                || brick_u > (1.0 - mortar_width)
                || brick_v < mortar_width
                || brick_v > (1.0 - mortar_width);

            let mut c = if is_mortar {
                let mortar_noise =
                    fbm_2d_periodic(u * 12.0, v * 12.0, 2, 0.3, 2.0, seed + 5, Some(12));
                mortar.scale(0.92 + mortar_noise * 0.08)
            } else {
                let brick_seed = hash_pair(((u + offset) * cols as f32).floor() as i32, row);
                let brick_var = (brick_seed as f32 / u32::MAX as f32) * 0.15 - 0.075;
                let noise = fbm_2d_periodic(u * 10.0, v * 10.0, 3, 0.5, 2.0, seed, Some(10));
                let factor = 0.85 + noise * 0.15;
                Rgb::new(
                    (brick_colour.r + brick_var) * factor,
                    (brick_colour.g + brick_var * 0.5) * factor,
                    brick_colour.b * factor,
                )
            };

            c = c.scale(edge_vignette(u, v));
            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

fn generate_warning_box() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let pair = rand::random::<u32>() % 3;
    let (colour_a, colour_b) = match pair {
        0 => (Rgb::new(0.90, 0.75, 0.15), Rgb::new(0.20, 0.20, 0.20)),
        1 => (Rgb::new(0.85, 0.25, 0.20), Rgb::new(0.90, 0.90, 0.85)),
        _ => (Rgb::new(0.25, 0.50, 0.80), Rgb::new(0.90, 0.90, 0.85)),
    };
    let seed = rand_u32();
    let num_stripes = rand_range(4.0, 8.0).round();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let stripe = ((u + v) * num_stripes).fract();
            let blend = if stripe < 0.5 { 0.0 } else { 1.0 };
            let mut c = colour_a.lerp(colour_b, blend);

            let wear = fbm_2d_periodic(u * 10.0, v * 10.0, 3, 0.5, 2.0, seed, Some(10));
            c = c.scale(0.88 + wear * 0.12);

            let border = border_band(u, v, 0.05);
            c = c.scale(1.0 - border * 0.3);

            c = c.scale(edge_vignette(u, v));
            c.write_rgba(&mut pixels);
        }
    }
    pixels
}
