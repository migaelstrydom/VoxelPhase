//! Box-family spawnables: Box, Crate, HeavyCrate, Plank.
//!
//! All share the cuboid model and box-style material system.

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Entity, World};

use super::shared::finish::{ColliderSurface, MaterialSurface};
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
use crate::rendering::physical_finish::PhysicalSurface;
use crate::resources::textures::TextureManager;
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

use specs::{Builder, WorldExt};

const TEXTURE_SIZE: u32 = 128;

/// Every box in the family shares this feel; only density separates them.
/// Densities here are gameplay values rather than physical ones — a crate at
/// 50 kg/m3 is a tenth of real timber — which is why none of them reads as
/// metal however metallic its texture looks.
const CRATE_RESTITUTION: f32 = 0.2;
const CRATE_FRICTION: f32 = 0.6;

/// A light wooden crate.
pub const CRATE_SURFACE: PhysicalSurface = PhysicalSurface {
    restitution: CRATE_RESTITUTION,
    friction: CRATE_FRICTION,
    density: 50.0,
};

/// The same crate, three times the mass.
pub const HEAVY_CRATE_SURFACE: PhysicalSurface = PhysicalSurface {
    restitution: CRATE_RESTITUTION,
    friction: CRATE_FRICTION,
    density: 150.0,
};

/// Solid timber, and the heaviest of the family.
pub const PLANK_SURFACE: PhysicalSurface = PhysicalSurface {
    restitution: CRATE_RESTITUTION,
    friction: CRATE_FRICTION,
    density: 500.0,
};

/// The crate feel at a caller-chosen density, for the spawnables that build
/// walls and towers out of boxes and let a level set how heavy they are.
pub const fn crate_surface_at(density: f32) -> PhysicalSurface {
    PhysicalSurface {
        restitution: CRATE_RESTITUTION,
        friction: CRATE_FRICTION,
        density,
    }
}

// ---------------------------------------------------------------------------
// Box styles — procedural texture generation
// ---------------------------------------------------------------------------

/// Generate texture pixels for a specific box style, varied by `seed`.
///
/// A pure function of its two arguments. The same style and seed give the same
/// pixels in any level, in any order, however many other objects were built
/// first — which is what lets a level author rely on what a box looks like.
fn generate_pixels_for_style(style: BoxStyle, seed: u32) -> Vec<u8> {
    let rng = &mut TextureRng::new(seed);
    match style {
        BoxStyle::WoodenCrate => generate_wooden_crate(rng),
        BoxStyle::Cardboard => generate_cardboard_box(rng),
        BoxStyle::Metal => generate_metal_container(rng),
        BoxStyle::Gift => generate_gift_box(rng),
        BoxStyle::Stone => generate_stone_block(rng),
        BoxStyle::Brick => generate_brick_block(rng),
        BoxStyle::Warning => generate_warning_box(rng),
        BoxStyle::Random => match rng.pick(7) {
            0 => generate_wooden_crate(rng),
            1 => generate_cardboard_box(rng),
            2 => generate_metal_container(rng),
            3 => generate_gift_box(rng),
            4 => generate_stone_block(rng),
            5 => generate_brick_block(rng),
            _ => generate_warning_box(rng),
        },
    }
}

/// Creates a single box material for the given style.
///
/// `seed` picks which of the style's variations this box gets. Derive it from
/// something stable about the object — where it spawns, its index in a stack —
/// so the same level file always produces the same level. A constant is the
/// right answer for equipment that should all match.
pub fn create_box_material_for_style(
    style: BoxStyle,
    surface: PhysicalSurface,
    seed: u32,
    texture_manager: &TextureManager,
    material_builder: &mut MaterialManagerBuilder,
) -> EngineResult<MaterialId> {
    let pixels = generate_pixels_for_style(style, seed);
    let texture = texture_manager.create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
    let material = Material::textured(texture).with_derived_finish(surface);
    Ok(material_builder.register(material))
}

// ---------------------------------------------------------------------------
// Shared spawn helper
// ---------------------------------------------------------------------------

/// Spawn a box entity with physics and optional flammability.
fn spawn_box_entity(
    world: &mut World,
    pos: Point3<f32>,
    half_extents: Vector3<f32>,
    yaw: Yaw,
    material: MaterialId,
    surface: PhysicalSurface,
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

        let collider_desc = ColliderDesc::box_shape(half_extents).with_physical_surface(surface);

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

    /// The one declaration of this box's physics, shading included.
    fn surface(&self) -> PhysicalSurface {
        PhysicalSurface {
            restitution: self.restitution,
            friction: self.friction,
            density: self.density,
        }
    }
}

impl Spawnable for BoxDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(vec![create_box_material_for_style(
            self.style,
            self.surface(),
            seed_from_position(self.pos, 0),
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
            self.surface(),
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
            CRATE_SURFACE,
            seed_from_position(self.pos, 0),
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
            CRATE_SURFACE,
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
            HEAVY_CRATE_SURFACE,
            seed_from_position(self.pos, 0),
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
            HEAVY_CRATE_SURFACE,
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
            PLANK_SURFACE,
            seed_from_position(self.pos, 0),
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
            PLANK_SURFACE,
            true,
        )]
    }
}

// ---------------------------------------------------------------------------
// Texture generators (moved from old box_entity.rs)
// ---------------------------------------------------------------------------

fn generate_wooden_crate(rng: &mut TextureRng) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let hue = rng.range(-0.06, 0.06);
    let base = Rgb::new(0.62 + hue, 0.44 + hue * 0.5, 0.25);
    let seed_a = rng.u32();
    let seed_b = rng.u32();

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

fn generate_cardboard_box(rng: &mut TextureRng) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let warmth = rng.range(-0.04, 0.04);
    let base = Rgb::new(0.76 + warmth, 0.65 + warmth, 0.48);
    let tape = Rgb::new(0.72, 0.62, 0.42);
    let seed = rng.u32();

    let tape_horizontal = rng.flip();

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

fn generate_metal_container(rng: &mut TextureRng) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let tint = rng.pick(3);
    let base = match tint {
        0 => Rgb::new(0.52, 0.55, 0.62),
        1 => Rgb::new(0.50, 0.58, 0.52),
        _ => Rgb::new(0.58, 0.55, 0.50),
    };
    let seed = rng.u32();

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

fn generate_gift_box(rng: &mut TextureRng) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let hue = rng.range(0.0, 6.0);
    let base = hue_to_rgb(hue, 0.65, 0.85);
    let ribbon = Rgb::new(0.95, 0.92, 0.55);
    let seed = rng.u32();

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

fn generate_stone_block(rng: &mut TextureRng) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let warmth = rng.range(-0.05, 0.05);
    let base = Rgb::new(0.58 + warmth, 0.56 + warmth, 0.54 + warmth);
    let seed = rng.u32();

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

fn generate_brick_block(rng: &mut TextureRng) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let hue = rng.range(-0.04, 0.04);
    let brick_colour = Rgb::new(0.72 + hue, 0.38 + hue * 0.3, 0.28);
    let mortar = Rgb::new(0.78, 0.75, 0.68);
    let seed = rng.u32();

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

fn generate_warning_box(rng: &mut TextureRng) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    // Hazard yellow on black, and only that. The colourway is what the style
    // *means* — a blue stripe is not a warning — so it is not a thing to vary.
    // What varies with the seed is the wear and how coarse the stripes are.
    let colour_a = Rgb::new(0.90, 0.75, 0.15);
    let colour_b = Rgb::new(0.20, 0.20, 0.20);
    let seed = rng.u32();
    let num_stripes = rng.range(4.0, 8.0).round();

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

#[cfg(test)]
mod tests {
    use super::*;

    /// The property the whole seed exists for. Before it, every generator drew
    /// from the global `rand`, so a level's textures depended on how many
    /// objects had been baked before them — three platforms asking for the same
    /// hazard stripe came back yellow, red and blue, and adding an unrelated
    /// object to the level file reshuffled which got which.
    #[test]
    fn a_style_and_a_seed_are_all_a_box_texture_depends_on() {
        for style in [
            BoxStyle::WoodenCrate,
            BoxStyle::Cardboard,
            BoxStyle::Metal,
            BoxStyle::Gift,
            BoxStyle::Stone,
            BoxStyle::Brick,
            BoxStyle::Warning,
            BoxStyle::Random,
        ] {
            let first = generate_pixels_for_style(style, 12345);
            // Bake something else in between: under the old scheme this is
            // exactly what moved the stream on and changed the answer.
            let _ = generate_pixels_for_style(BoxStyle::Gift, 999);
            let again = generate_pixels_for_style(style, 12345);
            assert_eq!(first, again, "{style:?} is not a function of its seed");
        }
    }

    /// And the seed must actually do something, or every crate in a wall would
    /// be the same crate.
    #[test]
    fn a_different_seed_is_a_different_box() {
        let a = generate_pixels_for_style(BoxStyle::WoodenCrate, 1);
        let b = generate_pixels_for_style(BoxStyle::WoodenCrate, 2);
        assert_ne!(a, b);
    }

    /// A warning stripe is yellow and black at every seed. The style names a
    /// meaning rather than a look, and the two colourways it used to also pick
    /// from — red on white, blue on white — do not carry that meaning.
    #[test]
    fn a_warning_box_is_hazard_yellow_whatever_its_seed() {
        for seed in [0, 1, 7, 4242, u32::MAX] {
            let pixels = generate_pixels_for_style(BoxStyle::Warning, seed);
            let brightest_blue = pixels
                .chunks_exact(4)
                .map(|px| px[2])
                .max()
                .expect("texture is not empty");
            assert!(
                brightest_blue < 128,
                "seed {seed} produced a texture with blue in it ({brightest_blue}), \
                 which means a colourway other than hazard yellow"
            );
        }
    }

    /// Two objects in different places differ; the same object is itself again
    /// on the next load, whatever else the level gained in between.
    #[test]
    fn a_position_seeds_a_box_by_where_it_is() {
        assert_eq!(
            seed_from_position((1.0, 2.0, 3.0), 0),
            seed_from_position((1.0, 2.0, 3.0), 0)
        );
        assert_ne!(
            seed_from_position((1.0, 2.0, 3.0), 0),
            seed_from_position((1.0, 2.0, 4.0), 0)
        );
        assert_ne!(
            seed_from_position((1.0, 2.0, 3.0), 0),
            seed_from_position((1.0, 2.0, 3.0), 1)
        );
    }
}
