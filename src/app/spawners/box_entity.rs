use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use specs::{Builder, Entity, World, WorldExt};

use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::fire::components::Flammable;
use crate::core::error::EngineResult;
use crate::geometry::{generate_cube_indices, generate_cube_vertices};
use crate::level::BoxStyle;
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId, MaterialManagerBuilder};
use crate::resources::textures::TextureManager;
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 128;

/// Physics parameters for a spawned box.
pub struct BoxPhysics {
    pub density: f32,
    pub restitution: f32,
    pub friction: f32,
}

impl Default for BoxPhysics {
    fn default() -> Self {
        Self {
            density: 50.0,
            restitution: 0.2,
            friction: 0.6,
        }
    }
}

// ---------------------------------------------------------------------------
// Colour helpers
// ---------------------------------------------------------------------------

/// RGB colour used during texture generation.
#[derive(Clone, Copy)]
struct Rgb {
    r: f32,
    g: f32,
    b: f32,
}

impl Rgb {
    fn new(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b }
    }

    fn scale(self, factor: f32) -> Self {
        Self::new(self.r * factor, self.g * factor, self.b * factor)
    }

    fn lerp(self, other: Self, t: f32) -> Self {
        Self::new(
            self.r + (other.r - self.r) * t,
            self.g + (other.g - self.g) * t,
            self.b + (other.b - self.b) * t,
        )
    }

    fn write_rgba(self, pixels: &mut Vec<u8>) {
        pixels.push((self.r.clamp(0.0, 1.0) * 255.0) as u8);
        pixels.push((self.g.clamp(0.0, 1.0) * 255.0) as u8);
        pixels.push((self.b.clamp(0.0, 1.0) * 255.0) as u8);
        pixels.push(255);
    }
}

fn rand_range(lo: f32, hi: f32) -> f32 {
    lo + rand::random::<f32>() * (hi - lo)
}

fn rand_u32() -> u32 {
    rand::random::<u32>()
}

// ---------------------------------------------------------------------------
// Box styles
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

/// Wooden crate with grain, plank borders and corner brackets.
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

            // Plank borders
            let border = plank_border(u, v, 0.03);
            c = c.scale(1.0 - border * 0.45);

            // Corner brackets
            c = c.scale(1.0 - corner_bracket(u, v) * 0.5);

            // Edge vignette
            c = c.scale(edge_vignette(u, v));

            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

/// Cardboard box with tape strips and printed labels.
fn generate_cardboard_box() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let warmth = rand_range(-0.04, 0.04);
    let base = Rgb::new(0.76 + warmth, 0.65 + warmth, 0.48);
    let tape = Rgb::new(0.72, 0.62, 0.42);
    let seed = rand_u32();

    // Random tape orientation: horizontal or vertical
    let tape_horizontal = rand::random::<bool>();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Cardboard texture: fine fibrous noise
            let fibre = fbm_2d_periodic(u * 16.0, v * 16.0, 3, 0.4, 2.5, seed, Some(16));
            let c_factor = 0.88 + fibre * 0.12;
            let mut c = base.scale(c_factor);

            // Tape strip across the middle
            let tape_coord = if tape_horizontal { v } else { u };
            let tape_dist = (tape_coord - 0.5).abs();
            if tape_dist < 0.06 {
                let tape_blend = 1.0 - (tape_dist / 0.06);
                c = c.lerp(tape, tape_blend * 0.7);
            }

            // Corrugation lines (subtle horizontal ridges)
            let ridge = ((v * size as f32 * 0.5).sin() * 0.5 + 0.5) * 0.05;
            c = c.scale(1.0 - ridge);

            // Flap fold lines near top/bottom edges
            let fold_v = fold_line(v, 0.18, 0.015);
            let fold_v2 = fold_line(v, 0.82, 0.015);
            c = c.scale(1.0 - (fold_v + fold_v2) * 0.25);

            c = c.scale(edge_vignette(u, v));
            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

/// Metal container with rivets and panel seams.
fn generate_metal_container() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    // Random metal tint: blue-grey, green-grey, or warm grey
    let tint = rand::random::<u32>() % 3;
    let base = match tint {
        0 => Rgb::new(0.52, 0.55, 0.62), // blue-grey
        1 => Rgb::new(0.50, 0.58, 0.52), // green-grey
        _ => Rgb::new(0.58, 0.55, 0.50), // warm grey
    };
    let seed = rand_u32();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Brushed metal: directional noise
            let brush = fbm_2d_periodic(u * 6.0, v * 30.0, 3, 0.5, 2.0, seed, Some(6));
            let scratches = fbm_2d_periodic(u * 20.0, v * 20.0, 2, 0.3, 2.0, seed + 7, Some(20));
            let c_factor = 0.82 + brush * 0.12 + scratches * 0.06;
            let mut c = base.scale(c_factor);

            // Panel seams: cross pattern
            let seam = plank_border(u, v, 0.02);
            c = c.scale(1.0 - seam * 0.35);

            // Rivets near the edges
            let rivet = rivet_pattern(u, v, size);
            c = c.scale(1.0 - rivet * 0.3);

            c = c.scale(edge_vignette(u, v));
            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

/// Colourful gift box with ribbon cross and bow dot.
fn generate_gift_box() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    // Random vibrant base colour
    let hue = rand_range(0.0, 6.0);
    let base = hue_to_rgb(hue, 0.65, 0.85);
    let ribbon = Rgb::new(0.95, 0.92, 0.55); // gold ribbon
    let seed = rand_u32();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Subtle pattern on wrapping paper: small polka dots or stars
            let pattern = fbm_2d_periodic(u * 12.0, v * 12.0, 2, 0.4, 2.0, seed, Some(12));
            let c_factor = 0.92 + pattern * 0.08;
            let mut c = base.scale(c_factor);

            // Ribbon cross
            let ribbon_h = (v - 0.5).abs() < 0.045;
            let ribbon_v = (u - 0.5).abs() < 0.045;
            if ribbon_h || ribbon_v {
                c = c.lerp(ribbon, 0.85);
            }

            // Bow at centre intersection
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

/// Rough stone block with cracks and surface variation.
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

            // Rocky surface: multi-octave noise
            let rock = fbm_2d_periodic(u * 8.0, v * 8.0, 5, 0.6, 2.0, seed, Some(8));
            let detail = fbm_2d_periodic(u * 16.0, v * 16.0, 3, 0.4, 2.0, seed + 3, Some(16));

            let c_factor = 0.70 + rock * 0.20 + detail * 0.10;
            let mut c = base.scale(c_factor);

            // Crack lines: sharp dark features
            let crack = crack_pattern(u, v, seed + 11);
            c = c.scale(1.0 - crack * 0.4);

            // Chiseled edge: border darkening
            let edge = border_band(u, v, 0.06);
            c = c.scale(1.0 - edge * 0.25);

            c = c.scale(edge_vignette(u, v));
            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

/// Brick block with mortar grid.
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

            // Brick grid with half-brick offset on alternating rows
            let row = (v * rows as f32).floor() as i32;
            let offset = if row % 2 == 0 { 0.0 } else { 0.5 / cols as f32 };
            let brick_u = ((u + offset) * cols as f32).fract();
            let brick_v = (v * rows as f32).fract();

            // Mortar lines
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
                // Per-brick colour variation using brick position as seed
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

/// Warning/hazard box with diagonal stripes.
fn generate_warning_box() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    // Random stripe colour pair
    let pair = rand::random::<u32>() % 3;
    let (colour_a, colour_b) = match pair {
        0 => (Rgb::new(0.90, 0.75, 0.15), Rgb::new(0.20, 0.20, 0.20)), // yellow/black
        1 => (Rgb::new(0.85, 0.25, 0.20), Rgb::new(0.90, 0.90, 0.85)), // red/white
        _ => (Rgb::new(0.25, 0.50, 0.80), Rgb::new(0.90, 0.90, 0.85)), // blue/white
    };
    let seed = rand_u32();
    let num_stripes = rand_range(4.0, 8.0).round();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Diagonal stripes
            let stripe = ((u + v) * num_stripes).fract();
            let blend = if stripe < 0.5 { 0.0 } else { 1.0 };
            let mut c = colour_a.lerp(colour_b, blend);

            // Surface wear
            let wear = fbm_2d_periodic(u * 10.0, v * 10.0, 3, 0.5, 2.0, seed, Some(10));
            c = c.scale(0.88 + wear * 0.12);

            // Border band
            let border = border_band(u, v, 0.05);
            c = c.scale(1.0 - border * 0.3);

            c = c.scale(edge_vignette(u, v));
            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

// ---------------------------------------------------------------------------
// Shared pattern helpers
// ---------------------------------------------------------------------------

/// Smooth border at the midpoint cross (plank divider).
fn plank_border(u: f32, v: f32, width: f32) -> f32 {
    let hx = (u - 0.5).abs();
    let hy = (v - 0.5).abs();
    let h = (1.0 - (hx / width).min(1.0)).powi(2);
    let vb = (1.0 - (hy / width).min(1.0)).powi(2);
    (h + vb).min(1.0)
}

/// L-shaped corner bracket intensity.
fn corner_bracket(u: f32, v: f32) -> f32 {
    let cu = if u < 0.5 { u } else { 1.0 - u };
    let cv = if v < 0.5 { v } else { 1.0 - v };

    let h_bar = cu < 0.22 && (cv - 0.06).abs() < 0.025;
    let v_bar = cv < 0.22 && (cu - 0.06).abs() < 0.025;

    if h_bar || v_bar {
        1.0
    } else {
        0.0
    }
}

/// Subtle vignette darkening at texture edges.
fn edge_vignette(u: f32, v: f32) -> f32 {
    let eu = (u - 0.5).abs() * 2.0;
    let ev = (v - 0.5).abs() * 2.0;
    let edge = ((eu.max(ev) - 0.85) / 0.15).clamp(0.0, 1.0);
    1.0 - edge * 0.25
}

/// Fold line at a given v-coordinate.
fn fold_line(v: f32, centre: f32, width: f32) -> f32 {
    let d = (v - centre).abs();
    (1.0 - (d / width).min(1.0)).powi(2)
}

/// Rivet dots near the edges of a panel.
fn rivet_pattern(u: f32, v: f32, size: u32) -> f32 {
    let margin = 0.08;
    let near_edge_u = u < margin || u > (1.0 - margin);
    let near_edge_v = v < margin || v > (1.0 - margin);
    if !near_edge_u && !near_edge_v {
        return 0.0;
    }

    // Rivet spacing
    let spacing = 0.12;
    let ru = ((u / spacing) + 0.5).fract() - 0.5;
    let rv = ((v / spacing) + 0.5).fract() - 0.5;
    let dist = (ru * ru + rv * rv).sqrt();
    let rivet_radius = 1.5 / size as f32;
    (1.0 - (dist / rivet_radius).min(1.0)).powi(2)
}

/// Sharp crack-like features from thresholded noise.
fn crack_pattern(u: f32, v: f32, seed: u32) -> f32 {
    let n = fbm_2d_periodic(u * 12.0, v * 12.0, 4, 0.7, 2.0, seed, Some(12));
    // Threshold to create thin dark lines
    let edge = ((n - 0.48).abs()).min(0.02) / 0.02;
    (1.0 - edge).powi(3)
}

/// Solid border band at a given inset distance.
fn border_band(u: f32, v: f32, width: f32) -> f32 {
    let eu = u.min(1.0 - u);
    let ev = v.min(1.0 - v);
    let d = eu.min(ev);
    (1.0 - (d / width).min(1.0)).powi(2)
}

/// Convert hue (0–6), saturation, value to RGB.
fn hue_to_rgb(h: f32, s: f32, v: f32) -> Rgb {
    let h = h % 6.0;
    let i = h.floor() as i32;
    let f = h - h.floor();
    let p = v * (1.0 - s);
    let q = v * (1.0 - f * s);
    let t = v * (1.0 - (1.0 - f) * s);
    match i % 6 {
        0 => Rgb::new(v, t, p),
        1 => Rgb::new(q, v, p),
        2 => Rgb::new(p, v, t),
        3 => Rgb::new(p, q, v),
        4 => Rgb::new(t, p, v),
        _ => Rgb::new(v, p, q),
    }
}

/// Simple integer hash for deterministic per-brick variation.
fn hash_pair(a: i32, b: i32) -> u32 {
    let mut n = (a as u32)
        .wrapping_mul(374761393)
        .wrapping_add((b as u32).wrapping_mul(668265263));
    n = (n ^ (n >> 13)).wrapping_mul(1274126177);
    n ^ (n >> 16)
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

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

/// Creates unique box materials with randomly chosen styles.
///
/// Call during initialisation while `MaterialManagerBuilder` is still mutable.
/// Each material gets a randomly chosen style and unique parameters.
pub fn create_box_materials(
    count: usize,
    texture_manager: &TextureManager,
    material_builder: &mut MaterialManagerBuilder,
) -> EngineResult<Vec<MaterialId>> {
    let mut ids = Vec::with_capacity(count);
    for _ in 0..count {
        ids.push(create_box_material_for_style(
            BoxStyle::Random,
            texture_manager,
            material_builder,
        )?);
    }
    Ok(ids)
}

/// Builds a box model with the given half-extents.
fn build_model(half_extents: Vector3<f32>, material: MaterialId) -> Arc<Model> {
    let parts = vec![ModelPart::new(vec![MeshPrimitive {
        vertices: generate_cube_vertices(half_extents, Colour::WHITE),
        indices: generate_cube_indices(),
        material,
    }])];

    Arc::new(Model::flat(parts))
}

/// Spawns a box entity with physics.
pub fn spawn_box(
    world: &mut World,
    initial_pos: nalgebra::Point3<f32>,
    half_extents: Vector3<f32>,
    material: MaterialId,
    phys: &BoxPhysics,
) -> Entity {
    let model = build_model(half_extents, material);

    let body_handle = {
        let mut physics = world.write_resource::<PhysicsResource>();

        let body_desc = RigidBodyDesc::dynamic()
            .position(initial_pos)
            .gravity_scale(1.0)
            .linear_damping(0.01)
            .angular_damping(0.005);

        let body_handle = physics.world.create_body(body_desc);

        let collider_desc = ColliderDesc::box_shape(half_extents)
            .density(phys.density)
            .restitution(phys.restitution)
            .friction(phys.friction);

        physics.world.attach_collider(body_handle, collider_desc);

        body_handle
    };

    world
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
        .with(Flammable::wood())
        .build()
}

/// Spawns a house structure built from boxes with varied proportions.
///
/// The house consists of:
/// - A foundation slab slightly wider than the walls
/// - A solid back wall (single tall box)
/// - A front wall with a centered door opening in the lower layer and a solid upper layer
/// - Two side walls each with a window opening (sill + lintel framing) in the lower layer
/// - A flat roof slab with overhang on all sides
///
/// Front/back walls span the full outer X (including corner thickness) so that side
/// walls can fill between them flush, with no overlapping geometry at the corners.
///
/// `base_pos` is ground level at the centre of the footprint.
/// `base_half_extents` (x, y, z) defines the interior half-extents: x/z set the
/// footprint, and y is used as the height reference unit.
pub fn spawn_house(
    world: &mut World,
    base_pos: Point3<f32>,
    base_half_extents: Vector3<f32>,
    materials: &[MaterialId],
) -> Vec<Entity> {
    let wx = base_half_extents.x;
    let hy = base_half_extents.y;
    let wz = base_half_extents.z;

    let wt = hy * 0.25; // wall half-thickness
    let fh = hy * 0.3; // foundation half-height
    let wh = hy; // wall layer half-height (two layers form the full wall)
    let rt = hy * 0.3; // roof half-thickness
    let ro = hy * 0.5; // roof overhang per side

    let door_half_w = wx * 0.4; // door half-width (centred on front wall)
    let win_half_z = wz * 0.35; // window half-width along Z (centred on side walls)
    let sill_h = wh * 0.3; // sill / lintel half-height

    let mut entities = Vec::new();
    let mat = |i: usize| materials[i % materials.len()];
    let phys = BoxPhysics::default();

    let y0 = base_pos.y;
    let foundation_cy = y0 + fh;
    let y_wall_base = y0 + 2.0 * fh;
    let lower_cy = y_wall_base + wh;
    let upper_cy = y_wall_base + 3.0 * wh;
    let walls_top = y_wall_base + 4.0 * wh;
    let roof_cy = walls_top + rt;

    // Foundation slab
    entities.push(spawn_box(
        world,
        Point3::new(base_pos.x, foundation_cy, base_pos.z),
        Vector3::new(wx + wt, fh, wz + wt),
        mat(0),
        &phys,
    ));

    // Back wall (−Z) — solid, single tall box spanning both layers
    entities.push(spawn_box(
        world,
        Point3::new(base_pos.x, y_wall_base + 2.0 * wh, base_pos.z - wz),
        Vector3::new(wx + wt, 2.0 * wh, wt),
        mat(1),
        &phys,
    ));

    // Front wall (+Z) — door opening in lower layer, solid upper layer
    //
    // The front wall spans X: [base_pos.x − (wx+wt), base_pos.x + (wx+wt)].
    // The door gap runs X: [base_pos.x − door_half_w, base_pos.x + door_half_w].
    // Each side piece therefore has half-extent = (wx + wt − door_half_w) / 2.
    let door_side_half = (wx + wt - door_half_w) / 2.0;
    let door_left_cx = base_pos.x - door_half_w - door_side_half;
    let door_right_cx = base_pos.x + door_half_w + door_side_half;

    entities.push(spawn_box(
        world,
        Point3::new(door_left_cx, lower_cy, base_pos.z + wz),
        Vector3::new(door_side_half, wh, wt),
        mat(2),
        &phys,
    ));
    entities.push(spawn_box(
        world,
        Point3::new(door_right_cx, lower_cy, base_pos.z + wz),
        Vector3::new(door_side_half, wh, wt),
        mat(2),
        &phys,
    ));
    entities.push(spawn_box(
        world,
        Point3::new(base_pos.x, upper_cy, base_pos.z + wz),
        Vector3::new(wx + wt, wh, wt),
        mat(3),
        &phys,
    ));

    // Side walls (±X) — window opening in lower layer with sill and lintel
    //
    // Side walls fill between the inner faces of the front and back walls, so their
    // Z half-extent is (wz − wt).  The window is centred in Z; the columns on each
    // side of the window have half-extent = (wz − wt − win_half_z) / 2.
    let win_col_half = (wz - wt - win_half_z) / 2.0;
    let win_col_neg_cz = base_pos.z - win_half_z - win_col_half;
    let win_col_pos_cz = base_pos.z + win_half_z + win_col_half;
    let sill_cy = y_wall_base + sill_h;
    let lintel_cy = y_wall_base + 2.0 * wh - sill_h;

    for (sign, mat_base) in [(-1.0_f32, 4_usize), (1.0_f32, 7_usize)] {
        let cx = base_pos.x + sign * wx;

        // Lower layer: columns to either side of the window opening
        entities.push(spawn_box(
            world,
            Point3::new(cx, lower_cy, win_col_neg_cz),
            Vector3::new(wt, wh, win_col_half),
            mat(mat_base),
            &phys,
        ));
        entities.push(spawn_box(
            world,
            Point3::new(cx, lower_cy, win_col_pos_cz),
            Vector3::new(wt, wh, win_col_half),
            mat(mat_base),
            &phys,
        ));

        // Window sill (bottom of opening)
        entities.push(spawn_box(
            world,
            Point3::new(cx, sill_cy, base_pos.z),
            Vector3::new(wt, sill_h, win_half_z),
            mat(mat_base + 1),
            &phys,
        ));

        // Window lintel (top of opening)
        entities.push(spawn_box(
            world,
            Point3::new(cx, lintel_cy, base_pos.z),
            Vector3::new(wt, sill_h, win_half_z),
            mat(mat_base + 1),
            &phys,
        ));

        // Upper layer: solid, spans full interior Z
        entities.push(spawn_box(
            world,
            Point3::new(cx, upper_cy, base_pos.z),
            Vector3::new(wt, wh, wz - wt),
            mat(mat_base + 2),
            &phys,
        ));
    }

    // Roof slab with overhang
    entities.push(spawn_box(
        world,
        Point3::new(base_pos.x, roof_cy, base_pos.z),
        Vector3::new(wx + ro, rt, wz + ro),
        mat(0),
        &phys,
    ));

    entities
}
