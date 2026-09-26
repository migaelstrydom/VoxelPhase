//! The barrel's two textures: staves bound by iron hoops for the side, and
//! boards across a chime for the heads.

use super::super::shared::textures::{Rgb, TextureRng};
use crate::utils::noise::fbm_2d_periodic;

/// Side texture width: once round the barrel, which is about twice its height.
pub const SIDE_WIDTH: u32 = 512;
/// Side texture height: from head to head.
pub const SIDE_HEIGHT: u32 = 256;
/// Head texture size, square.
pub const HEAD_SIZE: u32 = 256;

/// Staves round the barrel. The contact hull has one flat face per stave.
pub const STAVES: usize = 16;

/// Hoop centres, as a fraction of the height from the top head: a pair at
/// each chime and one each side of the belly.
const HOOPS: [f32; 4] = [0.07, 0.25, 0.75, 0.93];
/// Half a hoop's width, in the same fraction. About 2 cm on a 0.9 m barrel.
const HOOP_HALF_WIDTH: f32 = 0.024;
/// Half a stave joint's width, as a fraction of a stave's width.
const JOINT_HALF_WIDTH: f32 = 0.035;

const OAK_LIGHT: Rgb = Rgb {
    r: 0.74,
    g: 0.54,
    b: 0.32,
};
const OAK_DARK: Rgb = Rgb {
    r: 0.50,
    g: 0.34,
    b: 0.18,
};
const JOINT: Rgb = Rgb {
    r: 0.16,
    g: 0.10,
    b: 0.06,
};
const IRON: Rgb = Rgb {
    r: 0.20,
    g: 0.19,
    b: 0.18,
};
const IRON_EDGE: Rgb = Rgb {
    r: 0.34,
    g: 0.32,
    b: 0.30,
};
const RUST: Rgb = Rgb {
    r: 0.36,
    g: 0.20,
    b: 0.10,
};

/// Side texture: `STAVES` vertical staves, each its own shade, with the
/// fibre running head to head, dark joints between them and iron hoops
/// round them.
pub fn side_texture(seed: u32) -> Vec<u8> {
    let rng = &mut TextureRng::new(seed);
    let shades: Vec<f32> = (0..STAVES).map(|_| rng.range(-0.08, 0.08)).collect();
    let seed_fibre = rng.u32();
    let seed_figure = rng.u32();
    let seed_rust = rng.u32();

    let mut pixels = Vec::with_capacity((SIDE_WIDTH * SIDE_HEIGHT * 4) as usize);
    for y in 0..SIDE_HEIGHT {
        for x in 0..SIDE_WIDTH {
            let u = x as f32 / SIDE_WIDTH as f32;
            let v = y as f32 / SIDE_HEIGHT as f32;

            let across = u * STAVES as f32;
            let stave = (across as usize).min(STAVES - 1);
            let within = across.fract();

            let mut colour = stave_wood(u, v, shades[stave], seed_fibre, seed_figure);

            let joint = (within.min(1.0 - within) / JOINT_HALF_WIDTH).min(1.0);
            colour = JOINT.lerp(colour, joint.powf(0.6));

            if let Some(offset) = hoop_offset(v) {
                colour = hoop_iron(u, v, offset, within, seed_rust);
            } else if let Some(shadow) = hoop_shadow(v) {
                colour = colour.scale(1.0 - 0.3 * shadow);
            }

            colour.write_rgba(&mut pixels);
        }
    }
    pixels
}

/// Head texture: boards across the head with the fibre along them, and a
/// dark rim where the head sits in the staves' groove.
pub fn head_texture(seed: u32) -> Vec<u8> {
    const BOARDS: f32 = 5.0;

    let rng = &mut TextureRng::new(seed ^ 0x6865_6164);
    let shades: Vec<f32> = (0..BOARDS as usize)
        .map(|_| rng.range(-0.07, 0.07))
        .collect();
    let seed_fibre = rng.u32();
    let seed_figure = rng.u32();

    let mut pixels = Vec::with_capacity((HEAD_SIZE * HEAD_SIZE * 4) as usize);
    for y in 0..HEAD_SIZE {
        for x in 0..HEAD_SIZE {
            let u = x as f32 / HEAD_SIZE as f32;
            let v = y as f32 / HEAD_SIZE as f32;

            let across = v * BOARDS;
            let board = (across as usize).min(shades.len() - 1);
            let within = across.fract();

            // The boards' fibre runs along u, so the stave sampler is asked
            // with the axes swapped.
            let mut colour = stave_wood(v, u, shades[board], seed_fibre, seed_figure);

            let joint = (within.min(1.0 - within) / 0.03).min(1.0);
            colour = JOINT.lerp(colour, joint.powf(0.6));

            let radius = ((u - 0.5).powi(2) + (v - 0.5).powi(2)).sqrt() * 2.0;
            let rim = ((radius - 0.86) / 0.1).clamp(0.0, 1.0);
            colour = colour.lerp(OAK_DARK.scale(0.7), rim * 0.8);

            colour.write_rgba(&mut pixels);
        }
    }
    pixels
}

/// Oak at `(u, v)` with its fibre along `v`, lightened or darkened by
/// `shade` so neighbouring staves are told apart.
fn stave_wood(u: f32, v: f32, shade: f32, seed_fibre: u32, seed_figure: u32) -> Rgb {
    let fibre = fbm_2d_periodic(u * 96.0, v * 4.0, 3, 0.5, 2.0, seed_fibre, Some(96));
    let figure = fbm_2d_periodic(u * 8.0, v * 2.0, 2, 0.5, 2.0, seed_figure, Some(8));

    let t = (0.5 + 0.35 * figure + 0.25 * fibre).clamp(0.0, 1.0);
    OAK_DARK
        .lerp(OAK_LIGHT, t)
        .scale(1.0 + shade)
        .scale(0.9 + 0.1 * fibre)
}

/// Where `v` falls across a hoop, from -1 at its top edge to 1 at its
/// bottom, or `None` off every hoop.
fn hoop_offset(v: f32) -> Option<f32> {
    HOOPS
        .iter()
        .map(|centre| (v - centre) / HOOP_HALF_WIDTH)
        .find(|offset| offset.abs() <= 1.0)
}

/// How deep in a hoop's shadow `v` lies, just below its bottom edge, or
/// `None` clear of every hoop.
fn hoop_shadow(v: f32) -> Option<f32> {
    HOOPS
        .iter()
        .map(|centre| (v - centre - HOOP_HALF_WIDTH) / (0.5 * HOOP_HALF_WIDTH))
        .find(|below| (0.0..1.0).contains(below))
        .map(|below| 1.0 - below)
}

/// Hoop iron at `offset` across the hoop, with a rivet where each stave's
/// middle crosses it and a scatter of rust.
fn hoop_iron(u: f32, v: f32, offset: f32, within_stave: f32, seed_rust: u32) -> Rgb {
    let edge = offset.abs().powi(6);
    let mut colour = IRON.lerp(IRON_EDGE, edge);

    let rust = fbm_2d_periodic(u * 24.0, v * 24.0, 3, 0.6, 2.0, seed_rust, Some(24));
    colour = colour.lerp(RUST, ((rust - 0.1) / 0.5).clamp(0.0, 0.6));

    let rivet_u = (within_stave - 0.5) * (1.0 / STAVES as f32) * SIDE_WIDTH as f32;
    let rivet_v = offset * HOOP_HALF_WIDTH * SIDE_HEIGHT as f32;
    let rivet = (rivet_u * rivet_u + rivet_v * rivet_v).sqrt();
    if rivet < 1.8 {
        colour = colour.lerp(IRON_EDGE.scale(1.3), 1.0 - rivet / 1.8);
    }
    colour
}
