//! Patterns painted on a creature's parts by texture: the detail too fine
//! for the vertex colours of [`super::body_paint`] (freckles, pores, fine
//! veins).
//!
//! A part's texture coordinates are its sphere's (`u` round its own up
//! axis, `v` from top to bottom), so the pattern is baked by taking each
//! texel back to its point on the unit sphere and asking the fields there.
//! The pattern is then seamless round the part and does not crowd towards
//! its poles, and it travels with the part however the part moves.
//!
//! The texture multiplies the vertex colour: white leaves a part its
//! colour, and the pattern darkens or tints it.

use std::f32::consts::{PI, TAU};

use nalgebra::Vector3;
use rayon::prelude::*;

use crate::utils::noise::fbm_perlin_3d;

use super::body_paint::scatter;

/// A layer of round spots, scattered one or none to a cell of a 3D grid
/// over the unit sphere.
#[derive(Clone, Copy, Debug)]
pub struct Spots {
    /// Cells per unit of the sphere's radius.
    pub frequency: f32,
    /// Share of cells that hold a spot.
    pub presence: f32,
    /// Smallest and largest spot radius, as a share of a cell.
    pub radius: (f32, f32),
    /// The colour a spot multiplies the paint by at its heart (linear).
    pub tint: Vector3<f32>,
    /// Salt that keeps this layer's cells unlike any other layer's.
    pub salt: u32,
}

impl Spots {
    /// How much of a spot covers `point`: 1 at a spot's heart, 0 clear of
    /// every spot.
    fn cover(&self, point: Vector3<f32>) -> f32 {
        let at = point * self.frequency;
        let cell = at.map(f32::floor);
        let mut cover: f32 = 0.0;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let c = cell + Vector3::new(dx as f32, dy as f32, dz as f32);
                    let key = cell_key(c, self.salt);
                    if (scatter(key, 0) * 0.5 + 0.5) > self.presence {
                        continue;
                    }
                    let centre = c + Vector3::new(
                        scatter(key, 1) * 0.5 + 0.5,
                        scatter(key, 2) * 0.5 + 0.5,
                        scatter(key, 3) * 0.5 + 0.5,
                    );
                    let share = scatter(key, 4) * 0.5 + 0.5;
                    let radius = self.radius.0 + (self.radius.1 - self.radius.0) * share;
                    let t = ((at - centre).norm() / radius).min(1.0);
                    cover = cover.max(1.0 - t * t * (3.0 - 2.0 * t));
                }
            }
        }
        cover
    }
}

/// Thin wandering lines where a field of noise peaks: veins under a skin.
#[derive(Clone, Copy, Debug)]
pub struct Veins {
    /// Wanderings per unit of the sphere's radius.
    pub frequency: f32,
    /// How thin a vein is: the share of the field's range below its peak
    /// that still counts as vein. Smaller is thinner.
    pub width: f32,
    /// The colour a vein multiplies the paint by (linear).
    pub tint: Vector3<f32>,
    /// Seed of the field.
    pub seed: u32,
}

impl Veins {
    /// How much of a vein covers `point`, 0 to 1.
    fn cover(&self, point: Vector3<f32>) -> f32 {
        let at = point * self.frequency;
        let field = fbm_perlin_3d(at.x, at.y, at.z, 3, 0.5, self.seed);
        let ridge = 1.0 - field.abs();
        smoothstep(1.0 - self.width, 1.0, ridge)
    }
}

/// Soft clouds of lighter and darker paint, finer than the vertex mottling.
#[derive(Clone, Copy, Debug)]
pub struct Clouds {
    /// Clouds per unit of the sphere's radius.
    pub frequency: f32,
    /// How far a cloud lightens or darkens, as a share.
    pub value: f32,
    /// Seed of the clouds.
    pub seed: u32,
}

/// The pattern a part's skin is painted with: clouds, then veins, then two
/// layers of spots over them.
#[derive(Clone, Copy, Debug)]
pub struct SkinPattern {
    /// Texels round the sphere; half as many run from pole to pole, which
    /// keeps a texel square at the equator.
    pub width: u32,
    pub clouds: Clouds,
    pub veins: Veins,
    /// Large spots: freckles.
    pub freckles: Spots,
    /// Small spots: pores.
    pub pores: Spots,
}

impl SkinPattern {
    /// Texels from pole to pole.
    pub fn height(&self) -> u32 {
        self.width / 2
    }

    /// The pattern's multiplier on the paint at `point` on the unit sphere
    /// (linear).
    pub fn multiplier(&self, point: Vector3<f32>) -> Vector3<f32> {
        let at = point * self.clouds.frequency;
        let cloud =
            1.0 + self.clouds.value * fbm_perlin_3d(at.x, at.y, at.z, 3, 0.5, self.clouds.seed);
        let mut paint = Vector3::repeat(cloud);
        for (cover, tint) in [
            (self.veins.cover(point), self.veins.tint),
            (self.freckles.cover(point), self.freckles.tint),
            (self.pores.cover(point), self.pores.tint),
        ] {
            paint = paint.component_mul(&Vector3::repeat(1.0).lerp(&tint, cover));
        }
        paint
    }

    /// The texture: sRGB RGBA bytes, `width` by [`Self::height`], row by
    /// row from the sphere's top.
    pub fn texture(&self) -> Vec<u8> {
        let (width, height) = (self.width, self.height());
        (0..height)
            .into_par_iter()
            .flat_map_iter(|row| {
                let theta = (row as f32 + 0.5) / height as f32 * PI;
                (0..width).flat_map(move |column| {
                    let phi = (column as f32 + 0.5) / width as f32 * TAU;
                    // The sphere generator's own parameterisation.
                    let point = Vector3::new(
                        theta.sin() * phi.cos(),
                        theta.cos(),
                        theta.sin() * phi.sin(),
                    );
                    let paint = self.multiplier(point);
                    [
                        srgb_byte(paint.x),
                        srgb_byte(paint.y),
                        srgb_byte(paint.z),
                        255,
                    ]
                })
            })
            .collect()
    }
}

/// A key for the grid cell whose corner is `cell`, unlike any other cell's
/// or any other layer's.
fn cell_key(cell: Vector3<f32>, salt: u32) -> u32 {
    let [x, y, z] = [cell.x, cell.y, cell.z].map(|c| c as i32 as u32);
    x.wrapping_mul(73_856_093) ^ y.wrapping_mul(19_349_663) ^ z.wrapping_mul(83_492_791) ^ salt
}

fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// A linear value as an sRGB-encoded byte.
fn srgb_byte(linear: f32) -> u8 {
    let l = linear.clamp(0.0, 1.0);
    let encoded = if l <= 0.003_130_8 {
        l * 12.92
    } else {
        1.055 * l.powf(1.0 / 2.4) - 0.055
    };
    (encoded * 255.0).round() as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    const PATTERN: SkinPattern = SkinPattern {
        width: 64,
        clouds: Clouds {
            frequency: 3.0,
            value: 0.0,
            seed: 1,
        },
        veins: Veins {
            frequency: 3.0,
            width: 0.0,
            tint: Vector3::new(1.0, 1.0, 1.0),
            seed: 2,
        },
        freckles: Spots {
            frequency: 6.0,
            presence: 0.5,
            radius: (0.2, 0.4),
            tint: Vector3::new(0.5, 0.4, 0.3),
            salt: 3,
        },
        pores: Spots {
            frequency: 20.0,
            presence: 0.0,
            radius: (0.1, 0.2),
            tint: Vector3::new(0.5, 0.5, 0.5),
            salt: 4,
        },
    };

    #[test]
    fn the_texture_is_seamless_round_the_sphere() {
        let texture = PATTERN.texture();
        let width = PATTERN.width as usize;
        let row_of = |r: usize| &texture[r * width * 4..(r + 1) * width * 4];
        // The first and last columns are half a texel either side of the
        // seam, so they must agree about as well as any two neighbours.
        let gap = |a: usize, b: usize| {
            (0..PATTERN.height() as usize)
                .map(|r| {
                    let row = row_of(r);
                    (row[a * 4] as i32 - row[b * 4] as i32).abs()
                })
                .max()
                .unwrap()
        };
        assert!(gap(0, width - 1) <= gap(0, 1).max(gap(1, 2)) + 8);
    }

    #[test]
    fn freckles_darken_and_leave_the_rest_white() {
        let texture = PATTERN.texture();
        let reds: Vec<u8> = texture.chunks_exact(4).map(|t| t[0]).collect();
        assert_eq!(*reds.iter().max().unwrap(), 255, "clear skin stays white");
        assert!(*reds.iter().min().unwrap() < 200, "some freckle shows");
    }
}
