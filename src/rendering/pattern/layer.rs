//! Procedural surface patterns, built by stacking a few operations.
//!
//! Every stone spawnable in the game wrote its own texture generator, and they
//! were the same generator. Menhir, trilithon, temple, dolos and house all
//! layer a broad blotch, a mid undulation, a thresholded patch and a fine
//! speckle over a base colour — in that order, in eighty lines apiece, with
//! different constants and hand-managed seed offsets.
//!
//! ```text
//!   Palette ──┐
//!             ├──▶ Pattern (a stack of Layers) ──▶ RGBA8 tile
//!   seed ─────┘
//! ```
//!
//! Layers name their colours by *slot* rather than by value, so a pattern is
//! independent of what it is drawn in: one stone pattern over three palettes is
//! granite, limestone and sandstone. That is the reuse the duplicated
//! generators could never have — each had its greys baked in.
//!
//! Seeds are derived from the layer's index rather than authored, because
//! `seed.wrapping_add(17)` scattered through a generator is a decision nobody
//! makes deliberately and everybody has to preserve.

use crate::rendering::colour::Colour;
use crate::rendering::substance::Palette;
use crate::utils::noise::fbm_2d_periodic;

/// Which of a palette's colours a layer works towards.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Slot {
    Base,
    Light,
    Dark,
    Accent,
}

impl Slot {
    fn of(self, palette: &Palette) -> Colour {
        match self {
            Self::Base => palette.base,
            Self::Light => palette.light,
            Self::Dark => palette.dark,
            Self::Accent => palette.accent,
        }
    }
}

/// One operation in a pattern.
///
/// The set is deliberately small: it is what the generators this replaced
/// actually did, not what a texture language could do. A surface that needs
/// something genuinely different — a banana's ripeness gradient, a beach ball's
/// panels — should stay a bespoke function rather than growing a variant here
/// that only it uses.
#[derive(Clone, Copy, Debug)]
pub enum Layer {
    /// Broad colour variation: blend towards a slot by remapped noise.
    ///
    /// The large-scale mottling that decides what a surface reads as from
    /// across the room.
    Wash {
        /// Noise frequency in features per tile. Also its tiling period.
        scale: f32,
        octaves: u32,
        towards: Slot,
        /// Maximum fraction blended, at the noise's extreme.
        amount: f32,
    },

    /// Multiply brightness by noise, leaving hue alone.
    ///
    /// What gives a surface its bumpy, weathered look without shifting its
    /// colour. Cheap, and the layer most patterns end on.
    Shade {
        scale: f32,
        octaves: u32,
        /// Peak fractional darkening and brightening.
        amount: f32,
    },

    /// Blend towards a slot only where noise exceeds a threshold.
    ///
    /// Discrete features rather than continuous variation: lichen, damp
    /// staining, mineral deposits, quartz. The threshold is what makes them
    /// read as *patches* rather than as more mottling.
    Patch {
        scale: f32,
        octaves: u32,
        /// Noise value the patch starts at, in the field's own 0..1 range. The
        /// field rarely leaves 0.17..0.81, so a threshold outside that band
        /// means the layer never fires or always does.
        threshold: f32,
        /// Noise range over which it reaches full strength.
        span: f32,
        /// Maximum fraction blended.
        amount: f32,
        towards: Slot,
    },

    /// Blend towards a slot along the ridges of a noise field.
    ///
    /// Thin, branching lines — veins in marble, cracks in weathered rock. The
    /// ridge is `1 - |n|`, which is why this reads as a line rather than a
    /// blob: it picks out where the field crosses zero.
    Vein {
        scale: f32,
        octaves: u32,
        /// How tightly the ridge is confined around the field's midpoint.
        /// Higher is thinner; the ridge spans `1 / sharpness` either side.
        sharpness: f32,
        amount: f32,
        towards: Slot,
    },
}

impl Layer {
    /// Apply this layer to a colour at `(u, v)`.
    fn apply(&self, colour: Colour, u: f32, v: f32, palette: &Palette, seed: u32) -> Colour {
        match *self {
            Self::Wash {
                scale,
                octaves,
                towards,
                amount,
            } => {
                let n = noise(u, v, scale, octaves, seed);
                lerp(colour, towards.of(palette), unit(n) * amount)
            }

            Self::Shade {
                scale,
                octaves,
                amount,
            } => {
                let n = noise(u, v, scale, octaves, seed);
                scale_colour(colour, 1.0 - amount + unit(n) * amount * 2.0)
            }

            Self::Patch {
                scale,
                octaves,
                threshold,
                span,
                amount,
                towards,
            } => {
                let n = noise(u, v, scale, octaves, seed);
                if n <= threshold {
                    return colour;
                }
                let t = ((n - threshold) / span).clamp(0.0, 1.0) * amount;
                lerp(colour, towards.of(palette), t)
            }

            Self::Vein {
                scale,
                octaves,
                sharpness,
                amount,
                towards,
            } => {
                let n = noise(u, v, scale, octaves, seed);
                let ridge = (1.0 - (signed(n) * sharpness).abs()).max(0.0);
                lerp(colour, towards.of(palette), ridge * amount)
            }
        }
    }
}

/// A stack of layers, applied in order over a palette's base colour.
#[derive(Clone, Copy, Debug)]
pub struct Pattern {
    /// Identifies this pattern in a cache key. Two patterns must not share a
    /// name, or the cache will hand one out where the other was asked for.
    pub name: &'static str,

    /// Applied in order, each over the result of the last.
    pub layers: &'static [Layer],
}

impl Pattern {
    /// The colour this pattern produces at `(u, v)`.
    pub fn sample(&self, u: f32, v: f32, palette: &Palette, seed: u32) -> Colour {
        let mut colour = palette.base;
        for (index, layer) in self.layers.iter().enumerate() {
            // Derived from the layer's position rather than authored. Two
            // layers at the same frequency must not draw the same field, or
            // their features line up and the surface acquires a structure
            // nobody asked for.
            colour = layer.apply(
                colour,
                u,
                v,
                palette,
                seed.wrapping_add(index as u32 * 7919),
            );
        }
        colour
    }

    /// Bake this pattern into an RGBA8 tile.
    ///
    /// Seamless: every layer's noise is periodic at its own scale, so the tile
    /// wraps whatever it is built from.
    pub fn bake(&self, size: u32, palette: &Palette, seed: u32) -> Vec<u8> {
        let mut pixels = Vec::with_capacity((size * size * 4) as usize);

        for y in 0..size {
            for x in 0..size {
                let u = x as f32 / size as f32;
                let v = y as f32 / size as f32;
                let colour = self.sample(u, v, palette, seed);

                pixels.push(byte(colour.r));
                pixels.push(byte(colour.g));
                pixels.push(byte(colour.b));
                pixels.push(255);
            }
        }

        pixels
    }
}

/// Periodic fractal noise at a layer's scale.
///
/// **The range is 0..1**, centred near 0.5 and in practice spanning about
/// 0.17..0.81 — not the -1..1 that the name `fbm` suggests and that several of
/// the generators this library replaced assumed. That assumption was why the
/// veins in the old menhir and temple stone never appeared: their ridge term
/// computed `1 - |n * 4|` against a value near 0.5, which is negative
/// everywhere and clamps to nothing. Layers here take the range as it is, and
/// `signed` is what converts when a layer genuinely wants a deviation.
fn noise(u: f32, v: f32, scale: f32, octaves: u32, seed: u32) -> f32 {
    fbm_2d_periodic(
        u * scale,
        v * scale,
        octaves,
        0.5,
        2.0,
        seed,
        Some(scale.max(1.0) as i32),
    )
}

/// Noise as a 0..1 weight.
fn unit(n: f32) -> f32 {
    n.clamp(0.0, 1.0)
}

/// Noise as a deviation from its midpoint, in -1..1.
///
/// What a ridge is measured against: a vein is where the field *crosses* its
/// middle, and that crossing is invisible in the unsigned value.
fn signed(n: f32) -> f32 {
    (n - 0.5) * 2.0
}

fn lerp(a: Colour, b: Colour, t: f32) -> Colour {
    let t = t.clamp(0.0, 1.0);
    Colour::new(
        a.r + (b.r - a.r) * t,
        a.g + (b.g - a.g) * t,
        a.b + (b.b - a.b) * t,
        a.a,
    )
}

fn scale_colour(colour: Colour, factor: f32) -> Colour {
    Colour::new(
        colour.r * factor,
        colour.g * factor,
        colour.b * factor,
        colour.a,
    )
}

fn byte(value: f32) -> u8 {
    (value.clamp(0.0, 1.0) * 255.0) as u8
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rendering::pattern::library;

    fn palette() -> Palette {
        Palette::from_base(Colour::new(0.6, 0.58, 0.55, 1.0), 0.2)
    }

    /// The reason layers name colours by slot. One pattern over two palettes
    /// must give two different-looking materials, or the library has bought
    /// nothing over the generators it replaced.
    #[test]
    fn one_pattern_over_two_palettes_gives_two_materials() {
        let grey = Palette::from_base(Colour::new(0.6, 0.6, 0.6, 1.0), 0.2);
        let warm = Palette::from_base(Colour::new(0.74, 0.62, 0.45, 1.0), 0.2);

        let a = library::STONE.sample(0.3, 0.7, &grey, 1);
        let b = library::STONE.sample(0.3, 0.7, &warm, 1);

        assert!(
            (a.r - b.r).abs() + (a.g - b.g).abs() + (a.b - b.b).abs() > 0.1,
            "the palette barely changed the result: {a:?} vs {b:?}"
        );
    }

    /// A pattern is a texture, not a flat fill: it has to actually vary.
    #[test]
    fn a_pattern_varies_across_its_tile() {
        let palette = palette();
        let mut min = 1.0f32;
        let mut max = 0.0f32;

        for i in 0..64 {
            let u = (i % 8) as f32 / 8.0;
            let v = (i / 8) as f32 / 8.0;
            let c = library::STONE.sample(u, v, &palette, 3);
            min = min.min(c.r);
            max = max.max(c.r);
        }

        assert!(max - min > 0.08, "stone varies by only {}", max - min);
    }

    /// Seeds separate instances. Two menhirs in a level should not be the same
    /// rock twice, which is the whole reason the generators took a seed.
    #[test]
    fn a_different_seed_gives_a_different_surface() {
        let palette = palette();
        let mut difference = 0.0;

        for step in 0..64 {
            let u = (step % 8) as f32 / 8.0;
            let v = (step / 8) as f32 / 8.0;
            difference += (library::STONE.sample(u, v, &palette, 1).r
                - library::STONE.sample(u, v, &palette, 2).r)
                .abs();
        }

        assert!(
            difference / 64.0 > 0.01,
            "two seeds gave the same rock (mean difference {})",
            difference / 64.0
        );
    }

    /// The range the layers are written against, pinned because getting it
    /// wrong is silent: a threshold outside it makes a layer fire always or
    /// never, and a ridge measured against the wrong centre disappears
    /// entirely. That is exactly what happened to the veins in the generators
    /// this library replaced.
    #[test]
    fn the_noise_field_runs_from_zero_to_one_around_a_half() {
        let mut min = f32::MAX;
        let mut max = f32::MIN;
        let mut total = 0.0;
        let mut samples = 0;

        for step in 0..400 {
            let u = (step % 20) as f32 / 20.0;
            let v = (step / 20) as f32 / 20.0;
            let n = noise(u, v, 6.0, 3, 11);
            min = min.min(n);
            max = max.max(n);
            total += n;
            samples += 1;
        }

        assert!(min >= 0.0 && max <= 1.0, "field runs {min}..{max}");
        let mean = total / samples as f32;
        assert!(
            (0.4..0.6).contains(&mean),
            "field is centred on {mean}, not on a half"
        );
    }

    /// Layers at the same frequency must not draw the same field, or their
    /// features line up into a structure nobody authored.
    #[test]
    fn two_layers_at_one_frequency_do_not_align() {
        let palette = palette();
        const TWICE: Pattern = Pattern {
            name: "test/twice",
            layers: &[
                Layer::Shade {
                    scale: 8.0,
                    octaves: 2,
                    amount: 0.3,
                },
                Layer::Shade {
                    scale: 8.0,
                    octaves: 2,
                    amount: 0.3,
                },
            ],
        };
        const ONCE: Pattern = Pattern {
            name: "test/once",
            layers: &[Layer::Shade {
                scale: 8.0,
                octaves: 2,
                amount: 0.3,
            }],
        };

        // If both layers drew the same field, the second would square the
        // first and the result would be exactly the once-shaded value squared.
        let twice = TWICE.sample(0.3, 0.6, &palette, 5).r;
        let once = ONCE.sample(0.3, 0.6, &palette, 5).r;
        let squared_ratio = (once / palette.base.r).powi(2) * palette.base.r;

        assert!(
            (twice - squared_ratio).abs() > 1e-4,
            "the two layers drew the same noise field"
        );
    }

    /// The tile wraps, so opposite edges have to meet. A seam here shows as a
    /// visible line down every repeat of every stone in the game.
    #[test]
    fn a_baked_tile_is_seamless() {
        let palette = palette();

        for v in 0..16 {
            let v = v as f32 / 16.0;
            let left = library::STONE.sample(0.0, v, &palette, 9);
            let right = library::STONE.sample(1.0, v, &palette, 9);

            assert!(
                (left.r - right.r).abs() < 0.02,
                "seam at v={v}: {} vs {}",
                left.r,
                right.r
            );
        }
    }

    #[test]
    fn a_baked_tile_has_the_size_it_says() {
        let pixels = library::STONE.bake(16, &palette(), 1);
        assert_eq!(pixels.len(), 16 * 16 * 4);
        assert!(pixels.chunks(4).all(|p| p[3] == 255));
    }
}
