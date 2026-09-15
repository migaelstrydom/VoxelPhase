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
use crate::utils::noise::{fbm_2d_periodic, fbm_perlin_2d_periodic_xy};

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

    /// Veins drawn in a field that is stretched along the tile's `v` axis.
    ///
    /// The same ridge as [`Vein`], over noise sampled on an elongated lattice.
    /// That one change is the difference between cracks that wander in every
    /// direction and cracks that run — the fracture planes in a block of ice,
    /// the drag marks in brushed metal. Isotropic noise cannot express it at
    /// any frequency, which is why this is a layer rather than a `Vein` at
    /// different settings.
    ///
    /// The direction is the tile's, not the world's: a surface that wants its
    /// streaks running some other way orients its texture coordinates, exactly
    /// as wood grain does (see [`GrainSpec`](crate::rendering::grain::GrainSpec)).
    Streak {
        /// Noise frequency *across* the streaks, in features per tile. The
        /// one that sets how densely packed the lines are.
        scale: f32,
        octaves: u32,
        /// How much longer a feature is along the streak than across it.
        /// One is a plain [`Vein`]; six or more is what reads as a fracture
        /// running through something.
        elongation: f32,
        /// As [`Vein`]'s: higher is a thinner line.
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
                lerp(colour, towards.of(palette), ridge(n, sharpness) * amount)
            }

            Self::Streak {
                scale,
                octaves,
                elongation,
                sharpness,
                amount,
                towards,
            } => {
                let n = streaked_noise(u, v, scale, elongation, octaves, seed);
                lerp(colour, towards.of(palette), ridge(n, sharpness) * amount)
            }
        }
    }

    /// The same layer with its noise frequency multiplied.
    ///
    /// What a [`Spread`] is made of: every layer drawn `n` times finer over a
    /// tile `n` times larger leaves the features the size they were. Only the
    /// frequency moves — a threshold, a sharpness or an elongation describes
    /// the *shape* of a feature and has to survive the change untouched.
    fn scaled(self, factor: f32) -> Self {
        match self {
            Self::Wash {
                scale,
                octaves,
                towards,
                amount,
            } => Self::Wash {
                scale: scale * factor,
                octaves,
                towards,
                amount,
            },
            Self::Shade {
                scale,
                octaves,
                amount,
            } => Self::Shade {
                scale: scale * factor,
                octaves,
                amount,
            },
            Self::Patch {
                scale,
                octaves,
                threshold,
                span,
                amount,
                towards,
            } => Self::Patch {
                scale: scale * factor,
                octaves,
                threshold,
                span,
                amount,
                towards,
            },
            Self::Vein {
                scale,
                octaves,
                sharpness,
                amount,
                towards,
            } => Self::Vein {
                scale: scale * factor,
                octaves,
                sharpness,
                amount,
                towards,
            },
            Self::Streak {
                scale,
                octaves,
                elongation,
                sharpness,
                amount,
                towards,
            } => Self::Streak {
                scale: scale * factor,
                octaves,
                elongation,
                sharpness,
                amount,
                towards,
            },
        }
    }
}

/// How many tile-widths of a pattern one baked texture holds.
///
/// Whatever maps a pattern onto a surface decides how many metres one tile
/// covers, and anything wider than that repeats it. On a prop the size of a
/// tile nobody sees it; on a five-metre slab it is a grid of identical
/// blotches, and the eye finds the grid long before it finds the blotch.
///
/// A spread is the answer that does not involve making the features bigger:
/// bake `n × n` tiles' worth of the pattern into one texture, so a surface
/// gets `n` times as far before anything comes round again while every feature
/// keeps the size it had. It is paid for in texels — a spread of `n` is `n²`
/// times the memory and the bake — which is why it is an explicit request from
/// an object that knows how large it is, and why it is capped.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Spread(u32);

impl Spread {
    /// One tile: what every surface smaller than a tile wants, and the
    /// behaviour of every pattern before spreads existed.
    pub const ONE: Self = Self(1);

    /// The widest spread anything may ask for.
    ///
    /// A 256² tile at this spread is a megapixel, which is the point where the
    /// cost stops being free and the returns have mostly been had — a surface
    /// still wide enough to repeat at four tiles is wide enough that it is the
    /// feature size, not the period, that the eye is reading.
    pub const MAX: u32 = 4;

    /// The spread that covers `tiles` tile-widths of surface, as far as the
    /// cap allows. Fractions round up: covering most of a tile still needs the
    /// whole of it.
    pub fn covering(tiles: f32) -> Self {
        if !tiles.is_finite() || tiles <= 1.0 {
            return Self::ONE;
        }
        Self((tiles.ceil() as u32).min(Self::MAX))
    }

    /// How many tiles across this spread is.
    pub fn tiles(self) -> u32 {
        self.0
    }

    /// The texture a tile of `tile_size` bakes to at this spread.
    pub fn texture_size(self, tile_size: u32) -> u32 {
        tile_size * self.0
    }
}

impl Default for Spread {
    fn default() -> Self {
        Self::ONE
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
        self.sample_spread(u, v, palette, seed, Spread::ONE)
    }

    /// The colour at `(u, v)` of a texture holding `spread` tiles of the
    /// pattern, where `(u, v)` runs 0..1 across the whole of it.
    pub fn sample_spread(
        &self,
        u: f32,
        v: f32,
        palette: &Palette,
        seed: u32,
        spread: Spread,
    ) -> Colour {
        let factor = spread.tiles() as f32;
        let mut colour = palette.base;
        for (index, layer) in self.layers.iter().enumerate() {
            // Derived from the layer's position rather than authored. Two
            // layers at the same frequency must not draw the same field, or
            // their features line up and the surface acquires a structure
            // nobody asked for.
            colour = layer.scaled(factor).apply(
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
        self.bake_spread(size, palette, seed, Spread::ONE)
    }

    /// Bake `spread` tiles' worth of this pattern into one RGBA8 texture.
    ///
    /// `tile_size` is the resolution of a single tile, so the result is
    /// `spread.texture_size(tile_size)` on a side and every feature comes out
    /// at the same number of texels it would have at [`Spread::ONE`]. Seamless
    /// for the same reason `bake` is: multiplying a layer's frequency
    /// multiplies its period with it.
    pub fn bake_spread(
        &self,
        tile_size: u32,
        palette: &Palette,
        seed: u32,
        spread: Spread,
    ) -> Vec<u8> {
        let size = spread.texture_size(tile_size);
        let bands = bake_bands(size);
        let rows_per_band = size.div_ceil(bands);

        // Every texel is an independent evaluation of the same pure function,
        // so the only thing to decide is who evaluates which. Bands of rows,
        // one thread each, concatenated in order — the result is identical to
        // the serial bake, byte for byte, which is what lets the texture cache
        // go on assuming a bake depends only on its key.
        let mut strips: Vec<Vec<u8>> = std::thread::scope(|scope| {
            let workers: Vec<_> = (0..bands)
                .map(|band| {
                    let rows = (band * rows_per_band)..((band + 1) * rows_per_band).min(size);
                    scope.spawn(move || self.bake_rows(rows, size, palette, seed, spread))
                })
                .collect();

            workers
                .into_iter()
                .map(|worker| worker.join().expect("pattern bake panicked"))
                .collect()
        });

        let mut pixels = Vec::with_capacity((size * size * 4) as usize);
        for strip in &mut strips {
            pixels.append(strip);
        }
        pixels
    }

    /// The RGBA8 rows of a bake in `rows`, as one strip.
    fn bake_rows(
        &self,
        rows: std::ops::Range<u32>,
        size: u32,
        palette: &Palette,
        seed: u32,
        spread: Spread,
    ) -> Vec<u8> {
        let mut strip = Vec::with_capacity((rows.len() as u32 * size * 4) as usize);

        for y in rows {
            for x in 0..size {
                let u = x as f32 / size as f32;
                let v = y as f32 / size as f32;
                let colour = self.sample_spread(u, v, palette, seed, spread);

                strip.push(byte(colour.r));
                strip.push(byte(colour.g));
                strip.push(byte(colour.b));
                strip.push(255);
            }
        }

        strip
    }
}

/// How many threads a bake of this size is worth splitting across.
///
/// Small tiles are not: a 128² tile is a couple of milliseconds, and spawning
/// threads to share that out costs more than it saves. The bakes worth
/// splitting are the spread ones, which are the reason this exists — a
/// four-tile spread is sixteen times the work of the tile it replaced.
fn bake_bands(size: u32) -> u32 {
    const SERIAL_BELOW: u32 = 256;

    if size < SERIAL_BELOW {
        return 1;
    }

    std::thread::available_parallelism()
        .map(|threads| threads.get() as u32)
        .unwrap_or(1)
        .clamp(1, size)
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

/// Periodic fractal noise on a lattice stretched along `v`.
///
/// Stretching the *sampling* rather than smearing the result is what keeps the
/// field seamless, and it is the same trick the fibre grain in
/// [`grain`](crate::rendering::grain) uses. Each axis is given its own whole
/// number of features across the tile as its own period, because one square
/// period cannot wrap two different frequencies at the same tile edge: made to
/// share one, it can only divide both, and the field then repeats several times
/// inside the texture as rows of the same feature.
///
/// Gradient noise rather than the value noise the other layers draw, for the
/// reason the grain's height field uses it: value noise has an extremum at
/// every lattice point, so a *ridge* taken through it traces the lattice. On a
/// stretched lattice that is unmissable — it prints rows of identical
/// teardrops, which is what the first pass at ice's fractures produced.
fn streaked_noise(u: f32, v: f32, scale: f32, elongation: f32, octaves: u32, seed: u32) -> f32 {
    let across = scale.round().max(1.0);
    let along = (scale / elongation.max(1.0)).round().max(1.0);

    fbm_perlin_2d_periodic_xy(
        u * across,
        v * along,
        octaves,
        0.5,
        2.0,
        seed,
        Some((across as i32, along as i32)),
    )
}

/// Where a noise field crosses its midpoint, as a 0..1 line weight.
///
/// The shared half of [`Layer::Vein`] and [`Layer::Streak`]: what makes either
/// read as a line is the crossing, and the two differ only in the field they
/// measure it on.
fn ridge(n: f32, sharpness: f32) -> f32 {
    (1.0 - (signed(n) * sharpness).abs()).max(0.0)
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

    /// A streak's whole point is that it has a direction. Measured as how
    /// much faster the field changes across the tile than along it — isotropic
    /// noise would give a ratio near one, and that is exactly what a `Vein`
    /// with the elongation misapplied would produce.
    #[test]
    fn a_streak_runs_in_one_direction() {
        let mut across = 0.0;
        let mut along = 0.0;

        for step in 0..64 {
            let u = (step % 8) as f32 / 8.0;
            let v = (step / 8) as f32 / 8.0;
            let here = streaked_noise(u, v, 12.0, 6.0, 2, 4);
            across += (streaked_noise(u + 0.02, v, 12.0, 6.0, 2, 4) - here).abs();
            along += (streaked_noise(u, v + 0.02, 12.0, 6.0, 2, 4) - here).abs();
        }

        assert!(
            across > along * 3.0,
            "the field is barely directional: across {across}, along {along}"
        );
    }

    /// Elongation rounds both frequencies and divides by their common factor
    /// to keep the period whole. An elongation that does not divide the scale
    /// exactly must still tile, because nothing at the call site says which
    /// ratios are safe.
    #[test]
    fn a_streaked_tile_is_seamless_in_both_axes() {
        for elongation in [2.0, 5.0, 7.0, 8.0] {
            for step in 0..16 {
                let t = step as f32 / 16.0;

                let left = streaked_noise(0.0, t, 12.0, elongation, 2, 3);
                let right = streaked_noise(1.0, t, 12.0, elongation, 2, 3);
                assert!(
                    (left - right).abs() < 1e-4,
                    "u seam at v={t}, elongation {elongation}: {left} vs {right}"
                );

                let bottom = streaked_noise(t, 0.0, 12.0, elongation, 2, 3);
                let top = streaked_noise(t, 1.0, 12.0, elongation, 2, 3);
                assert!(
                    (bottom - top).abs() < 1e-4,
                    "v seam at u={t}, elongation {elongation}: {bottom} vs {top}"
                );
            }
        }
    }

    /// A spread has to wrap like a tile does, or a slab wide enough to need
    /// one shows a seam instead of the repeat it was given to avoid.
    #[test]
    fn a_spread_tile_is_seamless() {
        let palette = palette();
        let spread = Spread::covering(3.0);

        for step in 0..16 {
            let t = step as f32 / 16.0;
            let left = library::ICE.sample_spread(0.0, t, &palette, 4, spread);
            let right = library::ICE.sample_spread(1.0, t, &palette, 4, spread);
            assert!(
                (left.r - right.r).abs() < 0.02,
                "seam at v={t}: {} vs {}",
                left.r,
                right.r
            );
        }
    }

    /// The whole point: a texture holding three tiles must not be the same
    /// tile three times. Sampled a third of the way apart, which is exactly
    /// where a spread that only resized the texture would come out identical.
    #[test]
    fn a_spread_does_not_repeat_inside_itself() {
        let palette = palette();
        let spread = Spread::covering(3.0);
        let period = 1.0 / spread.tiles() as f32;
        let mut difference = 0.0;

        for step in 0..64 {
            let u = (step % 8) as f32 / 8.0 * period;
            let v = (step / 8) as f32 / 8.0;
            difference += (library::ICE.sample_spread(u, v, &palette, 4, spread).r
                - library::ICE
                    .sample_spread(u + period, v, &palette, 4, spread)
                    .r)
                .abs();
        }

        assert!(
            difference / 64.0 > 0.01,
            "a third of the way across is the same picture (mean difference {})",
            difference / 64.0
        );
    }

    /// And it must not do it by magnifying the pattern — the features have to
    /// come out the same number of texels across, which is the difference
    /// between more pattern and a bigger one.
    #[test]
    fn a_spread_keeps_features_the_size_they_were() {
        let palette = palette();
        let tile = 256.0;

        let roughness = |spread: Spread| {
            let step = 1.0 / (tile * spread.tiles() as f32);
            let mut total = 0.0;
            for i in 0..256 {
                let u = (i % 16) as f32 / 16.0;
                let v = (i / 16) as f32 / 16.0;
                total += (library::ICE.sample_spread(u, v, &palette, 4, spread).r
                    - library::ICE
                        .sample_spread(u + step, v, &palette, 4, spread)
                        .r)
                    .abs();
            }
            total / 256.0
        };

        let one = roughness(Spread::ONE);
        let three = roughness(Spread::covering(3.0));

        assert!(
            (one - three).abs() < one * 0.5 + 1e-4,
            "features changed size with the spread: {one} per texel at one tile, {three} at three"
        );
    }

    /// The cap is what stops an object asking for a texture nobody budgeted
    /// for, and covering less than a tile must stay at one.
    #[test]
    fn a_spread_is_bounded_at_both_ends() {
        assert_eq!(Spread::covering(0.3).tiles(), 1);
        assert_eq!(Spread::covering(1.0).tiles(), 1);
        assert_eq!(Spread::covering(2.1).tiles(), 3);
        assert_eq!(Spread::covering(100.0).tiles(), Spread::MAX);
        assert_eq!(Spread::covering(f32::NAN).tiles(), 1);
    }

    /// The bake is split across threads and stitched back together, so every
    /// row has to land where it was sampled from. A band assembled out of
    /// order is invisible in a size check and obvious on the object, and the
    /// rows that would move are the ones late in the texture — hence a texel
    /// from each band rather than from the first.
    #[test]
    fn a_baked_texture_holds_the_rows_it_sampled() {
        let palette = palette();
        let spread = Spread::covering(2.0);
        let tile = 256;
        let size = spread.texture_size(tile);
        let pixels = library::ICE.bake_spread(tile, &palette, 7, spread);

        assert_eq!(pixels.len(), (size * size * 4) as usize);

        for step in 0..32 {
            let x = (step * 37) % size;
            let y = (step * size) / 32;
            let sampled = library::ICE.sample_spread(
                x as f32 / size as f32,
                y as f32 / size as f32,
                &palette,
                7,
                spread,
            );
            let at = ((y * size + x) * 4) as usize;

            assert_eq!(
                pixels[at],
                byte(sampled.r),
                "texel ({x}, {y}) is not the colour sampled there"
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
