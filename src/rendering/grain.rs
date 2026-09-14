//! The microstructure a surface shows under a highlight.
//!
//! Terrain has had detail normals since its surface field arrived, and they are
//! most of why it reads as rock rather than as green plastic. Nothing else in
//! the game had any, because the flag that turned them on also turned on
//! world-projected albedo and terrain's roughness model
//! (see [`SurfaceSource`](crate::rendering::surface_source::SurfaceSource)).
//! With those decisions separated, every surface can have grain, and this is
//! the library it comes from.
//!
//! # One texture, two grains
//!
//! Grain is a *shared* resource. Every granite object in a level wants the same
//! rock microstructure, so it is generated once and bound once — at set 0
//! alongside the scene, not per material, which would put a descriptor and a
//! texture on every stone in the game.
//!
//! ```text
//!   RG ── stone: isotropic, sharp-edged, the grain a chisel leaves
//!   BA ── fibre: anisotropic, stretched along v — a sawn plank, a frozen block
//! ```
//!
//! Two grains share the four channels of one texture rather than occupying two
//! layers of an array, for the reason terrain packs its wash beside its normal:
//! it costs one sample instead of two, and the sample is the expensive part. A
//! third grain needs a second texture, and
//! [`GrainLayer`] is the index that would select it.
//!
//! # Why fibre cannot use the same projection as stone
//!
//! Stone grain is isotropic, so it can be projected from any direction and
//! still look like stone. A fibre has a direction, and that direction is a
//! property of the object, not of the world — a triplanar projection would run
//! the fibre along whichever axis the face happens to point at, which on a
//! tumbling crate changes as it tumbles. Fibre is therefore addressed by the
//! mesh's own texture coordinates, which already know which way the plank —
//! or the freeze front — runs.

use crate::core::error::EngineResult;
use crate::resources::textures::{TextureHandle, TextureManager};
use crate::utils::noise::fbm_perlin_2d_periodic;

/// Edge length of the grain texture, in texels.
///
/// Matches the terrain surface field. At the scales props project it, a texel
/// covers a few millimetres, which is the size microstructure has to be to read
/// as microstructure rather than as mottling.
const RESOLUTION: u32 = 512;

/// Layers of fractal noise in the height field the normals are derived from.
const OCTAVES: u32 = 5;

/// Feature size of the stone grain, in texture repeats. Also its tiling period.
const STONE_NOISE_SCALE: f32 = 24.0;

/// Feature size of the fibre grain across the fibre. Higher than stone's
/// because the field is then stretched along the fibre by [`FIBRE_STRETCH`],
/// and the two together are what make a ring rather than a blob.
const FIBRE_NOISE_SCALE: f32 = 32.0;

/// How much longer a fibre's features are along it than across it.
///
/// Eight to one is the ratio at which the field stops reading as stretched
/// noise and starts reading as grain: below about four the rings look like
/// smeared blobs, and much above ten they turn into stripes with no variation
/// to break them up.
const FIBRE_STRETCH: f32 = 8.0;

/// Fixed so that every run and every bench sheet shows the same grain.
const STONE_SEED: u32 = 7;
const FIBRE_SEED: u32 = 13;

/// Height of the stone microrelief, relative to the spacing of the texels it is
/// measured across. The single number that decides how pronounced stone grain
/// is at full strength.
const STONE_RELIEF: f32 = 16.0;

/// Height of the fibre microrelief. Lower than stone's: a planed plank is a
/// far smoother thing than a broken rock face, and a fibre that competes with
/// stone grain makes every material look like the same rough surface.
const FIBRE_RELIEF: f32 = 7.0;

/// Which grain a surface takes, as stored in the material control word.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GrainLayer {
    /// Isotropic rock microstructure. Granite, limestone, concrete, marble.
    Stone = 0,
    /// Directional fibre running along the mesh's v axis. Planks, crates and
    /// posts; also the fracture grain in a frozen block, which runs the same
    /// way for the same reason — it was laid down in a direction.
    Fibre = 1,
}

impl GrainLayer {
    /// Index the shader selects this grain's channel pair by.
    pub fn index(self) -> u32 {
        self as u32
    }
}

/// How strongly and at what scale a surface shows its grain.
///
/// Separated from the choice of grain because the same rock microstructure
/// reads as a boulder at one scale and as gravel at another, and because a
/// polished surface and a broken one differ only in strength.
#[derive(Clone, Copy, Debug)]
pub struct GrainSpec {
    /// Which of the packed grains to sample.
    pub layer: GrainLayer,

    /// Texture repeats per world unit (for an object-space projection) or per
    /// UV unit (for a UV-addressed one).
    pub scale: f32,

    /// Multiplier on the stored slope. Zero leaves the geometric normal
    /// untouched, which is what a surface with no microstructure should get.
    pub strength: f32,
}

impl GrainSpec {
    /// No microstructure at all. The finish of a moulded plastic toy, and the
    /// right answer for most of the bright props — grain on everything costs
    /// the stone its contrast.
    pub const NONE: Self = Self {
        layer: GrainLayer::Stone,
        scale: 0.0,
        strength: 0.0,
    };

    /// Broken rock: a quarried block, a standing stone, a rubble wall.
    ///
    /// The scale is in repeats per *world metre*, and the field carries
    /// [`STONE_NOISE_SCALE`] features per repeat — so this is about twelve
    /// mineral features per metre. Finer than that and rock reads as orange
    /// peel, which is what the first pass at these numbers produced; much
    /// coarser and the microstructure becomes lumps that the geometry should
    /// have carried instead.
    pub const STONE: Self = Self {
        layer: GrainLayer::Stone,
        scale: 0.5,
        strength: 1.0,
    };

    /// Dressed stone: a temple column, a cut voussoir. The same microstructure
    /// as `STONE` at half the strength, because a worked face has been taken
    /// down but not polished away.
    pub const DRESSED_STONE: Self = Self {
        layer: GrainLayer::Stone,
        scale: 0.65,
        strength: 0.55,
    };

    /// Cast concrete: coarser than dressed stone and rougher than rock, because
    /// the aggregate is bigger than the crystal.
    pub const CONCRETE: Self = Self {
        layer: GrainLayer::Stone,
        scale: 0.35,
        strength: 0.8,
    };

    /// Sawn timber. Addressed by UV, so `scale` is in repeats per UV unit and
    /// one repeat should span roughly one plank.
    pub const WOOD: Self = Self {
        layer: GrainLayer::Fibre,
        scale: 1.0,
        strength: 0.4,
    };

    /// A frozen block: the same directional fibre, coarser and much fainter.
    ///
    /// Coarser because the flaws a freeze leaves are centimetres apart rather
    /// than millimetres, and fainter because this is relief on a surface the
    /// eye is mostly looking *through* — grain at plank strength on clear ice
    /// reads as a frosted bathroom window, which is a real material and not
    /// this one.
    pub const ICE: Self = Self {
        layer: GrainLayer::Fibre,
        scale: 0.45,
        strength: 0.14,
    };

    /// Roughly how many mineral features this grain shows per world metre.
    ///
    /// The number the scale constants were chosen against, and the one to reach
    /// for when adding a substance: terrain sits near 2 at its own projection,
    /// broken stone near 12, and anything past about 30 stops reading as
    /// microstructure and starts reading as noise.
    pub fn features_per_metre(&self) -> f32 {
        self.scale
            * match self.layer {
                GrainLayer::Stone => STONE_NOISE_SCALE,
                GrainLayer::Fibre => FIBRE_NOISE_SCALE,
            }
    }

    /// Whether this grain does anything at all.
    pub fn is_enabled(&self) -> bool {
        self.scale > 0.0 && self.strength > 0.0
    }

    /// Same grain at a different strength.
    pub const fn with_strength(mut self, strength: f32) -> Self {
        self.strength = strength;
        self
    }

    /// Same grain at a different scale.
    pub const fn with_scale(mut self, scale: f32) -> Self {
        self.scale = scale;
        self
    }
}

impl Default for GrainSpec {
    fn default() -> Self {
        Self::NONE
    }
}

/// Generate the shared grain texture.
///
/// A data texture, not a colour one: all four channels are slopes. Mipmapped,
/// which is what fades grain out with distance — averaging the stored slopes
/// towards zero is exactly the statement that the bumps no longer cover a
/// pixel, and it is why grain does not sparkle on a receding surface.
pub fn create_grain_texture(textures: &TextureManager) -> EngineResult<TextureHandle> {
    let resolution = RESOLUTION as usize;
    let mut rgba = Vec::with_capacity(resolution * resolution * 4);

    for y in 0..RESOLUTION {
        for x in 0..RESOLUTION {
            let (stone_x, stone_y) = stone_normal(x, y);
            let (fibre_x, fibre_y) = fibre_normal(x, y);

            rgba.push(encode_signed(stone_x));
            rgba.push(encode_signed(stone_y));
            rgba.push(encode_signed(fibre_x));
            rgba.push(encode_signed(fibre_y));
        }
    }

    textures.create_data_from_rgba(RESOLUTION, RESOLUTION, &rgba, true)
}

/// The stone height field at a texel. Wraps, so the texture tiles.
///
/// Gradient noise rather than value noise, for the reason terrain's field uses
/// it: this is differentiated to produce a normal, and value noise has an
/// extremum at every lattice point, so its slope field prints the lattice as
/// axis-aligned banding — invisible in a colour, unmissable in a relief.
fn stone_height(x: u32, y: u32) -> f32 {
    let to_noise = STONE_NOISE_SCALE / RESOLUTION as f32;
    fbm_perlin_2d_periodic(
        (x % RESOLUTION) as f32 * to_noise,
        (y % RESOLUTION) as f32 * to_noise,
        OCTAVES,
        0.5,
        2.0,
        STONE_SEED,
        Some(STONE_NOISE_SCALE as i32),
    )
}

/// The fibre height field at a texel.
///
/// The same noise as stone's, sampled on a lattice stretched by
/// [`FIBRE_STRETCH`] along v. Stretching the *sampling* rather than filtering
/// the result keeps the field seamless, because the period is divided by the
/// same factor the coordinate is.
fn fibre_height(x: u32, y: u32) -> f32 {
    let across = FIBRE_NOISE_SCALE / RESOLUTION as f32;
    let along = across / FIBRE_STRETCH;

    let period_along = (FIBRE_NOISE_SCALE / FIBRE_STRETCH).max(1.0) as i32;

    fbm_perlin_2d_periodic(
        (x % RESOLUTION) as f32 * across,
        (y % RESOLUTION) as f32 * along,
        OCTAVES,
        0.5,
        2.0,
        FIBRE_SEED,
        Some(period_along.max(1)),
    )
}

/// Tangent-space xy of a height field's normal at a texel.
///
/// A central difference across neighbouring texels, wrapped so the derived
/// normals tile as seamlessly as the height does. The z component is not
/// stored: it is positive by construction and the shader reconstructs it.
fn slope_normal(height: impl Fn(u32, u32) -> f32, x: u32, y: u32, relief: f32) -> (f32, f32) {
    let last = RESOLUTION - 1;
    let left = height(if x == 0 { last } else { x - 1 }, y);
    let right = height(if x == last { 0 } else { x + 1 }, y);
    let down = height(x, if y == 0 { last } else { y - 1 });
    let up = height(x, if y == last { 0 } else { y + 1 });

    let slope_x = (right - left) * 0.5 * relief;
    let slope_y = (up - down) * 0.5 * relief;

    // Slope points uphill; a surface normal leans away from it.
    let length = (slope_x * slope_x + slope_y * slope_y + 1.0).sqrt();
    (-slope_x / length, -slope_y / length)
}

fn stone_normal(x: u32, y: u32) -> (f32, f32) {
    slope_normal(stone_height, x, y, STONE_RELIEF)
}

fn fibre_normal(x: u32, y: u32) -> (f32, f32) {
    slope_normal(fibre_height, x, y, FIBRE_RELIEF)
}

/// Encode a -1-to-1 value into a byte, as the shader's `value * 2 - 1` expects.
fn encode_signed(value: f32) -> u8 {
    ((value * 0.5 + 0.5).clamp(0.0, 1.0) * 255.0) as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Mean tilt of a grain, in degrees, sampled across the texture.
    fn mean_tilt(normal: impl Fn(u32, u32) -> (f32, f32)) -> f32 {
        let mut total = 0.0;
        let mut samples = 0;
        for y in (0..RESOLUTION).step_by(7) {
            for x in (0..RESOLUTION).step_by(7) {
                let (nx, ny) = normal(x, y);
                total += (nx * nx + ny * ny).sqrt().asin().to_degrees();
                samples += 1;
            }
        }
        total / samples as f32
    }

    /// The amount of relief is the whole visible effect, and it is easy to lose
    /// by accident: a change to the octave count or the resolution alters the
    /// slope between neighbouring texels without touching anything that looks
    /// like a strength dial.
    #[test]
    fn stone_grain_tilts_by_a_useful_amount() {
        let tilt = mean_tilt(stone_normal);
        assert!(
            (8.0..30.0).contains(&tilt),
            "stone grain tilts by a mean of {tilt}°"
        );
    }

    /// A fibre is a planed surface and stone is a broken one. If the two ever land
    /// on the same relief, every material in the game reads as the same rough
    /// thing under a highlight, which is the failure this library exists to
    /// avoid.
    #[test]
    fn fibre_is_smoother_than_stone() {
        assert!(
            mean_tilt(fibre_normal) < mean_tilt(stone_normal) * 0.75,
            "fibre {}° vs stone {}°",
            mean_tilt(fibre_normal),
            mean_tilt(stone_normal)
        );
    }

    /// A fibre's whole character is that it has a direction. Measured as the ratio
    /// of mean slope across the fibre to mean slope along it — isotropic noise
    /// would give a ratio near one.
    #[test]
    fn fibre_grain_runs_along_the_fibre() {
        let mut across = 0.0;
        let mut along = 0.0;
        let mut samples = 0;

        for y in (0..RESOLUTION).step_by(7) {
            for x in (0..RESOLUTION).step_by(7) {
                let (nx, ny) = fibre_normal(x, y);
                across += nx.abs();
                along += ny.abs();
                samples += 1;
            }
        }

        let ratio = (across / samples as f32) / (along / samples as f32).max(1e-6);
        assert!(
            ratio > 3.0,
            "fibre grain is barely directional: across/along = {ratio}"
        );
    }

    /// Stone must *not* be directional, or every rock face in the game acquires
    /// a comb pattern that follows whichever axis the projection picked.
    #[test]
    fn stone_grain_has_no_direction() {
        let mut across = 0.0;
        let mut along = 0.0;
        let mut samples = 0;

        for y in (0..RESOLUTION).step_by(7) {
            for x in (0..RESOLUTION).step_by(7) {
                let (nx, ny) = stone_normal(x, y);
                across += nx.abs();
                along += ny.abs();
                samples += 1;
            }
        }

        let ratio = (across / samples as f32) / (along / samples as f32).max(1e-6);
        assert!(
            (0.6..1.6).contains(&ratio),
            "stone grain has a direction: across/along = {ratio}"
        );
    }

    /// The stored xy must describe a normal completable by a positive z, or the
    /// shader's reconstruction takes the root of a negative number and the
    /// surface turns inside out.
    #[test]
    fn every_stored_normal_has_a_reconstructable_z() {
        for y in (0..RESOLUTION).step_by(13) {
            for x in (0..RESOLUTION).step_by(13) {
                for (nx, ny) in [stone_normal(x, y), fibre_normal(x, y)] {
                    let planar = nx * nx + ny * ny;
                    assert!(planar < 1.0, "normal at ({x}, {y}) has no z left: {planar}");
                }
            }
        }
    }

    /// The texture tiles, so opposite edges must agree. A one-sided difference
    /// at the edge would put a visible ridge along every repeat.
    #[test]
    fn the_grain_wraps_at_the_texture_edge() {
        let last = RESOLUTION - 1;
        for y in (0..RESOLUTION).step_by(31) {
            let (left_x, left_y) = stone_normal(0, y);
            let (right_x, right_y) = stone_normal(last, y);
            // Neighbouring texels, not identical ones: the wrap means texel 0
            // and texel `last` are adjacent, so their normals should be close.
            assert!(
                (left_x - right_x).abs() < 0.5 && (left_y - right_y).abs() < 0.5,
                "seam at y={y}: ({left_x}, {left_y}) vs ({right_x}, {right_y})"
            );
        }
    }

    #[test]
    fn a_flat_slope_encodes_to_the_middle_of_the_range() {
        let encoded = encode_signed(0.0);
        let decoded = (encoded as f32 / 255.0) * 2.0 - 1.0;
        assert!(decoded.abs() < 0.01, "flat encodes to {decoded}");
    }

    #[test]
    fn no_grain_is_the_default_and_does_nothing() {
        assert!(!GrainSpec::default().is_enabled());
        assert!(!GrainSpec::NONE.is_enabled());
        assert!(GrainSpec::STONE.is_enabled());
    }

    /// A worked face has been taken down but not polished away, so it must
    /// still show more than nothing and less than broken rock.
    #[test]
    fn dressed_stone_sits_between_rough_stone_and_nothing() {
        assert!(GrainSpec::DRESSED_STONE.strength < GrainSpec::STONE.strength);
        assert!(GrainSpec::DRESSED_STONE.strength > 0.0);
    }

    /// The scales were tuned by eye once and are easy to lose to an unrelated
    /// edit of the noise constants. Pinning the feature density rather than the
    /// scale is what keeps the intent legible: changing STONE_NOISE_SCALE must
    /// move the scales to compensate, not silently change how rock looks.
    #[test]
    fn the_stone_grains_sit_in_the_band_that_reads_as_mineral() {
        for (name, spec) in [
            ("stone", GrainSpec::STONE),
            ("dressed", GrainSpec::DRESSED_STONE),
            ("concrete", GrainSpec::CONCRETE),
        ] {
            let density = spec.features_per_metre();
            assert!(
                (5.0..30.0).contains(&density),
                "{name} shows {density} features per metre"
            );
        }
    }

    /// Cast concrete's aggregate is bigger than a crystal, so it must be the
    /// coarsest of the three; a worked face is the finest.
    #[test]
    fn the_stone_grains_are_ordered_by_how_worked_the_surface_is() {
        assert!(
            GrainSpec::CONCRETE.features_per_metre() < GrainSpec::STONE.features_per_metre(),
            "concrete should be coarser than broken rock"
        );
        assert!(
            GrainSpec::STONE.features_per_metre() < GrainSpec::DRESSED_STONE.features_per_metre(),
            "a dressed face should be finer than a broken one"
        );
    }

    #[test]
    fn the_grains_index_distinctly() {
        assert_eq!(GrainLayer::Stone.index(), 0);
        assert_eq!(GrainLayer::Fibre.index(), 1);
    }
}
