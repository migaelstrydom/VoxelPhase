//! How terrain is textured and shaded.
//!
//! Terrain's surface appearance is owned here rather than at the draw call, so
//! that the game and the visual bench cannot disagree about what terrain looks
//! like — the bench exists to predict the game, and it can only do that if both
//! ask the same place.
//!
//! # The surface field
//!
//! One texture carries two things, because terrain samples it three times (once
//! per triplanar axis) and those samples are the expensive part of drawing the
//! largest mesh in the game. Adding a second texture for detail normals would
//! have doubled that cost; packing them into the channels the wash was not
//! using costs nothing at all.
//!
//! ```text
//!   fbm height field h(u,v)
//!        ├── value ─────────────▶ R    albedo wash, 0.85 … 1.0
//!        └── central difference ─▶ GB   detail normal xy, tangent space
//!                                  A    unused
//! ```
//!
//! Both come from the same height field on purpose: the wash is that field's
//! low-frequency content and the detail normal is its slope, so the light and
//! dark of a surface always agree with the bumps on it rather than being two
//! unrelated patterns laid over each other.
//!
//! The normal is stored at a fixed reference amplitude and scaled per fragment
//! by the material's hardness (`shader/surface_character.glsl`), which is how
//! chalk ends up softer-featured than rock without a second texture.
//!
//! # Where the rest of the dials are
//!
//! Everything terrain's appearance is tuned by that can be decided on the CPU
//! is in this file. Three sets of numbers deliberately are not, because they
//! belong to something else and moving them here would be filing them under the
//! wrong owner:
//!
//! | Dial | Lives in | Why not here |
//! |---|---|---|
//! | Roughness and relief per hardness | `shader/surface_character.glsl` | Evaluated per fragment. Reaching them from the CPU means spending push-constant budget or a UBO on values that never change at runtime. |
//! | Specular anti-aliasing strength and ceiling | `shader/lighting.glsl` | Applies to every surface in the game, not just terrain. |
//! | Material colour, and the toughness-to-hardness curve | `src/terrain/voxel.rs` | Material identity, which destruction reads too. A renderer must not be the thing that defines what rock *is*. |
//!
//! Changing anything in the two shader files needs `glslc` re-run; see
//! CLAUDE.md for the command.

use crate::core::error::EngineResult;
use crate::rendering::material::SurfaceParams;
use crate::rendering::surface_source::SurfaceSource;
use crate::rendering::triplanar::TriplanarProjection;
use crate::resources::texture_encoding::TextureEncoding;
use crate::resources::textures::{TextureHandle, TextureManager};
use crate::utils::noise::fbm_perlin_2d_periodic;

/// Edge length of the surface texture, in texels.
///
/// At the projection scale terrain uses, one repeat covers 10 m, so this is
/// roughly 2 cm per texel — fine enough that the field's highest octave is
/// genuine close-range microstructure rather than something that reads as
/// large-scale mottling.
const NOISE_RESOLUTION: u32 = 512;

/// Layers of fractal noise. More octaves add finer detail at a lower amplitude.
const NOISE_OCTAVES: u32 = 5;

/// Feature size of the noise, in texture repeats. Also its tiling period, so
/// the texture is seamless.
const NOISE_SCALE: f32 = 20.0;

/// Fixed so that terrain looks the same in every run and on the bench.
const NOISE_SEED: u32 = 42;

/// Darkest the albedo wash goes. A narrow band around white, because this
/// modulates the vertex colours rather than replacing them.
///
/// Reaches the shader as written: the field is a data texture (see
/// `create_surface_texture`), so no transfer function stands between this
/// number and the multiply. Widening the band is the one dial for terrain's
/// large-scale mottling.
const WASH_FLOOR: f32 = 0.85;

/// Texture repeats per world unit — one repeat every 10 m.
///
/// Deliberately equal to the scale of the top-down UV that terrain vertices
/// already carried (`pos.xz * 0.1`), so that switching to a projection left
/// flat ground looking exactly as it did and changed only the steep faces the
/// old projection was smearing. A drift here is a silent change to the floor of
/// every level.
///
/// It also sets how coarse the detail normals are: at this scale a texel covers
/// about 2 cm of world, and the field's finest octave is a few centimetres
/// across.
const PROJECTION_SCALE: f32 = 0.1;

/// How sharply a surface commits to the axis plane it most nearly faces.
const BLEND_SHARPNESS: f32 = 4.0;

/// Height of the microrelief the stored normal describes, relative to the
/// spacing of the texels it is measured across.
///
/// This is the one number that decides how pronounced the detail is at full
/// strength. It was chosen by measuring the tilt it produces rather than by
/// eye — see `the_detail_normals_tilt_by_a_useful_amount`, which fails if a
/// change to the noise quietly flattens the surface or turns it into gravel.
const RELIEF: f32 = 14.0;

/// How the field's bytes are sampled.
///
/// Linear, and not negotiable: two of the three channels are a tangent-space
/// normal. An sRGB view would put a 2.4-power curve between what
/// `detail_normal` computes and what the shader reads, which is not a tuning
/// difference — it is a constant lean on every texel, because the byte that
/// means "flat" stops decoding to zero.
const FIELD_ENCODING: TextureEncoding = TextureEncoding::Linear;

/// Generate the packed surface field terrain samples.
///
/// See the module documentation for the channel layout. Mipmapped, which is
/// what makes the detail fade with distance: averaging the stored normal xy
/// towards zero is exactly the statement that the bumps have shrunk below what
/// a pixel can resolve.
pub fn create_surface_texture(textures: &TextureManager) -> EngineResult<TextureHandle> {
    let resolution = NOISE_RESOLUTION as usize;
    let mut rgba = Vec::with_capacity(resolution * resolution * 4);

    for y in 0..NOISE_RESOLUTION {
        for x in 0..NOISE_RESOLUTION {
            let height = sample_height(x, y);
            let (normal_x, normal_y) = detail_normal(x, y);

            rgba.push(encode_unit(WASH_FLOOR + height * (1.0 - WASH_FLOOR)));
            rgba.push(encode_signed(normal_x));
            rgba.push(encode_signed(normal_y));
            rgba.push(255);
        }
    }

    // A *data* texture, not a colour one. Two of its three channels are a
    // tangent-space normal, and an sRGB transfer function on the way in would
    // bend them through a 2.4-power curve: the byte 128 that means "no slope"
    // would arrive as 0.216, decoding to a permanent -0.57 lean on every texel
    // in the field. The wash rides along in the same texture and is therefore
    // authored at the value the shader should receive.
    textures.create_encoded_from_rgba(
        NOISE_RESOLUTION,
        NOISE_RESOLUTION,
        &rgba,
        true,
        FIELD_ENCODING,
    )
}

/// The height field, sampled at a texel. Wraps, so the texture tiles.
///
/// Gradient noise rather than the value noise the rest of the engine uses: this
/// field is differentiated to produce the detail normal, and value noise has an
/// extremum at every lattice point, so its slope field prints the lattice as
/// axis-aligned banding. It is invisible in the wash and unmissable in the
/// relief.
fn sample_height(x: u32, y: u32) -> f32 {
    let to_noise = NOISE_SCALE / NOISE_RESOLUTION as f32;
    fbm_perlin_2d_periodic(
        (x % NOISE_RESOLUTION) as f32 * to_noise,
        (y % NOISE_RESOLUTION) as f32 * to_noise,
        NOISE_OCTAVES,
        0.5,
        2.0,
        NOISE_SEED,
        Some(NOISE_SCALE as i32),
    )
}

/// Tangent-space xy of the surface normal at a texel.
///
/// A central difference across neighbouring texels, wrapped so that the derived
/// normals tile as seamlessly as the height does — a one-sided difference at
/// the edge would put a visible ridge along every repeat. The z component is
/// not stored: it is positive by construction and the shader reconstructs it.
fn detail_normal(x: u32, y: u32) -> (f32, f32) {
    let last = NOISE_RESOLUTION - 1;
    let left = sample_height(if x == 0 { last } else { x - 1 }, y);
    let right = sample_height(if x == last { 0 } else { x + 1 }, y);
    let down = sample_height(x, if y == 0 { last } else { y - 1 });
    let up = sample_height(x, if y == last { 0 } else { y + 1 });

    let slope_x = (right - left) * 0.5 * RELIEF;
    let slope_y = (up - down) * 0.5 * RELIEF;

    // Slope points uphill; a surface normal leans away from it.
    let length = (slope_x * slope_x + slope_y * slope_y + 1.0).sqrt();
    (-slope_x / length, -slope_y / length)
}

/// Encode a 0-to-1 value into a byte.
fn encode_unit(value: f32) -> u8 {
    (value.clamp(0.0, 1.0) * 255.0) as u8
}

/// Encode a -1-to-1 value into a byte, as the shader's `value * 2 - 1` expects.
fn encode_signed(value: f32) -> u8 {
    encode_unit(value * 0.5 + 0.5)
}

/// The shading parameters terrain is drawn with.
///
/// Textured by world position: marching cubes emits no usable UVs, and a
/// top-down projection smears into vertical streaks on anything steep.
///
/// The roughness in these params is not the one terrain shades with. A draw
/// carries a single finish and a terrain chunk carries many materials, so the
/// fragment shader derives roughness per fragment from the per-vertex hardness
/// instead (shader/surface_character.glsl). `MATTE` is what a terrain surface
/// of zero hardness would take, so the params remain the honest fallback rather
/// than a number that is quietly ignored.
pub fn surface_params() -> SurfaceParams {
    SurfaceParams::MATTE
        .with_projection(TriplanarProjection::new(PROJECTION_SCALE, BLEND_SHARPNESS))
        .with_source(SurfaceSource::TERRAIN)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The field is sampled as data, never as colour. Flipping this back is a
    /// one-word change with no compile error and no obvious symptom — terrain
    /// simply acquires a fixed lean and a wash darker than the one authored
    /// here — so the choice is asserted rather than left to the call site.
    #[test]
    fn the_surface_field_is_sampled_as_data_not_colour() {
        assert_eq!(FIELD_ENCODING, TextureEncoding::Linear);
    }

    /// The encoding fix is only complete if the flat case is actually flat: the
    /// byte written for zero slope must come back as zero slope after the
    /// shader's `value * 2 - 1`.
    #[test]
    fn a_texel_with_no_slope_decodes_to_no_slope() {
        let encoded = encode_signed(0.0);
        let decoded = (encoded as f32 / 255.0) * 2.0 - 1.0;
        assert!(
            decoded.abs() < 0.01,
            "a flat texel decodes to a slope of {decoded}"
        );
    }

    #[test]
    fn terrain_is_textured_by_world_position() {
        assert!(TriplanarProjection::new(PROJECTION_SCALE, BLEND_SHARPNESS).is_enabled());
        assert_eq!(
            surface_params().projection,
            [PROJECTION_SCALE, BLEND_SHARPNESS]
        );
    }

    /// Terrain vertices used to carry `tex_coords = pos.xz * 0.1`, which is the
    /// Y plane of this projection. Matching it is what kept flat ground looking
    /// unchanged when the projection replaced those UVs.
    #[test]
    fn the_projection_matches_the_uvs_it_replaced() {
        assert_eq!(PROJECTION_SCALE, 0.1);
    }

    /// The amount of relief is the whole visible effect of detail normals, and
    /// it is easy to lose by accident: a change to the octave count or the
    /// resolution alters the slope between neighbouring texels without touching
    /// anything that looks like a strength dial. Too flat and the surface has
    /// no microstructure for a highlight to break up against; too steep and it
    /// reads as gravel and aliases.
    #[test]
    fn the_detail_normals_tilt_by_a_useful_amount() {
        let mut total_tilt = 0.0;
        let mut samples = 0;

        for y in (0..NOISE_RESOLUTION).step_by(7) {
            for x in (0..NOISE_RESOLUTION).step_by(7) {
                let (nx, ny) = detail_normal(x, y);
                total_tilt += (nx * nx + ny * ny).sqrt().asin().to_degrees();
                samples += 1;
            }
        }

        let mean_tilt = total_tilt / samples as f32;
        assert!(
            (8.0..30.0).contains(&mean_tilt),
            "detail normals tilt by a mean of {mean_tilt}°"
        );
    }

    /// The stored xy must describe a normal that can be completed by a positive
    /// z, or the shader's reconstruction takes the square root of a negative
    /// number and the surface turns inside out.
    #[test]
    fn the_stored_normal_always_has_a_reconstructable_z() {
        for y in (0..NOISE_RESOLUTION).step_by(13) {
            for x in (0..NOISE_RESOLUTION).step_by(13) {
                let (nx, ny) = detail_normal(x, y);
                let planar = nx * nx + ny * ny;
                assert!(planar < 1.0, "normal at ({x}, {y}) has no z left: {planar}");
            }
        }
    }

    /// The texture tiles, so its detail normals have to tile too. Wrapping the
    /// central difference is what makes the seam invisible, and a one-sided
    /// difference at the edge would put a ridge along every repeat — at 10 m
    /// per repeat, a ridge every 10 m across the whole level.
    #[test]
    fn the_detail_normals_wrap_at_the_texture_edge() {
        let last = NOISE_RESOLUTION - 1;
        for y in (0..NOISE_RESOLUTION).step_by(29) {
            let (left_x, left_y) = detail_normal(0, y);
            let (wrapped_x, wrapped_y) = detail_normal(NOISE_RESOLUTION, y);
            assert!((left_x - wrapped_x).abs() < 1e-6, "x seam at row {y}");
            assert!((left_y - wrapped_y).abs() < 1e-6, "y seam at row {y}");

            // The edge texels are neighbours once the texture repeats, so their
            // normals must be close rather than identical.
            let (right_x, _) = detail_normal(last, y);
            assert!(
                (left_x - right_x).abs() < 0.6,
                "row {y} jumps across the repeat: {left_x} to {right_x}"
            );
        }
    }
}
