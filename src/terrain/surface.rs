//! How terrain is textured and shaded.
//!
//! Terrain's surface appearance is owned here rather than at the draw call, so
//! that the game and the visual bench cannot disagree about what terrain looks
//! like — the bench exists to predict the game, and it can only do that if both
//! ask the same place.

use crate::core::error::EngineResult;
use crate::rendering::material::SurfaceParams;
use crate::rendering::triplanar::TriplanarProjection;
use crate::resources::textures::{TextureHandle, TextureManager};

/// Edge length of the noise texture, in texels.
const NOISE_RESOLUTION: u32 = 512;

/// Layers of fractal noise. More octaves add finer detail at a lower amplitude.
const NOISE_OCTAVES: u32 = 5;

/// Feature size of the noise, in texture repeats. Also its tiling period, so
/// the texture is seamless.
const NOISE_SCALE: f32 = 20.0;

/// Fixed so that terrain looks the same in every run and on the bench.
const NOISE_SEED: u32 = 42;

/// Generate the procedural noise texture terrain samples for surface variation.
///
/// Greyscale, and mapped to a narrow band around white, so it modulates albedo
/// only slightly — it is a wash over the vertex colours, not a material.
pub fn create_surface_texture(textures: &TextureManager) -> EngineResult<TextureHandle> {
    textures.create_noise_texture(
        NOISE_RESOLUTION,
        NOISE_RESOLUTION,
        NOISE_OCTAVES,
        NOISE_SCALE,
        NOISE_SEED,
    )
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
    SurfaceParams::MATTE.with_projection(TriplanarProjection::TERRAIN)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn terrain_is_textured_by_world_position() {
        assert_eq!(
            surface_params().projection,
            TriplanarProjection::TERRAIN.packed()
        );
    }
}
