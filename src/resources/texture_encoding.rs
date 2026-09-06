//! Whether a texture's bytes are a colour or a measurement.
//!
//! The distinction decides one thing — whether the sampler applies the sRGB
//! transfer function on the way in — and getting it wrong is silent. Colour
//! read as data comes back too bright; data read as colour comes back through a
//! 2.4-power curve that no amount of downstream tuning can undo, because the
//! curve is not linear and the error therefore varies with the value.
//!
//! ```text
//!   Srgb    byte ──sRGB decode──▶ linear float   albedo, tints, anything seen
//!   Linear  byte ──── as-is ────▶ unit float     normals, masks, packed fields
//! ```
//!
//! A packed texture must pick one, so a field that mixes colour and data — as
//! the terrain surface field does, carrying an albedo wash beside a detail
//! normal — is stored [`Linear`](TextureEncoding::Linear) and writes its colour
//! channel at the value the shader should receive. Data cannot survive the
//! curve; a wash can simply be authored past it.

use ash::vk;

/// How a texture's bytes should be interpreted when sampled.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TextureEncoding {
    /// Bytes are sRGB-encoded colour. The sampler linearises them.
    ///
    /// The default for anything a player looks at directly: albedo maps,
    /// loaded image files, solid colours.
    Srgb,

    /// Bytes are values, taken at face value with no transfer function.
    ///
    /// For tangent-space normals, roughness and occlusion masks, and any packed
    /// field whose channels are numbers rather than light.
    Linear,
}

impl TextureEncoding {
    /// The Vulkan format an 8-bit RGBA texture of this encoding is created with.
    pub fn rgba8_format(self) -> vk::Format {
        match self {
            Self::Srgb => vk::Format::R8G8B8A8_SRGB,
            Self::Linear => vk::Format::R8G8B8A8_UNORM,
        }
    }
}

impl Default for TextureEncoding {
    fn default() -> Self {
        Self::Srgb
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn colour_and_data_take_different_formats() {
        assert_ne!(
            TextureEncoding::Srgb.rgba8_format(),
            TextureEncoding::Linear.rgba8_format()
        );
    }

    /// The whole point of the type. A texture created without an opinion is a
    /// colour texture, which is what every existing caller meant.
    #[test]
    fn the_default_is_colour() {
        assert_eq!(TextureEncoding::default(), TextureEncoding::Srgb);
        assert_eq!(
            TextureEncoding::default().rgba8_format(),
            vk::Format::R8G8B8A8_SRGB
        );
    }
}
