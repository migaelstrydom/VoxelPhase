//! Vertex types for overlay rendering.

use ash::vk;
use nalgebra::{Vector2, Vector4};

/// Vertex format for 2D overlay rendering.
///
/// Used for text quads and other 2D overlay elements.
/// Positions are in screen-space pixels, with (0,0) at top-left.
#[derive(Copy, Clone, Debug)]
#[repr(C)]
pub struct OverlayVertex {
    pub pos: Vector2<f32>,   // Screen-space position in pixels
    pub uv: Vector2<f32>,    // Texture coordinates
    pub color: Vector4<f32>, // RGBA color
}

impl OverlayVertex {
    /// Get the Vulkan binding description for this vertex format.
    pub fn binding_description() -> vk::VertexInputBindingDescription {
        vk::VertexInputBindingDescription {
            binding: 0,
            stride: std::mem::size_of::<Self>() as u32,
            input_rate: vk::VertexInputRate::VERTEX,
        }
    }

    /// Get the Vulkan attribute descriptions for this vertex format.
    pub fn attribute_descriptions() -> [vk::VertexInputAttributeDescription; 3] {
        [
            // Position (vec2)
            vk::VertexInputAttributeDescription {
                binding: 0,
                location: 0,
                format: vk::Format::R32G32_SFLOAT,
                offset: 0,
            },
            // UV (vec2)
            vk::VertexInputAttributeDescription {
                binding: 0,
                location: 1,
                format: vk::Format::R32G32_SFLOAT,
                offset: 8,
            },
            // Color (vec4)
            vk::VertexInputAttributeDescription {
                binding: 0,
                location: 2,
                format: vk::Format::R32G32B32A32_SFLOAT,
                offset: 16,
            },
        ]
    }
}
