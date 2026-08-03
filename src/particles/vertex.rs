//! Particle vertex format for GPU rendering.

use ash::vk;
use nalgebra::{Vector2, Vector3, Vector4};
use std::mem;

/// Vertex data for a single particle billboard corner.
///
/// Each particle is rendered as a quad (4 vertices, 6 indices).
/// The corner offset is used to expand the quad in the vertex shader.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ParticleVertex {
    /// World position of the particle center.
    pub center: Vector3<f32>,
    /// Billboard corner offset (-1,-1 to 1,1).
    pub corner: Vector2<f32>,
    /// Particle size (used to scale the billboard).
    pub size: f32,
    /// RGBA color with alpha.
    pub color: Vector4<f32>,
    /// Normalized lifetime (0.0 = just born, 1.0 = about to die).
    pub life: f32,
    /// World-space smear vector: how far the particle moves in the time its
    /// billboard is stretched over. Zero draws a round particle; otherwise the
    /// quad is drawn as a capsule-ish streak along this direction.
    pub motion: Vector3<f32>,
}

impl ParticleVertex {
    /// Get the Vulkan binding description for this vertex format.
    pub fn binding_description() -> vk::VertexInputBindingDescription {
        vk::VertexInputBindingDescription {
            binding: 0,
            stride: mem::size_of::<Self>() as u32,
            input_rate: vk::VertexInputRate::VERTEX,
        }
    }

    /// Get the Vulkan attribute descriptions for this vertex format.
    pub fn attribute_descriptions() -> [vk::VertexInputAttributeDescription; 6] {
        [
            // Center position (location 0) - vec3
            vk::VertexInputAttributeDescription {
                binding: 0,
                location: 0,
                format: vk::Format::R32G32B32_SFLOAT,
                offset: 0,
            },
            // Corner offset (location 1) - vec2
            vk::VertexInputAttributeDescription {
                binding: 0,
                location: 1,
                format: vk::Format::R32G32_SFLOAT,
                offset: mem::size_of::<Vector3<f32>>() as u32,
            },
            // Size (location 2) - float
            vk::VertexInputAttributeDescription {
                binding: 0,
                location: 2,
                format: vk::Format::R32_SFLOAT,
                offset: (mem::size_of::<Vector3<f32>>() + mem::size_of::<Vector2<f32>>()) as u32,
            },
            // Color (location 3) - vec4
            vk::VertexInputAttributeDescription {
                binding: 0,
                location: 3,
                format: vk::Format::R32G32B32A32_SFLOAT,
                offset: (mem::size_of::<Vector3<f32>>()
                    + mem::size_of::<Vector2<f32>>()
                    + mem::size_of::<f32>()) as u32,
            },
            // Life (location 4) - float
            vk::VertexInputAttributeDescription {
                binding: 0,
                location: 4,
                format: vk::Format::R32_SFLOAT,
                offset: (mem::size_of::<Vector3<f32>>()
                    + mem::size_of::<Vector2<f32>>()
                    + mem::size_of::<f32>()
                    + mem::size_of::<Vector4<f32>>()) as u32,
            },
            // Motion smear (location 5) - vec3
            vk::VertexInputAttributeDescription {
                binding: 0,
                location: 5,
                format: vk::Format::R32G32B32_SFLOAT,
                offset: (mem::size_of::<Vector3<f32>>()
                    + mem::size_of::<Vector2<f32>>()
                    + mem::size_of::<f32>()
                    + mem::size_of::<Vector4<f32>>()
                    + mem::size_of::<f32>()) as u32,
            },
        ]
    }
}
