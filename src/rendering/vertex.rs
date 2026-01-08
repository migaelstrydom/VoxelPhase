use ash::vk;
use nalgebra::{Vector2, Vector3, Vector4};
use std::mem;

#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct Vertex {
    pub pos: Vector4<f32>,
    pub color: Vector4<f32>,
    pub tex_coords: Vector2<f32>,
    pub normal: Vector3<f32>,
}

impl Vertex {
    /// Get the Vulkan vertex attribute descriptions for this vertex format.
    pub fn get_attribute_descriptions() -> [vk::VertexInputAttributeDescription; 4] {
        [
            // Position (location 0) - vec4
            vk::VertexInputAttributeDescription {
                location: 0,
                binding: 0,
                format: vk::Format::R32G32B32A32_SFLOAT,
                offset: 0,
            },
            // Color (location 1) - vec4
            vk::VertexInputAttributeDescription {
                location: 1,
                binding: 0,
                format: vk::Format::R32G32B32A32_SFLOAT,
                offset: mem::size_of::<Vector4<f32>>() as u32,
            },
            // Texture coordinates (location 2) - vec2
            vk::VertexInputAttributeDescription {
                location: 2,
                binding: 0,
                format: vk::Format::R32G32_SFLOAT,
                offset: (mem::size_of::<Vector4<f32>>() * 2) as u32,
            },
            // Normal (location 3) - vec3
            vk::VertexInputAttributeDescription {
                location: 3,
                binding: 0,
                format: vk::Format::R32G32B32_SFLOAT,
                offset: (mem::size_of::<Vector4<f32>>() * 2 + mem::size_of::<Vector2<f32>>())
                    as u32,
            },
        ]
    }
}
