use ash::vk;
use nalgebra::Vector2;

/// A water surface vertex. Its height is not stored: every draw pushes its
/// body's level, so the mesh outlives a change of level.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BasinVertex {
    /// World (x, z).
    pub xz: Vector2<f32>,
    /// The floor under this vertex, for depth tint and swell attenuation.
    pub floor: f32,
}

impl BasinVertex {
    pub fn binding_description() -> vk::VertexInputBindingDescription {
        vk::VertexInputBindingDescription {
            binding: 0,
            stride: std::mem::size_of::<Self>() as u32,
            input_rate: vk::VertexInputRate::VERTEX,
        }
    }

    pub fn attribute_descriptions() -> [vk::VertexInputAttributeDescription; 2] {
        [
            vk::VertexInputAttributeDescription {
                location: 0,
                binding: 0,
                format: vk::Format::R32G32_SFLOAT,
                offset: 0,
            },
            vk::VertexInputAttributeDescription {
                location: 1,
                binding: 0,
                format: vk::Format::R32_SFLOAT,
                offset: std::mem::size_of::<Vector2<f32>>() as u32,
            },
        ]
    }
}
