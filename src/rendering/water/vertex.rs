use ash::vk;
use nalgebra::Vector3;

/// Vertex for water mesh rendering.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct WaterVertex {
    pub position: Vector3<f32>,
    pub normal: Vector3<f32>,
}

impl WaterVertex {
    pub fn binding_description() -> vk::VertexInputBindingDescription {
        vk::VertexInputBindingDescription {
            binding: 0,
            stride: std::mem::size_of::<Self>() as u32,
            input_rate: vk::VertexInputRate::VERTEX,
        }
    }

    pub fn attribute_descriptions() -> [vk::VertexInputAttributeDescription; 2] {
        [
            // position: vec3
            vk::VertexInputAttributeDescription {
                location: 0,
                binding: 0,
                format: vk::Format::R32G32B32_SFLOAT,
                offset: 0,
            },
            // normal: vec3
            vk::VertexInputAttributeDescription {
                location: 1,
                binding: 0,
                format: vk::Format::R32G32B32_SFLOAT,
                offset: std::mem::size_of::<Vector3<f32>>() as u32,
            },
        ]
    }
}
