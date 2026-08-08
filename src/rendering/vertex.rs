use ash::vk;
use nalgebra::{Vector2, Vector3, Vector4};
use std::mem;

#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct Vertex {
    pub pos: Vector3<f32>,
    pub color: Vector4<f32>,
    pub tex_coords: Vector2<f32>,
    pub normal: Vector3<f32>,
    /// Baked ambient occlusion: `1.0` is fully open, lower is in shade from
    /// nearby geometry. Only terrain bakes it (`terrain::ao`); everything else
    /// carries `1.0`, which is both the neutral value and the honest one.
    pub ao: f32,
}

impl Vertex {
    /// Get the Vulkan vertex attribute descriptions for this vertex format.
    pub fn get_attribute_descriptions() -> [vk::VertexInputAttributeDescription; 5] {
        let [pos, color, tex_coords, normal] = Self::depth_only_attribute_descriptions();
        [
            pos,
            color,
            tex_coords,
            normal,
            // Ambient occlusion (location 4) - float
            vk::VertexInputAttributeDescription {
                location: 4,
                binding: 0,
                format: vk::Format::R32_SFLOAT,
                offset: mem::offset_of!(Vertex, ao) as u32,
            },
        ]
    }

    /// The attributes a depth-only pass consumes. `shadow.vert` writes no
    /// colour, so it declares no ambient occlusion input; handing the shadow
    /// pipeline the full set makes the validator flag location 4 as unconsumed.
    pub fn depth_only_attribute_descriptions() -> [vk::VertexInputAttributeDescription; 4] {
        [
            // Position (location 0) - vec3
            vk::VertexInputAttributeDescription {
                location: 0,
                binding: 0,
                format: vk::Format::R32G32B32_SFLOAT,
                offset: mem::offset_of!(Vertex, pos) as u32,
            },
            // Color (location 1) - vec4
            vk::VertexInputAttributeDescription {
                location: 1,
                binding: 0,
                format: vk::Format::R32G32B32A32_SFLOAT,
                offset: mem::offset_of!(Vertex, color) as u32,
            },
            // Texture coordinates (location 2) - vec2
            vk::VertexInputAttributeDescription {
                location: 2,
                binding: 0,
                format: vk::Format::R32G32_SFLOAT,
                offset: mem::offset_of!(Vertex, tex_coords) as u32,
            },
            // Normal (location 3) - vec3
            vk::VertexInputAttributeDescription {
                location: 3,
                binding: 0,
                format: vk::Format::R32G32B32_SFLOAT,
                offset: mem::offset_of!(Vertex, normal) as u32,
            },
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Ambient occlusion was added by narrowing `pos`, whose fourth component
    /// was `1.0` at every construction site and which both `triangle.vert` and
    /// `shadow.vert` already declared as a `vec3`. Terrain vertex buffers are
    /// re-uploaded every frame, so a vertex that grew would have cost bandwidth
    /// on the largest mesh in the game.
    #[test]
    fn ambient_occlusion_did_not_grow_the_vertex() {
        assert_eq!(mem::size_of::<Vertex>(), 52);
    }
}
