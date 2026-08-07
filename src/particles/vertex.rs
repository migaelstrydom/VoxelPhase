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
    /// How the fragment shader shapes and composites this particle, packed as
    /// `(rotation, additive, billow, seed)`.
    ///
    /// Packed into one attribute rather than four because they are only ever
    /// read together, by the fragment shader, and four scalar attributes cost
    /// four binding slots to say the same thing. See the fields of the same
    /// names on [`Particle`](super::particle::Particle).
    pub shape: Vector4<f32>,
}

impl ParticleVertex {
    /// Byte offset of each field, in declaration order.
    const CENTER_OFFSET: u32 = 0;
    const CORNER_OFFSET: u32 = Self::CENTER_OFFSET + mem::size_of::<Vector3<f32>>() as u32;
    const SIZE_OFFSET: u32 = Self::CORNER_OFFSET + mem::size_of::<Vector2<f32>>() as u32;
    const COLOR_OFFSET: u32 = Self::SIZE_OFFSET + mem::size_of::<f32>() as u32;
    const LIFE_OFFSET: u32 = Self::COLOR_OFFSET + mem::size_of::<Vector4<f32>>() as u32;
    const MOTION_OFFSET: u32 = Self::LIFE_OFFSET + mem::size_of::<f32>() as u32;
    const SHAPE_OFFSET: u32 = Self::MOTION_OFFSET + mem::size_of::<Vector3<f32>>() as u32;

    /// Get the Vulkan binding description for this vertex format.
    pub fn binding_description() -> vk::VertexInputBindingDescription {
        vk::VertexInputBindingDescription {
            binding: 0,
            stride: mem::size_of::<Self>() as u32,
            input_rate: vk::VertexInputRate::VERTEX,
        }
    }

    /// Get the Vulkan attribute descriptions for this vertex format.
    pub fn attribute_descriptions() -> [vk::VertexInputAttributeDescription; 7] {
        [
            attribute(0, vk::Format::R32G32B32_SFLOAT, Self::CENTER_OFFSET),
            attribute(1, vk::Format::R32G32_SFLOAT, Self::CORNER_OFFSET),
            attribute(2, vk::Format::R32_SFLOAT, Self::SIZE_OFFSET),
            attribute(3, vk::Format::R32G32B32A32_SFLOAT, Self::COLOR_OFFSET),
            attribute(4, vk::Format::R32_SFLOAT, Self::LIFE_OFFSET),
            attribute(5, vk::Format::R32G32B32_SFLOAT, Self::MOTION_OFFSET),
            attribute(6, vk::Format::R32G32B32A32_SFLOAT, Self::SHAPE_OFFSET),
        ]
    }
}

fn attribute(
    location: u32,
    format: vk::Format,
    offset: u32,
) -> vk::VertexInputAttributeDescription {
    vk::VertexInputAttributeDescription {
        binding: 0,
        location,
        format,
        offset,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The hand-written offsets must match what the compiler actually lays out,
    /// or every attribute past the mistake is read from the wrong bytes — which
    /// shows up as garbled particles rather than as any kind of error.
    #[test]
    fn declared_attribute_offsets_match_the_struct_layout() {
        let vertex = ParticleVertex {
            center: Vector3::zeros(),
            corner: Vector2::zeros(),
            size: 0.0,
            color: Vector4::zeros(),
            life: 0.0,
            motion: Vector3::zeros(),
            shape: Vector4::zeros(),
        };
        let base = &vertex as *const _ as usize;

        let field_offset = |field: *const f32| field as usize - base;

        assert_eq!(
            field_offset(vertex.center.as_ptr()),
            ParticleVertex::CENTER_OFFSET as usize
        );
        assert_eq!(
            field_offset(vertex.corner.as_ptr()),
            ParticleVertex::CORNER_OFFSET as usize
        );
        assert_eq!(
            field_offset(&vertex.size),
            ParticleVertex::SIZE_OFFSET as usize
        );
        assert_eq!(
            field_offset(vertex.color.as_ptr()),
            ParticleVertex::COLOR_OFFSET as usize
        );
        assert_eq!(
            field_offset(&vertex.life),
            ParticleVertex::LIFE_OFFSET as usize
        );
        assert_eq!(
            field_offset(vertex.motion.as_ptr()),
            ParticleVertex::MOTION_OFFSET as usize
        );
        assert_eq!(
            field_offset(vertex.shape.as_ptr()),
            ParticleVertex::SHAPE_OFFSET as usize
        );
    }
}
