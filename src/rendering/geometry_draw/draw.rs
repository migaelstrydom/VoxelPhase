use ash::vk;
use nalgebra::Matrix4;

use crate::rendering::frame::DrawInfo;
use crate::rendering::surface_buffer::SurfaceIndex;

/// One mesh draw, committed to the frame and ready to record.
///
/// Everything here is plain data that already lives in the frame — the
/// vertices are in its buffers and the shading parameters in its surface
/// table — so a draw can be held back and recorded later with nothing but
/// this struct.
#[derive(Clone, Copy, Debug)]
pub struct GeometryDraw {
    /// Model matrix, pushed as this draw's vertex transform.
    pub model: Matrix4<f32>,

    /// Where this draw's geometry landed in the frame's vertex and index
    /// buffers.
    pub draw: DrawInfo,

    /// Where its shading parameters landed in the frame's surface table.
    pub surface_index: SurfaceIndex,

    /// The descriptor set holding its albedo texture.
    pub texture_set: vk::DescriptorSet,
}

impl GeometryDraw {
    /// The push constants this draw is recorded with: its transform, no colour
    /// override, and its row of the surface table.
    pub fn push(&self) -> GeometryPush {
        GeometryPush {
            model: self.model,
            colour_override: GeometryPush::NO_OVERRIDE,
            surface_index: self.surface_index.0,
        }
    }
}

/// The geometry pipeline's push-constant block, laid out as the shaders read
/// it, so that a draw's constants go down in one push rather than one per
/// field.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct GeometryPush {
    /// Model matrix, read by the vertex stage, and by the fragment stage for
    /// the rotation that places an object-space grain.
    pub model: Matrix4<f32>,

    /// A flat colour that replaces the shaded one when its alpha is non-zero.
    pub colour_override: [f32; 4],

    /// The draw's row in the frame's surface table.
    pub surface_index: u32,
}

impl GeometryPush {
    /// A colour override that leaves the surface shaded normally.
    pub const NO_OVERRIDE: [f32; 4] = [0.0; 4];

    /// Byte offset of `colour_override`, for a push that replaces only it.
    pub const COLOUR_OVERRIDE_OFFSET: u32 = std::mem::offset_of!(Self, colour_override) as u32;

    pub fn as_bytes(&self) -> &[u8] {
        // SAFETY: `repr(C)` over f32 and u32 fields, all 4-byte aligned, so
        // the struct has no padding and every byte is initialised.
        unsafe {
            std::slice::from_raw_parts(
                (self as *const Self).cast::<u8>(),
                std::mem::size_of::<Self>(),
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rendering::material::SURFACE_INDEX_OFFSET;

    /// The block must match the offsets the shaders and the pipeline layout
    /// were built with, or every field past the mismatch reads the wrong bytes.
    #[test]
    fn the_block_matches_the_shader_layout() {
        assert_eq!(GeometryPush::COLOUR_OVERRIDE_OFFSET, 64);
        assert_eq!(
            std::mem::offset_of!(GeometryPush, surface_index) as u32,
            SURFACE_INDEX_OFFSET
        );
        assert_eq!(
            std::mem::size_of::<GeometryPush>() as u32,
            SURFACE_INDEX_OFFSET + 4
        );
    }
}
