//! Overlay rendering module for debug text and UI elements.
//!
//! Provides 2D overlay rendering capabilities using a dedicated pipeline
//! with alpha blending for text and UI elements.

use std::sync::Arc;

use ash::vk;
use nalgebra::{Matrix4, Vector2, Vector4};

use crate::core::device::ManagedDevice;
use crate::core::error::EngineResult;
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::frame::ManagedBuffer;
use crate::rendering::overlay::font::{FontAtlas, TextLayout};
use crate::rendering::overlay::geometry::OverlayGeometry;
use crate::rendering::overlay::pipeline::OverlayPipeline;
use crate::rendering::overlay::OverlayVertex;

/// Maximum quads the overlay can draw in one frame. Covers the debug text and
/// every HUD element together, since they share one buffer and one draw.
const MAX_OVERLAY_QUADS: usize = 1024;

/// Overlay renderer for debug text and UI elements.
///
/// Renders 2D elements on top of the 3D scene using a separate pipeline
/// with alpha blending and no depth testing.
pub struct OverlayRenderer {
    device: Arc<ManagedDevice>,
    pipeline: OverlayPipeline,
    font_atlas: FontAtlas,
    ortho_matrix: Matrix4<f32>,
    screen_size: Vector2<f32>,
    vertex_buffer: ManagedBuffer,
    index_buffer: ManagedBuffer,
}

impl OverlayRenderer {
    /// Create a new overlay renderer.
    ///
    /// # Arguments
    /// * `vulkan_context` - The Vulkan context
    /// * `render_pass` - The render pass to use (must be compatible with main scene render pass)
    /// * `width` - Screen width in pixels
    /// * `height` - Screen height in pixels
    pub fn new(
        vulkan_context: Arc<VulkanContext>,
        render_pass: vk::RenderPass,
        width: u32,
        height: u32,
    ) -> EngineResult<Self> {
        let device = Arc::clone(&vulkan_context.device);

        // Orthographic projection: (0,0) top-left, (width, height) bottom-right
        let ortho_matrix =
            Matrix4::new_orthographic(0.0, width as f32, 0.0, height as f32, -1.0, 1.0);

        let font_atlas = FontAtlas::new(Arc::clone(&vulkan_context))?;

        let pipeline = OverlayPipeline::new(
            Arc::clone(&device),
            render_pass,
            font_atlas.descriptor_set_layout(),
        )?;

        // Buffer sizes for MAX_OVERLAY_QUADS quads (4 verts, 6 indices each)
        let vertex_buffer_size =
            (MAX_OVERLAY_QUADS * 4 * std::mem::size_of::<OverlayVertex>()) as vk::DeviceSize;
        let index_buffer_size =
            (MAX_OVERLAY_QUADS * 6 * std::mem::size_of::<u32>()) as vk::DeviceSize;

        let vertex_buffer = ManagedBuffer::new(
            Arc::clone(&device),
            vertex_buffer_size,
            vk::BufferUsageFlags::VERTEX_BUFFER,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )?;

        let index_buffer = ManagedBuffer::new(
            Arc::clone(&device),
            index_buffer_size,
            vk::BufferUsageFlags::INDEX_BUFFER,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )?;

        Ok(Self {
            device,
            pipeline,
            font_atlas,
            ortho_matrix,
            screen_size: Vector2::new(width as f32, height as f32),
            vertex_buffer,
            index_buffer,
        })
    }

    /// Screen size the overlay's orthographic projection was built for, in
    /// pixels. HUD elements size themselves against this.
    pub fn screen_size(&self) -> Vector2<f32> {
        self.screen_size
    }

    /// UV of a fully-opaque texel, for drawing filled shapes through the same
    /// text pipeline.
    pub fn solid_uv(&self) -> Vector2<f32> {
        self.font_atlas.solid_uv()
    }

    /// Lay out debug key/value lines as overlay geometry.
    ///
    /// Laying out and drawing are separate so that a frame can gather text and
    /// HUD shapes into one batch before any of it is uploaded.
    pub fn layout_debug_lines<'a>(
        &self,
        entries: impl Iterator<Item = (&'a str, &'a str)>,
    ) -> OverlayGeometry {
        let line_height = self.font_atlas.line_height();
        let x = 10.0;
        let mut y = 10.0;
        let colour = Vector4::new(1.0, 1.0, 0.0, 1.0);

        let mut geometry = OverlayGeometry::new();
        for (key, value) in entries {
            let line = format!("{}: {}", key, value);
            let (vertices, indices) =
                TextLayout::layout_text(&self.font_atlas, &line, x, y, colour);
            geometry.extend(&vertices, &indices);
            y += line_height;
        }
        geometry
    }

    /// Upload and draw a frame's overlay geometry in one batch.
    ///
    /// Geometry past the buffer's capacity is dropped rather than written past
    /// the end of it; whole quads are kept, so nothing is ever drawn with half
    /// its vertices.
    pub fn render(
        &mut self,
        cb: vk::CommandBuffer,
        geometry: &OverlayGeometry,
    ) -> EngineResult<()> {
        if geometry.is_empty() {
            return Ok(());
        }

        let quads = (geometry.indices().len() / 6).min(MAX_OVERLAY_QUADS);
        let index_count = quads * 6;
        let vertex_count = geometry.vertices().len().min(MAX_OVERLAY_QUADS * 4);

        self.upload_geometry(
            &geometry.vertices()[..vertex_count],
            &geometry.indices()[..index_count],
        )?;
        self.record_draw_commands(cb, index_count as u32);

        Ok(())
    }

    fn upload_geometry(&self, vertices: &[OverlayVertex], indices: &[u32]) -> EngineResult<()> {
        unsafe {
            let vert_ptr = self
                .vertex_buffer
                .map_memory(0, vk::MemoryMapFlags::empty())?;
            std::ptr::copy_nonoverlapping(
                vertices.as_ptr(),
                vert_ptr as *mut OverlayVertex,
                vertices.len(),
            );
            self.vertex_buffer.unmap_memory();

            let idx_ptr = self
                .index_buffer
                .map_memory(0, vk::MemoryMapFlags::empty())?;
            std::ptr::copy_nonoverlapping(indices.as_ptr(), idx_ptr as *mut u32, indices.len());
            self.index_buffer.unmap_memory();
        }
        Ok(())
    }

    fn record_draw_commands(&self, cb: vk::CommandBuffer, index_count: u32) {
        unsafe {
            self.device.device.cmd_bind_pipeline(
                cb,
                vk::PipelineBindPoint::GRAPHICS,
                self.pipeline.pipeline(),
            );

            let ortho_bytes: &[u8] = std::slice::from_raw_parts(
                self.ortho_matrix.as_ptr() as *const u8,
                std::mem::size_of::<Matrix4<f32>>(),
            );
            self.device.device.cmd_push_constants(
                cb,
                self.pipeline.layout(),
                vk::ShaderStageFlags::VERTEX,
                0,
                ortho_bytes,
            );

            self.device.device.cmd_bind_descriptor_sets(
                cb,
                vk::PipelineBindPoint::GRAPHICS,
                self.pipeline.layout(),
                0,
                &[self.font_atlas.descriptor_set()],
                &[],
            );

            self.device
                .device
                .cmd_bind_vertex_buffers(cb, 0, &[self.vertex_buffer.buffer], &[0]);
            self.device.device.cmd_bind_index_buffer(
                cb,
                self.index_buffer.buffer,
                0,
                vk::IndexType::UINT32,
            );

            self.device
                .device
                .cmd_draw_indexed(cb, index_count, 1, 0, 0, 0);
        }
    }
}
