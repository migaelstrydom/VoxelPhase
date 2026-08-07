//! The images and framebuffers a frame is drawn into.
//!
//! Built on top of whatever `FrameOutput` provides, and identical either way:
//! the HDR scene target, the depth buffer and the framebuffers do not care
//! whether the final image ends up on a screen or in a file.

use std::sync::Arc;

use ash::vk;

use crate::core::command_buffer::ManagedCommandBuffer;
use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult};
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::target::images::{ColorTarget, DepthBuffer};
use crate::rendering::target::output::FrameOutput;
use crate::rendering::target::sync::FrameSync;

/// Depth format used by both render passes.
pub const DEPTH_FORMAT: vk::Format = vk::Format::D16_UNORM;

/// Per-frame render resources: what geometry is drawn into, and the
/// synchronization that orders one frame against the next.
pub struct FrameTargets {
    pub extent: vk::Extent2D,

    /// Offscreen HDR colour target for the opaque pass. Resolved to the output
    /// image by the post-processing chain between passes, and sampled by the
    /// water shader for refraction.
    pub color_target: ColorTarget,

    /// Depth buffer shared by both render passes.
    pub depth_buffer: DepthBuffer,

    /// Framebuffer for the opaque render pass (HDR colour target + depth).
    pub opaque_framebuffer: vk::Framebuffer,

    /// One per output image, for the transparent render pass.
    pub transparent_framebuffers: Vec<vk::Framebuffer>,

    pub sync: FrameSync,
    pub draw_command_buffer: ManagedCommandBuffer,

    device: Arc<ManagedDevice>,
}

impl FrameTargets {
    pub fn new(
        vulkan_context: &VulkanContext,
        output: &dyn FrameOutput,
        opaque_renderpass: vk::RenderPass,
        transparent_renderpass: vk::RenderPass,
        scene_color_format: vk::Format,
    ) -> EngineResult<Self> {
        let device = Arc::clone(&vulkan_context.device);
        let extent = output.extent();

        // The scene target is HDR, not the output format: the post-processing
        // resolve tonemaps it down to the displayable range.
        let color_target = ColorTarget::new(vulkan_context, extent, scene_color_format)?;
        let depth_buffer = DepthBuffer::new(vulkan_context, extent, DEPTH_FORMAT)?;

        let opaque_attachments = [color_target.view, depth_buffer.view];
        let opaque_fb_info = vk::FramebufferCreateInfo::default()
            .render_pass(opaque_renderpass)
            .attachments(&opaque_attachments)
            .width(extent.width)
            .height(extent.height)
            .layers(1);

        let opaque_framebuffer = unsafe { device.device.create_framebuffer(&opaque_fb_info, None) }
            .map_err(|e| EngineError::Framebuffer(format!("opaque creation: {:?}", e)))?;

        let transparent_framebuffers = output
            .image_views()
            .iter()
            .map(|&view| {
                let attachments = [view, depth_buffer.view];
                let fb_info = vk::FramebufferCreateInfo::default()
                    .render_pass(transparent_renderpass)
                    .attachments(&attachments)
                    .width(extent.width)
                    .height(extent.height)
                    .layers(1);
                unsafe { device.device.create_framebuffer(&fb_info, None) }
            })
            .collect::<Result<Vec<_>, _>>()
            .map_err(|e| EngineError::Framebuffer(format!("transparent creation: {:?}", e)))?;

        let sync = FrameSync::new(Arc::clone(&device))?;

        let draw_command_buffer = vulkan_context
            .command_buffer_manager
            .create_primary_buffer()?;

        Ok(Self {
            extent,
            color_target,
            depth_buffer,
            opaque_framebuffer,
            transparent_framebuffers,
            sync,
            draw_command_buffer,
            device,
        })
    }

    /// Full-target viewport, for the pipelines that take viewport dynamically.
    pub fn viewport(&self) -> vk::Viewport {
        vk::Viewport {
            x: 0.0,
            y: 0.0,
            width: self.extent.width as f32,
            height: self.extent.height as f32,
            min_depth: 0.0,
            max_depth: 1.0,
        }
    }

    /// Full-target scissor rectangle.
    pub fn scissor(&self) -> vk::Rect2D {
        vk::Rect2D {
            offset: vk::Offset2D { x: 0, y: 0 },
            extent: self.extent,
        }
    }
}

impl Drop for FrameTargets {
    fn drop(&mut self) {
        unsafe {
            self.device
                .device
                .destroy_framebuffer(self.opaque_framebuffer, None);
            for &fb in &self.transparent_framebuffers {
                self.device.device.destroy_framebuffer(fb, None);
            }
        }
    }
}
