//! Offscreen render target used by the post-processing chain.

use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult};
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::device_image::{DeviceImage, ImageSpec, ImageView};

/// A colour-only offscreen image that can be both rendered to and sampled,
/// with a framebuffer for rendering into it.
pub struct PostTarget {
    pub framebuffer: vk::Framebuffer,
    pub view: ImageView,
    pub image: DeviceImage,
    pub extent: vk::Extent2D,
    device: Arc<ManagedDevice>,
}

impl PostTarget {
    /// Allocate a target of the given size and format, with a framebuffer
    /// compatible with `render_pass`.
    pub fn new(
        vulkan_context: &VulkanContext,
        extent: vk::Extent2D,
        format: vk::Format,
        render_pass: vk::RenderPass,
    ) -> EngineResult<Self> {
        let device = Arc::clone(&vulkan_context.device);
        let image = DeviceImage::new(
            &device,
            ImageSpec::new_2d(
                format,
                extent,
                vk::ImageUsageFlags::COLOR_ATTACHMENT | vk::ImageUsageFlags::SAMPLED,
            ),
        )?;
        let view = image.full_view(vk::ImageViewType::TYPE_2D, vk::ImageAspectFlags::COLOR)?;

        let attachments = [view.handle()];
        let framebuffer_info = vk::FramebufferCreateInfo::default()
            .render_pass(render_pass)
            .attachments(&attachments)
            .width(extent.width)
            .height(extent.height)
            .layers(1);
        let framebuffer = unsafe { device.device.create_framebuffer(&framebuffer_info, None) }
            .map_err(|e| EngineError::Pipeline(format!("post framebuffer: {:?}", e)))?;

        Ok(Self {
            framebuffer,
            view,
            image,
            extent,
            device,
        })
    }
}

impl Drop for PostTarget {
    fn drop(&mut self) {
        unsafe {
            self.device
                .device
                .destroy_framebuffer(self.framebuffer, None);
        }
    }
}
