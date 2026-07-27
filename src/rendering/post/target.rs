//! Offscreen render target used by the post-processing chain.

use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult, ImageOperation};
use crate::core::vulkan_context::{find_memorytype_index, VulkanContext};

/// A colour-only offscreen image that can be both rendered to and sampled.
///
/// Owns its image, memory, view and framebuffer, and releases all four on drop.
pub struct PostTarget {
    pub image: vk::Image,
    pub view: vk::ImageView,
    pub framebuffer: vk::Framebuffer,
    pub extent: vk::Extent2D,
    memory: vk::DeviceMemory,
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

        let image_error = |operation: ImageOperation, reason: String| EngineError::Image {
            operation,
            width: extent.width,
            height: extent.height,
            reason,
        };

        unsafe {
            let image_info = vk::ImageCreateInfo::default()
                .image_type(vk::ImageType::TYPE_2D)
                .format(format)
                .extent(extent.into())
                .mip_levels(1)
                .array_layers(1)
                .samples(vk::SampleCountFlags::TYPE_1)
                .tiling(vk::ImageTiling::OPTIMAL)
                .usage(vk::ImageUsageFlags::COLOR_ATTACHMENT | vk::ImageUsageFlags::SAMPLED)
                .sharing_mode(vk::SharingMode::EXCLUSIVE);

            let image = device
                .device
                .create_image(&image_info, None)
                .map_err(|e| image_error(ImageOperation::Create, format!("{:?}", e)))?;

            let memory_req = device.device.get_image_memory_requirements(image);
            let memory_type_index = find_memorytype_index(
                &memory_req,
                &device.device_memory_properties,
                vk::MemoryPropertyFlags::DEVICE_LOCAL,
            )
            .ok_or_else(|| {
                image_error(
                    ImageOperation::AllocateMemory,
                    "no suitable memory type".to_string(),
                )
            })?;

            let alloc_info = vk::MemoryAllocateInfo::default()
                .allocation_size(memory_req.size)
                .memory_type_index(memory_type_index);

            let memory = device
                .device
                .allocate_memory(&alloc_info, None)
                .map_err(|e| image_error(ImageOperation::AllocateMemory, format!("{:?}", e)))?;

            device
                .device
                .bind_image_memory(image, memory, 0)
                .map_err(|e| image_error(ImageOperation::Bind, format!("{:?}", e)))?;

            let view_info = vk::ImageViewCreateInfo::default()
                .image(image)
                .view_type(vk::ImageViewType::TYPE_2D)
                .format(format)
                .subresource_range(
                    vk::ImageSubresourceRange::default()
                        .aspect_mask(vk::ImageAspectFlags::COLOR)
                        .level_count(1)
                        .layer_count(1),
                );

            let view = device
                .device
                .create_image_view(&view_info, None)
                .map_err(|e| image_error(ImageOperation::CreateView, format!("{:?}", e)))?;

            let framebuffer_info = vk::FramebufferCreateInfo::default()
                .render_pass(render_pass)
                .attachments(std::slice::from_ref(&view))
                .width(extent.width)
                .height(extent.height)
                .layers(1);

            let framebuffer = device
                .device
                .create_framebuffer(&framebuffer_info, None)
                .map_err(|e| EngineError::Pipeline(format!("post framebuffer: {:?}", e)))?;

            Ok(Self {
                image,
                view,
                framebuffer,
                extent,
                memory,
                device,
            })
        }
    }
}

impl Drop for PostTarget {
    fn drop(&mut self) {
        unsafe {
            self.device
                .device
                .destroy_framebuffer(self.framebuffer, None);
            self.device.device.destroy_image_view(self.view, None);
            self.device.device.destroy_image(self.image, None);
            self.device.device.free_memory(self.memory, None);
        }
    }
}
