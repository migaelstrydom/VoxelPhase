//! Device-local images owned by the frame targets.

use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult, ImageOperation};
use crate::core::vulkan_context::{find_memorytype_index, VulkanContext};

/// Depth buffer resources.
pub struct DepthBuffer {
    pub image: vk::Image,
    pub view: vk::ImageView,
    pub memory: vk::DeviceMemory,
    device: Arc<ManagedDevice>,
}

impl DepthBuffer {
    /// The scene's depth buffer: tested and written by the scene pass,
    /// sampled after it, and copied for the water to read.
    pub fn new(
        vulkan_context: &VulkanContext,
        extent: vk::Extent2D,
        format: vk::Format,
    ) -> EngineResult<Self> {
        Self::with_usage(
            vulkan_context,
            extent,
            format,
            vk::ImageUsageFlags::DEPTH_STENCIL_ATTACHMENT
                | vk::ImageUsageFlags::SAMPLED
                | vk::ImageUsageFlags::TRANSFER_SRC,
        )
    }

    pub fn with_usage(
        vulkan_context: &VulkanContext,
        extent: vk::Extent2D,
        format: vk::Format,
        usage: vk::ImageUsageFlags,
    ) -> EngineResult<Self> {
        let device = Arc::clone(&vulkan_context.device);

        unsafe {
            let image_info = vk::ImageCreateInfo::default()
                .image_type(vk::ImageType::TYPE_2D)
                .format(format)
                .extent(extent.into())
                .mip_levels(1)
                .array_layers(1)
                .samples(vk::SampleCountFlags::TYPE_1)
                .tiling(vk::ImageTiling::OPTIMAL)
                .usage(usage)
                .sharing_mode(vk::SharingMode::EXCLUSIVE);

            let image =
                device
                    .device
                    .create_image(&image_info, None)
                    .map_err(|e| EngineError::Image {
                        operation: ImageOperation::Create,
                        width: extent.width,
                        height: extent.height,
                        reason: format!("{:?}", e),
                    })?;

            let memory = allocate_and_bind(&device, image, extent)?;

            // A depth attachment starts in the layout the scene pass expects;
            // anything else is transitioned by whoever first writes it.
            if usage.contains(vk::ImageUsageFlags::DEPTH_STENCIL_ATTACHMENT) {
                Self::transition_layout(vulkan_context, image)?;
            }

            let view_info = vk::ImageViewCreateInfo::default()
                .image(image)
                .view_type(vk::ImageViewType::TYPE_2D)
                .format(format)
                .subresource_range(
                    vk::ImageSubresourceRange::default()
                        .aspect_mask(vk::ImageAspectFlags::DEPTH)
                        .level_count(1)
                        .layer_count(1),
                );

            let view = device
                .device
                .create_image_view(&view_info, None)
                .map_err(|e| EngineError::Image {
                    operation: ImageOperation::CreateView,
                    width: extent.width,
                    height: extent.height,
                    reason: format!("{:?}", e),
                })?;

            Ok(Self {
                image,
                view,
                memory,
                device,
            })
        }
    }

    /// Transitions a depth image from `UNDEFINED` to `DEPTH_STENCIL_ATTACHMENT_OPTIMAL` layout.
    ///
    /// In Vulkan, images must be in a specific layout to be used efficiently by the GPU. When an
    /// image is first created, it starts in the `UNDEFINED` layout, which means the image contents
    /// are undefined and the GPU may not access it in a meaningful way. Before using an image as a
    /// depth attachment in a render pass, it must be transitioned to `DEPTH_STENCIL_ATTACHMENT_OPTIMAL`,
    /// which tells the GPU that the image will be used for depth/stencil testing and should be
    /// arranged in memory for optimal depth buffer access.
    ///
    /// # Note
    /// This is a blocking operation that waits for the GPU to complete the transition before
    /// returning. This is safe for initialization code but should be avoided in hot paths.
    fn transition_layout(vulkan_context: &VulkanContext, image: vk::Image) -> EngineResult<()> {
        let cmd_buffer = vulkan_context
            .command_buffer_manager
            .create_one_time_submit_buffer()?;

        vulkan_context
            .command_buffer_manager
            .submit_graphics_commands_and_wait(&cmd_buffer, |device, cb| {
                let barrier = vk::ImageMemoryBarrier::default()
                    .image(image)
                    .dst_access_mask(
                        vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_READ
                            | vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_WRITE,
                    )
                    .new_layout(vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL)
                    .old_layout(vk::ImageLayout::UNDEFINED)
                    .subresource_range(
                        vk::ImageSubresourceRange::default()
                            .aspect_mask(vk::ImageAspectFlags::DEPTH)
                            .layer_count(1)
                            .level_count(1),
                    );

                unsafe {
                    device.cmd_pipeline_barrier(
                        cb,
                        vk::PipelineStageFlags::BOTTOM_OF_PIPE,
                        vk::PipelineStageFlags::LATE_FRAGMENT_TESTS,
                        vk::DependencyFlags::empty(),
                        &[],
                        &[],
                        &[barrier],
                    );
                }
            })
    }
}

impl Drop for DepthBuffer {
    fn drop(&mut self) {
        unsafe {
            self.device.device.destroy_image_view(self.view, None);
            self.device.device.destroy_image(self.image, None);
            self.device.device.free_memory(self.memory, None);
        }
    }
}

/// A colour image the engine owns outright, usable as a render target, as a
/// sampled texture, and as a transfer source.
///
/// Serves two roles. As the *scene* target it holds HDR radiance for the opaque
/// pass, which the post chain resolves and the water shader samples for
/// refraction. As an *output* image (offscreen rendering) it holds the finished
/// LDR frame, which readback copies to host memory. The usage flags are the
/// union of what those two need, which is small enough not to be worth
/// splitting.
pub struct ColorTarget {
    pub image: vk::Image,
    pub view: vk::ImageView,
    pub memory: vk::DeviceMemory,
    device: Arc<ManagedDevice>,
}

impl ColorTarget {
    pub fn new(
        vulkan_context: &VulkanContext,
        extent: vk::Extent2D,
        format: vk::Format,
    ) -> EngineResult<Self> {
        Self::with_usage(
            vulkan_context,
            extent,
            format,
            vk::ImageUsageFlags::COLOR_ATTACHMENT
                | vk::ImageUsageFlags::TRANSFER_SRC
                | vk::ImageUsageFlags::SAMPLED,
        )
    }

    pub fn with_usage(
        vulkan_context: &VulkanContext,
        extent: vk::Extent2D,
        format: vk::Format,
        usage: vk::ImageUsageFlags,
    ) -> EngineResult<Self> {
        let device = Arc::clone(&vulkan_context.device);

        unsafe {
            let image_info = vk::ImageCreateInfo::default()
                .image_type(vk::ImageType::TYPE_2D)
                .format(format)
                .extent(extent.into())
                .mip_levels(1)
                .array_layers(1)
                .samples(vk::SampleCountFlags::TYPE_1)
                .tiling(vk::ImageTiling::OPTIMAL)
                .usage(usage)
                .sharing_mode(vk::SharingMode::EXCLUSIVE);

            let image =
                device
                    .device
                    .create_image(&image_info, None)
                    .map_err(|e| EngineError::Image {
                        operation: ImageOperation::Create,
                        width: extent.width,
                        height: extent.height,
                        reason: format!("{:?}", e),
                    })?;

            let memory = allocate_and_bind(&device, image, extent)?;

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
                .map_err(|e| EngineError::Image {
                    operation: ImageOperation::CreateView,
                    width: extent.width,
                    height: extent.height,
                    reason: format!("{:?}", e),
                })?;

            Ok(Self {
                image,
                view,
                memory,
                device,
            })
        }
    }
}

impl Drop for ColorTarget {
    fn drop(&mut self) {
        unsafe {
            self.device.device.destroy_image_view(self.view, None);
            self.device.device.destroy_image(self.image, None);
            self.device.device.free_memory(self.memory, None);
        }
    }
}

/// Allocate device-local memory for `image` and bind it.
///
/// `extent` is carried only so a failure can report which image it was.
pub(crate) fn allocate_and_bind(
    device: &ManagedDevice,
    image: vk::Image,
    extent: vk::Extent2D,
) -> EngineResult<vk::DeviceMemory> {
    unsafe {
        let memory_req = device.device.get_image_memory_requirements(image);
        let memory_type_index = find_memorytype_index(
            &memory_req,
            &device.device_memory_properties,
            vk::MemoryPropertyFlags::DEVICE_LOCAL,
        )
        .ok_or_else(|| EngineError::Image {
            operation: ImageOperation::AllocateMemory,
            width: extent.width,
            height: extent.height,
            reason: "no suitable memory type".to_string(),
        })?;

        let alloc_info = vk::MemoryAllocateInfo::default()
            .allocation_size(memory_req.size)
            .memory_type_index(memory_type_index);

        let memory = device
            .device
            .allocate_memory(&alloc_info, None)
            .map_err(|e| EngineError::Image {
                operation: ImageOperation::AllocateMemory,
                width: extent.width,
                height: extent.height,
                reason: format!("{:?}", e),
            })?;

        device
            .device
            .bind_image_memory(image, memory, 0)
            .map_err(|e| EngineError::Image {
                operation: ImageOperation::Bind,
                width: extent.width,
                height: extent.height,
                reason: format!("{:?}", e),
            })?;

        Ok(memory)
    }
}
