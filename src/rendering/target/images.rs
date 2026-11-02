//! Device-local images owned by the frame targets.

use ash::vk;

use crate::core::error::EngineResult;
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::device_image::{DeviceImage, ImageSpec, ImageView};

/// Depth buffer resources.
pub struct DepthBuffer {
    pub view: ImageView,
    pub image: DeviceImage,
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
        let image = DeviceImage::new(
            &vulkan_context.device,
            ImageSpec::new_2d(format, extent, usage),
        )?;

        // A depth attachment starts in the layout the scene pass expects;
        // anything else is transitioned by whoever first writes it.
        if usage.contains(vk::ImageUsageFlags::DEPTH_STENCIL_ATTACHMENT) {
            Self::transition_layout(vulkan_context, image.handle())?;
        }

        let view = image.full_view(vk::ImageViewType::TYPE_2D, vk::ImageAspectFlags::DEPTH)?;
        Ok(Self { view, image })
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
    pub view: ImageView,
    pub image: DeviceImage,
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
        let image = DeviceImage::new(
            &vulkan_context.device,
            ImageSpec::new_2d(format, extent, usage),
        )?;
        let view = image.full_view(vk::ImageViewType::TYPE_2D, vk::ImageAspectFlags::COLOR)?;
        Ok(Self { view, image })
    }
}
