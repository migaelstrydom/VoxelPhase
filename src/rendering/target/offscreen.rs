//! Rendering to an image the engine owns, with readback to host memory.
//!
//! The counterpart to `SwapchainOutput`: same render passes, same shaders, same
//! everything — the finished frame simply stays in an image instead of going to
//! a window, and `read_pixels` copies it back so it can be written to a file.
//!
//! This is what lets the visual bench judge the *real* renderer rather than a
//! CPU transcription of it.

use std::sync::Arc;

use ash::vk;

use crate::core::error::EngineResult;
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::frame::ManagedBuffer;
use crate::rendering::target::images::ColorTarget;
use crate::rendering::target::output::{AcquiredFrame, FrameOutput};
use crate::rendering::target::sync::FrameSync;

/// Output format for offscreen rendering.
///
/// sRGB, matching what a window surface gives us, so the hardware applies the
/// same encoding on write and the bytes read back are directly the values a PNG
/// wants. Rendering to a linear format instead would make every offscreen image
/// come out darker than the game, which defeats the purpose of the tool.
const OFFSCREEN_FORMAT: vk::Format = vk::Format::R8G8B8A8_SRGB;

/// Bytes per pixel of `OFFSCREEN_FORMAT`.
const BYTES_PER_PIXEL: u32 = 4;

/// A single engine-owned image that finished frames land in.
///
/// One image rather than a chain: without a presentation engine to hand frames
/// to there is nothing to overlap with, and the caller reads each frame back
/// before rendering the next.
pub struct OffscreenOutput {
    vulkan_context: Arc<VulkanContext>,
    target: ColorTarget,
    extent: vk::Extent2D,

    /// Host-visible destination for `read_pixels`. Allocated once and reused,
    /// since the bench renders many frames at the same size.
    readback: ManagedBuffer,

    images: Vec<vk::Image>,
    image_views: Vec<vk::ImageView>,
}

impl OffscreenOutput {
    pub fn new(vulkan_context: Arc<VulkanContext>, width: u32, height: u32) -> EngineResult<Self> {
        let extent = vk::Extent2D { width, height };
        let target = ColorTarget::new(&vulkan_context, extent, OFFSCREEN_FORMAT)?;

        let readback = ManagedBuffer::new(
            Arc::clone(&vulkan_context.device),
            (width * height * BYTES_PER_PIXEL) as vk::DeviceSize,
            vk::BufferUsageFlags::TRANSFER_DST,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )?;

        let images = vec![target.image];
        let image_views = vec![target.view];

        Ok(Self {
            vulkan_context,
            target,
            extent,
            readback,
            images,
            image_views,
        })
    }

    /// Copy the rendered image back to host memory as tightly packed RGBA8.
    ///
    /// The caller must have finished the frame first — the image is expected to
    /// be in `TRANSFER_SRC_OPTIMAL`, which is where `final_layout` leaves it.
    fn read_pixels_impl(&self) -> EngineResult<Vec<u8>> {
        let cmd_buffer = self
            .vulkan_context
            .command_buffer_manager
            .create_one_time_submit_buffer()?;

        self.vulkan_context
            .command_buffer_manager
            .submit_graphics_commands_and_wait(&cmd_buffer, |device, cb| {
                let region = vk::BufferImageCopy::default()
                    .image_subresource(
                        vk::ImageSubresourceLayers::default()
                            .aspect_mask(vk::ImageAspectFlags::COLOR)
                            .layer_count(1),
                    )
                    .image_extent(self.extent.into());

                unsafe {
                    device.cmd_copy_image_to_buffer(
                        cb,
                        self.target.image,
                        vk::ImageLayout::TRANSFER_SRC_OPTIMAL,
                        self.readback.buffer,
                        &[region],
                    );

                    // Make the transfer write visible to the host map below.
                    let barrier = vk::BufferMemoryBarrier::default()
                        .src_access_mask(vk::AccessFlags::TRANSFER_WRITE)
                        .dst_access_mask(vk::AccessFlags::HOST_READ)
                        .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                        .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                        .buffer(self.readback.buffer)
                        .size(vk::WHOLE_SIZE);

                    device.cmd_pipeline_barrier(
                        cb,
                        vk::PipelineStageFlags::TRANSFER,
                        vk::PipelineStageFlags::HOST,
                        vk::DependencyFlags::empty(),
                        &[],
                        &[barrier],
                        &[],
                    );
                }
            })?;

        let byte_count = (self.extent.width * self.extent.height * BYTES_PER_PIXEL) as usize;

        unsafe {
            let mapped = self.readback.map_memory(0, vk::MemoryMapFlags::empty())?;
            let pixels = std::slice::from_raw_parts(mapped as *const u8, byte_count).to_vec();
            self.readback.unmap_memory();
            Ok(pixels)
        }
    }
}

impl FrameOutput for OffscreenOutput {
    fn extent(&self) -> vk::Extent2D {
        self.extent
    }

    fn format(&self) -> vk::Format {
        OFFSCREEN_FORMAT
    }

    fn images(&self) -> &[vk::Image] {
        &self.images
    }

    fn image_views(&self) -> &[vk::ImageView] {
        &self.image_views
    }

    fn final_layout(&self) -> vk::ImageLayout {
        vk::ImageLayout::TRANSFER_SRC_OPTIMAL
    }

    fn acquire(&self, _sync: &FrameSync) -> EngineResult<AcquiredFrame> {
        // The image is ours and always available, so there is nothing for the
        // submit to wait on and nothing downstream to signal.
        Ok(AcquiredFrame {
            index: 0,
            wait: None,
            signal: None,
        })
    }

    fn release(&self, _frame: &AcquiredFrame, _queue: vk::Queue) -> EngineResult<()> {
        Ok(())
    }

    fn read_pixels(&self) -> EngineResult<Vec<u8>> {
        self.read_pixels_impl()
    }
}
