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
use crate::rendering::target::readback::{read_back_image, BYTES_PER_PIXEL};
use crate::rendering::target::sync::FrameSync;

/// Output format for offscreen rendering.
///
/// sRGB, matching what a window surface gives us, so the hardware applies the
/// same encoding on write and the bytes read back are directly the values a PNG
/// wants. Rendering to a linear format instead would make every offscreen image
/// come out darker than the game, which defeats the purpose of the tool.
const OFFSCREEN_FORMAT: vk::Format = vk::Format::R8G8B8A8_SRGB;

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

        let images = vec![target.image.handle()];
        let image_views = vec![target.view.handle()];

        Ok(Self {
            vulkan_context,
            target,
            extent,
            readback,
            images,
            image_views,
        })
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
        // The image is left in `TRANSFER_SRC_OPTIMAL` by `final_layout`.
        read_back_image(
            &self.vulkan_context,
            self.target.image.handle(),
            self.final_layout(),
            self.extent,
            &self.readback,
        )
    }
}
