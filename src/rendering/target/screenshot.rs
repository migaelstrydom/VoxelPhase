//! A finished frame kept as a picture, on request.

use std::sync::Arc;

use ash::vk;
use image::RgbaImage;

use crate::core::error::{EngineError, EngineResult};
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::frame::ManagedBuffer;
use crate::rendering::target::output::{AcquiredFrame, FrameOutput};
use crate::rendering::target::readback::{read_back_image, BYTES_PER_PIXEL};
use crate::rendering::target::sync::FrameSync;

/// Takes the next finished frame as a picture once asked to: read back from
/// the output image after the frame's draw and before it is handed off, so a
/// window's frame is caught before it goes to the screen.
#[derive(Default)]
pub struct Screenshot {
    requested: bool,
    taken: Option<RgbaImage>,
}

impl Screenshot {
    /// Takes the next frame to finish.
    pub fn request(&mut self) {
        self.requested = true;
    }

    /// The frame taken since the last call, if one was.
    pub fn take(&mut self) -> Option<RgbaImage> {
        self.taken.take()
    }

    /// Reads `frame` back from `output` if a picture was asked for: waits for
    /// its draw (`sync`), which stalls this one frame.
    pub fn capture(
        &mut self,
        vulkan_context: &Arc<VulkanContext>,
        output: &dyn FrameOutput,
        frame: &AcquiredFrame,
        sync: &FrameSync,
    ) -> EngineResult<()> {
        if !std::mem::take(&mut self.requested) {
            return Ok(());
        }
        sync.wait()?;
        let extent = output.extent();
        let buffer = ManagedBuffer::new(
            Arc::clone(&vulkan_context.device),
            (extent.width * extent.height * BYTES_PER_PIXEL) as vk::DeviceSize,
            vk::BufferUsageFlags::TRANSFER_DST,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )?;
        let mut pixels = read_back_image(
            vulkan_context,
            output.images()[frame.index as usize],
            output.final_layout(),
            extent,
            &buffer,
        )?;
        if matches!(
            output.format(),
            vk::Format::B8G8R8A8_SRGB | vk::Format::B8G8R8A8_UNORM
        ) {
            for pixel in pixels.chunks_exact_mut(BYTES_PER_PIXEL as usize) {
                pixel.swap(0, 2);
            }
        }
        // A window's alpha is whatever the passes left; the picture is opaque.
        for pixel in pixels.chunks_exact_mut(BYTES_PER_PIXEL as usize) {
            pixel[3] = u8::MAX;
        }
        self.taken = Some(
            RgbaImage::from_raw(extent.width, extent.height, pixels).ok_or_else(|| {
                EngineError::InvalidState("readback produced the wrong number of pixels".into())
            })?,
        );
        Ok(())
    }
}
