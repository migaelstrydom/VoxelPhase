//! Where a finished frame goes.
//!
//! The renderer draws into `FrameTargets` and then hands the result to a
//! `FrameOutput`. Whether that means presenting to a window or leaving the
//! pixels in an image for readback is the only difference between running the
//! game and running the visual bench.

use ash::vk;

use crate::core::error::{EngineError, EngineResult};
use crate::rendering::target::sync::FrameSync;

/// The output image chosen for the current frame.
pub struct AcquiredFrame {
    /// Index into the output's image array.
    pub index: u32,

    /// Semaphore the submit must wait on before writing colour, if the image
    /// only becomes available asynchronously. `None` for outputs that own their
    /// images outright, where the draw fence alone is sufficient.
    pub wait: Option<vk::Semaphore>,

    /// Semaphore the submit signals when rendering completes, for a consumer
    /// that runs on the GPU timeline. `None` when nothing waits on the GPU side.
    pub signal: Option<vk::Semaphore>,
}

/// A sequence of images the renderer writes finished frames into.
///
/// Implementors own the images and the rules for taking and returning one. They
/// deliberately own nothing else: the depth buffer, HDR scene target and
/// framebuffers all belong to `FrameTargets`, which is built on top of whatever
/// this hands it.
///
/// `Send + Sync` because the renderer lives in the ECS world as a resource, and
/// specs requires its resources to be both.
pub trait FrameOutput: Send + Sync {
    fn extent(&self) -> vk::Extent2D;

    /// Pixel format of the output images. Drives the render-pass attachment
    /// formats of every stage that writes to them.
    fn format(&self) -> vk::Format;

    fn images(&self) -> &[vk::Image];

    fn image_views(&self) -> &[vk::ImageView];

    /// Layout the last pass of the frame must leave the output image in.
    ///
    /// `PRESENT_SRC_KHR` for a swapchain, `TRANSFER_SRC_OPTIMAL` for an image
    /// about to be copied back to host memory. The bloom overlay render pass is
    /// built against this, which is why it is a property of the output rather
    /// than a constant.
    fn final_layout(&self) -> vk::ImageLayout;

    /// Choose the image for this frame.
    fn acquire(&self, sync: &FrameSync) -> EngineResult<AcquiredFrame>;

    /// Hand the finished image off. Presents, or does nothing.
    fn release(&self, frame: &AcquiredFrame, queue: vk::Queue) -> EngineResult<()>;

    /// Copy the finished frame back to host memory as tightly packed RGBA8.
    ///
    /// Only outputs the engine owns can do this; a presentation output has
    /// handed its image to the windowing system and errors instead. The caller
    /// must have waited for the frame to complete first.
    fn read_pixels(&self) -> EngineResult<Vec<u8>> {
        Err(EngineError::InvalidState(
            "this frame output does not support readback".to_string(),
        ))
    }
}
