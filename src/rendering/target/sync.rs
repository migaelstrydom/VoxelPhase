//! Per-frame GPU synchronization primitives.

use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineResult, VkResultExt};

/// Frame synchronization primitives.
///
/// The two semaphores are only meaningful for outputs whose images become
/// available asynchronously — a swapchain acquire. An offscreen output owns its
/// images outright and leaves both unused, waiting on `draw_fence` alone.
pub struct FrameSync {
    pub present_complete: vk::Semaphore,
    pub rendering_complete: vk::Semaphore,
    pub draw_fence: vk::Fence,
    device: Arc<ManagedDevice>,
}

impl FrameSync {
    pub fn new(device: Arc<ManagedDevice>) -> EngineResult<Self> {
        let semaphore_info = vk::SemaphoreCreateInfo::default();
        let fence_info = vk::FenceCreateInfo::default().flags(vk::FenceCreateFlags::SIGNALED);

        unsafe {
            let present_complete = device
                .device
                .create_semaphore(&semaphore_info, None)
                .sync_context("create present semaphore")?;

            let rendering_complete = device
                .device
                .create_semaphore(&semaphore_info, None)
                .sync_context("create rendering semaphore")?;

            let draw_fence = device
                .device
                .create_fence(&fence_info, None)
                .sync_context("create draw fence")?;

            Ok(Self {
                present_complete,
                rendering_complete,
                draw_fence,
                device,
            })
        }
    }

    /// Wait for the draw fence and reset it.
    pub fn wait_and_reset(&self) -> EngineResult<()> {
        unsafe {
            self.device
                .device
                .wait_for_fences(&[self.draw_fence], true, u64::MAX)
                .sync_context("wait for draw fence")?;

            self.device
                .device
                .reset_fences(&[self.draw_fence])
                .sync_context("reset draw fence")?;
        }
        Ok(())
    }

    /// Wait for the draw fence without resetting it.
    ///
    /// Used by offscreen readback, which needs the frame to have finished
    /// before it maps the destination buffer, and which is followed by another
    /// `wait_and_reset` at the start of the next frame.
    pub fn wait(&self) -> EngineResult<()> {
        unsafe {
            self.device
                .device
                .wait_for_fences(&[self.draw_fence], true, u64::MAX)
                .sync_context("wait for draw fence")?;
        }
        Ok(())
    }
}

impl Drop for FrameSync {
    fn drop(&mut self) {
        unsafe {
            self.device
                .device
                .destroy_semaphore(self.present_complete, None);
            self.device
                .device
                .destroy_semaphore(self.rendering_complete, None);
            self.device.device.destroy_fence(self.draw_fence, None);
        }
    }
}
