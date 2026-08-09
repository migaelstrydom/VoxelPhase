//! Per-frame GPU synchronization primitives.

use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult, VkResultExt};

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
    /// How long `wait` blocks before declaring the frame lost. Generous enough
    /// that a heavy frame or a slow vsync never trips it.
    const FENCE_TIMEOUT_NS: u64 = 2_000_000_000;

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

    /// Wait for the draw fence, without resetting it.
    ///
    /// Idempotent by design: the fence is only reset in `reset()`, immediately
    /// before a submit that will signal it again. A frame that is abandoned
    /// between begin and submit therefore leaves the fence signalled, and the
    /// next wait returns straight away instead of blocking on a signal that is
    /// never coming.
    ///
    /// The timeout is deliberately finite. An infinite wait turns any lost
    /// submit into a silent, undebuggable freeze of the whole event loop;
    /// a bounded one surfaces it as an error with a log line naming the cause.
    pub fn wait(&self) -> EngineResult<()> {
        unsafe {
            match self.device.device.wait_for_fences(
                &[self.draw_fence],
                true,
                Self::FENCE_TIMEOUT_NS,
            ) {
                Ok(()) => Ok(()),
                Err(vk::Result::TIMEOUT) => {
                    log::error!(
                        "Draw fence still unsignalled after {}s — a submit was lost, \
                         or the GPU is hung. Abandoning this frame.",
                        Self::FENCE_TIMEOUT_NS / 1_000_000_000
                    );
                    Err(EngineError::Synchronization(
                        "draw fence wait timed out".to_string(),
                    ))
                }
                Err(e) => Err(e).sync_context("wait for draw fence"),
            }
        }
    }

    /// Reset the draw fence. Call immediately before the submit that signals it
    /// — never at the top of a frame, where an early return would strand it.
    pub fn reset(&self) -> EngineResult<()> {
        unsafe {
            self.device
                .device
                .reset_fences(&[self.draw_fence])
                .sync_context("reset draw fence")
        }
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
