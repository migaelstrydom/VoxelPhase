use std::sync::Arc;

use crate::core::command_buffer::ManagedCommandBuffer;
use crate::core::error::EngineResult;
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::frame::FrameData;
use crate::rendering::profile::{GpuTimer, RenderProfile};
use crate::rendering::surface_buffer::SurfaceBuffer;
use crate::rendering::target::FrameSync;

/// Everything the renderer writes while recording one frame, and so needs once
/// per frame in flight.
///
/// ```text
///   wait (fence) ─▶ rewind ─▶ record into data, surfaces, command_buffer
///        ▲                                   │
///        └── profile + GPU times ◀── park ◀── submit
/// ```
///
/// A slot is only rewritten after [`Self::wait`], which is what makes it safe
/// for the CPU to fill one slot while the GPU is still drawing another.
pub struct InFlightFrame {
    /// The fence the submission signals, and the acquire semaphore.
    pub sync: FrameSync,
    /// The geometry, post and overlay passes. The shadow pass records into
    /// its own buffer (see `ShadowRenderer`).
    pub command_buffer: ManagedCommandBuffer,
    /// Vertex, index and uniform buffers.
    pub data: FrameData,
    /// Per-draw surface parameters.
    pub surfaces: SurfaceBuffer,
    /// Timestamps the frame's GPU spans.
    pub timer: GpuTimer,
    /// The CPU side of the frame last submitted from this slot, waiting for
    /// its GPU times.
    parked: Option<RenderProfile>,
}

impl InFlightFrame {
    pub fn new(context: &VulkanContext) -> EngineResult<Self> {
        Ok(Self {
            sync: FrameSync::new(Arc::clone(&context.device))?,
            command_buffer: context.command_buffer_manager.create_primary_buffer()?,
            data: FrameData::new(Arc::clone(&context.device))?,
            surfaces: SurfaceBuffer::new(Arc::clone(&context.device))?,
            timer: GpuTimer::new(context)?,
            parked: None,
        })
    }

    /// Block until the GPU has finished the frame last submitted from this
    /// slot, and return that frame's profile completed with its GPU times.
    /// `None` when no frame was parked here.
    pub fn wait(&mut self) -> EngineResult<Option<RenderProfile>> {
        self.sync.wait()?;
        Ok(self.parked.take().map(|mut profile| {
            profile.gpu = self.timer.collect();
            profile
        }))
    }

    /// Rewind the buffers for a new frame. Only after [`Self::wait`].
    pub fn rewind(&mut self) {
        self.data.begin_frame();
        self.surfaces.begin_frame();
    }

    /// Hold the recorded frame's profile until its GPU times can be read.
    pub fn park(&mut self, profile: RenderProfile) {
        self.parked = Some(profile);
    }
}
