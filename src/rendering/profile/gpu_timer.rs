use std::sync::Arc;
use std::time::Duration;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult};
use crate::core::vulkan_context::VulkanContext;

use super::gpu_span::GpuSpan;

/// How long each span of one frame took on the GPU.
#[derive(Debug, Clone, Copy, Default)]
pub struct GpuTimings {
    /// Time per span, indexed by position in [`GpuSpan::ALL`]. `None` for a
    /// span whose timestamps never landed.
    spans: [Option<Duration>; GpuSpan::COUNT],
    /// From the first span opening to the last closing: the frame's whole GPU
    /// time, gaps between spans included.
    pub total: Option<Duration>,
}

impl GpuTimings {
    pub fn span(&self, span: GpuSpan) -> Option<Duration> {
        self.spans[span as usize]
    }

    /// Every span with its time, in the order the GPU runs them.
    pub fn iter(&self) -> impl Iterator<Item = (GpuSpan, Option<Duration>)> + '_ {
        GpuSpan::ALL.iter().map(|&span| (span, self.span(span)))
    }
}

/// Times a frame's GPU spans with timestamp queries.
///
/// ```text
///   open_frame (reset) ─▶ begin/end per span ─▶ submit ─▶ mark_submitted
///                                                              │
///   next begin_frame, after the fence ─▶ collect ◀─────────────┘
/// ```
///
/// Read back only once the frame's fence has signalled, so collecting never
/// stalls: every query the frame wrote is already available. On a device that
/// cannot time the graphics queue the timer records nothing and collects
/// `None`, and every call stays safe to make.
pub struct GpuTimer {
    device: Arc<ManagedDevice>,
    /// `None` when the graphics queue has no timestamp support.
    pool: Option<vk::QueryPool>,
    /// Nanoseconds per timestamp tick.
    tick_ns: f64,
    /// The bits of a timestamp the device actually fills.
    valid_mask: u64,
    /// Whether the queries were submitted since the last collect. Results of
    /// a pool that was never reset on the GPU are undefined, so without a
    /// submission there is nothing to read.
    submitted: bool,
}

impl GpuTimer {
    pub fn new(context: &VulkanContext) -> EngineResult<Self> {
        let device = Arc::clone(&context.device);
        let instance = &context.instance.instance;

        let (valid_bits, tick_ns) = unsafe {
            let families =
                instance.get_physical_device_queue_family_properties(device.physical_device);
            let graphics = device.queue_family_indices.graphics as usize;
            let valid_bits = families
                .get(graphics)
                .map_or(0, |family| family.timestamp_valid_bits);
            let limits = instance
                .get_physical_device_properties(device.physical_device)
                .limits;
            (valid_bits, limits.timestamp_period as f64)
        };

        let pool = if valid_bits == 0 {
            log::warn!("GpuTimer: graphics queue has no timestamps; GPU times will be absent");
            None
        } else {
            let create_info = vk::QueryPoolCreateInfo::default()
                .query_type(vk::QueryType::TIMESTAMP)
                .query_count(GpuSpan::QUERY_COUNT);
            let pool = unsafe { device.device.create_query_pool(&create_info, None) }
                .map_err(|e| EngineError::Query(format!("timestamp query pool: {:?}", e)))?;
            Some(pool)
        };

        let valid_mask = if valid_bits >= 64 {
            u64::MAX
        } else {
            (1u64 << valid_bits) - 1
        };

        Ok(Self {
            device,
            pool,
            tick_ns,
            valid_mask,
            submitted: false,
        })
    }

    /// Rewind the queries for a new frame.
    ///
    /// Must be recorded outside any render pass, into the first command buffer
    /// of the frame's submission, before any span opens.
    pub fn open_frame(&self, cb: vk::CommandBuffer) {
        if let Some(pool) = self.pool {
            unsafe {
                self.device
                    .device
                    .cmd_reset_query_pool(cb, pool, 0, GpuSpan::QUERY_COUNT);
            }
        }
    }

    /// Open a span. Record outside any render pass.
    pub fn begin(&self, cb: vk::CommandBuffer, span: GpuSpan) {
        self.write(cb, vk::PipelineStageFlags::TOP_OF_PIPE, span.begin_query());
    }

    /// Close a span. Record outside any render pass.
    pub fn end(&self, cb: vk::CommandBuffer, span: GpuSpan) {
        self.write(cb, vk::PipelineStageFlags::BOTTOM_OF_PIPE, span.end_query());
    }

    /// Note that the frame's queries went to the GPU.
    pub fn mark_submitted(&mut self) {
        self.submitted = true;
    }

    /// Read back the last submitted frame. Call only once its fence has
    /// signalled.
    pub fn collect(&mut self) -> Option<GpuTimings> {
        let pool = self.pool?;
        if !std::mem::take(&mut self.submitted) {
            return None;
        }

        // Each query comes back as its value and an availability word.
        let mut results = [[0u64; 2]; GpuSpan::QUERY_COUNT as usize];
        let read = unsafe {
            self.device.device.get_query_pool_results(
                pool,
                0,
                &mut results,
                vk::QueryResultFlags::TYPE_64 | vk::QueryResultFlags::WITH_AVAILABILITY,
            )
        };
        // NOT_READY only says some query is unavailable; the rest were still
        // written, and availability says which.
        match read {
            Ok(()) | Err(vk::Result::NOT_READY) => {}
            Err(e) => {
                log::warn!("GpuTimer: reading timestamps failed: {:?}", e);
                return None;
            }
        }

        let stamp = |query: u32| {
            let [value, available] = results[query as usize];
            (available != 0).then_some(value & self.valid_mask)
        };
        let between = |begin: Option<u64>, end: Option<u64>| {
            let ticks = end?.wrapping_sub(begin?) & self.valid_mask;
            Some(Duration::from_nanos((ticks as f64 * self.tick_ns) as u64))
        };

        let mut timings = GpuTimings::default();
        for (slot, span) in GpuSpan::ALL.iter().enumerate() {
            timings.spans[slot] = between(stamp(span.begin_query()), stamp(span.end_query()));
        }
        let first = GpuSpan::ALL[0];
        let last = GpuSpan::ALL[GpuSpan::COUNT - 1];
        timings.total = between(stamp(first.begin_query()), stamp(last.end_query()));

        Some(timings)
    }

    fn write(&self, cb: vk::CommandBuffer, stage: vk::PipelineStageFlags, query: u32) {
        if let Some(pool) = self.pool {
            unsafe {
                self.device
                    .device
                    .cmd_write_timestamp(cb, stage, pool, query);
            }
        }
    }
}

impl Drop for GpuTimer {
    fn drop(&mut self) {
        if let Some(pool) = self.pool {
            unsafe {
                self.device.device.destroy_query_pool(pool, None);
            }
        }
    }
}
