//! Where a draw's surface parameters live on the GPU.
//!
//! Surface parameters used to travel as fragment push constants, which put a
//! hard 128-byte ceiling over the whole material system: the model matrix and
//! the colour override already spent 80 of it, and the 40 bytes of parameters
//! left eight to grow into. Any new per-material dial — a detail grain, a tint,
//! a clearcoat — would not have fitted.
//!
//! So the parameters move into a storage buffer and the draw pushes an index
//! into it instead. One `uint` replaces 40 bytes, and the ceiling stops being a
//! design constraint.
//!
//! ```text
//!   draw ──▶ SurfaceBuffer::push(params) ──▶ SurfaceIndex ──push constant──▶ shader
//!                    │                                                        │
//!                    └──────────── set 0, binding 3 ──────────────────────────┘
//! ```
//!
//! # Why the capacity is fixed
//!
//! The buffer is bound to the scene descriptor set once, at startup, and never
//! rebound. Growing it would mean rewriting a descriptor that a frame still in
//! flight may be reading, and the synchronisation to make that safe costs more
//! than the memory it would save: the whole table is
//! `CAPACITY * size_of::<GpuSurface>()`, a few megabytes for a draw count no
//! scene approaches. Overflow is therefore handled by reusing the last entry
//! and logging, not by resizing.

use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::EngineResult;
use crate::rendering::frame::ManagedBuffer;
use crate::rendering::material::GpuSurface;

/// Entries the surface table holds, and so the maximum number of distinct
/// surfaces a single frame can draw.
///
/// Sized well past any plausible frame — the heaviest scenes in the game record
/// draws in the low thousands — because the cost of being generous is
/// `CAPACITY * 48` bytes of host-visible memory and the cost of being tight is a
/// mis-shaded draw.
pub const CAPACITY: u32 = 65536;

/// Index of one surface within the frame's table, pushed per draw.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SurfaceIndex(pub u32);

impl SurfaceIndex {
    /// View as raw bytes for `cmd_push_constants`.
    pub fn as_bytes(&self) -> [u8; 4] {
        self.0.to_ne_bytes()
    }
}

/// A frame-scoped table of surface parameters, bound to the scene descriptor set.
///
/// Filled by appending during recording and rewound at frame start. The
/// contents are written through a persistent mapping, so a push costs a memcpy
/// of 48 bytes and no Vulkan call.
pub struct SurfaceBuffer {
    buffer: ManagedBuffer,
    /// Persistent mapping of `buffer`. Host-coherent, so writes need no flush.
    mapped: *mut GpuSurface,
    /// Next free entry. Reset to zero each frame.
    cursor: u32,
    /// Whether an overflow has already been reported this frame, so a scene
    /// that overruns does not emit one log line per draw.
    overflow_reported: bool,
}

// The raw pointer is a mapping this struct exclusively owns for its lifetime;
// it is neither aliased nor derived from a shared reference. Nothing reachable
// through a `&SurfaceBuffer` touches it — every read and write of the mapping
// goes through `&mut self` — so sharing a reference across threads exposes no
// more than sharing a reference to the buffer handle already does. The
// renderer is an ECS resource and must be both.
unsafe impl Send for SurfaceBuffer {}
unsafe impl Sync for SurfaceBuffer {}

impl SurfaceBuffer {
    /// Size of the table in bytes, for the descriptor write.
    pub const SIZE: vk::DeviceSize =
        CAPACITY as vk::DeviceSize * std::mem::size_of::<GpuSurface>() as vk::DeviceSize;

    /// Allocate the table and map it for the lifetime of the renderer.
    pub fn new(device: Arc<ManagedDevice>) -> EngineResult<Self> {
        let buffer = ManagedBuffer::new(
            device,
            Self::SIZE,
            vk::BufferUsageFlags::STORAGE_BUFFER,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )?;

        // Mapped once and never unmapped: freeing mapped memory is legal, and
        // a map/unmap pair per draw would cost more than the write it guards.
        let mapped =
            unsafe { buffer.map_memory(0, vk::MemoryMapFlags::empty())? } as *mut GpuSurface;

        // Vulkan does not guarantee zeroed memory, and entry 0 is what an
        // overflowing draw falls back to. Write one honest entry so that even a
        // frame which pushes nothing has a defined surface to point at.
        unsafe {
            std::ptr::write(mapped, GpuSurface::MATTE);
        }

        Ok(Self {
            buffer,
            mapped,
            cursor: 0,
            overflow_reported: false,
        })
    }

    /// The underlying buffer, for the descriptor write that binds it.
    pub fn buffer(&self) -> &ManagedBuffer {
        &self.buffer
    }

    /// Rewind the table for a new frame.
    ///
    /// Safe to do without waiting on the GPU only because the caller has
    /// already waited on this frame's fence — the same guarantee that lets
    /// `FrameData::begin_frame` reset its vertex and index cursors.
    pub fn begin_frame(&mut self) {
        self.cursor = 0;
        self.overflow_reported = false;
    }

    /// Add one surface to this frame's table and return where it landed.
    ///
    /// On overflow the last valid entry is returned rather than growing the
    /// buffer or dropping the draw: one draw shaded with a neighbour's material
    /// is a far better failure than a lost frame, and it is visible enough to
    /// notice alongside the log line.
    pub fn push(&mut self, surface: GpuSurface) -> SurfaceIndex {
        if self.cursor >= CAPACITY {
            if !self.overflow_reported {
                log::warn!(
                    "surface table full at {} entries; further draws this frame reuse the last surface",
                    CAPACITY
                );
                self.overflow_reported = true;
            }
            return SurfaceIndex(CAPACITY - 1);
        }

        let index = self.cursor;
        unsafe {
            std::ptr::write(self.mapped.add(index as usize), surface);
        }
        self.cursor += 1;

        SurfaceIndex(index)
    }

    /// Entries written so far this frame. Exposed for the debug overlay.
    pub fn len(&self) -> u32 {
        self.cursor
    }

    pub fn is_empty(&self) -> bool {
        self.cursor == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The layout contract with the shader. std430 aligns a struct to its
    /// largest member, so a 40-byte struct of vec4s would be padded to a stride
    /// of 48 on the GPU and read back misaligned from entry one onwards — the
    /// classic silent version of this bug. `GpuSurface` is three vec4s so that
    /// its Rust size and its std430 stride are the same number.
    #[test]
    fn the_gpu_struct_matches_its_std430_stride() {
        assert_eq!(std::mem::size_of::<GpuSurface>(), 48);
        assert_eq!(std::mem::align_of::<GpuSurface>() % 4, 0);
    }

    #[test]
    fn the_table_is_a_sane_size() {
        // A few megabytes is the price of never having to resize it.
        assert!(SurfaceBuffer::SIZE < 8 * 1024 * 1024);
    }

    #[test]
    fn an_index_pushes_as_four_bytes() {
        assert_eq!(SurfaceIndex(7).as_bytes(), 7u32.to_ne_bytes());
    }
}
