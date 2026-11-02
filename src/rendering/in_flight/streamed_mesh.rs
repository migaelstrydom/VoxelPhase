use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::EngineResult;
use crate::rendering::frame::ManagedBuffer;

/// A fixed-capacity vertex and index buffer pair that the CPU refills each
/// frame: particle billboards, overlay quads, the water surface.
///
/// Held one per frame in flight (see [`super::PerFrame`]), so a refill never
/// lands in a buffer the GPU is still drawing from.
pub struct StreamedMesh {
    pub vertex_buffer: ManagedBuffer,
    pub index_buffer: ManagedBuffer,
}

impl StreamedMesh {
    /// Allocate room for `vertex_bytes` of vertices and `index_bytes` of
    /// 32-bit indices, both host-visible.
    pub fn new(
        device: &Arc<ManagedDevice>,
        vertex_bytes: vk::DeviceSize,
        index_bytes: vk::DeviceSize,
    ) -> EngineResult<Self> {
        let host = vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT;
        Ok(Self {
            vertex_buffer: ManagedBuffer::new(
                Arc::clone(device),
                vertex_bytes,
                vk::BufferUsageFlags::VERTEX_BUFFER,
                host,
            )?,
            index_buffer: ManagedBuffer::new(
                Arc::clone(device),
                index_bytes,
                vk::BufferUsageFlags::INDEX_BUFFER,
                host,
            )?,
        })
    }

    /// Write `vertices` and `indices` to the start of the two buffers. The
    /// caller caps both to the capacity it allocated.
    pub fn upload<V: Copy>(&self, vertices: &[V], indices: &[u32]) -> EngineResult<()> {
        self.upload_vertices(vertices)?;
        self.upload_indices(indices)
    }

    /// Write `vertices` to the start of the vertex buffer, leaving the indices
    /// as they were: for a mesh whose index pattern never changes.
    pub fn upload_vertices<V: Copy>(&self, vertices: &[V]) -> EngineResult<()> {
        write_start(&self.vertex_buffer, vertices)
    }

    /// Write `indices` to the start of the index buffer.
    pub fn upload_indices(&self, indices: &[u32]) -> EngineResult<()> {
        write_start(&self.index_buffer, indices)
    }

    /// Bind both buffers at offset zero.
    pub fn bind(&self, device: &ash::Device, cb: vk::CommandBuffer) {
        unsafe {
            device.cmd_bind_vertex_buffers(cb, 0, &[self.vertex_buffer.buffer], &[0]);
            device.cmd_bind_index_buffer(cb, self.index_buffer.buffer, 0, vk::IndexType::UINT32);
        }
    }
}

/// Copy `items` to the start of `buffer`, which must hold them.
fn write_start<T: Copy>(buffer: &ManagedBuffer, items: &[T]) -> EngineResult<()> {
    let bytes = std::mem::size_of_val(items);
    debug_assert!(bytes as vk::DeviceSize <= buffer.size);
    unsafe {
        let ptr = buffer.mapped()?;
        std::ptr::copy_nonoverlapping(items.as_ptr() as *const u8, ptr as *mut u8, bytes);
    }
    Ok(())
}
