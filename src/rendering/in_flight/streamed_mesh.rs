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
        let vertex_bytes = std::mem::size_of_val(vertices);
        let index_bytes = std::mem::size_of_val(indices);
        debug_assert!(vertex_bytes as vk::DeviceSize <= self.vertex_buffer.size);
        debug_assert!(index_bytes as vk::DeviceSize <= self.index_buffer.size);

        unsafe {
            let ptr = self
                .vertex_buffer
                .map_memory(0, vk::MemoryMapFlags::empty())?;
            std::ptr::copy_nonoverlapping(
                vertices.as_ptr() as *const u8,
                ptr as *mut u8,
                vertex_bytes,
            );
            self.vertex_buffer.unmap_memory();

            let ptr = self
                .index_buffer
                .map_memory(0, vk::MemoryMapFlags::empty())?;
            std::ptr::copy_nonoverlapping(
                indices.as_ptr() as *const u8,
                ptr as *mut u8,
                index_bytes,
            );
            self.index_buffer.unmap_memory();
        }
        Ok(())
    }

    /// Bind both buffers at offset zero.
    pub fn bind(&self, device: &ash::Device, cb: vk::CommandBuffer) {
        unsafe {
            device.cmd_bind_vertex_buffers(cb, 0, &[self.vertex_buffer.buffer], &[0]);
            device.cmd_bind_index_buffer(cb, self.index_buffer.buffer, 0, vk::IndexType::UINT32);
        }
    }
}
