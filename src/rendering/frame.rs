//! Per-frame data management.
//!
//! This module handles mutable per-frame resources like vertex/index buffers
//! and uniform buffers that change every frame.
//!
//! # Buffer Lifecycle
//!
//! When buffers need to be resized during rendering, we cannot destroy the old
//! buffer immediately because it may still be referenced by in-flight GPU commands.
//! Instead, old buffers are queued for deferred deletion via [`DeletionQueue`].
//!
//! The deletion queue should be flushed at frame start, after the fence wait
//! ensures the GPU has finished with previous frames.

use std::mem;
use std::sync::Arc;

use ash::vk;
use nalgebra::Matrix4;

use crate::core::device::ManagedDevice;
use crate::core::error::{BufferOperation, EngineError, EngineResult};
use crate::core::vulkan_context::find_memorytype_index;
use crate::rendering::deletion_queue::DeletionQueue;
use crate::rendering::vertex::Vertex;

/// Uniform buffer object for per-frame scene data.
/// Model matrix is now passed via push constants per draw call.
#[derive(Clone, Debug, Copy)]
#[repr(C)]
pub struct SceneUbo {
    pub view: Matrix4<f32>,
    pub proj: Matrix4<f32>,
}

/// A GPU buffer with RAII memory management.
pub struct ManagedBuffer {
    pub buffer: vk::Buffer,
    pub memory: vk::DeviceMemory,
    pub size: vk::DeviceSize,
    device: Arc<ManagedDevice>,
}

impl std::fmt::Debug for ManagedBuffer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ManagedBuffer")
            .field("buffer", &self.buffer)
            .field("memory", &self.memory)
            .field("size", &self.size)
            .finish()
    }
}

impl ManagedBuffer {
    pub fn new(
        device: Arc<ManagedDevice>,
        size: vk::DeviceSize,
        usage: vk::BufferUsageFlags,
        memory_properties: vk::MemoryPropertyFlags,
    ) -> EngineResult<Self> {
        unsafe {
            let buffer_info = vk::BufferCreateInfo::default()
                .size(size)
                .usage(usage)
                .sharing_mode(vk::SharingMode::EXCLUSIVE);

            let buffer = device
                .device
                .create_buffer(&buffer_info, None)
                .map_err(|e| EngineError::Buffer {
                    operation: BufferOperation::Create,
                    size,
                    reason: format!("{:?}", e),
                })?;

            let memory_req = device.device.get_buffer_memory_requirements(buffer);
            let memory_type_index = find_memorytype_index(
                &memory_req,
                &device.device_memory_properties,
                memory_properties,
            )
            .ok_or_else(|| EngineError::Buffer {
                operation: BufferOperation::Create,
                size,
                reason: "no suitable memory type".to_string(),
            })?;

            let allocate_info = vk::MemoryAllocateInfo::default()
                .allocation_size(memory_req.size)
                .memory_type_index(memory_type_index);

            let memory = device
                .device
                .allocate_memory(&allocate_info, None)
                .map_err(|e| EngineError::Buffer {
                    operation: BufferOperation::Create,
                    size,
                    reason: format!("memory allocation: {:?}", e),
                })?;

            device
                .device
                .bind_buffer_memory(buffer, memory, 0)
                .map_err(|e| EngineError::Buffer {
                    operation: BufferOperation::Bind,
                    size,
                    reason: format!("{:?}", e),
                })?;

            Ok(Self {
                buffer,
                memory,
                size,
                device,
            })
        }
    }

    /// Map buffer memory for CPU access.
    ///
    /// # Safety
    /// Caller must ensure proper usage of the returned pointer and call unmap_memory when done.
    pub unsafe fn map_memory(
        &self,
        offset: vk::DeviceSize,
        flags: vk::MemoryMapFlags,
    ) -> EngineResult<*mut std::ffi::c_void> {
        self.device
            .device
            .map_memory(self.memory, offset, self.size, flags)
            .map_err(|e| EngineError::Buffer {
                operation: BufferOperation::Map,
                size: self.size,
                reason: format!("{:?}", e),
            })
    }

    /// Unmap previously mapped buffer memory.
    ///
    /// # Safety
    /// Must only be called after a successful map_memory call.
    pub unsafe fn unmap_memory(&self) {
        self.device.device.unmap_memory(self.memory);
    }
}

impl Drop for ManagedBuffer {
    fn drop(&mut self) {
        unsafe {
            if self.buffer != vk::Buffer::null() {
                self.device.device.destroy_buffer(self.buffer, None);
            }
            if self.memory != vk::DeviceMemory::null() {
                self.device.device.free_memory(self.memory, None);
            }
        }
    }
}

/// Information needed to issue a draw call for a mesh.
#[derive(Debug, Clone, Copy)]
pub struct DrawInfo {
    pub index_count: u32,
    pub first_index: u32,
    pub vertex_offset: i32,
}

/// Manages per-frame mutable data: vertex buffer, index buffer, UBO.
///
/// Meshes are accumulated in the buffer during frame recording, then
/// the buffer is reset at the start of the next frame.
///
/// Handles buffer resizing safely by deferring destruction of old buffers
/// until the GPU is no longer using them.
pub struct FrameData {
    pub vertex_buffer: ManagedBuffer,
    pub index_buffer: ManagedBuffer,
    pub scene_ubo_buffer: ManagedBuffer,
    device: Arc<ManagedDevice>,
    /// Buffers pending deletion (deferred until GPU is done with them).
    buffer_deletion_queue: DeletionQueue<ManagedBuffer>,
    /// Current frame number for deletion queue tracking.
    frame_number: u64,
    /// Current write position in vertex buffer (in vertices, not bytes).
    current_vertex_count: u32,
    /// Current write position in index buffer (in indices, not bytes).
    current_index_count: u32,
}

impl FrameData {
    /// Number of frames to wait before destroying old buffers.
    /// This should match your frames-in-flight count.
    const FRAMES_IN_FLIGHT: u64 = 2;

    /// Create frame data with initial buffer sizes.
    pub fn new(device: Arc<ManagedDevice>) -> EngineResult<Self> {
        // Modest initial sizes - buffers will grow as needed via deferred resize.
        let initial_vertex_size = mem::size_of::<Vertex>() as u64 * 1024;
        let initial_index_size = mem::size_of::<u32>() as u64 * 4096;
        let ubo_size = mem::size_of::<SceneUbo>() as u64;

        let vertex_buffer = ManagedBuffer::new(
            Arc::clone(&device),
            initial_vertex_size,
            vk::BufferUsageFlags::VERTEX_BUFFER,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )?;

        let index_buffer = ManagedBuffer::new(
            Arc::clone(&device),
            initial_index_size,
            vk::BufferUsageFlags::INDEX_BUFFER,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )?;

        let scene_ubo_buffer = ManagedBuffer::new(
            Arc::clone(&device),
            ubo_size,
            vk::BufferUsageFlags::UNIFORM_BUFFER,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )?;

        Ok(Self {
            vertex_buffer,
            index_buffer,
            scene_ubo_buffer,
            device,
            buffer_deletion_queue: DeletionQueue::new(Self::FRAMES_IN_FLIGHT),
            frame_number: 0,
            current_vertex_count: 0,
            current_index_count: 0,
        })
    }

    /// Signal that a new frame is starting.
    ///
    /// Call this at frame start, AFTER waiting for the frame fence.
    /// This flushes old buffers that are safe to delete, advances the frame counter,
    /// and resets the mesh accumulation offsets.
    pub fn begin_frame(&mut self) {
        self.buffer_deletion_queue.flush(self.frame_number);
        self.frame_number += 1;
        // Reset for new frame - meshes will be accumulated fresh
        self.current_vertex_count = 0;
        self.current_index_count = 0;
    }

    /// Force cleanup of all pending deletions.
    ///
    /// Call this during shutdown after `device_wait_idle()`.
    pub fn cleanup(&mut self) {
        self.buffer_deletion_queue.flush_all();
    }

    /// Append mesh data to the frame's vertex/index buffers.
    ///
    /// Returns draw information (offsets and counts) needed for the draw call.
    /// Multiple meshes can be appended per frame; they accumulate in the buffer.
    ///
    /// If buffers need to be resized, the old buffers are queued for deferred
    /// deletion to avoid destroying them while the GPU may still be using them.
    pub fn append_mesh_data(
        &mut self,
        vertices: &[Vertex],
        indices: &[u32],
    ) -> EngineResult<DrawInfo> {
        let new_vertex_count = self.current_vertex_count + vertices.len() as u32;
        let new_index_count = self.current_index_count + indices.len() as u32;

        let required_vertex_size =
            (mem::size_of::<Vertex>() as u32 * new_vertex_count) as vk::DeviceSize;
        let required_index_size =
            (mem::size_of::<u32>() as u32 * new_index_count) as vk::DeviceSize;

        // Resize vertex buffer if needed (with deferred deletion of old buffer)
        if self.vertex_buffer.size < required_vertex_size {
            let old_size = self.vertex_buffer.size;
            // Double the size to reduce resize frequency
            let new_size = required_vertex_size.max(old_size * 2);
            let new_buffer = ManagedBuffer::new(
                Arc::clone(&self.device),
                new_size,
                vk::BufferUsageFlags::VERTEX_BUFFER,
                vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
            )?;
            // Swap in new buffer, queue old one for deferred deletion
            let old_buffer = mem::replace(&mut self.vertex_buffer, new_buffer);
            self.buffer_deletion_queue
                .queue(old_buffer, self.frame_number);
            log::debug!("Vertex buffer resized: {} -> {} bytes", old_size, new_size);
        }

        // Resize index buffer if needed (with deferred deletion of old buffer)
        if self.index_buffer.size < required_index_size {
            let old_size = self.index_buffer.size;
            // Double the size to reduce resize frequency
            let new_size = required_index_size.max(old_size * 2);
            let new_buffer = ManagedBuffer::new(
                Arc::clone(&self.device),
                new_size,
                vk::BufferUsageFlags::INDEX_BUFFER,
                vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
            )?;
            // Swap in new buffer, queue old one for deferred deletion
            let old_buffer = mem::replace(&mut self.index_buffer, new_buffer);
            self.buffer_deletion_queue
                .queue(old_buffer, self.frame_number);
            log::debug!("Index buffer resized: {} -> {} bytes", old_size, new_size);
        }

        // Calculate byte offsets for appending
        let vertex_byte_offset = (self.current_vertex_count as usize) * mem::size_of::<Vertex>();
        let index_byte_offset = (self.current_index_count as usize) * mem::size_of::<u32>();
        let vertex_size = mem::size_of::<Vertex>() * vertices.len();
        let index_size = mem::size_of::<u32>() * indices.len();

        // Append vertex data at current offset
        unsafe {
            let ptr = self
                .vertex_buffer
                .map_memory(0, vk::MemoryMapFlags::empty())?;
            let dst = (ptr as *mut u8).add(vertex_byte_offset);
            std::ptr::copy_nonoverlapping(vertices.as_ptr() as *const u8, dst, vertex_size);
            self.vertex_buffer.unmap_memory();
        }

        // Append index data at current offset
        unsafe {
            let ptr = self
                .index_buffer
                .map_memory(0, vk::MemoryMapFlags::empty())?;
            let dst = (ptr as *mut u8).add(index_byte_offset);
            std::ptr::copy_nonoverlapping(indices.as_ptr() as *const u8, dst, index_size);
            self.index_buffer.unmap_memory();
        }

        // Build draw info before updating offsets
        let draw_info = DrawInfo {
            index_count: indices.len() as u32,
            first_index: self.current_index_count,
            vertex_offset: self.current_vertex_count as i32,
        };

        // Update offsets for next mesh
        self.current_vertex_count = new_vertex_count;
        self.current_index_count = new_index_count;

        Ok(draw_info)
    }

    /// Update the scene uniform buffer with per-frame data (view/projection).
    /// Call this once per frame, not per draw call.
    pub fn update_scene_ubo(
        &mut self,
        view: &Matrix4<f32>,
        proj: &Matrix4<f32>,
    ) -> EngineResult<()> {
        let ubo = SceneUbo {
            view: *view,
            proj: *proj,
        };

        unsafe {
            let ptr = self
                .scene_ubo_buffer
                .map_memory(0, vk::MemoryMapFlags::empty())?;
            let slice = std::slice::from_raw_parts_mut(ptr as *mut SceneUbo, 1);
            slice[0] = ubo;
            self.scene_ubo_buffer.unmap_memory();
        }

        Ok(())
    }
}
