//! Per-frame data management.
//!
//! This module handles mutable per-frame resources like vertex/index buffers
//! and uniform buffers that change every frame.

use std::mem;
use std::sync::Arc;

use ash::{util::Align, vk};
use nalgebra::Matrix4;

use crate::core::device::ManagedDevice;
use crate::core::error::{BufferOperation, EngineError, EngineResult};
use crate::core::vulkan_context::find_memorytype_index;
use crate::rendering::vertex::Vertex;

/// Uniform buffer object for scene transforms.
#[derive(Clone, Debug, Copy)]
#[repr(C)]
pub struct SceneUbo {
    pub model: Matrix4<f32>,
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

            let memory =
                device
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

/// Manages per-frame mutable data: vertex buffer, index buffer, UBO.
pub struct FrameData {
    pub vertex_buffer: ManagedBuffer,
    pub index_buffer: ManagedBuffer,
    pub scene_ubo_buffer: ManagedBuffer,
    pub index_count: u32,
    device: Arc<ManagedDevice>,
}

impl FrameData {
    /// Create frame data with initial buffer sizes.
    pub fn new(device: Arc<ManagedDevice>) -> EngineResult<Self> {
        // Initial sizes for buffers (will be resized as needed)
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
            index_count: 0,
            device,
        })
    }

    /// Update vertex and index buffers with new mesh data.
    ///
    /// Resizes buffers if necessary.
    pub fn update_mesh_data(&mut self, vertices: &[Vertex], indices: &[u32]) -> EngineResult<()> {
        let vertex_size = (mem::size_of::<Vertex>() * vertices.len()) as vk::DeviceSize;
        let index_size = (mem::size_of::<u32>() * indices.len()) as vk::DeviceSize;

        // Resize vertex buffer if needed
        if self.vertex_buffer.size < vertex_size {
            self.vertex_buffer = ManagedBuffer::new(
                Arc::clone(&self.device),
                vertex_size,
                vk::BufferUsageFlags::VERTEX_BUFFER,
                vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
            )?;
        }

        // Resize index buffer if needed
        if self.index_buffer.size < index_size {
            self.index_buffer = ManagedBuffer::new(
                Arc::clone(&self.device),
                index_size,
                vk::BufferUsageFlags::INDEX_BUFFER,
                vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
            )?;
        }

        // Upload vertex data
        unsafe {
            let ptr = self
                .vertex_buffer
                .map_memory(0, vk::MemoryMapFlags::empty())?;
            let mut align = Align::new(ptr, mem::align_of::<Vertex>() as u64, vertex_size);
            align.copy_from_slice(vertices);
            self.vertex_buffer.unmap_memory();
        }

        // Upload index data
        unsafe {
            let ptr = self
                .index_buffer
                .map_memory(0, vk::MemoryMapFlags::empty())?;
            let mut align = Align::new(ptr, mem::align_of::<u32>() as u64, index_size);
            align.copy_from_slice(indices);
            self.index_buffer.unmap_memory();
        }

        self.index_count = indices.len() as u32;
        Ok(())
    }

    /// Update the scene uniform buffer with new transforms.
    pub fn update_transforms(
        &mut self,
        model: &Matrix4<f32>,
        view: &Matrix4<f32>,
        proj: &Matrix4<f32>,
    ) -> EngineResult<()> {
        let ubo = SceneUbo {
            model: *model,
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
