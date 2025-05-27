use ash::{vk, Device};
use std::error::Error;
use std::sync::Arc;

use super::device::ManagedDevice;

pub struct ManagedCommandBuffer {
    pub command_buffer: vk::CommandBuffer,
    device: Arc<ManagedDevice>,
    pool: Arc<ManagedCommandPool>,
}

impl ManagedCommandBuffer {
    pub fn new(
        device: Arc<ManagedDevice>,
        pool: Arc<ManagedCommandPool>,
        level: vk::CommandBufferLevel,
    ) -> Result<Self, vk::Result> {
        let allocate_info = vk::CommandBufferAllocateInfo::default()
            .command_pool(pool.pool) // Dereference Arc to get vk::CommandPool
            .level(level)
            .command_buffer_count(1);
        unsafe {
            let command_buffer = device.device.allocate_command_buffers(&allocate_info)?[0];
            Ok(Self {
                command_buffer,
                device,
                pool,
            })
        }
    }

    pub fn begin(&self, usage_flags: vk::CommandBufferUsageFlags) -> Result<(), vk::Result> {
        let begin_info = vk::CommandBufferBeginInfo::default().flags(usage_flags);
        unsafe {
            self.device
                .device
                .begin_command_buffer(self.command_buffer, &begin_info)?;
        }
        Ok(())
    }

    pub fn end(&self) -> Result<(), vk::Result> {
        unsafe {
            self.device.device.end_command_buffer(self.command_buffer)?;
        }
        Ok(())
    }

    pub fn reset(&self, flags: vk::CommandBufferResetFlags) -> Result<(), vk::Result> {
        unsafe {
            self.device
                .device
                .reset_command_buffer(self.command_buffer, flags)?;
        }
        Ok(())
    }

    pub fn raw(&self) -> vk::CommandBuffer {
        self.command_buffer
    }
}

impl Drop for ManagedCommandBuffer {
    fn drop(&mut self) {
        // The command buffer is freed when the pool it was allocated from is destroyed or reset.
        // If we want individual freeing, we'd do it here, but often command buffers are reset
        // or freed in bulk by resetting/destroying the pool.
        // For RAII safety with an Arc<vk::CommandPool>, we ensure the pool is valid when
        // device.free_command_buffers is called.
        // Given that the pool is an Arc, it won't be dropped before this ManagedCommandBuffer
        // if this buffer still holds an Arc to it.
        unsafe {
            self.device
                .device
                .free_command_buffers(self.pool.pool, &[self.command_buffer]);
        }
    }
}

pub struct ManagedCommandPool {
    pub pool: vk::CommandPool,
    device: Arc<ManagedDevice>,
}

impl ManagedCommandPool {
    pub fn new(device: Arc<ManagedDevice>) -> Result<Self, Box<dyn Error>> {
        let pool_create_info = vk::CommandPoolCreateInfo::default()
            .queue_family_index(device.queue_family_index)
            .flags(vk::CommandPoolCreateFlags::RESET_COMMAND_BUFFER);
        let pool = unsafe { device.device.create_command_pool(&pool_create_info, None)? };
        Ok(Self { pool, device })
    }
}

impl Drop for ManagedCommandPool {
    fn drop(&mut self) {
        unsafe { self.device.device.destroy_command_pool(self.pool, None) };
    }
}

pub struct CommandBufferManager {
    device: Arc<ManagedDevice>,
    pub graphics_queue: vk::Queue,
    pub compute_queue: vk::Queue,
    pub transfer_queue: vk::Queue,
    pub graphics_command_pool: Arc<ManagedCommandPool>,
    pub compute_command_pool: Arc<ManagedCommandPool>,
    pub transfer_command_pool: Arc<ManagedCommandPool>,
}

impl CommandBufferManager {
    pub fn new(device: Arc<ManagedDevice>) -> Result<Self, Box<dyn Error>> {
        let graphics_queue = unsafe {
            device
                .device
                .get_device_queue(device.queue_family_indices.graphics, 0)
        };
        let compute_queue = unsafe {
            device
                .device
                .get_device_queue(device.queue_family_indices.compute, 0)
        };
        let transfer_queue = unsafe {
            device
                .device
                .get_device_queue(device.queue_family_indices.transfer, 0)
        };

        let graphics_command_pool = Arc::new(ManagedCommandPool::new(Arc::clone(&device))?);
        let compute_command_pool = Arc::new(ManagedCommandPool::new(Arc::clone(&device))?);
        let transfer_command_pool = Arc::new(ManagedCommandPool::new(Arc::clone(&device))?);

        Ok(Self {
            device,
            graphics_queue,
            compute_queue,
            transfer_queue,
            graphics_command_pool,
            compute_command_pool,
            transfer_command_pool,
        })
    }

    pub fn create_primary_buffer(&self) -> Result<ManagedCommandBuffer, vk::Result> {
        ManagedCommandBuffer::new(
            Arc::clone(&self.device),
            Arc::clone(&self.graphics_command_pool),
            vk::CommandBufferLevel::PRIMARY,
        )
    }

    pub fn create_compute_buffer(&self) -> Result<ManagedCommandBuffer, vk::Result> {
        ManagedCommandBuffer::new(
            Arc::clone(&self.device),
            Arc::clone(&self.compute_command_pool),
            vk::CommandBufferLevel::PRIMARY,
        )
    }

    pub fn create_transfer_buffer(&self) -> Result<ManagedCommandBuffer, vk::Result> {
        ManagedCommandBuffer::new(
            Arc::clone(&self.device),
            Arc::clone(&self.transfer_command_pool),
            vk::CommandBufferLevel::PRIMARY,
        )
    }

    pub fn create_one_time_submit_buffer(&self) -> Result<ManagedCommandBuffer, vk::Result> {
        self.create_primary_buffer()
    }

    fn submit_commands_and_wait_internal<F: FnOnce(&Device, vk::CommandBuffer)>(
        &self,
        buffer: &ManagedCommandBuffer,
        queue: vk::Queue,
        record_commands_fn: F,
    ) -> Result<(), Box<dyn Error>> {
        unsafe {
            buffer.reset(vk::CommandBufferResetFlags::RELEASE_RESOURCES)?;
            buffer.begin(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT)?;

            record_commands_fn(&self.device.device, buffer.raw());

            buffer.end()?;

            let raw_buffer = buffer.raw();
            let submit_infos =
                [vk::SubmitInfo::default().command_buffers(std::slice::from_ref(&raw_buffer))];

            let fence_create_info = vk::FenceCreateInfo::default();
            let fence = self.device.device.create_fence(&fence_create_info, None)?;

            self.device
                .device
                .queue_submit(queue, &submit_infos, fence)?;
            self.device
                .device
                .wait_for_fences(&[fence], true, u64::MAX)?;
            self.device.device.destroy_fence(fence, None);
        }
        Ok(())
    }

    pub fn submit_graphics_commands_and_wait<F: FnOnce(&Device, vk::CommandBuffer)>(
        &self,
        buffer: &ManagedCommandBuffer,
        record_commands_fn: F,
    ) -> Result<(), Box<dyn Error>> {
        self.submit_commands_and_wait_internal(buffer, self.graphics_queue, record_commands_fn)
    }

    pub fn submit_compute_commands_and_wait<F: FnOnce(&Device, vk::CommandBuffer)>(
        &self,
        buffer: &ManagedCommandBuffer,
        record_commands_fn: F,
    ) -> Result<(), Box<dyn Error>> {
        self.submit_commands_and_wait_internal(buffer, self.compute_queue, record_commands_fn)
    }

    pub fn submit_transfer_commands_and_wait<F: FnOnce(&Device, vk::CommandBuffer)>(
        &self,
        buffer: &ManagedCommandBuffer,
        record_commands_fn: F,
    ) -> Result<(), Box<dyn Error>> {
        self.submit_commands_and_wait_internal(buffer, self.transfer_queue, record_commands_fn)
    }

    fn submit_commands_async_internal<F: FnOnce(&Device, vk::CommandBuffer)>(
        &self,
        buffer: &ManagedCommandBuffer,
        queue: vk::Queue,
        fence: vk::Fence,
        wait_semaphores: &[vk::Semaphore],
        signal_semaphores: &[vk::Semaphore],
        wait_dst_stage_mask: &[vk::PipelineStageFlags],
        record_commands_fn: F,
    ) -> Result<(), vk::Result> {
        unsafe {
            buffer.reset(vk::CommandBufferResetFlags::RELEASE_RESOURCES)?;
            buffer.begin(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT)?;

            record_commands_fn(&self.device.device, buffer.raw());

            buffer.end()?;

            let raw_buffer = buffer.raw();
            let command_buffers_slice = std::slice::from_ref(&raw_buffer);
            let submit_info = vk::SubmitInfo::default()
                .wait_semaphores(wait_semaphores)
                .wait_dst_stage_mask(wait_dst_stage_mask)
                .command_buffers(command_buffers_slice)
                .signal_semaphores(signal_semaphores);

            self.device
                .device
                .queue_submit(queue, &[submit_info], fence)?;
        }
        Ok(())
    }

    fn submit_recorded_commands_async_internal(
        &self,
        buffer: &ManagedCommandBuffer,
        queue: vk::Queue,
        fence: vk::Fence,
        wait_semaphores: &[vk::Semaphore],
        signal_semaphores: &[vk::Semaphore],
        wait_dst_stage_mask: &[vk::PipelineStageFlags],
    ) -> Result<(), vk::Result> {
        unsafe {
            let raw_buffer = buffer.raw();
            let command_buffers_slice = std::slice::from_ref(&raw_buffer);
            let submit_info = vk::SubmitInfo::default()
                .wait_semaphores(wait_semaphores)
                .wait_dst_stage_mask(wait_dst_stage_mask)
                .command_buffers(command_buffers_slice)
                .signal_semaphores(signal_semaphores);

            self.device
                .device
                .queue_submit(queue, &[submit_info], fence)?;
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    pub fn submit_graphics_commands_async<F: FnOnce(&Device, vk::CommandBuffer)>(
        &self,
        buffer: &ManagedCommandBuffer,
        fence: vk::Fence,
        wait_semaphores: &[vk::Semaphore],
        signal_semaphores: &[vk::Semaphore],
        wait_dst_stage_mask: &[vk::PipelineStageFlags],
        record_commands_fn: F,
    ) -> Result<(), vk::Result> {
        self.submit_commands_async_internal(
            buffer,
            self.graphics_queue,
            fence,
            wait_semaphores,
            signal_semaphores,
            wait_dst_stage_mask,
            record_commands_fn,
        )
    }

    pub fn submit_compute_commands_async<F: FnOnce(&Device, vk::CommandBuffer)>(
        &self,
        buffer: &ManagedCommandBuffer,
        fence: vk::Fence,
        wait_semaphores: &[vk::Semaphore],
        signal_semaphores: &[vk::Semaphore],
        wait_dst_stage_mask: &[vk::PipelineStageFlags],
        record_commands_fn: F,
    ) -> Result<(), vk::Result> {
        self.submit_commands_async_internal(
            buffer,
            self.compute_queue,
            fence,
            wait_semaphores,
            signal_semaphores,
            wait_dst_stage_mask,
            record_commands_fn,
        )
    }

    pub fn submit_transfer_commands_async<F: FnOnce(&Device, vk::CommandBuffer)>(
        &self,
        buffer: &ManagedCommandBuffer,
        fence: vk::Fence,
        wait_semaphores: &[vk::Semaphore],
        signal_semaphores: &[vk::Semaphore],
        wait_dst_stage_mask: &[vk::PipelineStageFlags],
        record_commands_fn: F,
    ) -> Result<(), vk::Result> {
        self.submit_commands_async_internal(
            buffer,
            self.transfer_queue,
            fence,
            wait_semaphores,
            signal_semaphores,
            wait_dst_stage_mask,
            record_commands_fn,
        )
    }

    pub fn submit_recorded_graphics_commands_async(
        &self,
        buffer: &ManagedCommandBuffer,
        fence: vk::Fence,
        wait_semaphores: &[vk::Semaphore],
        signal_semaphores: &[vk::Semaphore],
        wait_dst_stage_mask: &[vk::PipelineStageFlags],
    ) -> Result<(), vk::Result> {
        self.submit_recorded_commands_async_internal(
            buffer,
            self.graphics_queue,
            fence,
            wait_semaphores,
            signal_semaphores,
            wait_dst_stage_mask,
        )
    }

    pub fn submit_recorded_compute_commands_async(
        &self,
        buffer: &ManagedCommandBuffer,
        fence: vk::Fence,
        wait_semaphores: &[vk::Semaphore],
        signal_semaphores: &[vk::Semaphore],
        wait_dst_stage_mask: &[vk::PipelineStageFlags],
    ) -> Result<(), vk::Result> {
        self.submit_recorded_commands_async_internal(
            buffer,
            self.compute_queue,
            fence,
            wait_semaphores,
            signal_semaphores,
            wait_dst_stage_mask,
        )
    }

    pub fn submit_recorded_transfer_commands_async(
        &self,
        buffer: &ManagedCommandBuffer,
        fence: vk::Fence,
        wait_semaphores: &[vk::Semaphore],
        signal_semaphores: &[vk::Semaphore],
        wait_dst_stage_mask: &[vk::PipelineStageFlags],
    ) -> Result<(), vk::Result> {
        self.submit_recorded_commands_async_internal(
            buffer,
            self.transfer_queue,
            fence,
            wait_semaphores,
            signal_semaphores,
            wait_dst_stage_mask,
        )
    }
}
