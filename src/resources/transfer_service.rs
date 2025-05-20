use ash::{
    vk::{self},
    Device,
};
use std::{error::Error, sync::Arc};

use crate::core::vulkan_context::{record_submit_commandbuffer, VulkanContext};

/// Service responsible for GPU data transfer operations
#[derive(Clone)]
pub struct TransferService {
    vulkan_context: Arc<VulkanContext>,
    command_buffer: vk::CommandBuffer,
    fence: vk::Fence,
}

impl TransferService {
    /// Create a new transfer service
    pub fn new(vulkan_context: Arc<VulkanContext>) -> Result<Self, Box<dyn Error>> {
        // Create a command buffer for transfer operations
        let command_buffer_allocate_info = vk::CommandBufferAllocateInfo::default()
            .command_pool(vulkan_context.command_pool)
            .level(vk::CommandBufferLevel::PRIMARY)
            .command_buffer_count(1);

        let command_buffer = unsafe {
            vulkan_context
                .device
                .device
                .allocate_command_buffers(&command_buffer_allocate_info)?[0]
        };

        // Create a fence (in signaled state initially so first wait succeeds)
        let fence_create_info =
            vk::FenceCreateInfo::default().flags(vk::FenceCreateFlags::SIGNALED);

        let fence = unsafe {
            vulkan_context
                .device
                .device
                .create_fence(&fence_create_info, None)?
        };

        Ok(Self {
            vulkan_context,
            command_buffer,
            fence,
        })
    }

    fn device(&self) -> &Device {
        &self.vulkan_context.device.device
    }

    /// Copy data from a buffer to an image
    pub fn copy_buffer_to_image(
        &self,
        buffer: vk::Buffer,
        image: vk::Image,
        width: u32,
        height: u32,
    ) -> Result<(), Box<dyn Error>> {
        unsafe {
            // Wait for any previous operations to complete
            self.device()
                .wait_for_fences(&[self.fence], true, std::u64::MAX)?;
            self.device().reset_fences(&[self.fence])?;

            // Prepare buffer image copy info outside the closure to avoid lifetime issues
            let buffer_image_copy = vk::BufferImageCopy::default()
                .image_subresource(
                    vk::ImageSubresourceLayers::default()
                        .aspect_mask(vk::ImageAspectFlags::COLOR)
                        .mip_level(0)
                        .base_array_layer(0)
                        .layer_count(1),
                )
                .image_extent(vk::Extent3D {
                    width,
                    height,
                    depth: 1,
                });

            // Get values needed for the closure to avoid self capture
            let dst_image = image;

            record_submit_commandbuffer(
                &self.device(),
                self.command_buffer,
                self.fence,
                self.vulkan_context.queue,
                &[], // No wait stages
                &[], // No wait semaphores
                &[], // No signal semaphores
                &|device: &ash::Device, command_buffer: vk::CommandBuffer| {
                    device.cmd_copy_buffer_to_image(
                        command_buffer,
                        buffer,
                        dst_image,
                        vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                        &[buffer_image_copy],
                    );
                },
            );

            // Wait for the operation to complete
            self.device()
                .wait_for_fences(&[self.fence], true, std::u64::MAX)?;
        }
        Ok(())
    }

    /// Transition image layout from one layout to another
    pub fn transition_image_layout(
        &self,
        image: vk::Image,
        format: vk::Format,
        old_layout: vk::ImageLayout,
        new_layout: vk::ImageLayout,
        mip_levels: u32,
    ) -> Result<(), Box<dyn Error>> {
        unsafe {
            // Wait for any previous operations to complete
            self.device()
                .wait_for_fences(&[self.fence], true, std::u64::MAX)?;
            self.device().reset_fences(&[self.fence])?;

            // Prepare data for closure to avoid self capture
            let (src_access_mask, dst_access_mask, src_stage, dst_stage) =
                match (old_layout, new_layout) {
                    (vk::ImageLayout::UNDEFINED, vk::ImageLayout::TRANSFER_DST_OPTIMAL) => (
                        vk::AccessFlags::empty(),
                        vk::AccessFlags::TRANSFER_WRITE,
                        vk::PipelineStageFlags::TOP_OF_PIPE,
                        vk::PipelineStageFlags::TRANSFER,
                    ),
                    (
                        vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                        vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
                    ) => (
                        vk::AccessFlags::TRANSFER_WRITE,
                        vk::AccessFlags::SHADER_READ,
                        vk::PipelineStageFlags::TRANSFER,
                        vk::PipelineStageFlags::FRAGMENT_SHADER,
                    ),
                    (
                        vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                        vk::ImageLayout::TRANSFER_SRC_OPTIMAL,
                    ) => (
                        vk::AccessFlags::TRANSFER_WRITE,
                        vk::AccessFlags::TRANSFER_READ,
                        vk::PipelineStageFlags::TRANSFER,
                        vk::PipelineStageFlags::TRANSFER,
                    ),
                    _ => panic!("Unsupported layout transition!"),
                };

            let barrier = vk::ImageMemoryBarrier::default()
                .old_layout(old_layout)
                .new_layout(new_layout)
                .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                .image(image)
                .subresource_range(vk::ImageSubresourceRange {
                    aspect_mask: vk::ImageAspectFlags::COLOR,
                    base_mip_level: 0,
                    level_count: mip_levels,
                    base_array_layer: 0,
                    layer_count: 1,
                })
                .src_access_mask(src_access_mask)
                .dst_access_mask(dst_access_mask);

            record_submit_commandbuffer(
                &self.device(),
                self.command_buffer,
                self.fence,
                self.vulkan_context.queue,
                &[], // No wait stages
                &[], // No wait semaphores
                &[], // No signal semaphores
                &move |device: &ash::Device, command_buffer: vk::CommandBuffer| {
                    device.cmd_pipeline_barrier(
                        command_buffer,
                        src_stage,
                        dst_stage,
                        vk::DependencyFlags::empty(),
                        &[],
                        &[],
                        &[barrier],
                    );
                },
            );

            // Wait for the operation to complete
            self.device()
                .wait_for_fences(&[self.fence], true, std::u64::MAX)?;
        }
        Ok(())
    }

    /// Generate mipmaps for an image
    pub fn generate_mipmaps(
        &self,
        image: vk::Image,
        format: vk::Format,
        width: u32,
        height: u32,
        mip_levels: u32,
    ) -> Result<(), Box<dyn Error>> {
        unsafe {
            // Wait for any previous operations to complete
            self.device()
                .wait_for_fences(&[self.fence], true, std::u64::MAX)?;
            self.device().reset_fences(&[self.fence])?;

            // Store values for use in the closure to avoid capturing self
            let target_image = image;

            record_submit_commandbuffer(
                &self.device(),
                self.command_buffer,
                self.fence,
                self.vulkan_context.queue,
                &[], // No wait stages
                &[], // No wait semaphores
                &[], // No signal semaphores
                &|device: &ash::Device, command_buffer: vk::CommandBuffer| {
                    // First transition mip level 0 to TRANSFER_SRC_OPTIMAL
                    let barrier = vk::ImageMemoryBarrier::default()
                        .old_layout(vk::ImageLayout::TRANSFER_DST_OPTIMAL)
                        .new_layout(vk::ImageLayout::TRANSFER_SRC_OPTIMAL)
                        .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                        .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                        .image(target_image)
                        .subresource_range(vk::ImageSubresourceRange {
                            aspect_mask: vk::ImageAspectFlags::COLOR,
                            base_mip_level: 0,
                            level_count: 1,
                            base_array_layer: 0,
                            layer_count: 1,
                        })
                        .src_access_mask(vk::AccessFlags::TRANSFER_WRITE)
                        .dst_access_mask(vk::AccessFlags::TRANSFER_READ);

                    device.cmd_pipeline_barrier(
                        command_buffer,
                        vk::PipelineStageFlags::TRANSFER,
                        vk::PipelineStageFlags::TRANSFER,
                        vk::DependencyFlags::empty(),
                        &[],
                        &[],
                        &[barrier],
                    );

                    // For each mip level (starting from 1), blit from the previous level
                    let mut src_width = width;
                    let mut src_height = height;

                    for i in 1..mip_levels {
                        // Calculate destination dimensions
                        let dst_width = if src_width > 1 { src_width / 2 } else { 1 };
                        let dst_height = if src_height > 1 { src_height / 2 } else { 1 };

                        // Transition the destination mip level to TRANSFER_DST_OPTIMAL
                        let barrier = vk::ImageMemoryBarrier::default()
                            .old_layout(vk::ImageLayout::UNDEFINED)
                            .new_layout(vk::ImageLayout::TRANSFER_DST_OPTIMAL)
                            .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                            .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                            .image(target_image)
                            .subresource_range(vk::ImageSubresourceRange {
                                aspect_mask: vk::ImageAspectFlags::COLOR,
                                base_mip_level: i,
                                level_count: 1,
                                base_array_layer: 0,
                                layer_count: 1,
                            })
                            .src_access_mask(vk::AccessFlags::empty())
                            .dst_access_mask(vk::AccessFlags::TRANSFER_WRITE);

                        device.cmd_pipeline_barrier(
                            command_buffer,
                            vk::PipelineStageFlags::TRANSFER,
                            vk::PipelineStageFlags::TRANSFER,
                            vk::DependencyFlags::empty(),
                            &[],
                            &[],
                            &[barrier],
                        );

                        // Blit from the previous mip level to the current one
                        let blit = vk::ImageBlit::default()
                            .src_offsets([
                                vk::Offset3D { x: 0, y: 0, z: 0 },
                                vk::Offset3D {
                                    x: src_width as i32,
                                    y: src_height as i32,
                                    z: 1,
                                },
                            ])
                            .src_subresource(vk::ImageSubresourceLayers {
                                aspect_mask: vk::ImageAspectFlags::COLOR,
                                mip_level: i - 1,
                                base_array_layer: 0,
                                layer_count: 1,
                            })
                            .dst_offsets([
                                vk::Offset3D { x: 0, y: 0, z: 0 },
                                vk::Offset3D {
                                    x: dst_width as i32,
                                    y: dst_height as i32,
                                    z: 1,
                                },
                            ])
                            .dst_subresource(vk::ImageSubresourceLayers {
                                aspect_mask: vk::ImageAspectFlags::COLOR,
                                mip_level: i,
                                base_array_layer: 0,
                                layer_count: 1,
                            });

                        device.cmd_blit_image(
                            command_buffer,
                            target_image,
                            vk::ImageLayout::TRANSFER_SRC_OPTIMAL,
                            target_image,
                            vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                            &[blit],
                            vk::Filter::LINEAR,
                        );

                        // Transition the current mip level to TRANSFER_SRC_OPTIMAL for the next iteration
                        let barrier = vk::ImageMemoryBarrier::default()
                            .old_layout(vk::ImageLayout::TRANSFER_DST_OPTIMAL)
                            .new_layout(vk::ImageLayout::TRANSFER_SRC_OPTIMAL)
                            .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                            .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                            .image(target_image)
                            .subresource_range(vk::ImageSubresourceRange {
                                aspect_mask: vk::ImageAspectFlags::COLOR,
                                base_mip_level: i,
                                level_count: 1,
                                base_array_layer: 0,
                                layer_count: 1,
                            })
                            .src_access_mask(vk::AccessFlags::TRANSFER_WRITE)
                            .dst_access_mask(vk::AccessFlags::TRANSFER_READ);

                        device.cmd_pipeline_barrier(
                            command_buffer,
                            vk::PipelineStageFlags::TRANSFER,
                            vk::PipelineStageFlags::TRANSFER,
                            vk::DependencyFlags::empty(),
                            &[],
                            &[],
                            &[barrier],
                        );

                        // Update dimensions for next iteration
                        src_width = dst_width;
                        src_height = dst_height;
                    }

                    // Finally, transition all mip levels to SHADER_READ_ONLY_OPTIMAL
                    let barrier = vk::ImageMemoryBarrier::default()
                        .old_layout(vk::ImageLayout::TRANSFER_SRC_OPTIMAL)
                        .new_layout(vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL)
                        .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                        .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                        .image(target_image)
                        .subresource_range(vk::ImageSubresourceRange {
                            aspect_mask: vk::ImageAspectFlags::COLOR,
                            base_mip_level: 0,
                            level_count: mip_levels,
                            base_array_layer: 0,
                            layer_count: 1,
                        })
                        .src_access_mask(vk::AccessFlags::TRANSFER_READ)
                        .dst_access_mask(vk::AccessFlags::SHADER_READ);

                    device.cmd_pipeline_barrier(
                        command_buffer,
                        vk::PipelineStageFlags::TRANSFER,
                        vk::PipelineStageFlags::FRAGMENT_SHADER,
                        vk::DependencyFlags::empty(),
                        &[],
                        &[],
                        &[barrier],
                    );
                },
            );

            // Wait for the operation to complete
            self.device()
                .wait_for_fences(&[self.fence], true, std::u64::MAX)?;
        }
        Ok(())
    }

    /// Internal helper function for image layout transitions
    unsafe fn transition_image_layout_internal(
        &self,
        device: &ash::Device,
        command_buffer: vk::CommandBuffer,
        image: vk::Image,
        _format: vk::Format,
        old_layout: vk::ImageLayout,
        new_layout: vk::ImageLayout,
        mip_levels: u32,
    ) {
        let (src_access_mask, dst_access_mask, src_stage, dst_stage) =
            match (old_layout, new_layout) {
                (vk::ImageLayout::UNDEFINED, vk::ImageLayout::TRANSFER_DST_OPTIMAL) => (
                    vk::AccessFlags::empty(),
                    vk::AccessFlags::TRANSFER_WRITE,
                    vk::PipelineStageFlags::TOP_OF_PIPE,
                    vk::PipelineStageFlags::TRANSFER,
                ),
                (
                    vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                    vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
                ) => (
                    vk::AccessFlags::TRANSFER_WRITE,
                    vk::AccessFlags::SHADER_READ,
                    vk::PipelineStageFlags::TRANSFER,
                    vk::PipelineStageFlags::FRAGMENT_SHADER,
                ),
                (vk::ImageLayout::TRANSFER_DST_OPTIMAL, vk::ImageLayout::TRANSFER_SRC_OPTIMAL) => (
                    vk::AccessFlags::TRANSFER_WRITE,
                    vk::AccessFlags::TRANSFER_READ,
                    vk::PipelineStageFlags::TRANSFER,
                    vk::PipelineStageFlags::TRANSFER,
                ),
                _ => panic!("Unsupported layout transition!"),
            };

        let barrier = vk::ImageMemoryBarrier::default()
            .old_layout(old_layout)
            .new_layout(new_layout)
            .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
            .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
            .image(image)
            .subresource_range(vk::ImageSubresourceRange {
                aspect_mask: vk::ImageAspectFlags::COLOR,
                base_mip_level: 0,
                level_count: mip_levels,
                base_array_layer: 0,
                layer_count: 1,
            })
            .src_access_mask(src_access_mask)
            .dst_access_mask(dst_access_mask);

        device.cmd_pipeline_barrier(
            command_buffer,
            src_stage,
            dst_stage,
            vk::DependencyFlags::empty(),
            &[],
            &[],
            &[barrier],
        );
    }
}

impl Drop for TransferService {
    fn drop(&mut self) {
        unsafe {
            // Wait for any pending operations
            let _ = self
                .device()
                .wait_for_fences(&[self.fence], true, std::u64::MAX);

            // Clean up resources
            self.device().destroy_fence(self.fence, None);
            self.device()
                .free_command_buffers(self.vulkan_context.command_pool, &[self.command_buffer]);
            // Note: We don't destroy the command pool as it was passed in
        }
    }
}
