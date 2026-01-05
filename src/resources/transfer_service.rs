use ash::{
    vk::{self},
    Device,
};
use std::sync::Arc;

use crate::core::command_buffer::ManagedCommandBuffer;
use crate::core::error::{EngineError, EngineResult, VkResultExt};
use crate::core::vulkan_context::VulkanContext;

/// Service responsible for GPU data transfer operations
pub struct TransferService {
    vulkan_context: Arc<VulkanContext>,
    command_buffer: ManagedCommandBuffer,
    fence: vk::Fence,
}

impl TransferService {
    /// Create a new transfer service
    pub fn new(vulkan_context: Arc<VulkanContext>) -> EngineResult<Self> {
        let command_buffer = vulkan_context
            .command_buffer_manager
            .create_transfer_buffer()?;

        let fence_create_info =
            vk::FenceCreateInfo::default().flags(vk::FenceCreateFlags::SIGNALED);

        let fence = unsafe {
            vulkan_context
                .device
                .device
                .create_fence(&fence_create_info, None)
                .sync_context("create transfer fence")?
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

    /// Get the access masks and pipeline stages for a layout transition
    fn get_layout_transition_info(
        old_layout: vk::ImageLayout,
        new_layout: vk::ImageLayout,
    ) -> EngineResult<(
        vk::AccessFlags,
        vk::AccessFlags,
        vk::PipelineStageFlags,
        vk::PipelineStageFlags,
    )> {
        match (old_layout, new_layout) {
            (vk::ImageLayout::UNDEFINED, vk::ImageLayout::TRANSFER_DST_OPTIMAL) => Ok((
                vk::AccessFlags::empty(),
                vk::AccessFlags::TRANSFER_WRITE,
                vk::PipelineStageFlags::TOP_OF_PIPE,
                vk::PipelineStageFlags::TRANSFER,
            )),
            (vk::ImageLayout::TRANSFER_DST_OPTIMAL, vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL) => {
                Ok((
                    vk::AccessFlags::TRANSFER_WRITE,
                    vk::AccessFlags::SHADER_READ,
                    vk::PipelineStageFlags::TRANSFER,
                    vk::PipelineStageFlags::FRAGMENT_SHADER,
                ))
            }
            (vk::ImageLayout::TRANSFER_DST_OPTIMAL, vk::ImageLayout::TRANSFER_SRC_OPTIMAL) => Ok((
                vk::AccessFlags::TRANSFER_WRITE,
                vk::AccessFlags::TRANSFER_READ,
                vk::PipelineStageFlags::TRANSFER,
                vk::PipelineStageFlags::TRANSFER,
            )),
            (vk::ImageLayout::TRANSFER_SRC_OPTIMAL, vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL) => {
                Ok((
                    vk::AccessFlags::TRANSFER_READ,
                    vk::AccessFlags::SHADER_READ,
                    vk::PipelineStageFlags::TRANSFER,
                    vk::PipelineStageFlags::FRAGMENT_SHADER,
                ))
            }
            _ => Err(EngineError::Image {
                operation: crate::core::error::ImageOperation::TransitionLayout,
                width: 0,
                height: 0,
                reason: format!(
                    "unsupported layout transition: {:?} -> {:?}",
                    old_layout, new_layout
                ),
            }),
        }
    }

    /// Create an image memory barrier for layout transition
    fn create_layout_transition_barrier(
        image: vk::Image,
        old_layout: vk::ImageLayout,
        new_layout: vk::ImageLayout,
        src_access_mask: vk::AccessFlags,
        dst_access_mask: vk::AccessFlags,
        mip_levels: u32,
        base_mip_level: u32,
    ) -> vk::ImageMemoryBarrier<'static> {
        vk::ImageMemoryBarrier::default()
            .old_layout(old_layout)
            .new_layout(new_layout)
            .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
            .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
            .image(image)
            .subresource_range(vk::ImageSubresourceRange {
                aspect_mask: vk::ImageAspectFlags::COLOR,
                base_mip_level,
                level_count: mip_levels,
                base_array_layer: 0,
                layer_count: 1,
            })
            .src_access_mask(src_access_mask)
            .dst_access_mask(dst_access_mask)
    }

    /// Copy data from a buffer to an image
    pub fn copy_buffer_to_image(
        &self,
        buffer: vk::Buffer,
        image: vk::Image,
        width: u32,
        height: u32,
    ) -> EngineResult<()> {
        unsafe {
            self.device()
                .wait_for_fences(&[self.fence], true, u64::MAX)
                .sync_context("wait for transfer fence")?
        };

        unsafe {
            self.device()
                .reset_fences(&[self.fence])
                .sync_context("reset transfer fence")?
        };

        self.command_buffer
            .reset(vk::CommandBufferResetFlags::RELEASE_RESOURCES)?;

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

        self.vulkan_context
            .command_buffer_manager
            .submit_transfer_commands_async(
                &self.command_buffer,
                self.fence,
                &[],
                &[],
                &[],
                |device: &ash::Device, cb_raw: vk::CommandBuffer| unsafe {
                    device.cmd_copy_buffer_to_image(
                        cb_raw,
                        buffer,
                        image,
                        vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                        &[buffer_image_copy],
                    );
                },
            )?;

        Ok(())
    }

    /// Transition image layout from one layout to another
    pub fn transition_image_layout(
        &self,
        image: vk::Image,
        _format: vk::Format,
        old_layout: vk::ImageLayout,
        new_layout: vk::ImageLayout,
        mip_levels: u32,
    ) -> EngineResult<()> {
        unsafe {
            self.device()
                .wait_for_fences(&[self.fence], true, u64::MAX)
                .sync_context("wait for transfer fence")?
        };

        unsafe {
            self.device()
                .reset_fences(&[self.fence])
                .sync_context("reset transfer fence")?
        };

        self.command_buffer
            .reset(vk::CommandBufferResetFlags::RELEASE_RESOURCES)?;

        let (src_access_mask, dst_access_mask, src_stage, dst_stage) =
            Self::get_layout_transition_info(old_layout, new_layout)?;

        let barrier = Self::create_layout_transition_barrier(
            image,
            old_layout,
            new_layout,
            src_access_mask,
            dst_access_mask,
            mip_levels,
            0,
        );

        self.vulkan_context
            .command_buffer_manager
            .submit_transfer_commands_async(
                &self.command_buffer,
                self.fence,
                &[],
                &[],
                &[],
                move |device: &ash::Device, cb_raw: vk::CommandBuffer| unsafe {
                    device.cmd_pipeline_barrier(
                        cb_raw,
                        src_stage,
                        dst_stage,
                        vk::DependencyFlags::empty(),
                        &[],
                        &[],
                        &[barrier],
                    );
                },
            )?;

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
    ) -> EngineResult<()> {
        unsafe {
            // Check if image format supports linear blitting
            let format_properties = self
                .vulkan_context
                .instance
                .instance
                .get_physical_device_format_properties(
                    self.vulkan_context.device.physical_device,
                    format,
                );

            if !format_properties
                .optimal_tiling_features
                .contains(vk::FormatFeatureFlags::SAMPLED_IMAGE_FILTER_LINEAR)
            {
                return Err(EngineError::Image {
                    operation: crate::core::error::ImageOperation::GenerateMipmaps,
                    width,
                    height,
                    reason: "format does not support linear blitting".to_string(),
                });
            }
        }

        unsafe {
            self.device()
                .wait_for_fences(&[self.fence], true, u64::MAX)
                .sync_context("wait for transfer fence")?
        };

        unsafe {
            self.device()
                .reset_fences(&[self.fence])
                .sync_context("reset transfer fence")?
        };

        self.command_buffer
            .reset(vk::CommandBufferResetFlags::RELEASE_RESOURCES)?;

        self.vulkan_context
            .command_buffer_manager
            .submit_transfer_commands_async(
                &self.command_buffer,
                self.fence,
                &[],
                &[],
                &[],
                |device: &ash::Device, cb_raw: vk::CommandBuffer| {
                    let mut mip_width = width as i32;
                    let mut mip_height = height as i32;

                    for i in 1..mip_levels {
                        // Transition previous mip level to transfer source
                        let barrier_src = Self::create_layout_transition_barrier(
                            image,
                            vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                            vk::ImageLayout::TRANSFER_SRC_OPTIMAL,
                            vk::AccessFlags::TRANSFER_WRITE,
                            vk::AccessFlags::TRANSFER_READ,
                            1,
                            i - 1,
                        );

                        unsafe {
                            device.cmd_pipeline_barrier(
                                cb_raw,
                                vk::PipelineStageFlags::TRANSFER,
                                vk::PipelineStageFlags::TRANSFER,
                                vk::DependencyFlags::empty(),
                                &[],
                                &[],
                                &[barrier_src],
                            );
                        }

                        // Blit from previous mip level to current one
                        let blit = vk::ImageBlit::default()
                            .src_subresource(vk::ImageSubresourceLayers {
                                aspect_mask: vk::ImageAspectFlags::COLOR,
                                mip_level: i - 1,
                                base_array_layer: 0,
                                layer_count: 1,
                            })
                            .src_offsets([
                                vk::Offset3D { x: 0, y: 0, z: 0 },
                                vk::Offset3D {
                                    x: mip_width,
                                    y: mip_height,
                                    z: 1,
                                },
                            ])
                            .dst_subresource(vk::ImageSubresourceLayers {
                                aspect_mask: vk::ImageAspectFlags::COLOR,
                                mip_level: i,
                                base_array_layer: 0,
                                layer_count: 1,
                            })
                            .dst_offsets([
                                vk::Offset3D { x: 0, y: 0, z: 0 },
                                vk::Offset3D {
                                    x: if mip_width > 1 { mip_width / 2 } else { 1 },
                                    y: if mip_height > 1 { mip_height / 2 } else { 1 },
                                    z: 1,
                                },
                            ]);

                        unsafe {
                            device.cmd_blit_image(
                                cb_raw,
                                image,
                                vk::ImageLayout::TRANSFER_SRC_OPTIMAL,
                                image,
                                vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                                &[blit],
                                vk::Filter::LINEAR,
                            );
                        }

                        // Transition previous mip level to shader read
                        let barrier_shader_ro = Self::create_layout_transition_barrier(
                            image,
                            vk::ImageLayout::TRANSFER_SRC_OPTIMAL,
                            vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
                            vk::AccessFlags::TRANSFER_READ,
                            vk::AccessFlags::SHADER_READ,
                            1,
                            i - 1,
                        );

                        unsafe {
                            device.cmd_pipeline_barrier(
                                cb_raw,
                                vk::PipelineStageFlags::TRANSFER,
                                vk::PipelineStageFlags::FRAGMENT_SHADER,
                                vk::DependencyFlags::empty(),
                                &[],
                                &[],
                                &[barrier_shader_ro],
                            );
                        }

                        if mip_width > 1 {
                            mip_width /= 2;
                        }
                        if mip_height > 1 {
                            mip_height /= 2;
                        }
                    }

                    // Transition last mip level to shader read
                    let barrier_last_mip = Self::create_layout_transition_barrier(
                        image,
                        vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                        vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
                        vk::AccessFlags::TRANSFER_WRITE,
                        vk::AccessFlags::SHADER_READ,
                        1,
                        mip_levels - 1,
                    );

                    unsafe {
                        device.cmd_pipeline_barrier(
                            cb_raw,
                            vk::PipelineStageFlags::TRANSFER,
                            vk::PipelineStageFlags::FRAGMENT_SHADER,
                            vk::DependencyFlags::empty(),
                            &[],
                            &[],
                            &[barrier_last_mip],
                        );
                    }
                },
            )?;

        Ok(())
    }
}

impl Drop for TransferService {
    fn drop(&mut self) {
        unsafe {
            if self.fence != vk::Fence::null() {
                let device = self.device();
                let wait_result = device.wait_for_fences(&[self.fence], true, std::u64::MAX);
                if wait_result.is_err() {
                    // Handle or log error if waiting for fence fails, though in drop, options are limited.
                    // eprintln!("Error waiting for fence in TransferService drop: {:?}", wait_result.unwrap_err());
                }
                device.destroy_fence(self.fence, None);
            }
        }
    }
}
