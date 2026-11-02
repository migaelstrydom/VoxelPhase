//! Copying a finished image back to host memory.

use ash::vk;

use crate::core::error::EngineResult;
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::frame::ManagedBuffer;

/// Bytes per pixel of the 8-bit, four-channel images read back.
pub const BYTES_PER_PIXEL: u32 = 4;

/// Copies `image`, a finished frame of `extent` in 8-bit, four-channel
/// pixels, into `buffer` and returns its bytes, tightly packed. The image is
/// in `layout`, and is left in it. The caller must have waited for the frame
/// to complete first; this waits for the copy.
pub fn read_back_image(
    vulkan_context: &VulkanContext,
    image: vk::Image,
    layout: vk::ImageLayout,
    extent: vk::Extent2D,
    buffer: &ManagedBuffer,
) -> EngineResult<Vec<u8>> {
    let cmd_buffer = vulkan_context
        .command_buffer_manager
        .create_one_time_submit_buffer()?;
    let colour = vk::ImageSubresourceRange::default()
        .aspect_mask(vk::ImageAspectFlags::COLOR)
        .level_count(1)
        .layer_count(1);
    let transition = |from, to, src_access, dst_access| {
        vk::ImageMemoryBarrier::default()
            .old_layout(from)
            .new_layout(to)
            .src_access_mask(src_access)
            .dst_access_mask(dst_access)
            .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
            .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
            .image(image)
            .subresource_range(colour)
    };
    let moved = layout != vk::ImageLayout::TRANSFER_SRC_OPTIMAL;

    vulkan_context
        .command_buffer_manager
        .submit_graphics_commands_and_wait(&cmd_buffer, |device, cb| {
            let region = vk::BufferImageCopy::default()
                .image_subresource(
                    vk::ImageSubresourceLayers::default()
                        .aspect_mask(vk::ImageAspectFlags::COLOR)
                        .layer_count(1),
                )
                .image_extent(extent.into());

            unsafe {
                if moved {
                    device.cmd_pipeline_barrier(
                        cb,
                        vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT,
                        vk::PipelineStageFlags::TRANSFER,
                        vk::DependencyFlags::empty(),
                        &[],
                        &[],
                        &[transition(
                            layout,
                            vk::ImageLayout::TRANSFER_SRC_OPTIMAL,
                            vk::AccessFlags::COLOR_ATTACHMENT_WRITE,
                            vk::AccessFlags::TRANSFER_READ,
                        )],
                    );
                }

                device.cmd_copy_image_to_buffer(
                    cb,
                    image,
                    vk::ImageLayout::TRANSFER_SRC_OPTIMAL,
                    buffer.buffer,
                    &[region],
                );

                // Make the transfer write visible to the host map below.
                let barrier = vk::BufferMemoryBarrier::default()
                    .src_access_mask(vk::AccessFlags::TRANSFER_WRITE)
                    .dst_access_mask(vk::AccessFlags::HOST_READ)
                    .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                    .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
                    .buffer(buffer.buffer)
                    .size(vk::WHOLE_SIZE);
                let restored = moved.then(|| {
                    transition(
                        vk::ImageLayout::TRANSFER_SRC_OPTIMAL,
                        layout,
                        vk::AccessFlags::TRANSFER_READ,
                        vk::AccessFlags::empty(),
                    )
                });

                device.cmd_pipeline_barrier(
                    cb,
                    vk::PipelineStageFlags::TRANSFER,
                    vk::PipelineStageFlags::HOST | vk::PipelineStageFlags::BOTTOM_OF_PIPE,
                    vk::DependencyFlags::empty(),
                    &[],
                    &[barrier],
                    restored.as_slice(),
                );
            }
        })?;

    let byte_count = (extent.width * extent.height * BYTES_PER_PIXEL) as usize;

    unsafe {
        let mapped = buffer.mapped()?;
        let pixels = std::slice::from_raw_parts(mapped as *const u8, byte_count).to_vec();
        Ok(pixels)
    }
}
