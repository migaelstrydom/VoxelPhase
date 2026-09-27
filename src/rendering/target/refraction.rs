//! A copy of the scene as it stands before the water is drawn, for the water
//! to refract and to measure its depth against.
//!
//! The water is drawn into the scene target it reads, so it cannot sample that
//! target directly. The scene pass is ended once everything beyond the water
//! is down, both attachments are copied here, and a pass that loads them
//! resumes the scene:
//!
//! ```text
//!   scene pass ──▶ colour, depth ─copy─▶ RefractionCopy ──sampled by──▶ water
//!        │                                                               │
//!        └──────────────── resumed scene pass (loads both) ◀─────────────┘
//! ```
//!
//! Only the pixels the water can read are copied (`ScreenFootprint`).
//!
//! One copy shared by every frame in flight, like the targets it copies: the
//! barrier that opens [`RefractionCopy::capture`] waits for any earlier
//! frame's water to finish reading it.

use ash::vk;

use crate::core::error::EngineResult;
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::target::images::{ColorTarget, DepthBuffer};

/// The scene's colour and depth, copied between the halves of the scene pass.
pub struct RefractionCopy {
    colour: ColorTarget,
    depth: DepthBuffer,
}

impl RefractionCopy {
    pub fn new(
        vulkan_context: &VulkanContext,
        extent: vk::Extent2D,
        colour_format: vk::Format,
        depth_format: vk::Format,
    ) -> EngineResult<Self> {
        let usage = vk::ImageUsageFlags::TRANSFER_DST | vk::ImageUsageFlags::SAMPLED;
        Ok(Self {
            colour: ColorTarget::with_usage(vulkan_context, extent, colour_format, usage)?,
            depth: DepthBuffer::with_usage(vulkan_context, extent, depth_format, usage)?,
        })
    }

    pub fn colour_view(&self) -> vk::ImageView {
        self.colour.view
    }

    pub fn depth_view(&self) -> vk::ImageView {
        self.depth.view
    }

    /// Copy the scene's colour and depth within `region`, as the scene pass
    /// left them. The copies hold nothing meaningful outside it.
    ///
    /// Expects the scene colour in `SHADER_READ_ONLY_OPTIMAL` and its depth in
    /// `DEPTH_STENCIL_ATTACHMENT_OPTIMAL`, which is how the scene pass ends,
    /// and leaves both in `TRANSFER_SRC_OPTIMAL`, which is where the resumed
    /// pass picks them up. The copies are left `SHADER_READ_ONLY_OPTIMAL` for
    /// the water's fragment shader.
    pub fn capture(
        &self,
        device: &ash::Device,
        cb: vk::CommandBuffer,
        scene_colour: vk::Image,
        scene_depth: vk::Image,
        region: vk::Rect2D,
    ) {
        let colour = vk::ImageAspectFlags::COLOR;
        let depth = vk::ImageAspectFlags::DEPTH;
        let depth_writes = vk::PipelineStageFlags::EARLY_FRAGMENT_TESTS
            | vk::PipelineStageFlags::LATE_FRAGMENT_TESTS;

        let before = [
            barrier(
                scene_colour,
                colour,
                vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
                vk::ImageLayout::TRANSFER_SRC_OPTIMAL,
                vk::AccessFlags::COLOR_ATTACHMENT_WRITE,
                vk::AccessFlags::TRANSFER_READ,
            ),
            barrier(
                scene_depth,
                depth,
                vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                vk::ImageLayout::TRANSFER_SRC_OPTIMAL,
                vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_WRITE,
                vk::AccessFlags::TRANSFER_READ,
            ),
            // The old contents are last frame's, and are being replaced.
            barrier(
                self.colour.image,
                colour,
                vk::ImageLayout::UNDEFINED,
                vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                vk::AccessFlags::empty(),
                vk::AccessFlags::TRANSFER_WRITE,
            ),
            barrier(
                self.depth.image,
                depth,
                vk::ImageLayout::UNDEFINED,
                vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                vk::AccessFlags::empty(),
                vk::AccessFlags::TRANSFER_WRITE,
            ),
        ];
        let after = [
            barrier(
                self.colour.image,
                colour,
                vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
                vk::AccessFlags::TRANSFER_WRITE,
                vk::AccessFlags::SHADER_READ,
            ),
            barrier(
                self.depth.image,
                depth,
                vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
                vk::AccessFlags::TRANSFER_WRITE,
                vk::AccessFlags::SHADER_READ,
            ),
        ];

        unsafe {
            // The source stages cover the scene pass's writes and, for the
            // copies' own images, an earlier frame's water still sampling
            // them.
            device.cmd_pipeline_barrier(
                cb,
                vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT
                    | depth_writes
                    | vk::PipelineStageFlags::FRAGMENT_SHADER,
                vk::PipelineStageFlags::TRANSFER,
                vk::DependencyFlags::empty(),
                &[],
                &[],
                &before,
            );
            device.cmd_copy_image(
                cb,
                scene_colour,
                vk::ImageLayout::TRANSFER_SRC_OPTIMAL,
                self.colour.image,
                vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                &[copy_region(colour, region)],
            );
            device.cmd_copy_image(
                cb,
                scene_depth,
                vk::ImageLayout::TRANSFER_SRC_OPTIMAL,
                self.depth.image,
                vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                &[copy_region(depth, region)],
            );
            device.cmd_pipeline_barrier(
                cb,
                vk::PipelineStageFlags::TRANSFER,
                vk::PipelineStageFlags::FRAGMENT_SHADER,
                vk::DependencyFlags::empty(),
                &[],
                &[],
                &after,
            );
        }
    }
}

fn copy_region(aspect: vk::ImageAspectFlags, region: vk::Rect2D) -> vk::ImageCopy {
    let layers = vk::ImageSubresourceLayers::default()
        .aspect_mask(aspect)
        .layer_count(1);
    let offset = vk::Offset3D {
        x: region.offset.x,
        y: region.offset.y,
        z: 0,
    };
    vk::ImageCopy::default()
        .src_subresource(layers)
        .dst_subresource(layers)
        .src_offset(offset)
        .dst_offset(offset)
        .extent(region.extent.into())
}

fn barrier(
    image: vk::Image,
    aspect: vk::ImageAspectFlags,
    old_layout: vk::ImageLayout,
    new_layout: vk::ImageLayout,
    src_access: vk::AccessFlags,
    dst_access: vk::AccessFlags,
) -> vk::ImageMemoryBarrier<'static> {
    vk::ImageMemoryBarrier::default()
        .image(image)
        .old_layout(old_layout)
        .new_layout(new_layout)
        .src_access_mask(src_access)
        .dst_access_mask(dst_access)
        .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
        .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
        .subresource_range(
            vk::ImageSubresourceRange::default()
                .aspect_mask(aspect)
                .level_count(1)
                .layer_count(1),
        )
}
