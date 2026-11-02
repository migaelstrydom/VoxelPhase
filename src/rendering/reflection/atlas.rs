//! The cube-map array every probe is captured into and sampled from.

use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult, ImageOperation};
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::device_image::{DeviceImage, ImageSpec, ImageView};
use crate::rendering::reflection::config::ProbeConfig;
use crate::rendering::reflection::face::CubeFace;
use crate::rendering::reflection::pool::ProbeSlot;

/// Format of a probe's faces: the same HDR radiance the scene target holds,
/// so a reflection of a glowing object glows.
///
/// Alpha is coverage. A face is cleared to transparent and every surface
/// captured into it writes 1, so after the mips are filtered a texel's alpha
/// is the fraction of its cone that hit something, and the colour is
/// premultiplied by it. The shader composites the probe over the analytic
/// sky with that alpha, which keeps the sky exact rather than resampling it
/// at 32 texels a face.
pub const PROBE_FORMAT: vk::Format = vk::Format::R16G16B16A16_SFLOAT;

/// Depth format of the scratch buffer faces are captured against.
const PROBE_DEPTH_FORMAT: vk::Format = vk::Format::D32_SFLOAT;

/// Texels along each face's edge.
///
/// Small on purpose: a probe serves a rough reflection on a small object,
/// and what makes it read is the colour of the surroundings, not their
/// detail. Fixed rather than configured because the mip filter's workgroup is
/// shaped to it; `probe_mips.comp` must agree.
pub const PROBE_RESOLUTION: u32 = 32;

/// Mips in each face's chain, 32 texels down to 1.
pub const PROBE_MIP_LEVELS: u32 = 6;

/// The layout the whole atlas lives in, always.
///
/// A face is sampled by the scene, read by the mip filter at mip 0 and
/// written by it at the rest, all through views that span every layer. With
/// a layout per use, those views would span faces in different layouts
/// whenever some faces were mid-capture, which a descriptor cannot express.
/// `GENERAL` is valid for all three, and on a tiled GPU — MoltenVK ignores
/// layouts — it costs nothing. Only a capture's render pass moves a face out
/// of it, and back.
pub const PROBE_LAYOUT: vk::ImageLayout = vk::ImageLayout::GENERAL;

/// A cube-compatible image array with six layers per probe and a full mip
/// chain, plus what it takes to render into one face at a time.
///
/// ```text
///   layer:   0  1  2  3  4  5 | 6  7 ...
///   face:   +X −X +Y −Y +Z −Z |+X −X ...
///   probe:  ─────── 0 ─────── |── 1 ...
/// ```
///
/// A face is rendered into its mip 0 through a framebuffer of its own; its
/// smaller mips are then filtered down from it by `ProbeMipFilter`, which
/// stands in for prefiltering by roughness.
///
/// The image is shared by the frames in flight, like the shadow map: a face is
/// rewritten only after the render pass's incoming dependency has waited for
/// every earlier sampling of it.
pub struct ProbeAtlas {
    /// Every probe as a cube, every mip: what the scene samples.
    pub view: ImageView,
    pub sampler: vk::Sampler,
    /// One per layer, mip 0 only: what a face is rendered into. Held for
    /// the framebuffers built on them.
    #[allow(dead_code)]
    face_views: Vec<ImageView>,
    /// One per mip level, every layer: what the mip filter reads and writes.
    level_views: Vec<ImageView>,
    framebuffers: Vec<vk::Framebuffer>,
    image: DeviceImage,
    /// The depth faces are captured against, held for the framebuffers.
    #[allow(dead_code)]
    depth_view: ImageView,
    #[allow(dead_code)]
    depth_image: DeviceImage,
    pub render_pass: vk::RenderPass,
    pub extent: vk::Extent2D,
    device: Arc<ManagedDevice>,
}

impl ProbeAtlas {
    pub fn new(vulkan_context: &VulkanContext, config: &ProbeConfig) -> EngineResult<Self> {
        let device = Arc::clone(&vulkan_context.device);
        let extent = vk::Extent2D {
            width: PROBE_RESOLUTION,
            height: PROBE_RESOLUTION,
        };
        let layers = config.capacity * CubeFace::COUNT as u32;
        let mip_levels = PROBE_MIP_LEVELS;

        let image = DeviceImage::new(
            &device,
            ImageSpec::new_2d(
                PROBE_FORMAT,
                extent,
                vk::ImageUsageFlags::COLOR_ATTACHMENT
                    | vk::ImageUsageFlags::SAMPLED
                    | vk::ImageUsageFlags::STORAGE,
            )
            .mip_levels(mip_levels)
            .array_layers(layers, vk::ImageCreateFlags::CUBE_COMPATIBLE),
        )?;
        let colour = vk::ImageAspectFlags::COLOR;
        let view = image.full_view(vk::ImageViewType::CUBE_ARRAY, colour)?;
        let face_views = (0..layers)
            .map(|layer| image.view(vk::ImageViewType::TYPE_2D, colour, 0..1, layer..layer + 1))
            .collect::<EngineResult<Vec<_>>>()?;
        let level_views = (0..mip_levels)
            .map(|level| {
                image.view(
                    vk::ImageViewType::TYPE_2D_ARRAY,
                    colour,
                    level..level + 1,
                    0..layers,
                )
            })
            .collect::<EngineResult<Vec<_>>>()?;

        let depth_image = DeviceImage::new(
            &device,
            ImageSpec::new_2d(
                PROBE_DEPTH_FORMAT,
                extent,
                vk::ImageUsageFlags::DEPTH_STENCIL_ATTACHMENT,
            ),
        )?;
        let depth_view =
            depth_image.full_view(vk::ImageViewType::TYPE_2D, vk::ImageAspectFlags::DEPTH)?;

        let render_pass = create_render_pass(&device)?;
        let framebuffers = face_views
            .iter()
            .map(|face_view| {
                let attachments = [face_view.handle(), depth_view.handle()];
                let info = vk::FramebufferCreateInfo::default()
                    .render_pass(render_pass)
                    .attachments(&attachments)
                    .width(extent.width)
                    .height(extent.height)
                    .layers(1);
                unsafe { device.device.create_framebuffer(&info, None) }
                    .map_err(|e| EngineError::Framebuffer(format!("probe face: {:?}", e)))
            })
            .collect::<EngineResult<Vec<_>>>()?;
        let sampler = create_sampler(&device, mip_levels)?;

        let atlas = Self {
            view,
            sampler,
            face_views,
            level_views,
            framebuffers,
            image,
            depth_view,
            depth_image,
            render_pass,
            extent,
            device,
        };
        atlas.enter_layout(vulkan_context)?;
        Ok(atlas)
    }

    /// Put every face into [`PROBE_LAYOUT`], once, at creation.
    ///
    /// The descriptors name the whole array in that layout, so it has to be
    /// true of the faces no probe has been captured into yet. Their contents
    /// are undefined, which is harmless: a draw only samples the cube of a
    /// probe that has been captured.
    fn enter_layout(&self, vulkan_context: &VulkanContext) -> EngineResult<()> {
        let commands = vulkan_context
            .command_buffer_manager
            .create_one_time_submit_buffer()?;
        vulkan_context
            .command_buffer_manager
            .submit_graphics_commands_and_wait(&commands, |device, cb| {
                let barrier = vk::ImageMemoryBarrier::default()
                    .image(self.image.handle())
                    .old_layout(vk::ImageLayout::UNDEFINED)
                    .new_layout(PROBE_LAYOUT)
                    .dst_access_mask(vk::AccessFlags::SHADER_READ)
                    .subresource_range(
                        vk::ImageSubresourceRange::default()
                            .aspect_mask(vk::ImageAspectFlags::COLOR)
                            .level_count(vk::REMAINING_MIP_LEVELS)
                            .layer_count(vk::REMAINING_ARRAY_LAYERS),
                    );
                unsafe {
                    device.cmd_pipeline_barrier(
                        cb,
                        vk::PipelineStageFlags::TOP_OF_PIPE,
                        vk::PipelineStageFlags::FRAGMENT_SHADER,
                        vk::DependencyFlags::empty(),
                        &[],
                        &[],
                        &[barrier],
                    );
                }
            })
    }

    /// Every layer at one mip `level`, as a 2D array.
    pub fn level_view(&self, level: u32) -> vk::ImageView {
        self.level_views[level as usize].handle()
    }

    /// The layer holding `face` of the probe in `slot`.
    pub fn layer(slot: ProbeSlot, face: CubeFace) -> u32 {
        slot.0 * CubeFace::COUNT as u32 + face.layer()
    }

    /// Start rendering into one face, cleared to transparent and to the far
    /// plane, with the viewport over the whole face.
    pub fn begin_face(&self, cb: vk::CommandBuffer, layer: u32) {
        let clear_values = [
            vk::ClearValue {
                color: vk::ClearColorValue { float32: [0.0; 4] },
            },
            vk::ClearValue {
                depth_stencil: vk::ClearDepthStencilValue {
                    depth: 1.0,
                    stencil: 0,
                },
            },
        ];
        let begin_info = vk::RenderPassBeginInfo::default()
            .render_pass(self.render_pass)
            .framebuffer(self.framebuffers[layer as usize])
            .render_area(self.extent.into())
            .clear_values(&clear_values);
        unsafe {
            self.device
                .device
                .cmd_begin_render_pass(cb, &begin_info, vk::SubpassContents::INLINE);
        }
    }

    /// Finish rendering a face. Its mip 0 is left ready for its smaller mips
    /// to be filtered down from it.
    pub fn end_face(&self, cb: vk::CommandBuffer) {
        unsafe {
            self.device.device.cmd_end_render_pass(cb);
        }
    }
}

impl Drop for ProbeAtlas {
    fn drop(&mut self) {
        let device = &self.device.device;
        unsafe {
            for &framebuffer in &self.framebuffers {
                device.destroy_framebuffer(framebuffer, None);
            }
            device.destroy_render_pass(self.render_pass, None);
            device.destroy_sampler(self.sampler, None);
        }
    }
}

/// Trilinear, so a roughness between two mips blends them rather than
/// stepping. Cube lookups are always seamless in Vulkan, so the address mode
/// only matters within a face.
fn create_sampler(device: &ManagedDevice, mip_levels: u32) -> EngineResult<vk::Sampler> {
    let info = vk::SamplerCreateInfo::default()
        .mag_filter(vk::Filter::LINEAR)
        .min_filter(vk::Filter::LINEAR)
        .mipmap_mode(vk::SamplerMipmapMode::LINEAR)
        .address_mode_u(vk::SamplerAddressMode::CLAMP_TO_EDGE)
        .address_mode_v(vk::SamplerAddressMode::CLAMP_TO_EDGE)
        .address_mode_w(vk::SamplerAddressMode::CLAMP_TO_EDGE)
        .max_lod(mip_levels as f32);

    unsafe { device.device.create_sampler(&info, None) }.map_err(|e| EngineError::Image {
        operation: ImageOperation::Create,
        width: 0,
        height: 0,
        reason: format!("reflection probe sampler: {:?}", e),
    })
}

/// One colour and one depth attachment, both cleared on entry.
///
/// The colour ends back in [`PROBE_LAYOUT`]. The incoming dependency is what
/// makes rewriting a face safe while the frame before may still be sampling
/// it, or the mip filter reading it; the outgoing one orders the render
/// before the mip filter reads it.
fn create_render_pass(device: &ManagedDevice) -> EngineResult<vk::RenderPass> {
    let attachments = [
        vk::AttachmentDescription {
            format: PROBE_FORMAT,
            samples: vk::SampleCountFlags::TYPE_1,
            load_op: vk::AttachmentLoadOp::CLEAR,
            store_op: vk::AttachmentStoreOp::STORE,
            stencil_load_op: vk::AttachmentLoadOp::DONT_CARE,
            stencil_store_op: vk::AttachmentStoreOp::DONT_CARE,
            initial_layout: vk::ImageLayout::UNDEFINED,
            final_layout: PROBE_LAYOUT,
            ..Default::default()
        },
        vk::AttachmentDescription {
            format: PROBE_DEPTH_FORMAT,
            samples: vk::SampleCountFlags::TYPE_1,
            load_op: vk::AttachmentLoadOp::CLEAR,
            store_op: vk::AttachmentStoreOp::DONT_CARE,
            stencil_load_op: vk::AttachmentLoadOp::DONT_CARE,
            stencil_store_op: vk::AttachmentStoreOp::DONT_CARE,
            initial_layout: vk::ImageLayout::UNDEFINED,
            final_layout: vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
            ..Default::default()
        },
    ];

    let colour_ref = [vk::AttachmentReference {
        attachment: 0,
        layout: vk::ImageLayout::COLOR_ATTACHMENT_OPTIMAL,
    }];
    let depth_ref = vk::AttachmentReference {
        attachment: 1,
        layout: vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
    };
    let subpass = vk::SubpassDescription::default()
        .pipeline_bind_point(vk::PipelineBindPoint::GRAPHICS)
        .color_attachments(&colour_ref)
        .depth_stencil_attachment(&depth_ref);

    let dependencies = [
        // Before a face is rewritten: every earlier sampling of it (the frame
        // before's scene), read of it (its last mip filtering), and use of
        // the shared depth buffer (the face captured before it) is done.
        vk::SubpassDependency {
            src_subpass: vk::SUBPASS_EXTERNAL,
            dst_subpass: 0,
            src_stage_mask: vk::PipelineStageFlags::FRAGMENT_SHADER
                | vk::PipelineStageFlags::COMPUTE_SHADER
                | vk::PipelineStageFlags::LATE_FRAGMENT_TESTS,
            src_access_mask: vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_WRITE,
            dst_stage_mask: vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT
                | vk::PipelineStageFlags::EARLY_FRAGMENT_TESTS,
            dst_access_mask: vk::AccessFlags::COLOR_ATTACHMENT_WRITE
                | vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_READ
                | vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_WRITE,
            ..Default::default()
        },
        // The face is written before its mips are filtered from it.
        vk::SubpassDependency {
            src_subpass: 0,
            dst_subpass: vk::SUBPASS_EXTERNAL,
            src_stage_mask: vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT,
            src_access_mask: vk::AccessFlags::COLOR_ATTACHMENT_WRITE,
            dst_stage_mask: vk::PipelineStageFlags::COMPUTE_SHADER,
            dst_access_mask: vk::AccessFlags::SHADER_READ,
            ..Default::default()
        },
    ];

    let info = vk::RenderPassCreateInfo::default()
        .attachments(&attachments)
        .subpasses(std::slice::from_ref(&subpass))
        .dependencies(&dependencies);

    unsafe { device.device.create_render_pass(&info, None) }
        .map_err(|e| EngineError::RenderPass(format!("probe capture pass: {:?}", e)))
}
