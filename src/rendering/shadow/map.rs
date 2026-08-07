//! The depth image the sun's view is rendered into, and the pass that fills it.

use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult, ImageOperation};
use crate::core::vulkan_context::{find_memorytype_index, VulkanContext};

/// Depth format for the shadow map.
///
/// Full float rather than the `D16_UNORM` the scene depth buffer uses: the
/// light's box is over a hundred metres deep, and the comparison happens
/// against a biased value, so the extra range keeps acne out of large scenes
/// without the bias having to grow enough to detach contact shadows.
pub const SHADOW_DEPTH_FORMAT: vk::Format = vk::Format::D32_SFLOAT;

/// Layout the map sits in whenever it is not being rendered into. Depth images
/// have their own read-only layout, which is what the sampler expects.
pub const SHADOW_SAMPLED_LAYOUT: vk::ImageLayout = vk::ImageLayout::DEPTH_STENCIL_READ_ONLY_OPTIMAL;

/// A square depth target plus the render pass, framebuffer and comparison
/// sampler that go with it.
///
/// Owns no pipeline: what is drawn into it is the caller's business, which is
/// what lets the shadow pass replay the scene's ordinary draw calls.
pub struct ShadowMap {
    pub image: vk::Image,
    pub view: vk::ImageView,
    pub sampler: vk::Sampler,
    pub render_pass: vk::RenderPass,
    pub framebuffer: vk::Framebuffer,
    pub extent: vk::Extent2D,
    memory: vk::DeviceMemory,
    device: Arc<ManagedDevice>,
}

impl ShadowMap {
    pub fn new(vulkan_context: &VulkanContext, resolution: u32) -> EngineResult<Self> {
        let device = Arc::clone(&vulkan_context.device);
        let extent = vk::Extent2D {
            width: resolution,
            height: resolution,
        };

        let image = create_image(&device, extent)?;
        let memory = allocate_and_bind(&device, image, extent)?;
        let view = create_view(&device, image, extent)?;
        let sampler = create_comparison_sampler(&device)?;
        let render_pass = create_render_pass(&device)?;

        let attachments = [view];
        let fb_info = vk::FramebufferCreateInfo::default()
            .render_pass(render_pass)
            .attachments(&attachments)
            .width(extent.width)
            .height(extent.height)
            .layers(1);

        let framebuffer = unsafe { device.device.create_framebuffer(&fb_info, None) }
            .map_err(|e| EngineError::Framebuffer(format!("shadow map creation: {:?}", e)))?;

        Ok(Self {
            image,
            view,
            sampler,
            render_pass,
            framebuffer,
            extent,
            memory,
            device,
        })
    }

    /// Start the depth-only pass and set the viewport to the whole map.
    pub fn begin_pass(&self, cb: vk::CommandBuffer) {
        let clear_values = [vk::ClearValue {
            depth_stencil: vk::ClearDepthStencilValue {
                depth: 1.0,
                stencil: 0,
            },
        }];

        let begin_info = vk::RenderPassBeginInfo::default()
            .render_pass(self.render_pass)
            .framebuffer(self.framebuffer)
            .render_area(self.extent.into())
            .clear_values(&clear_values);

        let viewports = [vk::Viewport {
            x: 0.0,
            y: 0.0,
            width: self.extent.width as f32,
            height: self.extent.height as f32,
            min_depth: 0.0,
            max_depth: 1.0,
        }];
        let scissors = [self.extent.into()];

        unsafe {
            self.device
                .device
                .cmd_begin_render_pass(cb, &begin_info, vk::SubpassContents::INLINE);
            self.device.device.cmd_set_viewport(cb, 0, &viewports);
            self.device.device.cmd_set_scissor(cb, 0, &scissors);
        }
    }

    /// End the depth-only pass. The render pass leaves the image in
    /// [`SHADOW_SAMPLED_LAYOUT`], ready for the geometry pass to read.
    pub fn end_pass(&self, cb: vk::CommandBuffer) {
        unsafe {
            self.device.device.cmd_end_render_pass(cb);
        }
    }
}

impl Drop for ShadowMap {
    fn drop(&mut self) {
        unsafe {
            self.device
                .device
                .destroy_framebuffer(self.framebuffer, None);
            self.device
                .device
                .destroy_render_pass(self.render_pass, None);
            self.device.device.destroy_sampler(self.sampler, None);
            self.device.device.destroy_image_view(self.view, None);
            self.device.device.destroy_image(self.image, None);
            self.device.device.free_memory(self.memory, None);
        }
    }
}

fn create_image(device: &ManagedDevice, extent: vk::Extent2D) -> EngineResult<vk::Image> {
    let image_info = vk::ImageCreateInfo::default()
        .image_type(vk::ImageType::TYPE_2D)
        .format(SHADOW_DEPTH_FORMAT)
        .extent(extent.into())
        .mip_levels(1)
        .array_layers(1)
        .samples(vk::SampleCountFlags::TYPE_1)
        .tiling(vk::ImageTiling::OPTIMAL)
        .usage(vk::ImageUsageFlags::DEPTH_STENCIL_ATTACHMENT | vk::ImageUsageFlags::SAMPLED)
        .sharing_mode(vk::SharingMode::EXCLUSIVE);

    unsafe { device.device.create_image(&image_info, None) }.map_err(|e| EngineError::Image {
        operation: ImageOperation::Create,
        width: extent.width,
        height: extent.height,
        reason: format!("shadow map: {:?}", e),
    })
}

fn allocate_and_bind(
    device: &ManagedDevice,
    image: vk::Image,
    extent: vk::Extent2D,
) -> EngineResult<vk::DeviceMemory> {
    unsafe {
        let requirements = device.device.get_image_memory_requirements(image);
        let memory_type_index = find_memorytype_index(
            &requirements,
            &device.device_memory_properties,
            vk::MemoryPropertyFlags::DEVICE_LOCAL,
        )
        .ok_or_else(|| EngineError::Image {
            operation: ImageOperation::AllocateMemory,
            width: extent.width,
            height: extent.height,
            reason: "shadow map: no suitable memory type".to_string(),
        })?;

        let alloc_info = vk::MemoryAllocateInfo::default()
            .allocation_size(requirements.size)
            .memory_type_index(memory_type_index);

        let memory = device
            .device
            .allocate_memory(&alloc_info, None)
            .map_err(|e| EngineError::Image {
                operation: ImageOperation::AllocateMemory,
                width: extent.width,
                height: extent.height,
                reason: format!("shadow map: {:?}", e),
            })?;

        device
            .device
            .bind_image_memory(image, memory, 0)
            .map_err(|e| EngineError::Image {
                operation: ImageOperation::Bind,
                width: extent.width,
                height: extent.height,
                reason: format!("shadow map: {:?}", e),
            })?;

        Ok(memory)
    }
}

fn create_view(
    device: &ManagedDevice,
    image: vk::Image,
    extent: vk::Extent2D,
) -> EngineResult<vk::ImageView> {
    let view_info = vk::ImageViewCreateInfo::default()
        .image(image)
        .view_type(vk::ImageViewType::TYPE_2D)
        .format(SHADOW_DEPTH_FORMAT)
        .subresource_range(
            vk::ImageSubresourceRange::default()
                .aspect_mask(vk::ImageAspectFlags::DEPTH)
                .level_count(1)
                .layer_count(1),
        );

    unsafe { device.device.create_image_view(&view_info, None) }.map_err(|e| EngineError::Image {
        operation: ImageOperation::CreateView,
        width: extent.width,
        height: extent.height,
        reason: format!("shadow map: {:?}", e),
    })
}

/// A sampler that compares rather than fetches.
///
/// With `LINEAR` filtering on a comparison sampler the hardware bilinearly
/// blends the four comparison *results*, so one tap already returns a partial
/// occlusion. That is what makes a 3x3 kernel affordable and soft.
///
/// Clamping to a white border matters: it is what makes geometry outside the
/// map read as lit. `CLAMP_TO_EDGE` would instead smear the map's outermost
/// row across the entire rest of the world.
fn create_comparison_sampler(device: &ManagedDevice) -> EngineResult<vk::Sampler> {
    let info = vk::SamplerCreateInfo::default()
        .mag_filter(vk::Filter::LINEAR)
        .min_filter(vk::Filter::LINEAR)
        .mipmap_mode(vk::SamplerMipmapMode::NEAREST)
        .address_mode_u(vk::SamplerAddressMode::CLAMP_TO_BORDER)
        .address_mode_v(vk::SamplerAddressMode::CLAMP_TO_BORDER)
        .address_mode_w(vk::SamplerAddressMode::CLAMP_TO_BORDER)
        .border_color(vk::BorderColor::FLOAT_OPAQUE_WHITE)
        .compare_enable(true)
        .compare_op(vk::CompareOp::LESS_OR_EQUAL)
        .max_lod(0.0);

    unsafe { device.device.create_sampler(&info, None) }.map_err(|e| EngineError::Image {
        operation: ImageOperation::Create,
        width: 0,
        height: 0,
        reason: format!("shadow comparison sampler: {:?}", e),
    })
}

/// A single depth attachment, cleared on entry and left readable on exit.
///
/// The dependency out to `EXTERNAL` is what orders the write against the
/// geometry pass's sampling of it — the two are recorded into separate command
/// buffers, so nothing else would.
fn create_render_pass(device: &ManagedDevice) -> EngineResult<vk::RenderPass> {
    let attachments = [vk::AttachmentDescription {
        format: SHADOW_DEPTH_FORMAT,
        samples: vk::SampleCountFlags::TYPE_1,
        load_op: vk::AttachmentLoadOp::CLEAR,
        store_op: vk::AttachmentStoreOp::STORE,
        stencil_load_op: vk::AttachmentLoadOp::DONT_CARE,
        stencil_store_op: vk::AttachmentStoreOp::DONT_CARE,
        // Contents are cleared every frame, so the previous layout is worthless
        // and declaring it UNDEFINED lets the driver skip a decompress.
        initial_layout: vk::ImageLayout::UNDEFINED,
        final_layout: SHADOW_SAMPLED_LAYOUT,
        ..Default::default()
    }];

    let depth_ref = vk::AttachmentReference {
        attachment: 0,
        layout: vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
    };

    let subpass = vk::SubpassDescription::default()
        .depth_stencil_attachment(&depth_ref)
        .pipeline_bind_point(vk::PipelineBindPoint::GRAPHICS);

    let dependencies = [
        // Last frame's sampling must finish before this frame overwrites it.
        vk::SubpassDependency {
            src_subpass: vk::SUBPASS_EXTERNAL,
            dst_subpass: 0,
            src_stage_mask: vk::PipelineStageFlags::FRAGMENT_SHADER,
            src_access_mask: vk::AccessFlags::SHADER_READ,
            dst_stage_mask: vk::PipelineStageFlags::EARLY_FRAGMENT_TESTS,
            dst_access_mask: vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_WRITE,
            ..Default::default()
        },
        // This frame's write must finish before the geometry pass samples it.
        vk::SubpassDependency {
            src_subpass: 0,
            dst_subpass: vk::SUBPASS_EXTERNAL,
            src_stage_mask: vk::PipelineStageFlags::LATE_FRAGMENT_TESTS,
            src_access_mask: vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_WRITE,
            dst_stage_mask: vk::PipelineStageFlags::FRAGMENT_SHADER,
            dst_access_mask: vk::AccessFlags::SHADER_READ,
            ..Default::default()
        },
    ];

    let create_info = vk::RenderPassCreateInfo::default()
        .attachments(&attachments)
        .subpasses(std::slice::from_ref(&subpass))
        .dependencies(&dependencies);

    unsafe { device.device.create_render_pass(&create_info, None) }
        .map_err(|e| EngineError::RenderPass(format!("shadow pass creation: {:?}", e)))
}
