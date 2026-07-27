//! Render passes and pipelines for the post-processing chain.
//!
//! Every stage is a fullscreen triangle that samples one or two textures and
//! writes one colour attachment, so they share a single pipeline builder and
//! differ only in fragment shader, descriptor layout and target render pass.

use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult};
use crate::rendering::shaders::ShaderManager;

/// Size of the `vec4` push-constant block every post stage uses for tuning.
pub const POST_PUSH_CONSTANT_SIZE: u32 = 16;

/// How a post stage combines its output with the target.
#[derive(Clone, Copy, Debug)]
enum BlendMode {
    /// Overwrite the target.
    Replace,
    /// Add to what is already there.
    Additive,
}

/// Render passes, pipelines and descriptor layouts for post-processing.
pub struct PostPipelines {
    device: Arc<ManagedDevice>,

    /// Renders into an offscreen `PostTarget`, leaving it ready to sample.
    pub offscreen_pass: vk::RenderPass,

    /// Renders into a swapchain image, leaving it as a colour attachment so the
    /// existing transparent pass can continue compositing onto it.
    pub composite_pass: vk::RenderPass,

    /// One combined image sampler — used by the bright pass and the blur.
    pub single_sampler_layout: vk::DescriptorSetLayout,

    pub bright_pass: vk::Pipeline,
    pub bright_pass_layout: vk::PipelineLayout,

    pub blur: vk::Pipeline,
    pub blur_layout: vk::PipelineLayout,

    pub composite: vk::Pipeline,
    pub composite_layout: vk::PipelineLayout,

    /// Adds bloom over the finished frame and leaves it ready to present.
    pub bloom_overlay_pass: vk::RenderPass,
    pub bloom_overlay: vk::Pipeline,
    pub bloom_overlay_layout: vk::PipelineLayout,
}

impl PostPipelines {
    pub fn new(
        device: Arc<ManagedDevice>,
        bloom_format: vk::Format,
        swapchain_format: vk::Format,
    ) -> EngineResult<Self> {
        let offscreen_pass = Self::create_offscreen_pass(&device, bloom_format)?;
        let composite_pass = Self::create_composite_pass(&device, swapchain_format)?;
        let bloom_overlay_pass = Self::create_bloom_overlay_pass(&device, swapchain_format)?;

        // Every stage samples exactly one texture.
        let single_sampler_layout = Self::create_sampler_layout(&device, 1)?;

        let bright_pass_layout = Self::create_pipeline_layout(&device, single_sampler_layout)?;
        let blur_layout = Self::create_pipeline_layout(&device, single_sampler_layout)?;
        let composite_layout = Self::create_pipeline_layout(&device, single_sampler_layout)?;
        let bloom_overlay_layout = Self::create_pipeline_layout(&device, single_sampler_layout)?;

        let bright_pass = Self::create_fullscreen_pipeline(
            &device,
            offscreen_pass,
            bright_pass_layout,
            ShaderManager::load_post_bright_pass(&device)?,
            BlendMode::Replace,
        )?;

        let blur = Self::create_fullscreen_pipeline(
            &device,
            offscreen_pass,
            blur_layout,
            ShaderManager::load_post_blur(&device)?,
            BlendMode::Replace,
        )?;

        let composite = Self::create_fullscreen_pipeline(
            &device,
            composite_pass,
            composite_layout,
            ShaderManager::load_post_composite(&device)?,
            BlendMode::Replace,
        )?;

        let bloom_overlay = Self::create_fullscreen_pipeline(
            &device,
            bloom_overlay_pass,
            bloom_overlay_layout,
            ShaderManager::load_post_bloom_overlay(&device)?,
            BlendMode::Additive,
        )?;

        Ok(Self {
            device,
            offscreen_pass,
            composite_pass,
            single_sampler_layout,
            bright_pass,
            bright_pass_layout,
            blur,
            blur_layout,
            composite,
            composite_layout,
            bloom_overlay_pass,
            bloom_overlay,
            bloom_overlay_layout,
        })
    }

    /// Colour-only pass whose attachment is fully overwritten, so its previous
    /// contents are discarded and it ends ready for sampling.
    fn create_offscreen_pass(
        device: &ManagedDevice,
        format: vk::Format,
    ) -> EngineResult<vk::RenderPass> {
        let attachment = vk::AttachmentDescription {
            format,
            samples: vk::SampleCountFlags::TYPE_1,
            load_op: vk::AttachmentLoadOp::DONT_CARE,
            store_op: vk::AttachmentStoreOp::STORE,
            initial_layout: vk::ImageLayout::UNDEFINED,
            final_layout: vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
            ..Default::default()
        };

        Self::create_single_attachment_pass(device, attachment, "post offscreen")
    }

    /// Writes the tonemapped result into a swapchain image, leaving it in
    /// `COLOR_ATTACHMENT_OPTIMAL` for the transparent pass that follows.
    fn create_composite_pass(
        device: &ManagedDevice,
        format: vk::Format,
    ) -> EngineResult<vk::RenderPass> {
        let attachment = vk::AttachmentDescription {
            format,
            samples: vk::SampleCountFlags::TYPE_1,
            load_op: vk::AttachmentLoadOp::DONT_CARE,
            store_op: vk::AttachmentStoreOp::STORE,
            initial_layout: vk::ImageLayout::UNDEFINED,
            final_layout: vk::ImageLayout::COLOR_ATTACHMENT_OPTIMAL,
            ..Default::default()
        };

        Self::create_single_attachment_pass(device, attachment, "post composite")
    }

    /// Blends bloom onto the completed frame and hands it to the presentation
    /// engine. Loads existing contents rather than discarding them.
    fn create_bloom_overlay_pass(
        device: &ManagedDevice,
        format: vk::Format,
    ) -> EngineResult<vk::RenderPass> {
        let attachment = vk::AttachmentDescription {
            format,
            samples: vk::SampleCountFlags::TYPE_1,
            load_op: vk::AttachmentLoadOp::LOAD,
            store_op: vk::AttachmentStoreOp::STORE,
            initial_layout: vk::ImageLayout::COLOR_ATTACHMENT_OPTIMAL,
            final_layout: vk::ImageLayout::PRESENT_SRC_KHR,
            ..Default::default()
        };

        Self::create_single_attachment_pass(device, attachment, "bloom overlay")
    }

    fn create_single_attachment_pass(
        device: &ManagedDevice,
        attachment: vk::AttachmentDescription,
        label: &str,
    ) -> EngineResult<vk::RenderPass> {
        let color_ref = vk::AttachmentReference {
            attachment: 0,
            layout: vk::ImageLayout::COLOR_ATTACHMENT_OPTIMAL,
        };

        let subpass = vk::SubpassDescription::default()
            .pipeline_bind_point(vk::PipelineBindPoint::GRAPHICS)
            .color_attachments(std::slice::from_ref(&color_ref));

        // The source texture was written by the previous stage's colour output;
        // wait for it before sampling in the fragment shader.
        let dependencies = [
            vk::SubpassDependency {
                src_subpass: vk::SUBPASS_EXTERNAL,
                dst_subpass: 0,
                src_stage_mask: vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT,
                dst_stage_mask: vk::PipelineStageFlags::FRAGMENT_SHADER
                    | vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT,
                src_access_mask: vk::AccessFlags::COLOR_ATTACHMENT_WRITE,
                dst_access_mask: vk::AccessFlags::SHADER_READ
                    | vk::AccessFlags::COLOR_ATTACHMENT_WRITE,
                ..Default::default()
            },
            vk::SubpassDependency {
                src_subpass: 0,
                dst_subpass: vk::SUBPASS_EXTERNAL,
                src_stage_mask: vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT,
                dst_stage_mask: vk::PipelineStageFlags::FRAGMENT_SHADER,
                src_access_mask: vk::AccessFlags::COLOR_ATTACHMENT_WRITE,
                dst_access_mask: vk::AccessFlags::SHADER_READ,
                ..Default::default()
            },
        ];

        let create_info = vk::RenderPassCreateInfo::default()
            .attachments(std::slice::from_ref(&attachment))
            .subpasses(std::slice::from_ref(&subpass))
            .dependencies(&dependencies);

        unsafe { device.device.create_render_pass(&create_info, None) }
            .map_err(|e| EngineError::Pipeline(format!("{} render pass: {:?}", label, e)))
    }

    fn create_sampler_layout(
        device: &ManagedDevice,
        count: u32,
    ) -> EngineResult<vk::DescriptorSetLayout> {
        let bindings: Vec<_> = (0..count)
            .map(|binding| {
                vk::DescriptorSetLayoutBinding::default()
                    .binding(binding)
                    .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                    .descriptor_count(1)
                    .stage_flags(vk::ShaderStageFlags::FRAGMENT)
            })
            .collect();

        let create_info = vk::DescriptorSetLayoutCreateInfo::default().bindings(&bindings);

        unsafe {
            device
                .device
                .create_descriptor_set_layout(&create_info, None)
        }
        .map_err(|e| EngineError::Pipeline(format!("post descriptor layout: {:?}", e)))
    }

    fn create_pipeline_layout(
        device: &ManagedDevice,
        set_layout: vk::DescriptorSetLayout,
    ) -> EngineResult<vk::PipelineLayout> {
        let push_constant_range = vk::PushConstantRange {
            stage_flags: vk::ShaderStageFlags::FRAGMENT,
            offset: 0,
            size: POST_PUSH_CONSTANT_SIZE,
        };

        let create_info = vk::PipelineLayoutCreateInfo::default()
            .set_layouts(std::slice::from_ref(&set_layout))
            .push_constant_ranges(std::slice::from_ref(&push_constant_range));

        unsafe { device.device.create_pipeline_layout(&create_info, None) }
            .map_err(|e| EngineError::Pipeline(format!("post pipeline layout: {:?}", e)))
    }

    /// Build a fullscreen-triangle pipeline: no vertex input, no depth, no blend.
    /// Consumes `fragment_module`, destroying it once the pipeline is built.
    fn create_fullscreen_pipeline(
        device: &ManagedDevice,
        render_pass: vk::RenderPass,
        layout: vk::PipelineLayout,
        fragment_module: vk::ShaderModule,
        blend: BlendMode,
    ) -> EngineResult<vk::Pipeline> {
        let vertex_module = ShaderManager::load_post_fullscreen_vertex(device)?;

        let entry_name = c"main";
        let shader_stages = [
            vk::PipelineShaderStageCreateInfo::default()
                .stage(vk::ShaderStageFlags::VERTEX)
                .module(vertex_module)
                .name(entry_name),
            vk::PipelineShaderStageCreateInfo::default()
                .stage(vk::ShaderStageFlags::FRAGMENT)
                .module(fragment_module)
                .name(entry_name),
        ];

        let vertex_input_state = vk::PipelineVertexInputStateCreateInfo::default();

        let input_assembly = vk::PipelineInputAssemblyStateCreateInfo::default()
            .topology(vk::PrimitiveTopology::TRIANGLE_LIST);

        let viewport_state = vk::PipelineViewportStateCreateInfo::default()
            .viewport_count(1)
            .scissor_count(1);

        let rasterization = vk::PipelineRasterizationStateCreateInfo::default()
            .polygon_mode(vk::PolygonMode::FILL)
            .cull_mode(vk::CullModeFlags::NONE)
            .front_face(vk::FrontFace::COUNTER_CLOCKWISE)
            .line_width(1.0);

        let multisample = vk::PipelineMultisampleStateCreateInfo::default()
            .rasterization_samples(vk::SampleCountFlags::TYPE_1);

        let depth_stencil = vk::PipelineDepthStencilStateCreateInfo::default()
            .depth_test_enable(false)
            .depth_write_enable(false);

        let color_blend_attachment = match blend {
            BlendMode::Replace => vk::PipelineColorBlendAttachmentState {
                blend_enable: vk::FALSE,
                color_write_mask: vk::ColorComponentFlags::RGBA,
                ..Default::default()
            },
            BlendMode::Additive => vk::PipelineColorBlendAttachmentState {
                blend_enable: vk::TRUE,
                src_color_blend_factor: vk::BlendFactor::ONE,
                dst_color_blend_factor: vk::BlendFactor::ONE,
                color_blend_op: vk::BlendOp::ADD,
                src_alpha_blend_factor: vk::BlendFactor::ONE,
                dst_alpha_blend_factor: vk::BlendFactor::ONE,
                alpha_blend_op: vk::BlendOp::ADD,
                color_write_mask: vk::ColorComponentFlags::RGBA,
            },
        };

        let color_blend = vk::PipelineColorBlendStateCreateInfo::default()
            .attachments(std::slice::from_ref(&color_blend_attachment));

        let dynamic_states = [vk::DynamicState::VIEWPORT, vk::DynamicState::SCISSOR];
        let dynamic_state =
            vk::PipelineDynamicStateCreateInfo::default().dynamic_states(&dynamic_states);

        let pipeline_info = vk::GraphicsPipelineCreateInfo::default()
            .stages(&shader_stages)
            .vertex_input_state(&vertex_input_state)
            .input_assembly_state(&input_assembly)
            .viewport_state(&viewport_state)
            .rasterization_state(&rasterization)
            .multisample_state(&multisample)
            .depth_stencil_state(&depth_stencil)
            .color_blend_state(&color_blend)
            .dynamic_state(&dynamic_state)
            .layout(layout)
            .render_pass(render_pass);

        unsafe {
            let result = device.device.create_graphics_pipelines(
                vk::PipelineCache::null(),
                &[pipeline_info],
                None,
            );

            device.device.destroy_shader_module(vertex_module, None);
            device.device.destroy_shader_module(fragment_module, None);

            let pipelines = result
                .map_err(|(_, e)| EngineError::Pipeline(format!("post creation: {:?}", e)))?;

            Ok(pipelines[0])
        }
    }
}

impl Drop for PostPipelines {
    fn drop(&mut self) {
        unsafe {
            let device = &self.device.device;
            device.destroy_pipeline(self.bright_pass, None);
            device.destroy_pipeline(self.blur, None);
            device.destroy_pipeline(self.composite, None);
            device.destroy_pipeline(self.bloom_overlay, None);
            device.destroy_pipeline_layout(self.bright_pass_layout, None);
            device.destroy_pipeline_layout(self.blur_layout, None);
            device.destroy_pipeline_layout(self.composite_layout, None);
            device.destroy_pipeline_layout(self.bloom_overlay_layout, None);
            device.destroy_descriptor_set_layout(self.single_sampler_layout, None);
            device.destroy_render_pass(self.offscreen_pass, None);
            device.destroy_render_pass(self.composite_pass, None);
            device.destroy_render_pass(self.bloom_overlay_pass, None);
        }
    }
}
