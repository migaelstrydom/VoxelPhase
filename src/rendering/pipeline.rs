//! Graphics pipeline management.
//!
//! This module encapsulates the immutable graphics pipeline state including
//! shaders, pipeline layout, render pass, and the pipeline itself.

use std::mem;
use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult};
use crate::rendering::shaders::ShaderManager;
use crate::rendering::vertex::Vertex;

/// Immutable graphics pipeline configuration and state.
///
/// Once created, this structure is immutable and can be shared across frames.
/// Contains the render pass, pipeline layout, shader modules, and the pipeline itself.
pub struct GraphicsPipeline {
    pub pipeline: vk::Pipeline,
    /// Wireframe pipeline that renders only backfaces (front-face culled) in wireframe mode.
    pub wireframe_backface_pipeline: vk::Pipeline,
    pub layout: vk::PipelineLayout,
    pub renderpass: vk::RenderPass,
    /// Separate render pass for transparent geometry (water, particles, overlay).
    /// Uses the swapchain image directly and has depth as an input attachment.
    pub transparent_renderpass: vk::RenderPass,
    pub scene_ubo_descriptor_set_layout: vk::DescriptorSetLayout,
    pub sampler_descriptor_set_layout: vk::DescriptorSetLayout,
    vertex_shader_module: vk::ShaderModule,
    fragment_shader_module: vk::ShaderModule,
    device: Arc<ManagedDevice>,
}

/// Configuration for creating a graphics pipeline.
pub struct GraphicsPipelineConfig {
    pub color_format: vk::Format,
    pub depth_format: vk::Format,
    pub extent: vk::Extent2D,
}

impl GraphicsPipeline {
    /// Create a new graphics pipeline with the given configuration.
    pub fn new(device: Arc<ManagedDevice>, config: &GraphicsPipelineConfig) -> EngineResult<Self> {
        unsafe {
            // Load shaders
            let (vertex_shader_module, fragment_shader_module) = Self::load_shaders(&device)?;

            // Create descriptor set layouts
            let scene_ubo_descriptor_set_layout = Self::create_ubo_descriptor_layout(&device)?;
            let sampler_descriptor_set_layout = Self::create_sampler_descriptor_layout(&device)?;

            // Create pipeline layout with push constants for per-object model matrix
            let set_layouts = [
                scene_ubo_descriptor_set_layout,
                sampler_descriptor_set_layout,
            ];

            // Push constant ranges:
            // - Vertex: mat4 model (offset 0, 64 bytes)
            // - Fragment: vec4 colorOverride (offset 64, 16 bytes)
            let push_constant_ranges = [
                vk::PushConstantRange {
                    stage_flags: vk::ShaderStageFlags::VERTEX,
                    offset: 0,
                    size: 64,
                },
                vk::PushConstantRange {
                    stage_flags: vk::ShaderStageFlags::FRAGMENT,
                    offset: 64,
                    size: 16,
                },
            ];

            let pipeline_layout_create_info = vk::PipelineLayoutCreateInfo::default()
                .set_layouts(&set_layouts)
                .push_constant_ranges(&push_constant_ranges);
            let layout = device
                .device
                .create_pipeline_layout(&pipeline_layout_create_info, None)
                .map_err(|e| EngineError::Pipeline(format!("layout creation: {:?}", e)))?;

            // Create render passes
            let renderpass = Self::create_render_pass(&device, config)?;
            let transparent_renderpass = Self::create_transparent_render_pass(
                &device,
                config.color_format,
                config.depth_format,
            )?;

            // Create the graphics pipeline
            let pipeline = Self::create_pipeline(
                &device,
                renderpass,
                layout,
                vertex_shader_module,
                fragment_shader_module,
                config,
                vk::PolygonMode::FILL,
                vk::CullModeFlags::BACK,
            )?;

            // Create the wireframe backface pipeline (wireframe, cull front faces)
            let wireframe_backface_pipeline = Self::create_pipeline(
                &device,
                renderpass,
                layout,
                vertex_shader_module,
                fragment_shader_module,
                config,
                vk::PolygonMode::LINE,
                vk::CullModeFlags::FRONT,
            )?;

            Ok(Self {
                pipeline,
                wireframe_backface_pipeline,
                layout,
                renderpass,
                transparent_renderpass,
                scene_ubo_descriptor_set_layout,
                sampler_descriptor_set_layout,
                vertex_shader_module,
                fragment_shader_module,
                device,
            })
        }
    }

    fn load_shaders(device: &ManagedDevice) -> EngineResult<(vk::ShaderModule, vk::ShaderModule)> {
        let vertex_module = ShaderManager::load_main_vertex(device)?;
        let fragment_module = ShaderManager::load_main_fragment(device)?;
        Ok((vertex_module, fragment_module))
    }

    fn create_ubo_descriptor_layout(
        device: &ManagedDevice,
    ) -> EngineResult<vk::DescriptorSetLayout> {
        let bindings = [vk::DescriptorSetLayoutBinding::default()
            .binding(0)
            .descriptor_type(vk::DescriptorType::UNIFORM_BUFFER)
            .descriptor_count(1)
            .stage_flags(vk::ShaderStageFlags::VERTEX)];

        let create_info = vk::DescriptorSetLayoutCreateInfo::default().bindings(&bindings);

        unsafe {
            device
                .device
                .create_descriptor_set_layout(&create_info, None)
                .map_err(|e| EngineError::Descriptor(format!("UBO layout creation: {:?}", e)))
        }
    }

    fn create_sampler_descriptor_layout(
        device: &ManagedDevice,
    ) -> EngineResult<vk::DescriptorSetLayout> {
        let bindings = [vk::DescriptorSetLayoutBinding::default()
            .binding(0)
            .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
            .descriptor_count(1)
            .stage_flags(vk::ShaderStageFlags::FRAGMENT)];

        let create_info = vk::DescriptorSetLayoutCreateInfo::default().bindings(&bindings);

        unsafe {
            device
                .device
                .create_descriptor_set_layout(&create_info, None)
                .map_err(|e| EngineError::Descriptor(format!("sampler layout creation: {:?}", e)))
        }
    }

    /// Render pass for opaque geometry (sky, terrain, models, debug overlays).
    ///
    /// Writes to the offscreen `ColorTarget` and depth buffer. The color attachment
    /// transitions to `TRANSFER_SRC_OPTIMAL` at the end, ready to be blitted to
    /// the swapchain image and sampled as a texture for water refraction.
    fn create_render_pass(
        device: &ManagedDevice,
        config: &GraphicsPipelineConfig,
    ) -> EngineResult<vk::RenderPass> {
        let attachments = [
            vk::AttachmentDescription {
                format: config.color_format,
                samples: vk::SampleCountFlags::TYPE_1,
                load_op: vk::AttachmentLoadOp::CLEAR,
                store_op: vk::AttachmentStoreOp::STORE,
                final_layout: vk::ImageLayout::TRANSFER_SRC_OPTIMAL,
                ..Default::default()
            },
            vk::AttachmentDescription {
                format: config.depth_format,
                samples: vk::SampleCountFlags::TYPE_1,
                load_op: vk::AttachmentLoadOp::CLEAR,
                initial_layout: vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                final_layout: vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                ..Default::default()
            },
        ];

        let color_refs = [vk::AttachmentReference {
            attachment: 0,
            layout: vk::ImageLayout::COLOR_ATTACHMENT_OPTIMAL,
        }];

        let depth_ref = vk::AttachmentReference {
            attachment: 1,
            layout: vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
        };

        let dependencies = [vk::SubpassDependency {
            src_subpass: vk::SUBPASS_EXTERNAL,
            src_stage_mask: vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT,
            dst_access_mask: vk::AccessFlags::COLOR_ATTACHMENT_READ
                | vk::AccessFlags::COLOR_ATTACHMENT_WRITE,
            dst_stage_mask: vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT,
            ..Default::default()
        }];

        let subpass = vk::SubpassDescription::default()
            .color_attachments(&color_refs)
            .depth_stencil_attachment(&depth_ref)
            .pipeline_bind_point(vk::PipelineBindPoint::GRAPHICS);

        let create_info = vk::RenderPassCreateInfo::default()
            .attachments(&attachments)
            .subpasses(std::slice::from_ref(&subpass))
            .dependencies(&dependencies);

        unsafe {
            device
                .device
                .create_render_pass(&create_info, None)
                .map_err(|e| EngineError::RenderPass(format!("creation: {:?}", e)))
        }
    }

    /// Render pass for transparent geometry (water, particles, overlay).
    ///
    /// Writes to the swapchain image (loaded from the blit of the opaque pass).
    /// Depth is loaded from the opaque pass and available as both a read-only
    /// depth-stencil attachment and an input attachment (for water volumetric depth).
    pub fn create_transparent_render_pass(
        device: &ManagedDevice,
        color_format: vk::Format,
        depth_format: vk::Format,
    ) -> EngineResult<vk::RenderPass> {
        let attachments = [
            vk::AttachmentDescription {
                format: color_format,
                samples: vk::SampleCountFlags::TYPE_1,
                load_op: vk::AttachmentLoadOp::LOAD,
                store_op: vk::AttachmentStoreOp::STORE,
                initial_layout: vk::ImageLayout::COLOR_ATTACHMENT_OPTIMAL,
                final_layout: vk::ImageLayout::PRESENT_SRC_KHR,
                ..Default::default()
            },
            vk::AttachmentDescription {
                format: depth_format,
                samples: vk::SampleCountFlags::TYPE_1,
                load_op: vk::AttachmentLoadOp::LOAD,
                store_op: vk::AttachmentStoreOp::DONT_CARE,
                initial_layout: vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                final_layout: vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                ..Default::default()
            },
        ];

        let color_refs = [vk::AttachmentReference {
            attachment: 0,
            layout: vk::ImageLayout::COLOR_ATTACHMENT_OPTIMAL,
        }];

        let depth_ref = vk::AttachmentReference {
            attachment: 1,
            layout: vk::ImageLayout::DEPTH_STENCIL_READ_ONLY_OPTIMAL,
        };

        let dependencies = [vk::SubpassDependency {
            src_subpass: vk::SUBPASS_EXTERNAL,
            src_stage_mask: vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT
                | vk::PipelineStageFlags::TRANSFER,
            dst_stage_mask: vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT
                | vk::PipelineStageFlags::FRAGMENT_SHADER
                | vk::PipelineStageFlags::EARLY_FRAGMENT_TESTS,
            dst_access_mask: vk::AccessFlags::COLOR_ATTACHMENT_WRITE
                | vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_READ,
            ..Default::default()
        }];

        let subpass = vk::SubpassDescription::default()
            .color_attachments(&color_refs)
            .depth_stencil_attachment(&depth_ref)
            .pipeline_bind_point(vk::PipelineBindPoint::GRAPHICS);

        let create_info = vk::RenderPassCreateInfo::default()
            .attachments(&attachments)
            .subpasses(std::slice::from_ref(&subpass))
            .dependencies(&dependencies);

        unsafe {
            device
                .device
                .create_render_pass(&create_info, None)
                .map_err(|e| EngineError::RenderPass(format!("transparent pass creation: {:?}", e)))
        }
    }

    fn create_pipeline(
        device: &ManagedDevice,
        renderpass: vk::RenderPass,
        layout: vk::PipelineLayout,
        vertex_module: vk::ShaderModule,
        fragment_module: vk::ShaderModule,
        config: &GraphicsPipelineConfig,
        polygon_mode: vk::PolygonMode,
        cull_mode: vk::CullModeFlags,
    ) -> EngineResult<vk::Pipeline> {
        // Vertex input configuration
        let binding_descriptions = [vk::VertexInputBindingDescription {
            binding: 0,
            stride: mem::size_of::<Vertex>() as u32,
            input_rate: vk::VertexInputRate::VERTEX,
        }];

        let attribute_descriptions = Vertex::get_attribute_descriptions();

        let vertex_input_state = vk::PipelineVertexInputStateCreateInfo::default()
            .vertex_binding_descriptions(&binding_descriptions)
            .vertex_attribute_descriptions(&attribute_descriptions);

        // Shader stages
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

        // Fixed function state
        let input_assembly = vk::PipelineInputAssemblyStateCreateInfo::default()
            .topology(vk::PrimitiveTopology::TRIANGLE_LIST);

        let viewports = [vk::Viewport {
            x: 0.0,
            y: 0.0,
            width: config.extent.width as f32,
            height: config.extent.height as f32,
            min_depth: 0.0,
            max_depth: 1.0,
        }];

        let scissors = [config.extent.into()];

        let viewport_state = vk::PipelineViewportStateCreateInfo::default()
            .viewports(&viewports)
            .scissors(&scissors);

        let rasterization = vk::PipelineRasterizationStateCreateInfo::default()
            .polygon_mode(polygon_mode)
            .cull_mode(cull_mode)
            .front_face(vk::FrontFace::COUNTER_CLOCKWISE)
            .line_width(1.0);

        let multisample = vk::PipelineMultisampleStateCreateInfo::default()
            .rasterization_samples(vk::SampleCountFlags::TYPE_1);

        let stencil_op = vk::StencilOpState {
            fail_op: vk::StencilOp::KEEP,
            pass_op: vk::StencilOp::KEEP,
            depth_fail_op: vk::StencilOp::KEEP,
            compare_op: vk::CompareOp::ALWAYS,
            ..Default::default()
        };

        let depth_stencil = vk::PipelineDepthStencilStateCreateInfo::default()
            .depth_test_enable(true)
            .depth_write_enable(true)
            .depth_compare_op(vk::CompareOp::LESS_OR_EQUAL)
            .front(stencil_op)
            .back(stencil_op)
            .max_depth_bounds(1.0);

        let color_blend_attachments = [vk::PipelineColorBlendAttachmentState {
            blend_enable: vk::FALSE,
            color_write_mask: vk::ColorComponentFlags::RGBA,
            ..Default::default()
        }];

        let color_blend =
            vk::PipelineColorBlendStateCreateInfo::default().attachments(&color_blend_attachments);

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
            .render_pass(renderpass);

        unsafe {
            let pipelines = device
                .device
                .create_graphics_pipelines(vk::PipelineCache::null(), &[pipeline_info], None)
                .map_err(|(_pipelines, e)| EngineError::Pipeline(format!("creation: {:?}", e)))?;

            Ok(pipelines[0])
        }
    }
}

impl Drop for GraphicsPipeline {
    fn drop(&mut self) {
        unsafe {
            log::debug!("GraphicsPipeline::drop - cleaning up pipeline resources");

            self.device.device.destroy_pipeline(self.pipeline, None);
            self.device
                .device
                .destroy_pipeline(self.wireframe_backface_pipeline, None);
            self.device
                .device
                .destroy_pipeline_layout(self.layout, None);
            self.device
                .device
                .destroy_render_pass(self.renderpass, None);
            self.device
                .device
                .destroy_render_pass(self.transparent_renderpass, None);
            self.device
                .device
                .destroy_descriptor_set_layout(self.scene_ubo_descriptor_set_layout, None);
            self.device
                .device
                .destroy_descriptor_set_layout(self.sampler_descriptor_set_layout, None);
            self.device
                .device
                .destroy_shader_module(self.vertex_shader_module, None);
            self.device
                .device
                .destroy_shader_module(self.fragment_shader_module, None);
        }
    }
}
