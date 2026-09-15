//! Particle rendering pipeline.
//!
//! Creates a graphics pipeline configured for particle system rendering
//! with premultiplied alpha blending, depth testing (read-only), and billboard
//! geometry.

use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult};
use crate::rendering::shaders::ShaderManager;

use super::vertex::ParticleVertex;

/// Graphics pipeline for particle rendering.
///
/// Configured for particle effects with:
/// - Premultiplied alpha blending (per-particle glow vs. occlusion)
/// - Depth test enabled (particles occluded by geometry)
/// - Depth write disabled (particles don't occlude each other)
/// - No backface culling (billboards visible from all angles)
/// - Push constants for view and projection matrices (128 bytes)
pub struct ParticlePipeline {
    device: Arc<ManagedDevice>,
    pipeline: vk::Pipeline,
    pipeline_layout: vk::PipelineLayout,
}

impl ParticlePipeline {
    /// Create a new particle pipeline.
    ///
    /// The pipeline is created with the provided render pass to ensure compatibility
    /// when drawing within an existing render pass instance.
    pub fn new(device: Arc<ManagedDevice>, render_pass: vk::RenderPass) -> EngineResult<Self> {
        let pipeline_layout = Self::create_pipeline_layout(&device)?;

        let pipeline = Self::create_pipeline(&device, render_pass, pipeline_layout)?;

        Ok(Self {
            device,
            pipeline,
            pipeline_layout,
        })
    }

    fn create_pipeline_layout(device: &ManagedDevice) -> EngineResult<vk::PipelineLayout> {
        // Vertex: view matrix (64 bytes) + projection matrix (64 bytes).
        // Fragment: the scene exposure, which the shader inverts to turn an
        // authored display colour into the radiance that resolves back to it.
        let push_constant_ranges = [
            vk::PushConstantRange {
                stage_flags: vk::ShaderStageFlags::VERTEX,
                offset: 0,
                size: 128, // 2 * sizeof(mat4)
            },
            vk::PushConstantRange {
                stage_flags: vk::ShaderStageFlags::FRAGMENT,
                offset: 128,
                size: std::mem::size_of::<f32>() as u32,
            },
        ];

        let create_info =
            vk::PipelineLayoutCreateInfo::default().push_constant_ranges(&push_constant_ranges);

        unsafe { device.device.create_pipeline_layout(&create_info, None) }
            .map_err(|e| EngineError::Pipeline(format!("particle layout creation: {:?}", e)))
    }

    fn create_pipeline(
        device: &ManagedDevice,
        render_pass: vk::RenderPass,
        layout: vk::PipelineLayout,
    ) -> EngineResult<vk::Pipeline> {
        let vert_module = ShaderManager::load_particle_vertex(device)?;
        let frag_module = ShaderManager::load_particle_fragment(device)?;

        let entry_name = c"main";
        let shader_stages = [
            vk::PipelineShaderStageCreateInfo::default()
                .stage(vk::ShaderStageFlags::VERTEX)
                .module(vert_module)
                .name(entry_name),
            vk::PipelineShaderStageCreateInfo::default()
                .stage(vk::ShaderStageFlags::FRAGMENT)
                .module(frag_module)
                .name(entry_name),
        ];

        let binding_description = ParticleVertex::binding_description();
        let attribute_descriptions = ParticleVertex::attribute_descriptions();

        let vertex_input_state = vk::PipelineVertexInputStateCreateInfo::default()
            .vertex_binding_descriptions(std::slice::from_ref(&binding_description))
            .vertex_attribute_descriptions(&attribute_descriptions);

        let input_assembly = vk::PipelineInputAssemblyStateCreateInfo::default()
            .topology(vk::PrimitiveTopology::TRIANGLE_LIST);

        let viewport_state = vk::PipelineViewportStateCreateInfo::default()
            .viewport_count(1)
            .scissor_count(1);

        // No backface culling - billboards should be visible from all angles
        let rasterization = vk::PipelineRasterizationStateCreateInfo::default()
            .polygon_mode(vk::PolygonMode::FILL)
            .cull_mode(vk::CullModeFlags::NONE)
            .front_face(vk::FrontFace::COUNTER_CLOCKWISE)
            .line_width(1.0);

        let multisample = vk::PipelineMultisampleStateCreateInfo::default()
            .rasterization_samples(vk::SampleCountFlags::TYPE_1);

        // Depth test enabled (particles occluded by geometry)
        // Depth write disabled (particles don't occlude each other)
        let depth_stencil = vk::PipelineDepthStencilStateCreateInfo::default()
            .depth_test_enable(true)
            .depth_write_enable(false)
            .depth_compare_op(vk::CompareOp::LESS_OR_EQUAL);

        // Premultiplied alpha:
        //   result = src_color + dst_color * (1 - src_alpha)
        //
        // The fragment shader premultiplies, which makes this one blend state
        // cover both compositing modes a particle system needs. A fragment
        // emitting alpha 0 adds its colour without attenuating the scene
        // (fire, sparks); one emitting its full coverage attenuates and
        // replaces it (smoke, dust); anything between cross-fades. Two
        // pipelines and two sorted draws would buy nothing over it.
        let color_blend_attachment = vk::PipelineColorBlendAttachmentState {
            blend_enable: vk::TRUE,
            src_color_blend_factor: vk::BlendFactor::ONE,
            dst_color_blend_factor: vk::BlendFactor::ONE_MINUS_SRC_ALPHA,
            color_blend_op: vk::BlendOp::ADD,
            src_alpha_blend_factor: vk::BlendFactor::ONE,
            dst_alpha_blend_factor: vk::BlendFactor::ZERO,
            alpha_blend_op: vk::BlendOp::ADD,
            color_write_mask: vk::ColorComponentFlags::RGBA,
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
            .render_pass(render_pass)
            .subpass(0);

        let pipeline = unsafe {
            let pipelines = device
                .device
                .create_graphics_pipelines(vk::PipelineCache::null(), &[pipeline_info], None)
                .map_err(|(_, e)| EngineError::Pipeline(format!("particle creation: {:?}", e)))?;

            // Clean up shader modules after pipeline creation
            device.device.destroy_shader_module(vert_module, None);
            device.device.destroy_shader_module(frag_module, None);

            pipelines[0]
        };

        Ok(pipeline)
    }

    pub fn pipeline(&self) -> vk::Pipeline {
        self.pipeline
    }

    pub fn layout(&self) -> vk::PipelineLayout {
        self.pipeline_layout
    }
}

impl Drop for ParticlePipeline {
    fn drop(&mut self) {
        unsafe {
            self.device.device.destroy_pipeline(self.pipeline, None);
            self.device
                .device
                .destroy_pipeline_layout(self.pipeline_layout, None);
        }
    }
}
