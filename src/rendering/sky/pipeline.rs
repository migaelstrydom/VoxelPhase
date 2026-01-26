//! Sky rendering pipeline.
//!
//! Creates a graphics pipeline configured for procedural sky rendering
//! with no depth testing/writing and fullscreen triangle generation.

use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult};
use crate::rendering::shaders::ShaderManager;

/// Graphics pipeline for sky rendering.
///
/// Configured for procedural sky with:
/// - No vertex input (fullscreen triangle generated from vertex ID)
/// - No depth test/write (sky renders behind everything)
/// - No backface culling
/// - Push constants for inverse view matrix (64 bytes) + sun direction/time (16 bytes) + FOV params (16 bytes)
pub struct SkyPipeline {
    device: Arc<ManagedDevice>,
    pipeline: vk::Pipeline,
    pipeline_layout: vk::PipelineLayout,
}

impl SkyPipeline {
    /// Create a new sky pipeline.
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
        // Push constants:
        // - mat4 inv_view (64 bytes)
        // - vec4 sun_direction (xyz = direction, w = time) (16 bytes)
        // - vec4 tan_fov (x = tan(fov/2)*aspect, y = tan(fov/2), zw = unused) (16 bytes)
        // Total: 96 bytes
        let push_constant_range = vk::PushConstantRange {
            stage_flags: vk::ShaderStageFlags::VERTEX | vk::ShaderStageFlags::FRAGMENT,
            offset: 0,
            size: 96,
        };

        let create_info = vk::PipelineLayoutCreateInfo::default()
            .push_constant_ranges(std::slice::from_ref(&push_constant_range));

        unsafe { device.device.create_pipeline_layout(&create_info, None) }
            .map_err(|e| EngineError::Pipeline(format!("sky layout creation: {:?}", e)))
    }

    fn create_pipeline(
        device: &ManagedDevice,
        render_pass: vk::RenderPass,
        layout: vk::PipelineLayout,
    ) -> EngineResult<vk::Pipeline> {
        let vert_module = ShaderManager::load_sky_vertex(device)?;
        let frag_module = ShaderManager::load_sky_fragment(device)?;

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

        // No vertex input - fullscreen triangle generated from gl_VertexIndex
        let vertex_input_state = vk::PipelineVertexInputStateCreateInfo::default();

        let input_assembly = vk::PipelineInputAssemblyStateCreateInfo::default()
            .topology(vk::PrimitiveTopology::TRIANGLE_LIST);

        let viewport_state = vk::PipelineViewportStateCreateInfo::default()
            .viewport_count(1)
            .scissor_count(1);

        // No culling for fullscreen triangle
        let rasterization = vk::PipelineRasterizationStateCreateInfo::default()
            .polygon_mode(vk::PolygonMode::FILL)
            .cull_mode(vk::CullModeFlags::NONE)
            .front_face(vk::FrontFace::COUNTER_CLOCKWISE)
            .line_width(1.0);

        let multisample = vk::PipelineMultisampleStateCreateInfo::default()
            .rasterization_samples(vk::SampleCountFlags::TYPE_1);

        // No depth test or write - sky is always behind everything
        let depth_stencil = vk::PipelineDepthStencilStateCreateInfo::default()
            .depth_test_enable(false)
            .depth_write_enable(false);

        // No blending needed - sky is opaque background
        let color_blend_attachment = vk::PipelineColorBlendAttachmentState {
            blend_enable: vk::FALSE,
            color_write_mask: vk::ColorComponentFlags::RGBA,
            ..Default::default()
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

        let pipeline = unsafe {
            let pipelines = device
                .device
                .create_graphics_pipelines(vk::PipelineCache::null(), &[pipeline_info], None)
                .map_err(|(_, e)| EngineError::Pipeline(format!("sky creation: {:?}", e)))?;

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

impl Drop for SkyPipeline {
    fn drop(&mut self) {
        unsafe {
            self.device.device.destroy_pipeline(self.pipeline, None);
            self.device
                .device
                .destroy_pipeline_layout(self.pipeline_layout, None);
        }
    }
}
