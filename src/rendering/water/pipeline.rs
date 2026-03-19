//! Water rendering pipeline.
//!
//! Creates a graphics pipeline configured for water surface rendering
//! with alpha blending, depth testing (read-only), and volumetric depth
//! via a depth input attachment from the opaque geometry pass.

use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult};
use crate::rendering::shaders::ShaderManager;

use super::vertex::WaterVertex;

/// Graphics pipeline for water surface rendering.
///
/// Configured with:
/// - Alpha blending (translucent water surface)
/// - Depth test enabled (water occluded by terrain)
/// - Depth write disabled (terrain behind water still visible through alpha)
/// - No backface culling (water visible from both sides)
/// - Input attachment for reading the opaque depth buffer (volumetric depth)
pub struct WaterPipeline {
    device: Arc<ManagedDevice>,
    pipeline: vk::Pipeline,
    pipeline_layout: vk::PipelineLayout,
    descriptor_set_layout: vk::DescriptorSetLayout,
    descriptor_pool: vk::DescriptorPool,
    descriptor_set: vk::DescriptorSet,
}

impl WaterPipeline {
    pub fn new(
        device: Arc<ManagedDevice>,
        render_pass: vk::RenderPass,
        depth_view: vk::ImageView,
    ) -> EngineResult<Self> {
        let descriptor_set_layout = Self::create_descriptor_set_layout(&device)?;
        let pipeline_layout = Self::create_pipeline_layout(&device, descriptor_set_layout)?;
        let pipeline = Self::create_pipeline(&device, render_pass, pipeline_layout)?;
        let descriptor_pool = Self::create_descriptor_pool(&device)?;

        let descriptor_set =
            Self::allocate_descriptor_set(&device, descriptor_pool, descriptor_set_layout)?;

        Self::update_descriptor_set(&device, descriptor_set, depth_view);

        Ok(Self {
            device,
            pipeline,
            pipeline_layout,
            descriptor_set_layout,
            descriptor_pool,
            descriptor_set,
        })
    }

    fn create_descriptor_set_layout(
        device: &ManagedDevice,
    ) -> EngineResult<vk::DescriptorSetLayout> {
        let binding = vk::DescriptorSetLayoutBinding::default()
            .binding(0)
            .descriptor_type(vk::DescriptorType::INPUT_ATTACHMENT)
            .descriptor_count(1)
            .stage_flags(vk::ShaderStageFlags::FRAGMENT);

        let create_info = vk::DescriptorSetLayoutCreateInfo::default()
            .bindings(std::slice::from_ref(&binding));

        unsafe { device.device.create_descriptor_set_layout(&create_info, None) }
            .map_err(|e| EngineError::Pipeline(format!("water descriptor layout: {:?}", e)))
    }

    fn create_pipeline_layout(
        device: &ManagedDevice,
        descriptor_set_layout: vk::DescriptorSetLayout,
    ) -> EngineResult<vk::PipelineLayout> {
        // Push constants layout:
        //   0..128  — view matrix (64) + projection matrix (64) [vertex]
        // 128..160  — camera_pos (vec4) + sun_dir (vec4) + proj_params (vec4) [fragment]
        let push_constant_ranges = [
            vk::PushConstantRange {
                stage_flags: vk::ShaderStageFlags::VERTEX,
                offset: 0,
                size: 128,
            },
            vk::PushConstantRange {
                stage_flags: vk::ShaderStageFlags::FRAGMENT,
                offset: 128,
                size: 48,
            },
        ];

        let set_layouts = [descriptor_set_layout];

        let create_info = vk::PipelineLayoutCreateInfo::default()
            .set_layouts(&set_layouts)
            .push_constant_ranges(&push_constant_ranges);

        unsafe { device.device.create_pipeline_layout(&create_info, None) }
            .map_err(|e| EngineError::Pipeline(format!("water layout creation: {:?}", e)))
    }

    fn create_descriptor_pool(device: &ManagedDevice) -> EngineResult<vk::DescriptorPool> {
        let pool_size = vk::DescriptorPoolSize {
            ty: vk::DescriptorType::INPUT_ATTACHMENT,
            descriptor_count: 1,
        };

        let create_info = vk::DescriptorPoolCreateInfo::default()
            .max_sets(1)
            .pool_sizes(std::slice::from_ref(&pool_size));

        unsafe { device.device.create_descriptor_pool(&create_info, None) }
            .map_err(|e| EngineError::Pipeline(format!("water descriptor pool: {:?}", e)))
    }

    fn allocate_descriptor_set(
        device: &ManagedDevice,
        pool: vk::DescriptorPool,
        layout: vk::DescriptorSetLayout,
    ) -> EngineResult<vk::DescriptorSet> {
        let alloc_info = vk::DescriptorSetAllocateInfo::default()
            .descriptor_pool(pool)
            .set_layouts(std::slice::from_ref(&layout));

        let sets = unsafe { device.device.allocate_descriptor_sets(&alloc_info) }
            .map_err(|e| EngineError::Pipeline(format!("water descriptor alloc: {:?}", e)))?;

        Ok(sets[0])
    }

    fn update_descriptor_set(
        device: &ManagedDevice,
        descriptor_set: vk::DescriptorSet,
        depth_view: vk::ImageView,
    ) {
        let image_info = vk::DescriptorImageInfo::default()
            .image_view(depth_view)
            .image_layout(vk::ImageLayout::DEPTH_STENCIL_READ_ONLY_OPTIMAL);

        let write = vk::WriteDescriptorSet::default()
            .dst_set(descriptor_set)
            .dst_binding(0)
            .descriptor_type(vk::DescriptorType::INPUT_ATTACHMENT)
            .image_info(std::slice::from_ref(&image_info));

        unsafe {
            device
                .device
                .update_descriptor_sets(std::slice::from_ref(&write), &[]);
        }
    }

    fn create_pipeline(
        device: &ManagedDevice,
        render_pass: vk::RenderPass,
        layout: vk::PipelineLayout,
    ) -> EngineResult<vk::Pipeline> {
        let vert_module = ShaderManager::load_water_vertex(device)?;
        let frag_module = ShaderManager::load_water_fragment(device)?;

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

        let binding_description = WaterVertex::binding_description();
        let attribute_descriptions = WaterVertex::attribute_descriptions();

        let vertex_input_state = vk::PipelineVertexInputStateCreateInfo::default()
            .vertex_binding_descriptions(std::slice::from_ref(&binding_description))
            .vertex_attribute_descriptions(&attribute_descriptions);

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

        // Depth test enabled, depth write disabled (transparent surface)
        let depth_stencil = vk::PipelineDepthStencilStateCreateInfo::default()
            .depth_test_enable(true)
            .depth_write_enable(false)
            .depth_compare_op(vk::CompareOp::LESS_OR_EQUAL);

        let color_blend_attachment = vk::PipelineColorBlendAttachmentState {
            blend_enable: vk::TRUE,
            src_color_blend_factor: vk::BlendFactor::SRC_ALPHA,
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
            .subpass(1);

        let pipeline = unsafe {
            let pipelines = device
                .device
                .create_graphics_pipelines(vk::PipelineCache::null(), &[pipeline_info], None)
                .map_err(|(_, e)| EngineError::Pipeline(format!("water creation: {:?}", e)))?;

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

    pub fn descriptor_set(&self) -> vk::DescriptorSet {
        self.descriptor_set
    }
}

impl Drop for WaterPipeline {
    fn drop(&mut self) {
        unsafe {
            self.device.device.destroy_pipeline(self.pipeline, None);
            self.device
                .device
                .destroy_pipeline_layout(self.pipeline_layout, None);
            self.device
                .device
                .destroy_descriptor_pool(self.descriptor_pool, None);
            self.device
                .device
                .destroy_descriptor_set_layout(self.descriptor_set_layout, None);
        }
    }
}
