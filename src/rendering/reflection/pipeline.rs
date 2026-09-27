//! The pipeline that renders the scene's opaque draws into a probe face.

use std::mem;
use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult};
use crate::rendering::material::SURFACE_INDEX_OFFSET;
use crate::rendering::renderer::PUSH_CONSTANT_STAGES;
use crate::rendering::shaders::ShaderManager;
use crate::rendering::vertex::Vertex;

/// The capture pipeline and its layout.
///
/// The layout is the geometry pipeline's with one set added: sets 0 and 1
/// and the push-constant range are declared identically, which makes the two
/// layouts compatible for those sets. That is what lets a draw committed for
/// the scene — its surface-table row, its texture set, its push constants —
/// be recorded again into a probe unchanged. Set 2 carries the face.
pub struct ProbePipeline {
    pub pipeline: vk::Pipeline,
    pub layout: vk::PipelineLayout,
    /// Set 2: the face being captured, one dynamic uniform buffer.
    pub face_set_layout: vk::DescriptorSetLayout,
    device: Arc<ManagedDevice>,
}

impl ProbePipeline {
    pub fn new(
        device: Arc<ManagedDevice>,
        render_pass: vk::RenderPass,
        scene_set_layout: vk::DescriptorSetLayout,
        texture_set_layout: vk::DescriptorSetLayout,
    ) -> EngineResult<Self> {
        let face_set_layout = create_face_set_layout(&device)?;

        let set_layouts = [scene_set_layout, texture_set_layout, face_set_layout];
        let push_constant_ranges = [vk::PushConstantRange {
            stage_flags: PUSH_CONSTANT_STAGES,
            offset: 0,
            size: SURFACE_INDEX_OFFSET + mem::size_of::<u32>() as u32,
        }];
        let layout_info = vk::PipelineLayoutCreateInfo::default()
            .set_layouts(&set_layouts)
            .push_constant_ranges(&push_constant_ranges);
        let layout = unsafe { device.device.create_pipeline_layout(&layout_info, None) }
            .map_err(|e| EngineError::Pipeline(format!("probe layout: {:?}", e)))?;

        let vertex_module = ShaderManager::load_probe_vertex(&device)?;
        let fragment_module = ShaderManager::load_probe_fragment(&device)?;
        let pipeline =
            create_pipeline(&device, render_pass, layout, vertex_module, fragment_module);
        unsafe {
            device.device.destroy_shader_module(vertex_module, None);
            device.device.destroy_shader_module(fragment_module, None);
        }

        Ok(Self {
            pipeline: pipeline?,
            layout,
            face_set_layout,
            device,
        })
    }
}

impl Drop for ProbePipeline {
    fn drop(&mut self) {
        unsafe {
            self.device.device.destroy_pipeline(self.pipeline, None);
            self.device
                .device
                .destroy_pipeline_layout(self.layout, None);
            self.device
                .device
                .destroy_descriptor_set_layout(self.face_set_layout, None);
        }
    }
}

fn create_face_set_layout(device: &ManagedDevice) -> EngineResult<vk::DescriptorSetLayout> {
    let bindings = [vk::DescriptorSetLayoutBinding::default()
        .binding(0)
        .descriptor_type(vk::DescriptorType::UNIFORM_BUFFER_DYNAMIC)
        .descriptor_count(1)
        .stage_flags(vk::ShaderStageFlags::VERTEX | vk::ShaderStageFlags::FRAGMENT)];
    let info = vk::DescriptorSetLayoutCreateInfo::default().bindings(&bindings);
    unsafe { device.device.create_descriptor_set_layout(&info, None) }
        .map_err(|e| EngineError::Descriptor(format!("probe face layout: {:?}", e)))
}

fn create_pipeline(
    device: &ManagedDevice,
    render_pass: vk::RenderPass,
    layout: vk::PipelineLayout,
    vertex_module: vk::ShaderModule,
    fragment_module: vk::ShaderModule,
) -> EngineResult<vk::Pipeline> {
    let binding_descriptions = [vk::VertexInputBindingDescription {
        binding: 0,
        stride: mem::size_of::<Vertex>() as u32,
        input_rate: vk::VertexInputRate::VERTEX,
    }];
    let attribute_descriptions = Vertex::get_attribute_descriptions();
    let vertex_input_state = vk::PipelineVertexInputStateCreateInfo::default()
        .vertex_binding_descriptions(&binding_descriptions)
        .vertex_attribute_descriptions(&attribute_descriptions);

    let entry_name = c"main";
    let stages = [
        vk::PipelineShaderStageCreateInfo::default()
            .stage(vk::ShaderStageFlags::VERTEX)
            .module(vertex_module)
            .name(entry_name),
        vk::PipelineShaderStageCreateInfo::default()
            .stage(vk::ShaderStageFlags::FRAGMENT)
            .module(fragment_module)
            .name(entry_name),
    ];

    let input_assembly = vk::PipelineInputAssemblyStateCreateInfo::default()
        .topology(vk::PrimitiveTopology::TRIANGLE_LIST);

    // Set per face from the atlas's extent, like the scene's.
    let viewport_state = vk::PipelineViewportStateCreateInfo::default()
        .viewport_count(1)
        .scissor_count(1);

    // Back faces culled, as in the scene pass, but with the winding reversed:
    // a cube face is a mirrored view (see `CubeFace::view_proj`), so a
    // triangle facing the probe arrives clockwise.
    let rasterization = vk::PipelineRasterizationStateCreateInfo::default()
        .polygon_mode(vk::PolygonMode::FILL)
        .cull_mode(vk::CullModeFlags::BACK)
        .front_face(vk::FrontFace::CLOCKWISE)
        .line_width(1.0);

    let multisample = vk::PipelineMultisampleStateCreateInfo::default()
        .rasterization_samples(vk::SampleCountFlags::TYPE_1);

    let depth_stencil = vk::PipelineDepthStencilStateCreateInfo::default()
        .depth_test_enable(true)
        .depth_write_enable(true)
        .depth_compare_op(vk::CompareOp::LESS_OR_EQUAL)
        .max_depth_bounds(1.0);

    let blend_attachments = [vk::PipelineColorBlendAttachmentState {
        blend_enable: vk::FALSE,
        color_write_mask: vk::ColorComponentFlags::RGBA,
        ..Default::default()
    }];
    let color_blend =
        vk::PipelineColorBlendStateCreateInfo::default().attachments(&blend_attachments);

    let dynamic_states = [vk::DynamicState::VIEWPORT, vk::DynamicState::SCISSOR];
    let dynamic_state =
        vk::PipelineDynamicStateCreateInfo::default().dynamic_states(&dynamic_states);

    let info = vk::GraphicsPipelineCreateInfo::default()
        .stages(&stages)
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
        device
            .device
            .create_graphics_pipelines(vk::PipelineCache::null(), &[info], None)
            .map(|pipelines| pipelines[0])
            .map_err(|(_, e)| EngineError::Pipeline(format!("probe capture: {:?}", e)))
    }
}
