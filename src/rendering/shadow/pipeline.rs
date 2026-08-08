//! The depth-only pipeline that fills the shadow map.

use std::mem;
use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult};
use crate::rendering::shaders::ShaderManager;
use crate::rendering::vertex::Vertex;

/// Constant depth offset applied to every shadow-map fragment, in units of the
/// depth format's smallest representable increment.
///
/// Together with the slope factor this is the caster-side half of the bias.
/// Kept small on purpose: the receiver-side normal offset in shadow.glsl does
/// the heavy lifting, because moving the sample across the surface costs no
/// contact whereas pushing depth away detaches it.
const DEPTH_BIAS_CONSTANT: f32 = 1.5;

/// Extra depth offset proportional to the fragment's depth slope.
///
/// A polygon at a grazing angle to the light spans many depth values inside one
/// texel, and the single stored value is wrong for most of it. This scales the
/// correction with that span, so flat-on surfaces are not over-biased.
const DEPTH_BIAS_SLOPE: f32 = 2.5;

/// A pipeline that renders geometry from the sun's point of view, writing depth
/// and nothing else.
///
/// Deliberately built over the *main* pipeline layout rather than one of its
/// own: the vertex format, the `mat4 model` push constant and the scene UBO are
/// the same, which is what lets the shadow pass replay the geometry pass's draw
/// calls without a parallel set of draw entry points.
pub struct ShadowPipeline {
    pub pipeline: vk::Pipeline,
    device: Arc<ManagedDevice>,
}

impl ShadowPipeline {
    pub fn new(
        device: Arc<ManagedDevice>,
        render_pass: vk::RenderPass,
        layout: vk::PipelineLayout,
    ) -> EngineResult<Self> {
        let vertex_module = ShaderManager::load_shadow_vertex(&device)?;
        let fragment_module = ShaderManager::load_shadow_fragment(&device)?;

        let result = Self::create(&device, render_pass, layout, vertex_module, fragment_module);

        unsafe {
            device.device.destroy_shader_module(vertex_module, None);
            device.device.destroy_shader_module(fragment_module, None);
        }

        Ok(Self {
            pipeline: result?,
            device,
        })
    }

    fn create(
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
        let attribute_descriptions = Vertex::depth_only_attribute_descriptions();

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

        // Both are dynamic and set from the map's extent when the pass begins,
        // so the pipeline does not need to know the resolution.
        let viewport_state = vk::PipelineViewportStateCreateInfo::default()
            .viewport_count(1)
            .scissor_count(1);

        // No culling: the scene carries open surfaces (character meshes, debug
        // geometry) whose back faces are the only thing a light behind them
        // would see, and front-face culling would drop their shadows entirely.
        let rasterization = vk::PipelineRasterizationStateCreateInfo::default()
            .polygon_mode(vk::PolygonMode::FILL)
            .cull_mode(vk::CullModeFlags::NONE)
            .front_face(vk::FrontFace::COUNTER_CLOCKWISE)
            .line_width(1.0)
            .depth_bias_enable(true)
            .depth_bias_constant_factor(DEPTH_BIAS_CONSTANT)
            .depth_bias_slope_factor(DEPTH_BIAS_SLOPE);

        let multisample = vk::PipelineMultisampleStateCreateInfo::default()
            .rasterization_samples(vk::SampleCountFlags::TYPE_1);

        let depth_stencil = vk::PipelineDepthStencilStateCreateInfo::default()
            .depth_test_enable(true)
            .depth_write_enable(true)
            .depth_compare_op(vk::CompareOp::LESS_OR_EQUAL)
            .max_depth_bounds(1.0);

        // The pass has no colour attachment, so there is nothing to blend.
        let color_blend = vk::PipelineColorBlendStateCreateInfo::default();

        let dynamic_states = [vk::DynamicState::VIEWPORT, vk::DynamicState::SCISSOR];
        let dynamic_state =
            vk::PipelineDynamicStateCreateInfo::default().dynamic_states(&dynamic_states);

        let pipeline_info = vk::GraphicsPipelineCreateInfo::default()
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
            let pipelines = device
                .device
                .create_graphics_pipelines(vk::PipelineCache::null(), &[pipeline_info], None)
                .map_err(|(_, e)| EngineError::Pipeline(format!("shadow creation: {:?}", e)))?;
            Ok(pipelines[0])
        }
    }
}

impl Drop for ShadowPipeline {
    fn drop(&mut self) {
        unsafe {
            self.device.device.destroy_pipeline(self.pipeline, None);
        }
    }
}
