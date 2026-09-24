//! Water rendering pipelines.
//!
//! Four pipelines over one layout. Three share the water fragment shader:
//! the coarse surface (a quad per column), the fine surface of an awake
//! ripple tile (a static grid displaced from the ripple storage buffer) and a
//! reach's surface. The fourth draws a fall's sheet with a fragment shader of
//! its own. All depth-test against the opaque scene without writing depth,
//! read that depth for volumetric tint, and sample the opaque colour target
//! at offset UVs for refraction.

use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult};
use crate::rendering::shaders::ShaderManager;

use crate::rendering::in_flight::FRAMES_IN_FLIGHT;

use super::vertex::{BasinVertex, FallVertex, FineVertex, RiverVertex};

/// Where each draw's constants start (its body, then its tile), and how many
/// bytes they are.
pub const BODY_PUSH_OFFSET: u32 = 128;
pub const BODY_PUSH_SIZE: u32 = 32;

/// Where the fragment stage's constants start.
pub const FRAGMENT_PUSH_OFFSET: u32 = BODY_PUSH_OFFSET + BODY_PUSH_SIZE;

/// Graphics pipeline for water surface rendering.
///
/// Configured with:
/// - Alpha blending (translucent water surface)
/// - Depth test enabled (water occluded by terrain)
/// - Depth write disabled (terrain behind water still visible through alpha)
/// - No backface culling (water visible from both sides)
/// - Input attachment for reading the opaque depth buffer (volumetric depth)
/// - Combined image sampler for the opaque color target (screen-space refraction)
pub struct WaterPipeline {
    device: Arc<ManagedDevice>,
    pipeline: vk::Pipeline,
    /// The fine surface of an awake ripple tile.
    fine_pipeline: vk::Pipeline,
    /// A reach's surface.
    river_pipeline: vk::Pipeline,
    fall_pipeline: vk::Pipeline,
    pipeline_layout: vk::PipelineLayout,
    descriptor_set_layout: vk::DescriptorSetLayout,
    /// Set 1: the ripple storage buffer.
    ripple_set_layout: vk::DescriptorSetLayout,
    descriptor_pool: vk::DescriptorPool,
    descriptor_set: vk::DescriptorSet,
    color_sampler: vk::Sampler,
    depth_sampler: vk::Sampler,
}

impl WaterPipeline {
    pub fn new(
        device: Arc<ManagedDevice>,
        render_pass: vk::RenderPass,
        depth_view: vk::ImageView,
        color_view: vk::ImageView,
    ) -> EngineResult<Self> {
        let color_sampler = Self::create_color_sampler(&device)?;
        let depth_sampler = Self::create_depth_sampler(&device)?;
        let descriptor_set_layout = Self::create_descriptor_set_layout(&device)?;
        let ripple_set_layout = Self::create_ripple_set_layout(&device)?;
        let pipeline_layout =
            Self::create_pipeline_layout(&device, descriptor_set_layout, ripple_set_layout)?;
        let pipeline = Self::create_pipeline(
            &device,
            render_pass,
            pipeline_layout,
            ShaderManager::load_water_vertex(&device)?,
            BasinVertex::binding_description(),
            &BasinVertex::attribute_descriptions(),
            ShaderManager::load_water_fragment(&device)?,
        )?;
        let fine_pipeline = Self::create_pipeline(
            &device,
            render_pass,
            pipeline_layout,
            ShaderManager::load_ripple_vertex(&device)?,
            FineVertex::binding_description(),
            &FineVertex::attribute_descriptions(),
            ShaderManager::load_water_fragment(&device)?,
        )?;
        let river_pipeline = Self::create_pipeline(
            &device,
            render_pass,
            pipeline_layout,
            ShaderManager::load_river_vertex(&device)?,
            RiverVertex::binding_description(),
            &RiverVertex::attribute_descriptions(),
            ShaderManager::load_water_fragment(&device)?,
        )?;
        let fall_pipeline = Self::create_pipeline(
            &device,
            render_pass,
            pipeline_layout,
            ShaderManager::load_fall_vertex(&device)?,
            FallVertex::binding_description(),
            &FallVertex::attribute_descriptions(),
            ShaderManager::load_fall_fragment(&device)?,
        )?;
        let descriptor_pool = Self::create_descriptor_pool(&device)?;

        let descriptor_set =
            Self::allocate_descriptor_set(&device, descriptor_pool, descriptor_set_layout)?;
        Self::update_descriptor_set(
            &device,
            descriptor_set,
            depth_view,
            color_view,
            color_sampler,
            depth_sampler,
        );

        Ok(Self {
            device,
            pipeline,
            fine_pipeline,
            river_pipeline,
            fall_pipeline,
            pipeline_layout,
            descriptor_set_layout,
            ripple_set_layout,
            descriptor_pool,
            descriptor_set,
            color_sampler,
            depth_sampler,
        })
    }

    fn create_color_sampler(device: &ManagedDevice) -> EngineResult<vk::Sampler> {
        let sampler_info = vk::SamplerCreateInfo::default()
            .mag_filter(vk::Filter::LINEAR)
            .min_filter(vk::Filter::LINEAR)
            .address_mode_u(vk::SamplerAddressMode::CLAMP_TO_EDGE)
            .address_mode_v(vk::SamplerAddressMode::CLAMP_TO_EDGE)
            .address_mode_w(vk::SamplerAddressMode::CLAMP_TO_EDGE);

        unsafe { device.device.create_sampler(&sampler_info, None) }
            .map_err(|e| EngineError::Pipeline(format!("water color sampler: {:?}", e)))
    }

    fn create_depth_sampler(device: &ManagedDevice) -> EngineResult<vk::Sampler> {
        let sampler_info = vk::SamplerCreateInfo::default()
            .mag_filter(vk::Filter::NEAREST)
            .min_filter(vk::Filter::NEAREST)
            .address_mode_u(vk::SamplerAddressMode::CLAMP_TO_EDGE)
            .address_mode_v(vk::SamplerAddressMode::CLAMP_TO_EDGE)
            .address_mode_w(vk::SamplerAddressMode::CLAMP_TO_EDGE);

        unsafe { device.device.create_sampler(&sampler_info, None) }
            .map_err(|e| EngineError::Pipeline(format!("water depth sampler: {:?}", e)))
    }

    fn create_descriptor_set_layout(
        device: &ManagedDevice,
    ) -> EngineResult<vk::DescriptorSetLayout> {
        let bindings = [
            // Binding 0: opaque color target (sampled at offset UVs for refraction)
            vk::DescriptorSetLayoutBinding::default()
                .binding(0)
                .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                .descriptor_count(1)
                .stage_flags(vk::ShaderStageFlags::FRAGMENT),
            // Binding 1: depth buffer (sampled at both current and refracted UVs)
            vk::DescriptorSetLayoutBinding::default()
                .binding(1)
                .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                .descriptor_count(1)
                .stage_flags(vk::ShaderStageFlags::FRAGMENT),
        ];

        let create_info = vk::DescriptorSetLayoutCreateInfo::default().bindings(&bindings);

        unsafe {
            device
                .device
                .create_descriptor_set_layout(&create_info, None)
        }
        .map_err(|e| EngineError::Pipeline(format!("water descriptor layout: {:?}", e)))
    }

    fn create_ripple_set_layout(device: &ManagedDevice) -> EngineResult<vk::DescriptorSetLayout> {
        let bindings = [vk::DescriptorSetLayoutBinding::default()
            .binding(0)
            .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
            .descriptor_count(1)
            .stage_flags(vk::ShaderStageFlags::VERTEX | vk::ShaderStageFlags::FRAGMENT)];
        let create_info = vk::DescriptorSetLayoutCreateInfo::default().bindings(&bindings);
        unsafe {
            device
                .device
                .create_descriptor_set_layout(&create_info, None)
        }
        .map_err(|e| EngineError::Pipeline(format!("water ripple layout: {:?}", e)))
    }

    fn create_pipeline_layout(
        device: &ManagedDevice,
        descriptor_set_layout: vk::DescriptorSetLayout,
        ripple_set_layout: vk::DescriptorSetLayout,
    ) -> EngineResult<vk::PipelineLayout> {
        // Push constants layout:
        //   0..128  — view matrix (64) + projection matrix (64) [vertex]
        // 128..160  — body (vec4: level, swell amplitude, swell phase, clock)
        //             + tile (vec4: origin x, origin z, ripple layer, sealed edges),
        //             one per draw [vertex]
        // 160..224  — camera_pos (vec4) + sun_dir (vec4) + proj_params (vec4)
        //             + screen_params (vec4) [fragment]
        let push_constant_ranges = [
            vk::PushConstantRange {
                stage_flags: vk::ShaderStageFlags::VERTEX,
                offset: 0,
                size: BODY_PUSH_OFFSET + BODY_PUSH_SIZE,
            },
            vk::PushConstantRange {
                stage_flags: vk::ShaderStageFlags::FRAGMENT,
                offset: FRAGMENT_PUSH_OFFSET,
                size: 64,
            },
        ];

        let set_layouts = [descriptor_set_layout, ripple_set_layout];

        let create_info = vk::PipelineLayoutCreateInfo::default()
            .set_layouts(&set_layouts)
            .push_constant_ranges(&push_constant_ranges);

        unsafe { device.device.create_pipeline_layout(&create_info, None) }
            .map_err(|e| EngineError::Pipeline(format!("water layout creation: {:?}", e)))
    }

    fn create_descriptor_pool(device: &ManagedDevice) -> EngineResult<vk::DescriptorPool> {
        let pool_sizes = [
            vk::DescriptorPoolSize {
                ty: vk::DescriptorType::COMBINED_IMAGE_SAMPLER,
                descriptor_count: 2,
            },
            vk::DescriptorPoolSize {
                ty: vk::DescriptorType::STORAGE_BUFFER,
                descriptor_count: FRAMES_IN_FLIGHT as u32,
            },
        ];

        let create_info = vk::DescriptorPoolCreateInfo::default()
            .max_sets(1 + FRAMES_IN_FLIGHT as u32)
            .pool_sizes(&pool_sizes);

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
        color_view: vk::ImageView,
        color_sampler: vk::Sampler,
        depth_sampler: vk::Sampler,
    ) {
        let color_info = vk::DescriptorImageInfo::default()
            .sampler(color_sampler)
            .image_view(color_view)
            .image_layout(vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL);

        let depth_info = vk::DescriptorImageInfo::default()
            .sampler(depth_sampler)
            .image_view(depth_view)
            .image_layout(vk::ImageLayout::DEPTH_STENCIL_READ_ONLY_OPTIMAL);

        let writes = [
            vk::WriteDescriptorSet::default()
                .dst_set(descriptor_set)
                .dst_binding(0)
                .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                .image_info(std::slice::from_ref(&color_info)),
            vk::WriteDescriptorSet::default()
                .dst_set(descriptor_set)
                .dst_binding(1)
                .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                .image_info(std::slice::from_ref(&depth_info)),
        ];

        unsafe {
            device.device.update_descriptor_sets(&writes, &[]);
        }
    }

    fn create_pipeline(
        device: &ManagedDevice,
        render_pass: vk::RenderPass,
        layout: vk::PipelineLayout,
        vert_module: vk::ShaderModule,
        binding_description: vk::VertexInputBindingDescription,
        attribute_descriptions: &[vk::VertexInputAttributeDescription],
        frag_module: vk::ShaderModule,
    ) -> EngineResult<vk::Pipeline> {
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

        let vertex_input_state = vk::PipelineVertexInputStateCreateInfo::default()
            .vertex_binding_descriptions(std::slice::from_ref(&binding_description))
            .vertex_attribute_descriptions(attribute_descriptions);

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

        // No blending — the shader composites the final color internally
        // (samples the opaque color target, applies absorption/Fresnel/etc.)
        // and outputs alpha = 1.0, fully replacing the destination pixel.
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
            .render_pass(render_pass)
            .subpass(0);

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

    /// The pipeline for an awake ripple tile's fine surface.
    pub fn fine_pipeline(&self) -> vk::Pipeline {
        self.fine_pipeline
    }

    /// The pipeline for a reach's surface.
    pub fn river_pipeline(&self) -> vk::Pipeline {
        self.river_pipeline
    }

    /// The pipeline for a fall's sheet, with a fragment shader of its own.
    pub fn fall_pipeline(&self) -> vk::Pipeline {
        self.fall_pipeline
    }

    /// A descriptor set pointing set 1 at a ripple storage buffer. One per
    /// frame slot, allocated once: a set is never rewritten while a frame may
    /// be reading it.
    pub fn allocate_ripple_set(
        &self,
        buffer: vk::Buffer,
        size: vk::DeviceSize,
    ) -> EngineResult<vk::DescriptorSet> {
        let set = Self::allocate_descriptor_set(
            &self.device,
            self.descriptor_pool,
            self.ripple_set_layout,
        )?;
        let info = vk::DescriptorBufferInfo::default()
            .buffer(buffer)
            .offset(0)
            .range(size);
        let write = vk::WriteDescriptorSet::default()
            .dst_set(set)
            .dst_binding(0)
            .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
            .buffer_info(std::slice::from_ref(&info));
        unsafe {
            self.device.device.update_descriptor_sets(&[write], &[]);
        }
        Ok(set)
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
                .destroy_pipeline(self.fine_pipeline, None);
            self.device
                .device
                .destroy_pipeline(self.river_pipeline, None);
            self.device
                .device
                .destroy_pipeline(self.fall_pipeline, None);
            self.device
                .device
                .destroy_pipeline_layout(self.pipeline_layout, None);
            self.device
                .device
                .destroy_descriptor_pool(self.descriptor_pool, None);
            self.device
                .device
                .destroy_descriptor_set_layout(self.descriptor_set_layout, None);
            self.device
                .device
                .destroy_descriptor_set_layout(self.ripple_set_layout, None);
            self.device.device.destroy_sampler(self.color_sampler, None);
            self.device.device.destroy_sampler(self.depth_sampler, None);
        }
    }
}
