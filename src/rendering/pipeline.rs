//! Graphics pipeline management.
//!
//! Provides a [`PipelineFactory`] for creating standard-geometry pipeline
//! variants from [`PipelineVariantConfig`] descriptors, and a
//! [`GraphicsPipeline`] that owns the render passes, descriptor set layouts,
//! and the standard set of pipeline variants used by the engine.

use std::mem;
use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult};
use crate::rendering::material::SURFACE_INDEX_OFFSET;
use crate::rendering::shaders::ShaderManager;
use crate::rendering::vertex::Vertex;

/// How the fragment colour is blended with the framebuffer.
#[derive(Debug, Clone, Copy)]
pub enum BlendMode {
    /// No blending -- fragment overwrites framebuffer.
    Opaque,
    /// Standard alpha blending: src*a + dst*(1-a).
    Alpha,
    /// Additive blending: src + dst.
    Additive,
}

/// Configurable fixed-function state for a standard geometry pipeline variant.
///
/// All variants share the same vertex format ([`Vertex`]), shaders
/// (`triangle.vert/frag`), pipeline layout, and descriptor set layouts.
/// Only rasterization, depth, and blend state vary between variants.
pub struct PipelineVariantConfig {
    /// Polygon fill mode (fill, line, point).
    pub polygon_mode: vk::PolygonMode,
    /// Which faces to cull (back, front, none).
    pub cull_mode: vk::CullModeFlags,
    /// Whether fragments write to the depth buffer.
    pub depth_write: bool,
    /// How fragment colour is combined with the framebuffer.
    pub blend_mode: BlendMode,
}

/// Creates standard-geometry `vk::Pipeline` handles from variant configs.
///
/// Owns the shared shader modules and stamps out pipelines that all use the
/// same vertex format ([`Vertex`]) and shader pair (`triangle.vert/frag`).
pub struct PipelineFactory {
    device: Arc<ManagedDevice>,
    vertex_shader_module: vk::ShaderModule,
    fragment_shader_module: vk::ShaderModule,
    extent: vk::Extent2D,
}

impl PipelineFactory {
    fn new(device: Arc<ManagedDevice>, extent: vk::Extent2D) -> EngineResult<Self> {
        let vertex_shader_module = ShaderManager::load_main_vertex(&device)?;
        let fragment_shader_module = ShaderManager::load_main_fragment(&device)?;
        Ok(Self {
            device,
            vertex_shader_module,
            fragment_shader_module,
            extent,
        })
    }

    /// Create a pipeline variant with the given fixed-function configuration.
    pub fn create(
        &self,
        render_pass: vk::RenderPass,
        layout: vk::PipelineLayout,
        config: &PipelineVariantConfig,
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
        let shader_stages = [
            vk::PipelineShaderStageCreateInfo::default()
                .stage(vk::ShaderStageFlags::VERTEX)
                .module(self.vertex_shader_module)
                .name(entry_name),
            vk::PipelineShaderStageCreateInfo::default()
                .stage(vk::ShaderStageFlags::FRAGMENT)
                .module(self.fragment_shader_module)
                .name(entry_name),
        ];

        let input_assembly = vk::PipelineInputAssemblyStateCreateInfo::default()
            .topology(vk::PrimitiveTopology::TRIANGLE_LIST);

        let viewports = [vk::Viewport {
            x: 0.0,
            y: 0.0,
            width: self.extent.width as f32,
            height: self.extent.height as f32,
            min_depth: 0.0,
            max_depth: 1.0,
        }];

        let scissors = [self.extent.into()];

        let viewport_state = vk::PipelineViewportStateCreateInfo::default()
            .viewports(&viewports)
            .scissors(&scissors);

        let rasterization = vk::PipelineRasterizationStateCreateInfo::default()
            .polygon_mode(config.polygon_mode)
            .cull_mode(config.cull_mode)
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
            .depth_write_enable(config.depth_write)
            .depth_compare_op(vk::CompareOp::LESS_OR_EQUAL)
            .front(stencil_op)
            .back(stencil_op)
            .max_depth_bounds(1.0);

        let color_blend_attachment = match config.blend_mode {
            BlendMode::Opaque => vk::PipelineColorBlendAttachmentState {
                blend_enable: vk::FALSE,
                color_write_mask: vk::ColorComponentFlags::RGBA,
                ..Default::default()
            },
            BlendMode::Alpha => vk::PipelineColorBlendAttachmentState {
                blend_enable: vk::TRUE,
                src_color_blend_factor: vk::BlendFactor::SRC_ALPHA,
                dst_color_blend_factor: vk::BlendFactor::ONE_MINUS_SRC_ALPHA,
                color_blend_op: vk::BlendOp::ADD,
                src_alpha_blend_factor: vk::BlendFactor::ONE,
                dst_alpha_blend_factor: vk::BlendFactor::ONE_MINUS_SRC_ALPHA,
                alpha_blend_op: vk::BlendOp::ADD,
                color_write_mask: vk::ColorComponentFlags::RGBA,
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

        let color_blend_attachments = [color_blend_attachment];
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
            .render_pass(render_pass);

        unsafe {
            let pipelines = self
                .device
                .device
                .create_graphics_pipelines(vk::PipelineCache::null(), &[pipeline_info], None)
                .map_err(|(_pipelines, e)| EngineError::Pipeline(format!("creation: {:?}", e)))?;

            Ok(pipelines[0])
        }
    }
}

impl Drop for PipelineFactory {
    fn drop(&mut self) {
        unsafe {
            self.device
                .device
                .destroy_shader_module(self.vertex_shader_module, None);
            self.device
                .device
                .destroy_shader_module(self.fragment_shader_module, None);
        }
    }
}

/// Configuration for creating a [`GraphicsPipeline`].
pub struct GraphicsPipelineConfig {
    /// Format of the offscreen target the opaque pass renders into. Floating
    /// point, so emissive surfaces can carry radiance above 1.0 through to the
    /// post-processing resolve.
    pub scene_color_format: vk::Format,
    /// Format of the swapchain images the transparent pass renders into.
    pub swapchain_format: vk::Format,
    pub depth_format: vk::Format,
    pub extent: vk::Extent2D,
    /// The comparison sampler the shadow map is read through.
    ///
    /// Baked into the scene descriptor set layout as an immutable sampler.
    /// Metal — and so MoltenVK — cannot take a comparison function from a
    /// descriptor write, only from the layout, so this is not merely an
    /// optimisation: writing one at runtime fails on macOS.
    pub shadow_sampler: vk::Sampler,
}

/// Immutable graphics pipeline configuration and state.
///
/// Owns the render passes, descriptor set layouts, pipeline layout, and the
/// standard set of pipeline variants (opaque, wireframe, transparent).
pub struct GraphicsPipeline {
    /// Standard opaque fill pipeline (back-face culled, depth write, no blend).
    pub opaque: vk::Pipeline,
    /// Wireframe pipeline rendering only backfaces (front-face culled).
    pub wireframe_backface: vk::Pipeline,
    /// Alpha-blended pipeline for transparent geometry (no cull, no depth write).
    pub transparent: vk::Pipeline,
    /// Blended scene geometry, back faces only (front-face culled).
    ///
    /// The first of the two draws every blended mesh takes. Recorded into the
    /// HDR scene pass rather than the post-tonemap one, so glass is exposed
    /// and bloomed with the rest of the scene instead of being pasted on after.
    /// Writes depth, because the passes drawn after the resolve test against
    /// it — see the pipeline's configuration for why that is safe here.
    pub scene_blended_back: vk::Pipeline,
    /// Blended scene geometry, front faces only. The second of the two draws.
    pub scene_blended_front: vk::Pipeline,
    /// Pipeline layout shared by all standard geometry variants.
    pub layout: vk::PipelineLayout,
    /// Render pass for opaque geometry.
    pub renderpass: vk::RenderPass,
    /// Render pass for transparent geometry (loads existing colour + depth read-only).
    pub transparent_renderpass: vk::RenderPass,
    pub scene_ubo_descriptor_set_layout: vk::DescriptorSetLayout,
    pub sampler_descriptor_set_layout: vk::DescriptorSetLayout,
    #[allow(dead_code)]
    factory: PipelineFactory,
    device: Arc<ManagedDevice>,
}

impl GraphicsPipeline {
    /// Create a new graphics pipeline with the given configuration.
    pub fn new(device: Arc<ManagedDevice>, config: &GraphicsPipelineConfig) -> EngineResult<Self> {
        // Create descriptor set layouts
        let scene_ubo_descriptor_set_layout =
            Self::create_ubo_descriptor_layout(&device, config.shadow_sampler)?;
        let sampler_descriptor_set_layout = Self::create_sampler_descriptor_layout(&device)?;

        // Create pipeline layout with push constants for per-object data
        let set_layouts = [
            scene_ubo_descriptor_set_layout,
            sampler_descriptor_set_layout,
        ];

        // Push constants, one range covering both stages:
        // - mat4 model            (offset  0, 64 bytes) — vertex, and fragment,
        //     which reads its rotation to place an object-space grain
        // - vec4 colourOverride   (offset 64, 16 bytes) — fragment
        // - uint surfaceIndex     (offset 80,  4 bytes) — fragment
        //
        // A single range rather than one per stage: two ranges may not declare
        // the same stage, and the fragment block now starts at offset 0.
        //
        // The parameters themselves live in the surface table (set 0, binding
        // 3); only the index travels here. See `rendering::surface_buffer`.
        let push_constant_ranges = [vk::PushConstantRange {
            stage_flags: vk::ShaderStageFlags::VERTEX | vk::ShaderStageFlags::FRAGMENT,
            offset: 0,
            size: SURFACE_INDEX_OFFSET + std::mem::size_of::<u32>() as u32,
        }];

        let pipeline_layout_create_info = vk::PipelineLayoutCreateInfo::default()
            .set_layouts(&set_layouts)
            .push_constant_ranges(&push_constant_ranges);
        let layout = unsafe {
            device
                .device
                .create_pipeline_layout(&pipeline_layout_create_info, None)
                .map_err(|e| EngineError::Pipeline(format!("layout creation: {:?}", e)))?
        };

        // Create render passes
        let renderpass = Self::create_render_pass(&device, config)?;
        let transparent_renderpass = Self::create_transparent_render_pass(
            &device,
            config.swapchain_format,
            config.depth_format,
        )?;

        // Create pipeline factory and stamp out standard variants
        let factory = PipelineFactory::new(Arc::clone(&device), config.extent)?;

        let opaque = factory.create(
            renderpass,
            layout,
            &PipelineVariantConfig {
                polygon_mode: vk::PolygonMode::FILL,
                cull_mode: vk::CullModeFlags::BACK,
                depth_write: true,
                blend_mode: BlendMode::Opaque,
            },
        )?;

        let wireframe_backface = factory.create(
            renderpass,
            layout,
            &PipelineVariantConfig {
                polygon_mode: vk::PolygonMode::LINE,
                cull_mode: vk::CullModeFlags::FRONT,
                depth_write: true,
                blend_mode: BlendMode::Opaque,
            },
        )?;

        // Blended scene geometry, drawn as two passes over the same mesh.
        // Within one draw the rasteriser has no ordering to offer — a mesh's
        // own far and near faces arrive in index order — so the far half is
        // separated out by culling and recorded first. Exact for a convex
        // shape, which is what an ice cube is.
        //
        // Depth *is* written, which is unusual for blended geometry and is
        // affordable only because these draws are sorted back to front before
        // they are recorded: each one is nearer than everything already in the
        // buffer, so it passes the test it would otherwise have to be excused
        // from. What that buys is the passes that come after the scene
        // resolves — water and fire — which test against this depth and have
        // no other way of knowing the glass is there. Without it the water
        // surface paints straight over an ice cube standing in a pond.
        //
        // It also means anything drawn after a blended mesh is occluded by it,
        // which is why particles are recorded in this pass, interleaved with
        // these draws, rather than after the resolve.
        let blended_scene_config = |cull_mode| PipelineVariantConfig {
            polygon_mode: vk::PolygonMode::FILL,
            cull_mode,
            depth_write: true,
            blend_mode: BlendMode::Alpha,
        };

        let scene_blended_back = factory.create(
            renderpass,
            layout,
            &blended_scene_config(vk::CullModeFlags::FRONT),
        )?;

        let scene_blended_front = factory.create(
            renderpass,
            layout,
            &blended_scene_config(vk::CullModeFlags::BACK),
        )?;

        let transparent = factory.create(
            transparent_renderpass,
            layout,
            &PipelineVariantConfig {
                polygon_mode: vk::PolygonMode::FILL,
                cull_mode: vk::CullModeFlags::NONE,
                depth_write: false,
                blend_mode: BlendMode::Alpha,
            },
        )?;

        Ok(Self {
            opaque,
            wireframe_backface,
            transparent,
            scene_blended_back,
            scene_blended_front,
            layout,
            renderpass,
            transparent_renderpass,
            scene_ubo_descriptor_set_layout,
            sampler_descriptor_set_layout,
            factory,
            device,
        })
    }

    fn create_ubo_descriptor_layout(
        device: &ManagedDevice,
        shadow_sampler: vk::Sampler,
    ) -> EngineResult<vk::DescriptorSetLayout> {
        // Binding 0: the vertex stage reads the matrices; the fragment stage
        // reads the camera position and sun lighting from the same block.
        // Binding 1: the per-frame point light set, fragment stage only.
        // Binding 2: the sun shadow map, sampled through an immutable
        // comparison sampler (see `GraphicsPipelineConfig::shadow_sampler`).
        // Binding 3: the frame's surface table, indexed by the push constant.
        //   A storage buffer rather than a uniform one because it is sized for
        //   the worst frame rather than for a fixed small count.
        // Binding 4: the shared grain texture, one microstructure atlas for
        //   every material in the scene rather than one per material.
        // Binding 5: the reflection probes, one cube-map array holding every
        //   live probe (see `rendering::reflection`).
        let immutable_shadow_sampler = [shadow_sampler];
        let bindings = [
            vk::DescriptorSetLayoutBinding::default()
                .binding(0)
                .descriptor_type(vk::DescriptorType::UNIFORM_BUFFER)
                .descriptor_count(1)
                .stage_flags(vk::ShaderStageFlags::VERTEX | vk::ShaderStageFlags::FRAGMENT),
            vk::DescriptorSetLayoutBinding::default()
                .binding(1)
                .descriptor_type(vk::DescriptorType::UNIFORM_BUFFER)
                .descriptor_count(1)
                .stage_flags(vk::ShaderStageFlags::FRAGMENT),
            vk::DescriptorSetLayoutBinding::default()
                .binding(2)
                .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                .stage_flags(vk::ShaderStageFlags::FRAGMENT)
                // Sets descriptor_count from the slice length, so it must come
                // after any explicit count rather than before it.
                .immutable_samplers(&immutable_shadow_sampler),
            vk::DescriptorSetLayoutBinding::default()
                .binding(3)
                .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
                .descriptor_count(1)
                .stage_flags(vk::ShaderStageFlags::FRAGMENT),
            vk::DescriptorSetLayoutBinding::default()
                .binding(4)
                .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                .descriptor_count(1)
                .stage_flags(vk::ShaderStageFlags::FRAGMENT),
            vk::DescriptorSetLayoutBinding::default()
                .binding(5)
                .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                .descriptor_count(1)
                .stage_flags(vk::ShaderStageFlags::FRAGMENT),
        ];

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
    /// Writes to the offscreen HDR `ColorTarget` and depth buffer. The color
    /// attachment transitions to `SHADER_READ_ONLY_OPTIMAL` at the end, ready to
    /// be sampled by the post-processing resolve and by the water shader for
    /// refraction.
    fn create_render_pass(
        device: &ManagedDevice,
        config: &GraphicsPipelineConfig,
    ) -> EngineResult<vk::RenderPass> {
        let attachments = [
            vk::AttachmentDescription {
                format: config.scene_color_format,
                samples: vk::SampleCountFlags::TYPE_1,
                load_op: vk::AttachmentLoadOp::CLEAR,
                store_op: vk::AttachmentStoreOp::STORE,
                final_layout: vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
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

        // The previous frame may still be on the GPU, reading the depth
        // buffer in its transparent pass and sampling the colour target in its
        // resolve and water passes. Both finish by its colour output, and the
        // depth clear here must also wait for its depth writes.
        let depth_stages = vk::PipelineStageFlags::EARLY_FRAGMENT_TESTS
            | vk::PipelineStageFlags::LATE_FRAGMENT_TESTS;
        let dependencies = [vk::SubpassDependency {
            src_subpass: vk::SUBPASS_EXTERNAL,
            src_stage_mask: vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT | depth_stages,
            src_access_mask: vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_WRITE,
            dst_access_mask: vk::AccessFlags::COLOR_ATTACHMENT_READ
                | vk::AccessFlags::COLOR_ATTACHMENT_WRITE
                | vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_READ
                | vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_WRITE,
            dst_stage_mask: vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT | depth_stages,
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

    /// Render pass for what is composited after the HDR resolve (water, fire,
    /// overlay).
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
                // Stays a colour attachment: the bloom overlay pass runs after
                // this one and is what finally transitions to PRESENT_SRC_KHR.
                final_layout: vk::ImageLayout::COLOR_ATTACHMENT_OPTIMAL,
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

        // Depth goes read-only for this pass and back afterwards, and both
        // layout transitions are writes: the first must follow the opaque
        // pass's depth writes, and the next frame's opaque pass must follow
        // the second.
        let depth_stages = vk::PipelineStageFlags::EARLY_FRAGMENT_TESTS
            | vk::PipelineStageFlags::LATE_FRAGMENT_TESTS;
        let dependencies = [
            vk::SubpassDependency {
                src_subpass: vk::SUBPASS_EXTERNAL,
                dst_subpass: 0,
                src_stage_mask: vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT
                    | vk::PipelineStageFlags::TRANSFER
                    | vk::PipelineStageFlags::LATE_FRAGMENT_TESTS,
                src_access_mask: vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_WRITE,
                dst_stage_mask: vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT
                    | vk::PipelineStageFlags::FRAGMENT_SHADER
                    | vk::PipelineStageFlags::EARLY_FRAGMENT_TESTS,
                dst_access_mask: vk::AccessFlags::COLOR_ATTACHMENT_WRITE
                    | vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_READ,
                ..Default::default()
            },
            vk::SubpassDependency {
                src_subpass: 0,
                dst_subpass: vk::SUBPASS_EXTERNAL,
                src_stage_mask: vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT | depth_stages,
                // Discarding depth at the end of the pass is a store, and so a
                // write the transition back must follow.
                src_access_mask: vk::AccessFlags::COLOR_ATTACHMENT_WRITE
                    | vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_WRITE,
                dst_stage_mask: vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT
                    | vk::PipelineStageFlags::FRAGMENT_SHADER
                    | depth_stages,
                dst_access_mask: vk::AccessFlags::COLOR_ATTACHMENT_READ
                    | vk::AccessFlags::COLOR_ATTACHMENT_WRITE
                    | vk::AccessFlags::SHADER_READ
                    | vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_READ
                    | vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_WRITE,
                ..Default::default()
            },
        ];

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
}

impl Drop for GraphicsPipeline {
    fn drop(&mut self) {
        unsafe {
            log::debug!("GraphicsPipeline::drop - cleaning up pipeline resources");

            self.device.device.destroy_pipeline(self.opaque, None);
            self.device
                .device
                .destroy_pipeline(self.wireframe_backface, None);
            self.device.device.destroy_pipeline(self.transparent, None);
            self.device
                .device
                .destroy_pipeline(self.scene_blended_back, None);
            self.device
                .device
                .destroy_pipeline(self.scene_blended_front, None);
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
            // PipelineFactory Drop handles shader module cleanup
        }
    }
}
