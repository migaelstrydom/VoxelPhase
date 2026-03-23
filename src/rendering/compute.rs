//! Compute pipeline infrastructure.
//!
//! Provides a reusable `ComputePipeline` abstraction for dispatching compute
//! shaders. This is the engine's first compute pipeline support, introduced
//! for the volumetric fire simulation but designed to be general-purpose.

use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult};

/// A single Vulkan compute pipeline with its layout.
///
/// Follows the same RAII pattern as `GraphicsPipeline` — pipeline and layout
/// are cleaned up automatically on drop. Descriptor set layouts are **not**
/// owned here — the caller manages their lifetime, since they are typically
/// needed for descriptor set allocation as well.
pub struct ComputePipeline {
    pub pipeline: vk::Pipeline,
    pub layout: vk::PipelineLayout,
    device: Arc<ManagedDevice>,
}

/// Configuration for creating a compute pipeline.
pub struct ComputePipelineConfig<'a> {
    /// Compiled SPIR-V shader module for the compute stage.
    pub shader_module: vk::ShaderModule,
    /// Descriptor set layouts (bindings for textures, buffers, etc.).
    /// These are referenced by the pipeline layout but not owned by the pipeline.
    pub descriptor_set_layouts: &'a [vk::DescriptorSetLayout],
    /// Push constant ranges for per-dispatch parameters.
    pub push_constant_ranges: &'a [vk::PushConstantRange],
}

impl ComputePipeline {
    /// Create a new compute pipeline.
    ///
    /// The caller is responsible for destroying the `shader_module` after this
    /// call returns, and for keeping the descriptor set layouts alive for the
    /// lifetime of this pipeline.
    pub fn new(
        device: Arc<ManagedDevice>,
        config: &ComputePipelineConfig,
    ) -> EngineResult<Self> {
        let layout = Self::create_pipeline_layout(&device, config)?;
        let pipeline = Self::create_pipeline(&device, layout, config.shader_module)?;

        Ok(Self {
            pipeline,
            layout,
            device,
        })
    }

    fn create_pipeline_layout(
        device: &ManagedDevice,
        config: &ComputePipelineConfig,
    ) -> EngineResult<vk::PipelineLayout> {
        let create_info = vk::PipelineLayoutCreateInfo::default()
            .set_layouts(config.descriptor_set_layouts)
            .push_constant_ranges(config.push_constant_ranges);

        unsafe {
            device
                .device
                .create_pipeline_layout(&create_info, None)
                .map_err(|e| EngineError::Pipeline(format!("compute layout creation: {:?}", e)))
        }
    }

    fn create_pipeline(
        device: &ManagedDevice,
        layout: vk::PipelineLayout,
        shader_module: vk::ShaderModule,
    ) -> EngineResult<vk::Pipeline> {
        let entry_name = c"main";
        let stage = vk::PipelineShaderStageCreateInfo::default()
            .stage(vk::ShaderStageFlags::COMPUTE)
            .module(shader_module)
            .name(entry_name);

        let create_info = vk::ComputePipelineCreateInfo::default()
            .stage(stage)
            .layout(layout);

        unsafe {
            let pipelines = device
                .device
                .create_compute_pipelines(vk::PipelineCache::null(), &[create_info], None)
                .map_err(|(_pipelines, e)| {
                    EngineError::Pipeline(format!("compute pipeline creation: {:?}", e))
                })?;

            Ok(pipelines[0])
        }
    }
}

impl Drop for ComputePipeline {
    fn drop(&mut self) {
        unsafe {
            self.device.device.destroy_pipeline(self.pipeline, None);
            self.device
                .device
                .destroy_pipeline_layout(self.layout, None);
        }
    }
}
