//! Filters captured probe faces down their mip chains, in one dispatch.

use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult};
use crate::rendering::compute::{ComputePipeline, ComputePipelineConfig};
use crate::rendering::reflection::atlas::{ProbeAtlas, PROBE_LAYOUT, PROBE_MIP_LEVELS};
use crate::rendering::reflection::faces::MAX_FACES_PER_FRAME;
use crate::rendering::shaders::ShaderManager;

mod bytecode {
    use crate::embedded_spirv;

    pub const PROBE_MIPS: &[u8] = embedded_spirv!("probe_mips.comp");
}

/// Mips the filter writes: every one but the captured one.
const FILTERED_LEVELS: u32 = PROBE_MIP_LEVELS - 1;

/// Fills the smaller mips of the faces captured this frame from their mip 0.
///
/// ```text
///   faces captured ──render pass──▶ mip 0 ──▶ dispatch, one workgroup a face
///                                              └─▶ mips 1..5, box-filtered
/// ```
///
/// A compute pass rather than a chain of blits: MoltenVK runs each blit as a
/// render pass of its own, and with six faces a frame the chain measured
/// 0.8 ms of GPU time where this takes a few microseconds.
pub struct ProbeMipFilter {
    pipeline: ComputePipeline,
    set_layout: vk::DescriptorSetLayout,
    pool: vk::DescriptorPool,
    /// Mip 0 of every layer, and each smaller mip of every layer. Written
    /// once: the atlas never moves, and its layout never changes.
    set: vk::DescriptorSet,
    /// Reads mip 0 texel by texel.
    sampler: vk::Sampler,
    device: Arc<ManagedDevice>,
}

impl ProbeMipFilter {
    pub fn new(device: Arc<ManagedDevice>, atlas: &ProbeAtlas) -> EngineResult<Self> {
        let set_layout = create_set_layout(&device)?;
        let shader = ShaderManager::load_compute(&device, bytecode::PROBE_MIPS)?;
        let push_constant_ranges = [vk::PushConstantRange {
            stage_flags: vk::ShaderStageFlags::COMPUTE,
            offset: 0,
            size: MAX_FACES_PER_FRAME * std::mem::size_of::<u32>() as u32,
        }];
        let pipeline = ComputePipeline::new(
            Arc::clone(&device),
            &ComputePipelineConfig {
                shader_module: shader,
                descriptor_set_layouts: &[set_layout],
                push_constant_ranges: &push_constant_ranges,
            },
        );
        unsafe { device.device.destroy_shader_module(shader, None) };
        let pipeline = pipeline?;

        let pool_sizes = [
            vk::DescriptorPoolSize::default()
                .ty(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                .descriptor_count(1),
            vk::DescriptorPoolSize::default()
                .ty(vk::DescriptorType::STORAGE_IMAGE)
                .descriptor_count(FILTERED_LEVELS),
        ];
        let pool_info = vk::DescriptorPoolCreateInfo::default()
            .max_sets(1)
            .pool_sizes(&pool_sizes);
        let pool = unsafe { device.device.create_descriptor_pool(&pool_info, None) }
            .map_err(|e| EngineError::Descriptor(format!("probe mip pool: {:?}", e)))?;
        let layouts = [set_layout];
        let alloc_info = vk::DescriptorSetAllocateInfo::default()
            .descriptor_pool(pool)
            .set_layouts(&layouts);
        let set = unsafe { device.device.allocate_descriptor_sets(&alloc_info) }
            .map_err(|e| EngineError::Descriptor(format!("probe mip set: {:?}", e)))?[0];

        let sampler = create_sampler(&device)?;
        let source = [vk::DescriptorImageInfo::default()
            .image_layout(PROBE_LAYOUT)
            .image_view(atlas.level_view(0))
            .sampler(sampler)];
        let targets: Vec<vk::DescriptorImageInfo> = (1..PROBE_MIP_LEVELS)
            .map(|level| {
                vk::DescriptorImageInfo::default()
                    .image_layout(PROBE_LAYOUT)
                    .image_view(atlas.level_view(level))
            })
            .collect();
        let writes = [
            vk::WriteDescriptorSet::default()
                .dst_set(set)
                .dst_binding(0)
                .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                .image_info(&source),
            vk::WriteDescriptorSet::default()
                .dst_set(set)
                .dst_binding(1)
                .descriptor_type(vk::DescriptorType::STORAGE_IMAGE)
                .image_info(&targets),
        ];
        unsafe { device.device.update_descriptor_sets(&writes, &[]) };

        Ok(Self {
            pipeline,
            set_layout,
            pool,
            set,
            sampler,
            device,
        })
    }

    /// Filter the mips of the faces in `layers`, all captured earlier in `cb`,
    /// and make them ready for the scene to sample.
    pub fn record(&self, cb: vk::CommandBuffer, layers: &[u32]) {
        if layers.is_empty() {
            return;
        }
        let mut push = [0u32; MAX_FACES_PER_FRAME as usize];
        push[..layers.len()].copy_from_slice(layers);
        let push_bytes: Vec<u8> = push.iter().flat_map(|layer| layer.to_ne_bytes()).collect();

        let device = &self.device.device;
        unsafe {
            // The smaller mips are about to be overwritten: whatever frame
            // last sampled or filtered them must be done. Mip 0's capture is
            // ordered before this by the capture pass's own outgoing
            // dependency.
            let previous = vk::MemoryBarrier::default()
                .src_access_mask(vk::AccessFlags::SHADER_WRITE)
                .dst_access_mask(vk::AccessFlags::SHADER_WRITE);
            device.cmd_pipeline_barrier(
                cb,
                vk::PipelineStageFlags::FRAGMENT_SHADER | vk::PipelineStageFlags::COMPUTE_SHADER,
                vk::PipelineStageFlags::COMPUTE_SHADER,
                vk::DependencyFlags::empty(),
                &[previous],
                &[],
                &[],
            );
            device.cmd_bind_pipeline(cb, vk::PipelineBindPoint::COMPUTE, self.pipeline.pipeline);
            device.cmd_bind_descriptor_sets(
                cb,
                vk::PipelineBindPoint::COMPUTE,
                self.pipeline.layout,
                0,
                &[self.set],
                &[],
            );
            device.cmd_push_constants(
                cb,
                self.pipeline.layout,
                vk::ShaderStageFlags::COMPUTE,
                0,
                &push_bytes,
            );
            device.cmd_dispatch(cb, layers.len() as u32, 1, 1);

            // The scene samples every mip of them next.
            let written = vk::MemoryBarrier::default()
                .src_access_mask(vk::AccessFlags::SHADER_WRITE)
                .dst_access_mask(vk::AccessFlags::SHADER_READ);
            device.cmd_pipeline_barrier(
                cb,
                vk::PipelineStageFlags::COMPUTE_SHADER,
                vk::PipelineStageFlags::FRAGMENT_SHADER,
                vk::DependencyFlags::empty(),
                &[written],
                &[],
                &[],
            );
        }
    }
}

impl Drop for ProbeMipFilter {
    fn drop(&mut self) {
        unsafe {
            self.device.device.destroy_descriptor_pool(self.pool, None);
            self.device
                .device
                .destroy_descriptor_set_layout(self.set_layout, None);
            self.device.device.destroy_sampler(self.sampler, None);
        }
    }
}

fn create_set_layout(device: &ManagedDevice) -> EngineResult<vk::DescriptorSetLayout> {
    let bindings = [
        vk::DescriptorSetLayoutBinding::default()
            .binding(0)
            .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
            .descriptor_count(1)
            .stage_flags(vk::ShaderStageFlags::COMPUTE),
        vk::DescriptorSetLayoutBinding::default()
            .binding(1)
            .descriptor_type(vk::DescriptorType::STORAGE_IMAGE)
            .descriptor_count(FILTERED_LEVELS)
            .stage_flags(vk::ShaderStageFlags::COMPUTE),
    ];
    let info = vk::DescriptorSetLayoutCreateInfo::default().bindings(&bindings);
    unsafe { device.device.create_descriptor_set_layout(&info, None) }
        .map_err(|e| EngineError::Descriptor(format!("probe mip layout: {:?}", e)))
}

fn create_sampler(device: &ManagedDevice) -> EngineResult<vk::Sampler> {
    let info = vk::SamplerCreateInfo::default()
        .mag_filter(vk::Filter::NEAREST)
        .min_filter(vk::Filter::NEAREST)
        .mipmap_mode(vk::SamplerMipmapMode::NEAREST)
        .address_mode_u(vk::SamplerAddressMode::CLAMP_TO_EDGE)
        .address_mode_v(vk::SamplerAddressMode::CLAMP_TO_EDGE)
        .address_mode_w(vk::SamplerAddressMode::CLAMP_TO_EDGE)
        .max_lod(0.0);
    unsafe { device.device.create_sampler(&info, None) }
        .map_err(|e| EngineError::Descriptor(format!("probe mip sampler: {:?}", e)))
}
