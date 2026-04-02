//! Fire volume rendering via raymarching.
//!
//! Combines the compute simulation (`FireSimPipelines`) with a graphics pipeline
//! that raymarches through the 3D volume textures in the transparent pass.

use std::sync::Arc;

use ash::vk;
use nalgebra::{Matrix4, Vector3};

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult};
use crate::rendering::frame::ManagedBuffer;
use crate::rendering::shaders::ShaderManager;

use super::pipeline::{FireSimParams, FireSimPipelines, FireVolumeDescriptors};
use super::volume::FireVolume;

/// Embedded shader bytecode for the raymarching graphics pipeline.
mod bytecode {
    pub const FIRE_VERTEX: &[u8] = include_bytes!("../../shader/fire/fire.vert.spv");
    pub const FIRE_FRAGMENT: &[u8] = include_bytes!("../../shader/fire/fire.frag.spv");
}

/// Maximum number of fire volumes that can be rendered simultaneously.
const MAX_FIRE_VOLUMES: usize = 32;

/// Push constants for the fire raymarching pipeline.
/// Shared by both vertex and fragment stages.
#[repr(C)]
#[derive(Clone, Copy)]
struct FireRenderPushConstants {
    view: [[f32; 4]; 4],
    proj: [[f32; 4]; 4],
    volume_to_world: [[f32; 4]; 4],
    camera_pos: [f32; 4],
}

/// Number of simulation slots in the shared pool.
/// Each slot owns full GPU resources (6 textures + descriptors).
/// Multiple fires share slots so compute cost is O(pool_size), not O(fire_count).
pub const SIM_POOL_SIZE: usize = 5;

/// A GPU simulation slot with its own volume textures and descriptors.
/// Allocated once at startup and reused for the lifetime of the renderer.
struct SimSlot {
    volume: FireVolume,
    sim_descriptors: FireVolumeDescriptors,
    render_descriptor_set: vk::DescriptorSet,
    /// Random noise seed so each slot evolves differently.
    noise_seed: f32,
}

/// Lightweight metadata for an active fire instance.
/// Multiple fires can reference the same sim slot, sharing its GPU volume
/// but rendering with different world transforms.
pub struct ActiveFire {
    /// Index into the sim slot pool.
    pub sim_slot: usize,
    /// World-space transform of the volume (origin + scale).
    pub volume_to_world: Matrix4<f32>,
    /// Remaining fuel (synced from OnFire component each frame).
    pub fuel_remaining: f32,
    /// Initial fuel at ignition (used to compute source intensity ratio).
    pub initial_fuel: f32,
    /// Accumulated time the fire has been burning.
    pub burn_time: f32,
}

/// Orchestrates fire simulation and rendering.
///
/// Owns a fixed pool of simulation slots (GPU volumes + descriptors) and
/// the raymarching graphics pipeline. The `Renderer` calls `simulate()`
/// before the render pass and `render()` during the transparent pass.
/// Compute cost is constant regardless of how many fires are active.
pub struct FireRenderer {
    sim_pipelines: FireSimPipelines,
    sim_slots: Vec<SimSlot>,
    graphics_pipeline: vk::Pipeline,
    graphics_pipeline_layout: vk::PipelineLayout,
    graphics_descriptor_set_layout: vk::DescriptorSetLayout,
    graphics_descriptor_pool: vk::DescriptorPool,
    depth_sampler: vk::Sampler,
    cube_vertex_buffer: ManagedBuffer,
    cube_index_buffer: ManagedBuffer,
    device: Arc<ManagedDevice>,
}

impl FireRenderer {
    pub fn new(
        device: Arc<ManagedDevice>,
        render_pass: vk::RenderPass,
        depth_view: vk::ImageView,
    ) -> EngineResult<Self> {
        let sim_pipelines = FireSimPipelines::new(device.clone())?;

        let depth_sampler = Self::create_depth_sampler(&device)?;
        let graphics_descriptor_set_layout = Self::create_graphics_descriptor_layout(&device)?;
        let graphics_pipeline_layout =
            Self::create_graphics_pipeline_layout(&device, graphics_descriptor_set_layout)?;
        let graphics_pipeline =
            Self::create_graphics_pipeline(&device, render_pass, graphics_pipeline_layout)?;
        let graphics_descriptor_pool = Self::create_graphics_descriptor_pool(&device)?;

        let (cube_vertex_buffer, cube_index_buffer) = Self::create_cube_buffers(&device)?;

        // Allocate the shared simulation pool upfront
        let mut sim_slots = Vec::with_capacity(SIM_POOL_SIZE);
        for i in 0..SIM_POOL_SIZE {
            let volume = FireVolume::new(device.clone())?;
            let sim_descriptors = sim_pipelines.allocate_volume_descriptors()?;
            sim_pipelines.update_volume_descriptors(&volume, &sim_descriptors);

            let render_descriptor_set = Self::allocate_render_descriptor_set_from_pool(
                &device,
                graphics_descriptor_pool,
                graphics_descriptor_set_layout,
            )?;
            Self::write_render_descriptor_set(
                &device,
                render_descriptor_set,
                &volume,
                depth_sampler,
                depth_view,
            );

            sim_slots.push(SimSlot {
                volume,
                sim_descriptors,
                render_descriptor_set,
                noise_seed: i as f32 * 200.0,
            });
        }

        Ok(Self {
            sim_pipelines,
            sim_slots,
            graphics_pipeline,
            graphics_pipeline_layout,
            graphics_descriptor_set_layout,
            graphics_descriptor_pool,
            depth_sampler,
            cube_vertex_buffer,
            cube_index_buffer,
            device,
        })
    }

    /// Assign a sim slot to a new fire using least-used allocation.
    pub fn assign_slot(&self, existing_fires: &[(specs::Entity, ActiveFire)]) -> usize {
        let mut counts = [0usize; SIM_POOL_SIZE];
        for (_, fire) in existing_fires {
            counts[fire.sim_slot] += 1;
        }
        counts.iter().enumerate().min_by_key(|(_, &c)| c).unwrap().0
    }

    /// Simulation time scale. Values > 1.0 make fire evolve faster
    /// (more energetic flames) without extra compute dispatches.
    const SIM_TIME_SCALE: f32 = 1.5;

    /// Record compute dispatches for active sim slots.
    /// Only slots referenced by at least one fire are simulated.
    /// Source intensity per slot is the max across all fires sharing it.
    /// Must be called before the render pass begins.
    pub fn simulate(
        &mut self,
        cb: vk::CommandBuffer,
        fires: &[&ActiveFire],
        dt: f32,
        total_time: f32,
    ) {
        // Compute per-slot max source intensity
        let mut slot_intensities = [0.0f32; SIM_POOL_SIZE];
        for fire in fires {
            let intensity = if fire.initial_fuel > 0.0 {
                (fire.fuel_remaining / fire.initial_fuel).clamp(0.0, 1.0)
            } else {
                0.0
            };
            slot_intensities[fire.sim_slot] = slot_intensities[fire.sim_slot].max(intensity);
        }

        let mut params = FireSimParams::default();
        params.dt = dt * Self::SIM_TIME_SCALE;
        params.time = total_time * Self::SIM_TIME_SCALE;

        let device = &self.device.device;
        for (i, slot) in self.sim_slots.iter_mut().enumerate() {
            if slot_intensities[i] <= 0.0 {
                continue;
            }
            params.source_intensity = slot_intensities[i];
            params.noise_seed = slot.noise_seed;

            transition_volumes_for_compute(device, cb, &slot.volume);
            self.sim_pipelines
                .simulate(cb, &mut slot.volume, &slot.sim_descriptors, &params);
        }

        // After simulation, update render descriptors to point at the current
        // source texture (advection swaps read/write indices each frame), then
        // transition field textures to SHADER_READ_ONLY for fragment sampling.
        // Both must happen here (outside the render pass), not in render().
        for (i, slot) in self.sim_slots.iter().enumerate() {
            if slot_intensities[i] <= 0.0 {
                continue;
            }
            update_render_volume_binding(device, slot.render_descriptor_set, &slot.volume);
            transition_field_for_read(device, cb, &slot.volume);
        }
    }

    /// Record raymarching draw calls for all active fires.
    /// Each fire binds its sim slot's descriptor set and draws with its own transform.
    /// Must be called during the transparent pass, after water and before particles.
    pub fn render(
        &self,
        cb: vk::CommandBuffer,
        fires: &[&ActiveFire],
        view_matrix: &Matrix4<f32>,
        proj_matrix: &Matrix4<f32>,
        camera_pos: &Vector3<f32>,
    ) {
        if fires.is_empty() {
            return;
        }

        let device = &self.device.device;

        unsafe {
            device.cmd_bind_pipeline(cb, vk::PipelineBindPoint::GRAPHICS, self.graphics_pipeline);

            device.cmd_bind_vertex_buffers(cb, 0, &[self.cube_vertex_buffer.buffer], &[0]);
            device.cmd_bind_index_buffer(
                cb,
                self.cube_index_buffer.buffer,
                0,
                vk::IndexType::UINT16,
            );

            for fire in fires {
                let slot = &self.sim_slots[fire.sim_slot];

                let push = FireRenderPushConstants {
                    view: matrix4_to_array(view_matrix),
                    proj: matrix4_to_array(proj_matrix),
                    volume_to_world: matrix4_to_array(&fire.volume_to_world),
                    camera_pos: [camera_pos.x, camera_pos.y, camera_pos.z, 0.0],
                };

                device.cmd_push_constants(
                    cb,
                    self.graphics_pipeline_layout,
                    vk::ShaderStageFlags::VERTEX | vk::ShaderStageFlags::FRAGMENT,
                    0,
                    std::slice::from_raw_parts(
                        &push as *const FireRenderPushConstants as *const u8,
                        std::mem::size_of::<FireRenderPushConstants>(),
                    ),
                );

                device.cmd_bind_descriptor_sets(
                    cb,
                    vk::PipelineBindPoint::GRAPHICS,
                    self.graphics_pipeline_layout,
                    0,
                    &[slot.render_descriptor_set],
                    &[],
                );

                // Draw the unit cube (36 indices = 12 triangles)
                device.cmd_draw_indexed(cb, 36, 1, 0, 0, 0);
            }
        }
    }

    // --- Descriptor set management (static helpers for use during init) ---

    fn allocate_render_descriptor_set_from_pool(
        device: &ManagedDevice,
        pool: vk::DescriptorPool,
        layout: vk::DescriptorSetLayout,
    ) -> EngineResult<vk::DescriptorSet> {
        let alloc_info = vk::DescriptorSetAllocateInfo::default()
            .descriptor_pool(pool)
            .set_layouts(std::slice::from_ref(&layout));

        let sets = unsafe {
            device
                .device
                .allocate_descriptor_sets(&alloc_info)
                .map_err(|e| {
                    EngineError::Descriptor(format!("fire render descriptor allocation: {:?}", e))
                })?
        };

        Ok(sets[0])
    }

    fn write_render_descriptor_set(
        device: &ManagedDevice,
        set: vk::DescriptorSet,
        volume: &FireVolume,
        depth_sampler: vk::Sampler,
        depth_view: vk::ImageView,
    ) {
        let src = volume.src();
        let volume_info = vk::DescriptorImageInfo::default()
            .sampler(volume.field[src].sampler)
            .image_view(volume.field[src].image_view)
            .image_layout(vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL);

        let depth_info = vk::DescriptorImageInfo::default()
            .sampler(depth_sampler)
            .image_view(depth_view)
            .image_layout(vk::ImageLayout::DEPTH_STENCIL_READ_ONLY_OPTIMAL);

        let writes = [
            vk::WriteDescriptorSet::default()
                .dst_set(set)
                .dst_binding(0)
                .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                .image_info(std::slice::from_ref(&volume_info)),
            vk::WriteDescriptorSet::default()
                .dst_set(set)
                .dst_binding(1)
                .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                .image_info(std::slice::from_ref(&depth_info)),
        ];

        unsafe {
            device.device.update_descriptor_sets(&writes, &[]);
        }
    }

    // --- Resource creation helpers ---

    fn create_depth_sampler(device: &ManagedDevice) -> EngineResult<vk::Sampler> {
        let sampler_info = vk::SamplerCreateInfo::default()
            .mag_filter(vk::Filter::NEAREST)
            .min_filter(vk::Filter::NEAREST)
            .address_mode_u(vk::SamplerAddressMode::CLAMP_TO_EDGE)
            .address_mode_v(vk::SamplerAddressMode::CLAMP_TO_EDGE)
            .address_mode_w(vk::SamplerAddressMode::CLAMP_TO_EDGE);

        unsafe { device.device.create_sampler(&sampler_info, None) }
            .map_err(|e| EngineError::Pipeline(format!("fire depth sampler: {:?}", e)))
    }

    fn create_graphics_descriptor_layout(
        device: &ManagedDevice,
    ) -> EngineResult<vk::DescriptorSetLayout> {
        let bindings = [
            // Binding 0: 3D volume texture (fire field data)
            vk::DescriptorSetLayoutBinding::default()
                .binding(0)
                .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                .descriptor_count(1)
                .stage_flags(vk::ShaderStageFlags::FRAGMENT),
            // Binding 1: Depth buffer (occlusion testing)
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
        .map_err(|e| EngineError::Pipeline(format!("fire render descriptor layout: {:?}", e)))
    }

    fn create_graphics_pipeline_layout(
        device: &ManagedDevice,
        descriptor_set_layout: vk::DescriptorSetLayout,
    ) -> EngineResult<vk::PipelineLayout> {
        // Push constants: view(64) + proj(64) + volume_to_world(64) + camera_pos(16) = 208 bytes
        let push_constant_range = vk::PushConstantRange {
            stage_flags: vk::ShaderStageFlags::VERTEX | vk::ShaderStageFlags::FRAGMENT,
            offset: 0,
            size: std::mem::size_of::<FireRenderPushConstants>() as u32,
        };

        let set_layouts = [descriptor_set_layout];
        let create_info = vk::PipelineLayoutCreateInfo::default()
            .set_layouts(&set_layouts)
            .push_constant_ranges(std::slice::from_ref(&push_constant_range));

        unsafe { device.device.create_pipeline_layout(&create_info, None) }
            .map_err(|e| EngineError::Pipeline(format!("fire render layout: {:?}", e)))
    }

    fn create_graphics_pipeline(
        device: &ManagedDevice,
        render_pass: vk::RenderPass,
        layout: vk::PipelineLayout,
    ) -> EngineResult<vk::Pipeline> {
        let vert_module = ShaderManager::load_compute(device, bytecode::FIRE_VERTEX)?;
        let frag_module = ShaderManager::load_compute(device, bytecode::FIRE_FRAGMENT)?;

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

        // Simple vec3 position input for cube vertices
        let binding_description = vk::VertexInputBindingDescription {
            binding: 0,
            stride: std::mem::size_of::<[f32; 3]>() as u32,
            input_rate: vk::VertexInputRate::VERTEX,
        };

        let attribute_descriptions = [vk::VertexInputAttributeDescription {
            location: 0,
            binding: 0,
            format: vk::Format::R32G32B32_SFLOAT,
            offset: 0,
        }];

        let vertex_input_state = vk::PipelineVertexInputStateCreateInfo::default()
            .vertex_binding_descriptions(std::slice::from_ref(&binding_description))
            .vertex_attribute_descriptions(&attribute_descriptions);

        let input_assembly = vk::PipelineInputAssemblyStateCreateInfo::default()
            .topology(vk::PrimitiveTopology::TRIANGLE_LIST);

        let viewport_state = vk::PipelineViewportStateCreateInfo::default()
            .viewport_count(1)
            .scissor_count(1);

        // No backface culling — camera can be inside the volume
        let rasterization = vk::PipelineRasterizationStateCreateInfo::default()
            .polygon_mode(vk::PolygonMode::FILL)
            .cull_mode(vk::CullModeFlags::NONE)
            .front_face(vk::FrontFace::COUNTER_CLOCKWISE)
            .line_width(1.0);

        let multisample = vk::PipelineMultisampleStateCreateInfo::default()
            .rasterization_samples(vk::SampleCountFlags::TYPE_1);

        // Depth test enabled (fire occluded by closer geometry), depth write disabled
        let depth_stencil = vk::PipelineDepthStencilStateCreateInfo::default()
            .depth_test_enable(true)
            .depth_write_enable(false)
            .depth_compare_op(vk::CompareOp::LESS_OR_EQUAL);

        // Alpha blending: premultiplied alpha for emissive fire + absorptive smoke
        let color_blend_attachment = vk::PipelineColorBlendAttachmentState {
            blend_enable: vk::TRUE,
            src_color_blend_factor: vk::BlendFactor::ONE,
            dst_color_blend_factor: vk::BlendFactor::ONE_MINUS_SRC_ALPHA,
            color_blend_op: vk::BlendOp::ADD,
            src_alpha_blend_factor: vk::BlendFactor::ONE,
            dst_alpha_blend_factor: vk::BlendFactor::ONE_MINUS_SRC_ALPHA,
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
                .map_err(|(_, e)| {
                    EngineError::Pipeline(format!("fire render creation: {:?}", e))
                })?;

            device.device.destroy_shader_module(vert_module, None);
            device.device.destroy_shader_module(frag_module, None);

            pipelines[0]
        };

        Ok(pipeline)
    }

    fn create_graphics_descriptor_pool(device: &ManagedDevice) -> EngineResult<vk::DescriptorPool> {
        // 2 combined image samplers per fire volume (volume + depth)
        let pool_size = vk::DescriptorPoolSize {
            ty: vk::DescriptorType::COMBINED_IMAGE_SAMPLER,
            descriptor_count: 2 * MAX_FIRE_VOLUMES as u32,
        };

        let create_info = vk::DescriptorPoolCreateInfo::default()
            .flags(vk::DescriptorPoolCreateFlags::FREE_DESCRIPTOR_SET)
            .max_sets(MAX_FIRE_VOLUMES as u32)
            .pool_sizes(std::slice::from_ref(&pool_size));

        unsafe { device.device.create_descriptor_pool(&create_info, None) }
            .map_err(|e| EngineError::Pipeline(format!("fire render descriptor pool: {:?}", e)))
    }

    /// Create vertex and index buffers for a unit cube [0,1]^3.
    fn create_cube_buffers(
        device: &Arc<ManagedDevice>,
    ) -> EngineResult<(ManagedBuffer, ManagedBuffer)> {
        #[rustfmt::skip]
        let vertices: [f32; 24] = [
            0.0, 0.0, 0.0,  // 0: near-bottom-left
            1.0, 0.0, 0.0,  // 1: near-bottom-right
            1.0, 1.0, 0.0,  // 2: near-top-right
            0.0, 1.0, 0.0,  // 3: near-top-left
            0.0, 0.0, 1.0,  // 4: far-bottom-left
            1.0, 0.0, 1.0,  // 5: far-bottom-right
            1.0, 1.0, 1.0,  // 6: far-top-right
            0.0, 1.0, 1.0,  // 7: far-top-left
        ];

        #[rustfmt::skip]
        let indices: [u16; 36] = [
            // Front
            0, 1, 2,  2, 3, 0,
            // Back
            5, 4, 7,  7, 6, 5,
            // Left
            4, 0, 3,  3, 7, 4,
            // Right
            1, 5, 6,  6, 2, 1,
            // Top
            3, 2, 6,  6, 7, 3,
            // Bottom
            4, 5, 1,  1, 0, 4,
        ];

        let vertex_size = std::mem::size_of_val(&vertices) as vk::DeviceSize;
        let index_size = std::mem::size_of_val(&indices) as vk::DeviceSize;

        let vertex_buffer = ManagedBuffer::new(
            device.clone(),
            vertex_size,
            vk::BufferUsageFlags::VERTEX_BUFFER,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )?;

        let index_buffer = ManagedBuffer::new(
            device.clone(),
            index_size,
            vk::BufferUsageFlags::INDEX_BUFFER,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )?;

        // Upload cube geometry
        unsafe {
            let ptr = vertex_buffer.map_memory(0, vk::MemoryMapFlags::empty())?;
            std::ptr::copy_nonoverlapping(
                vertices.as_ptr() as *const u8,
                ptr as *mut u8,
                vertex_size as usize,
            );
            vertex_buffer.unmap_memory();

            let ptr = index_buffer.map_memory(0, vk::MemoryMapFlags::empty())?;
            std::ptr::copy_nonoverlapping(
                indices.as_ptr() as *const u8,
                ptr as *mut u8,
                index_size as usize,
            );
            index_buffer.unmap_memory();
        }

        Ok((vertex_buffer, index_buffer))
    }
}

impl Drop for FireRenderer {
    fn drop(&mut self) {
        unsafe {
            self.device
                .device
                .destroy_pipeline(self.graphics_pipeline, None);
            self.device
                .device
                .destroy_pipeline_layout(self.graphics_pipeline_layout, None);
            self.device
                .device
                .destroy_descriptor_pool(self.graphics_descriptor_pool, None);
            self.device
                .device
                .destroy_descriptor_set_layout(self.graphics_descriptor_set_layout, None);
            self.device.device.destroy_sampler(self.depth_sampler, None);
        }
    }
}

// --- Free functions for layout transitions and descriptor updates ---
// These are free functions (not methods) so they can be called while
// `sim_slots` is borrowed mutably in `simulate()`.

fn transition_volumes_for_compute(
    device: &ash::Device,
    cb: vk::CommandBuffer,
    volume: &FireVolume,
) {
    let images: Vec<vk::Image> = volume
        .field
        .iter()
        .chain(volume.velocity.iter())
        .chain(volume.pressure.iter())
        .map(|tex| tex.image)
        .collect();

    let barriers: Vec<vk::ImageMemoryBarrier> = images
        .iter()
        .map(|&image| {
            vk::ImageMemoryBarrier::default()
                .image(image)
                .old_layout(vk::ImageLayout::UNDEFINED)
                .new_layout(vk::ImageLayout::GENERAL)
                .src_access_mask(vk::AccessFlags::empty())
                .dst_access_mask(vk::AccessFlags::SHADER_READ | vk::AccessFlags::SHADER_WRITE)
                .subresource_range(vk::ImageSubresourceRange {
                    aspect_mask: vk::ImageAspectFlags::COLOR,
                    base_mip_level: 0,
                    level_count: 1,
                    base_array_layer: 0,
                    layer_count: 1,
                })
        })
        .collect();

    unsafe {
        device.cmd_pipeline_barrier(
            cb,
            vk::PipelineStageFlags::TOP_OF_PIPE,
            vk::PipelineStageFlags::COMPUTE_SHADER,
            vk::DependencyFlags::empty(),
            &[],
            &[],
            &barriers,
        );
    }
}

fn transition_field_for_read(device: &ash::Device, cb: vk::CommandBuffer, volume: &FireVolume) {
    let src = volume.src();
    let barrier = vk::ImageMemoryBarrier::default()
        .image(volume.field[src].image)
        .old_layout(vk::ImageLayout::GENERAL)
        .new_layout(vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL)
        .src_access_mask(vk::AccessFlags::SHADER_WRITE)
        .dst_access_mask(vk::AccessFlags::SHADER_READ)
        .subresource_range(vk::ImageSubresourceRange {
            aspect_mask: vk::ImageAspectFlags::COLOR,
            base_mip_level: 0,
            level_count: 1,
            base_array_layer: 0,
            layer_count: 1,
        });

    unsafe {
        device.cmd_pipeline_barrier(
            cb,
            vk::PipelineStageFlags::COMPUTE_SHADER,
            vk::PipelineStageFlags::FRAGMENT_SHADER,
            vk::DependencyFlags::empty(),
            &[],
            &[],
            &[barrier],
        );
    }
}

fn update_render_volume_binding(device: &ash::Device, set: vk::DescriptorSet, volume: &FireVolume) {
    let src = volume.src();
    let volume_info = vk::DescriptorImageInfo::default()
        .sampler(volume.field[src].sampler)
        .image_view(volume.field[src].image_view)
        .image_layout(vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL);

    let write = vk::WriteDescriptorSet::default()
        .dst_set(set)
        .dst_binding(0)
        .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
        .image_info(std::slice::from_ref(&volume_info));

    unsafe {
        device.update_descriptor_sets(std::slice::from_ref(&write), &[]);
    }
}

/// Convert a nalgebra Matrix4 to a column-major f32 array for push constants.
fn matrix4_to_array(m: &Matrix4<f32>) -> [[f32; 4]; 4] {
    let s = m.as_slice();
    [
        [s[0], s[1], s[2], s[3]],
        [s[4], s[5], s[6], s[7]],
        [s[8], s[9], s[10], s[11]],
        [s[12], s[13], s[14], s[15]],
    ]
}
