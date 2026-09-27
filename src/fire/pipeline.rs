//! Compute pipelines for fire fluid simulation.
//!
//! Wraps the four simulation passes (advection, forces, pressure projection,
//! velocity correction) and provides a single `simulate()` method that records
//! all compute dispatches into a command buffer.

use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult};
use crate::rendering::compute::{ComputePipeline, ComputePipelineConfig};
use crate::rendering::shaders::ShaderManager;

use super::volume::FireVolume;

/// Embedded compute shader bytecode.
mod bytecode {
    use crate::embedded_spirv;

    pub const ADVECT: &[u8] = embedded_spirv!("fire/advect.comp");
    pub const FORCES: &[u8] = embedded_spirv!("fire/forces.comp");
    pub const PRESSURE: &[u8] = embedded_spirv!("fire/pressure.comp");
    pub const CORRECT: &[u8] = embedded_spirv!("fire/correct.comp");
}

/// Number of Jacobi iterations for pressure projection.
const JACOBI_ITERATIONS: u32 = 25;

/// Maximum number of simultaneously active fire volumes.
const MAX_FIRE_VOLUMES: u32 = 32;

/// Push constants shared by all fire simulation shaders.
///
/// These control the look and feel of the volumetric fire. The simulation runs
/// per-voxel each frame: source injection feeds fuel+heat at the base, combustion
/// converts fuel→heat+smoke, buoyancy carries hot gas upward, turbulence adds
/// chaotic flicker, and cooling dissipates temperature over time.
///
/// # Tuning guide
///
/// - **Taller flames**: increase `buoyancy_strength`, decrease `cooling_rate`
/// - **More flickery**: increase `turbulence_amplitude` and `turbulence_frequency`
/// - **Brighter core**: increase `combustion_rate`
/// - **Longer-lasting voxel fuel**: decrease `burn_rate`
/// - **More/less smoke**: adjust `smoke_production`
/// - **Sharper flame shape**: increase `cooling_rate` (fire dies faster at edges)
#[repr(C)]
#[derive(Clone, Copy)]
pub struct FireSimParams {
    /// Simulation time step (seconds). Typically matches frame delta.
    pub dt: f32,
    /// Upward velocity added per unit of temperature per second.
    /// Higher = taller, faster-rising flames. Range: 2–15.
    pub buoyancy_strength: f32,
    /// Temperature lost per second (multiplied by edge proximity).
    /// Higher = fire dies faster, sharper flame edges. Range: 0.3–3.0.
    pub cooling_rate: f32,
    /// Temperature gained per unit of fuel burned.
    /// Higher = brighter, hotter core. Range: 2–10.
    pub combustion_rate: f32,
    /// Fraction of voxel fuel consumed per second (when above ignition threshold).
    /// Lower = each voxel burns longer, flame has time to rise. Range: 0.02–0.5.
    pub burn_rate: f32,
    /// Curl noise velocity magnitude scaled by temperature.
    /// Higher = more chaotic, flickery flames. Range: 0.5–5.0.
    pub turbulence_amplitude: f32,
    /// Curl noise spatial frequency (higher = finer detail).
    /// Range: 1–8.
    pub turbulence_frequency: f32,
    /// Smoke density produced per unit of fuel burned.
    /// Higher = thicker, darker smoke. Range: 0.1–1.0.
    pub smoke_production: f32,
    /// Total elapsed time (seconds). Drives animated noise for flickering.
    pub time: f32,
    /// Current Jacobi pressure solver iteration (internal, not for tuning).
    pub jacobi_iteration: i32,
    /// Per-fire source injection multiplier (0.0 = dying out, 1.0 = full burn).
    /// Set automatically from fuel_remaining / initial_fuel.
    pub source_intensity: f32,
    /// Per-fire random seed that offsets noise sampling so simultaneous fires
    /// don't evolve identically. Set automatically at fire creation.
    pub noise_seed: f32,
}

impl Default for FireSimParams {
    fn default() -> Self {
        Self {
            dt: 1.0 / 60.0,
            buoyancy_strength: 8.0,
            cooling_rate: 0.6,
            combustion_rate: 6.0,
            burn_rate: 0.05,
            turbulence_amplitude: 6.0,
            turbulence_frequency: 6.0,
            smoke_production: 2.5,
            time: 0.0,
            jacobi_iteration: 0,
            source_intensity: 1.0,
            noise_seed: 0.0,
        }
    }
}

/// Descriptor sets for a single fire volume across all simulation passes.
pub struct FireVolumeDescriptors {
    /// Advection pass: [field_src, field_dst, vel_src, vel_dst]
    pub advect_sets: [vk::DescriptorSet; 2],
    /// Forces pass: [field, velocity]
    pub forces_set: vk::DescriptorSet,
    /// Pressure pass: [velocity, pressure_src, pressure_dst] — no dedicated set,
    /// updated each iteration via the two ping-pong sets.
    pub pressure_sets: [vk::DescriptorSet; 2],
    /// Correction pass: [velocity, pressure]
    pub correct_set: vk::DescriptorSet,
}

/// The four compute pipelines for fire fluid simulation.
pub struct FireSimPipelines {
    advect: ComputePipeline,
    forces: ComputePipeline,
    pressure: ComputePipeline,
    correct: ComputePipeline,
    /// Descriptor set layouts (one per pass). Owned by this struct.
    advect_layout: vk::DescriptorSetLayout,
    forces_layout: vk::DescriptorSetLayout,
    pressure_layout: vk::DescriptorSetLayout,
    correct_layout: vk::DescriptorSetLayout,
    descriptor_pool: vk::DescriptorPool,
    device: Arc<ManagedDevice>,
}

impl FireSimPipelines {
    pub fn new(device: Arc<ManagedDevice>) -> EngineResult<Self> {
        let push_constant_range = vk::PushConstantRange {
            stage_flags: vk::ShaderStageFlags::COMPUTE,
            offset: 0,
            size: std::mem::size_of::<FireSimParams>() as u32,
        };

        let advect_layout = Self::create_storage_image_layout(&device, 4)?;
        let forces_layout = Self::create_storage_image_layout(&device, 2)?;
        let pressure_layout = Self::create_storage_image_layout(&device, 3)?;
        let correct_layout = Self::create_storage_image_layout(&device, 2)?;

        let advect = Self::create_pipeline(
            &device,
            bytecode::ADVECT,
            advect_layout,
            &push_constant_range,
        )?;
        let forces = Self::create_pipeline(
            &device,
            bytecode::FORCES,
            forces_layout,
            &push_constant_range,
        )?;
        let pressure = Self::create_pipeline(
            &device,
            bytecode::PRESSURE,
            pressure_layout,
            &push_constant_range,
        )?;
        let correct = Self::create_pipeline(
            &device,
            bytecode::CORRECT,
            correct_layout,
            &push_constant_range,
        )?;

        let descriptor_pool = Self::create_descriptor_pool(&device)?;

        Ok(Self {
            advect,
            forces,
            pressure,
            correct,
            advect_layout,
            forces_layout,
            pressure_layout,
            correct_layout,
            descriptor_pool,
            device,
        })
    }

    pub fn descriptor_pool(&self) -> vk::DescriptorPool {
        self.descriptor_pool
    }

    /// Allocate descriptor sets for a fire volume.
    pub fn allocate_volume_descriptors(&self) -> EngineResult<FireVolumeDescriptors> {
        let layouts = [
            self.advect_layout,   // advect set A (src=0, dst=1)
            self.advect_layout,   // advect set B (src=1, dst=0)
            self.forces_layout,   // forces
            self.pressure_layout, // pressure ping A
            self.pressure_layout, // pressure ping B
            self.correct_layout,  // correct
        ];

        let alloc_info = vk::DescriptorSetAllocateInfo::default()
            .descriptor_pool(self.descriptor_pool)
            .set_layouts(&layouts);

        let sets = unsafe {
            self.device
                .device
                .allocate_descriptor_sets(&alloc_info)
                .map_err(|e| {
                    EngineError::Descriptor(format!("fire volume descriptor allocation: {:?}", e))
                })?
        };

        Ok(FireVolumeDescriptors {
            advect_sets: [sets[0], sets[1]],
            forces_set: sets[2],
            pressure_sets: [sets[3], sets[4]],
            correct_set: sets[5],
        })
    }

    /// Update all descriptor sets for a fire volume to point at its current textures.
    pub fn update_volume_descriptors(
        &self,
        volume: &FireVolume,
        descriptors: &FireVolumeDescriptors,
    ) {
        // Advect set A: reads from [0], writes to [1]
        self.write_advect_descriptors(descriptors.advect_sets[0], volume, 0, 1);
        // Advect set B: reads from [1], writes to [0]
        self.write_advect_descriptors(descriptors.advect_sets[1], volume, 1, 0);

        // Forces: reads+writes current source
        self.write_storage_images(
            descriptors.forces_set,
            &[volume.field[0].image_view, volume.velocity[0].image_view],
        );

        // Pressure ping A: reads vel[0], pressure[0] → pressure[1]
        self.write_storage_images(
            descriptors.pressure_sets[0],
            &[
                volume.velocity[0].image_view,
                volume.pressure[0].image_view,
                volume.pressure[1].image_view,
            ],
        );
        // Pressure ping B: reads vel[0], pressure[1] → pressure[0]
        self.write_storage_images(
            descriptors.pressure_sets[1],
            &[
                volume.velocity[0].image_view,
                volume.pressure[1].image_view,
                volume.pressure[0].image_view,
            ],
        );

        // Correct: reads vel[0], pressure[0]
        self.write_storage_images(
            descriptors.correct_set,
            &[volume.velocity[0].image_view, volume.pressure[0].image_view],
        );
    }

    /// Record all simulation dispatches for a fire volume into the command buffer.
    ///
    /// Executes: advection → forces → pressure projection (N iterations) → velocity correction.
    /// Inserts pipeline barriers between passes.
    pub fn simulate(
        &self,
        cb: vk::CommandBuffer,
        volume: &mut FireVolume,
        descriptors: &FireVolumeDescriptors,
        params: &FireSimParams,
    ) {
        let (gx, gy, gz) = volume.dispatch_size();
        let device = &self.device.device;

        unsafe {
            // Pass 1: Advection (reads src, writes dst)
            let advect_set = descriptors.advect_sets[volume.src()];
            self.dispatch(cb, device, &self.advect, advect_set, params, gx, gy, gz);
            volume.swap();
            Self::barrier_compute_to_compute(cb, device);

            // Pass 2: Forces (reads+writes field/velocity in place)
            self.dispatch(
                cb,
                device,
                &self.forces,
                descriptors.forces_set,
                params,
                gx,
                gy,
                gz,
            );
            Self::barrier_compute_to_compute(cb, device);

            // Pass 3: Pressure projection (Jacobi iterations with ping-pong)
            for i in 0..JACOBI_ITERATIONS {
                let mut p = *params;
                p.jacobi_iteration = i as i32;
                let pressure_set = descriptors.pressure_sets[(i % 2) as usize];
                self.dispatch(cb, device, &self.pressure, pressure_set, &p, gx, gy, gz);
                if i < JACOBI_ITERATIONS - 1 {
                    Self::barrier_compute_to_compute(cb, device);
                }
            }
            Self::barrier_compute_to_compute(cb, device);

            // Pass 4: Velocity correction
            self.dispatch(
                cb,
                device,
                &self.correct,
                descriptors.correct_set,
                params,
                gx,
                gy,
                gz,
            );
        }
    }

    /// Dispatch a single compute pass.
    unsafe fn dispatch(
        &self,
        cb: vk::CommandBuffer,
        device: &ash::Device,
        pipeline: &ComputePipeline,
        descriptor_set: vk::DescriptorSet,
        params: &FireSimParams,
        gx: u32,
        gy: u32,
        gz: u32,
    ) {
        device.cmd_bind_pipeline(cb, vk::PipelineBindPoint::COMPUTE, pipeline.pipeline);
        device.cmd_bind_descriptor_sets(
            cb,
            vk::PipelineBindPoint::COMPUTE,
            pipeline.layout,
            0,
            &[descriptor_set],
            &[],
        );
        device.cmd_push_constants(
            cb,
            pipeline.layout,
            vk::ShaderStageFlags::COMPUTE,
            0,
            std::slice::from_raw_parts(
                params as *const FireSimParams as *const u8,
                std::mem::size_of::<FireSimParams>(),
            ),
        );
        device.cmd_dispatch(cb, gx, gy, gz);
    }

    /// Insert a compute-to-compute pipeline barrier for read-after-write sync.
    unsafe fn barrier_compute_to_compute(cb: vk::CommandBuffer, device: &ash::Device) {
        let barrier = vk::MemoryBarrier::default()
            .src_access_mask(vk::AccessFlags::SHADER_WRITE)
            .dst_access_mask(vk::AccessFlags::SHADER_READ);

        device.cmd_pipeline_barrier(
            cb,
            vk::PipelineStageFlags::COMPUTE_SHADER,
            vk::PipelineStageFlags::COMPUTE_SHADER,
            vk::DependencyFlags::empty(),
            &[barrier],
            &[],
            &[],
        );
    }

    // --- Descriptor write helpers ---

    fn write_advect_descriptors(
        &self,
        set: vk::DescriptorSet,
        volume: &FireVolume,
        src: usize,
        dst: usize,
    ) {
        self.write_storage_images(
            set,
            &[
                volume.field[src].image_view,
                volume.field[dst].image_view,
                volume.velocity[src].image_view,
                volume.velocity[dst].image_view,
            ],
        );
    }

    fn write_storage_images(&self, set: vk::DescriptorSet, image_views: &[vk::ImageView]) {
        let image_infos: Vec<vk::DescriptorImageInfo> = image_views
            .iter()
            .map(|&view| {
                vk::DescriptorImageInfo::default()
                    .image_view(view)
                    .image_layout(vk::ImageLayout::GENERAL)
            })
            .collect();

        let writes: Vec<vk::WriteDescriptorSet> = image_infos
            .iter()
            .enumerate()
            .map(|(i, info)| {
                vk::WriteDescriptorSet::default()
                    .dst_set(set)
                    .dst_binding(i as u32)
                    .descriptor_type(vk::DescriptorType::STORAGE_IMAGE)
                    .image_info(std::slice::from_ref(info))
            })
            .collect();

        unsafe {
            self.device.device.update_descriptor_sets(&writes, &[]);
        }
    }

    // --- Pipeline/layout creation helpers ---

    fn create_storage_image_layout(
        device: &ManagedDevice,
        binding_count: u32,
    ) -> EngineResult<vk::DescriptorSetLayout> {
        let bindings: Vec<vk::DescriptorSetLayoutBinding> = (0..binding_count)
            .map(|i| {
                vk::DescriptorSetLayoutBinding::default()
                    .binding(i)
                    .descriptor_type(vk::DescriptorType::STORAGE_IMAGE)
                    .descriptor_count(1)
                    .stage_flags(vk::ShaderStageFlags::COMPUTE)
            })
            .collect();

        let create_info = vk::DescriptorSetLayoutCreateInfo::default().bindings(&bindings);

        unsafe {
            device
                .device
                .create_descriptor_set_layout(&create_info, None)
                .map_err(|e| {
                    EngineError::Descriptor(format!("fire compute layout creation: {:?}", e))
                })
        }
    }

    fn create_pipeline(
        device: &Arc<ManagedDevice>,
        shader_bytecode: &[u8],
        descriptor_set_layout: vk::DescriptorSetLayout,
        push_constant_range: &vk::PushConstantRange,
    ) -> EngineResult<ComputePipeline> {
        let shader_module = ShaderManager::load_compute(device, shader_bytecode)?;

        let result = ComputePipeline::new(
            device.clone(),
            &ComputePipelineConfig {
                shader_module,
                descriptor_set_layouts: &[descriptor_set_layout],
                push_constant_ranges: &[*push_constant_range],
            },
        );

        unsafe {
            device.device.destroy_shader_module(shader_module, None);
        }

        result
    }

    fn create_descriptor_pool(device: &ManagedDevice) -> EngineResult<vk::DescriptorPool> {
        // Per volume: advect(2*4) + forces(2) + pressure(2*3) + correct(2) = 18 storage images
        // 6 descriptor sets per volume
        let max_sets = 6 * MAX_FIRE_VOLUMES;
        let pool_size = vk::DescriptorPoolSize {
            ty: vk::DescriptorType::STORAGE_IMAGE,
            descriptor_count: 18 * MAX_FIRE_VOLUMES,
        };

        let create_info = vk::DescriptorPoolCreateInfo::default()
            .flags(vk::DescriptorPoolCreateFlags::FREE_DESCRIPTOR_SET)
            .max_sets(max_sets)
            .pool_sizes(std::slice::from_ref(&pool_size));

        unsafe {
            device
                .device
                .create_descriptor_pool(&create_info, None)
                .map_err(|e| {
                    EngineError::Descriptor(format!("fire descriptor pool creation: {:?}", e))
                })
        }
    }
}

impl Drop for FireSimPipelines {
    fn drop(&mut self) {
        unsafe {
            self.device
                .device
                .destroy_descriptor_pool(self.descriptor_pool, None);
            self.device
                .device
                .destroy_descriptor_set_layout(self.advect_layout, None);
            self.device
                .device
                .destroy_descriptor_set_layout(self.forces_layout, None);
            self.device
                .device
                .destroy_descriptor_set_layout(self.pressure_layout, None);
            self.device
                .device
                .destroy_descriptor_set_layout(self.correct_layout, None);
        }
    }
}
