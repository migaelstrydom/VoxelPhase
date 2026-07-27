//! Centralized descriptor set management.
//!
//! This module handles descriptor pool creation, descriptor set allocation,
//! and updates for UBOs and texture samplers. All descriptor logic is consolidated here.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineResult, VkResultExt};
use crate::rendering::frame::ManagedBuffer;

// Constants for texture pool management
const INITIAL_TEXTURE_POOL_SIZE: u32 = 100;
const POOL_GROWTH_FACTOR: f32 = 1.5;

/// Internal tracking for a single descriptor pool
struct DescriptorPool {
    pool: vk::DescriptorPool,
    capacity: u32,
    allocated: u32,
}

/// Internal mutable state for texture descriptor management
struct TextureDescriptorState {
    pools: Vec<DescriptorPool>,
    /// Maps each allocated descriptor set back to its pool index.
    set_to_pool: HashMap<vk::DescriptorSet, usize>,
    texture_layout: vk::DescriptorSetLayout,
}

/// Manages all descriptor pools and sets for the renderer.
///
/// This centralizes:
/// - Scene UBO descriptor set (single, pre-allocated)
/// - Texture descriptor sets (dynamically allocated with pool growth)
pub struct DescriptorManager {
    pub scene_ubo_set: vk::DescriptorSet,
    ubo_pool: vk::DescriptorPool,
    texture_state: Mutex<TextureDescriptorState>,
    device: Arc<ManagedDevice>,
}

impl DescriptorManager {
    /// Create a descriptor manager with separate pools for UBOs and textures.
    pub fn new(
        device: Arc<ManagedDevice>,
        ubo_layout: vk::DescriptorSetLayout,
        texture_layout: vk::DescriptorSetLayout,
        initial_texture_capacity: u32,
    ) -> EngineResult<Self> {
        // Create UBO pool. One descriptor set holding two uniform buffer
        // descriptors: the scene block (binding 0) and the light set (binding 1).
        let ubo_pool_sizes = [vk::DescriptorPoolSize::default()
            .ty(vk::DescriptorType::UNIFORM_BUFFER)
            .descriptor_count(2)];

        let ubo_pool_info = vk::DescriptorPoolCreateInfo::default()
            .max_sets(1)
            .pool_sizes(&ubo_pool_sizes);

        let ubo_pool = unsafe {
            device
                .device
                .create_descriptor_pool(&ubo_pool_info, None)
                .descriptor_context("create UBO descriptor pool")?
        };

        // Allocate the scene UBO descriptor set
        let alloc_info = vk::DescriptorSetAllocateInfo::default()
            .descriptor_pool(ubo_pool)
            .set_layouts(std::slice::from_ref(&ubo_layout));

        let sets = unsafe {
            device
                .device
                .allocate_descriptor_sets(&alloc_info)
                .descriptor_context("allocate UBO descriptor set")?
        };

        // Create initial texture pool
        let initial_capacity = initial_texture_capacity.max(INITIAL_TEXTURE_POOL_SIZE);
        let texture_pool = Self::create_texture_pool(&device, initial_capacity)?;

        Ok(Self {
            scene_ubo_set: sets[0],
            ubo_pool,
            texture_state: Mutex::new(TextureDescriptorState {
                pools: vec![DescriptorPool {
                    pool: texture_pool,
                    capacity: initial_capacity,
                    allocated: 0,
                }],
                set_to_pool: HashMap::new(),
                texture_layout,
            }),
            device,
        })
    }

    /// Create a new texture descriptor pool.
    fn create_texture_pool(
        device: &ManagedDevice,
        capacity: u32,
    ) -> EngineResult<vk::DescriptorPool> {
        let pool_sizes = [vk::DescriptorPoolSize::default()
            .ty(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
            .descriptor_count(capacity)];

        let pool_info = vk::DescriptorPoolCreateInfo::default()
            .flags(vk::DescriptorPoolCreateFlags::FREE_DESCRIPTOR_SET)
            .max_sets(capacity)
            .pool_sizes(&pool_sizes);

        unsafe {
            device
                .device
                .create_descriptor_pool(&pool_info, None)
                .descriptor_context("create texture descriptor pool")
        }
    }

    /// Update the scene UBO descriptor (set 0, binding 0) to point at a buffer.
    pub fn update_scene_ubo(&self, buffer: &ManagedBuffer, size: vk::DeviceSize) {
        self.write_ubo_binding(0, buffer, size);
    }

    /// Update the light UBO descriptor (set 0, binding 1) to point at a buffer.
    pub fn update_light_ubo(&self, buffer: &ManagedBuffer, size: vk::DeviceSize) {
        self.write_ubo_binding(1, buffer, size);
    }

    /// Point one binding of the scene descriptor set at a uniform buffer.
    ///
    /// Descriptor writes only rebind the buffer; per-frame *contents* are
    /// written through the mapped buffer, so this runs once at startup.
    fn write_ubo_binding(&self, binding: u32, buffer: &ManagedBuffer, size: vk::DeviceSize) {
        let buffer_info = [vk::DescriptorBufferInfo::default()
            .buffer(buffer.buffer)
            .offset(0)
            .range(size)];

        let write = [vk::WriteDescriptorSet::default()
            .dst_set(self.scene_ubo_set)
            .dst_binding(binding)
            .descriptor_type(vk::DescriptorType::UNIFORM_BUFFER)
            .buffer_info(&buffer_info)];

        unsafe {
            self.device.device.update_descriptor_sets(&write, &[]);
        }
    }

    /// Allocate a texture descriptor set, growing pools if needed.
    pub fn allocate_texture_set(&self) -> EngineResult<vk::DescriptorSet> {
        let mut state = self.texture_state.lock().unwrap();

        // Find a pool with available space
        let pool_index = {
            let available_pool = state
                .pools
                .iter()
                .enumerate()
                .find(|(_, pool)| pool.allocated < pool.capacity);

            match available_pool {
                Some((idx, _)) => idx,
                None => {
                    // All pools are full, create a new one
                    let new_capacity = (state
                        .pools
                        .last()
                        .map(|p| p.capacity)
                        .unwrap_or(INITIAL_TEXTURE_POOL_SIZE)
                        as f32
                        * POOL_GROWTH_FACTOR) as u32;

                    log::info!(
                        "Creating new texture descriptor pool with capacity {}",
                        new_capacity
                    );

                    let new_pool = Self::create_texture_pool(&self.device, new_capacity)?;
                    state.pools.push(DescriptorPool {
                        pool: new_pool,
                        capacity: new_capacity,
                        allocated: 0,
                    });

                    state.pools.len() - 1
                }
            }
        };

        // Allocate from the selected pool
        let (pool_handle, texture_layout) = {
            let pool = &state.pools[pool_index];
            (pool.pool, state.texture_layout)
        };

        let alloc_info = vk::DescriptorSetAllocateInfo::default()
            .descriptor_pool(pool_handle)
            .set_layouts(std::slice::from_ref(&texture_layout));

        let sets = unsafe {
            self.device
                .device
                .allocate_descriptor_sets(&alloc_info)
                .descriptor_context("allocate texture descriptor set")?
        };

        // Update allocation count and record ownership
        let set = sets[0];
        state.set_to_pool.insert(set, pool_index);
        state.pools[pool_index].allocated += 1;

        let pool = &state.pools[pool_index];
        log::debug!(
            "Allocated texture descriptor set from pool {} ({}/{})",
            pool_index,
            pool.allocated,
            pool.capacity
        );

        Ok(set)
    }

    /// Free a texture descriptor set back to the pool that allocated it.
    pub fn free_texture_set(&self, set: vk::DescriptorSet) -> EngineResult<()> {
        let mut state = self.texture_state.lock().unwrap();

        if let Some(pool_index) = state.set_to_pool.remove(&set) {
            let pool = &mut state.pools[pool_index];
            unsafe {
                let _ = self.device.device.free_descriptor_sets(pool.pool, &[set]);
            }
            pool.allocated = pool.allocated.saturating_sub(1);
        } else {
            log::warn!("Attempted to free untracked descriptor set {:?}", set);
        }

        Ok(())
    }

    /// Update a texture descriptor set with an image view and sampler.
    pub fn update_texture_set(
        &self,
        set: vk::DescriptorSet,
        image_view: vk::ImageView,
        sampler: vk::Sampler,
    ) {
        let image_info = [vk::DescriptorImageInfo::default()
            .image_layout(vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL)
            .image_view(image_view)
            .sampler(sampler)];

        let write = [vk::WriteDescriptorSet::default()
            .dst_set(set)
            .dst_binding(0)
            .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
            .image_info(&image_info)];

        unsafe {
            self.device.device.update_descriptor_sets(&write, &[]);
        }
    }
}

impl Drop for DescriptorManager {
    fn drop(&mut self) {
        unsafe {
            // Destroy UBO pool
            if self.ubo_pool != vk::DescriptorPool::null() {
                self.device
                    .device
                    .destroy_descriptor_pool(self.ubo_pool, None);
            }

            // Destroy all texture pools
            let state = self.texture_state.lock().unwrap();
            for pool in &state.pools {
                self.device.device.destroy_descriptor_pool(pool.pool, None);
            }
        }
    }
}
