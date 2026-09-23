//! Centralized descriptor set management.
//!
//! This module handles descriptor pool creation, descriptor set allocation,
//! and updates for UBOs and texture samplers. All descriptor logic is consolidated here.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineResult, VkResultExt};
use crate::rendering::deletion_queue::DeletionQueue;
use crate::rendering::frame::ManagedBuffer;
use crate::rendering::in_flight::{FrameSlot, PerFrame, FRAMES_IN_FLIGHT};
use crate::rendering::texture::ManagedTexture;

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

/// A texture whose last handle has gone, held until no frame in flight can
/// still be sampling it.
struct RetiredTexture {
    /// Its descriptor set, if one was ever allocated.
    set: Option<vk::DescriptorSet>,
    /// Held only so the image outlives the frames that draw with it.
    _texture: Arc<ManagedTexture>,
}

/// Textures released while frames may still be using them.
struct Retirement {
    queue: DeletionQueue<RetiredTexture>,
    /// Frames begun so far; what retired textures are stamped with.
    frame: u64,
}

/// Manages all descriptor pools and sets for the renderer.
///
/// This centralizes:
/// - Scene descriptor sets (one per frame in flight, pre-allocated)
/// - Texture descriptor sets (dynamically allocated with pool growth)
pub struct DescriptorManager {
    /// Set 0, one per frame in flight. Each points at its own frame's uniform
    /// buffers and surface table; the shadow map and grain texture are shared.
    scene_sets: PerFrame<vk::DescriptorSet>,
    ubo_pool: vk::DescriptorPool,
    texture_state: Mutex<TextureDescriptorState>,
    /// Textures released by their handles, freed once the frames that were in
    /// flight at the time have finished. See [`Self::retire_texture`].
    retired: Mutex<Retirement>,
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
        // Create UBO pool. One descriptor set per frame in flight, each
        // holding two uniform buffer descriptors — the scene block (binding 0)
        // and the light set (binding 1) — plus the sun shadow map (binding 2)
        // and the frame's surface table (binding 3) and the shared grain
        // texture (binding 4).
        let frames = FRAMES_IN_FLIGHT as u32;
        let ubo_pool_sizes = [
            vk::DescriptorPoolSize::default()
                .ty(vk::DescriptorType::UNIFORM_BUFFER)
                .descriptor_count(2 * frames),
            vk::DescriptorPoolSize::default()
                .ty(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                .descriptor_count(2 * frames),
            vk::DescriptorPoolSize::default()
                .ty(vk::DescriptorType::STORAGE_BUFFER)
                .descriptor_count(frames),
        ];

        let ubo_pool_info = vk::DescriptorPoolCreateInfo::default()
            .max_sets(frames)
            .pool_sizes(&ubo_pool_sizes);

        let ubo_pool = unsafe {
            device
                .device
                .create_descriptor_pool(&ubo_pool_info, None)
                .descriptor_context("create UBO descriptor pool")?
        };

        let layouts = [ubo_layout; FRAMES_IN_FLIGHT];
        let alloc_info = vk::DescriptorSetAllocateInfo::default()
            .descriptor_pool(ubo_pool)
            .set_layouts(&layouts);

        let sets = unsafe {
            device
                .device
                .allocate_descriptor_sets(&alloc_info)
                .descriptor_context("allocate UBO descriptor sets")?
        };

        // Create initial texture pool
        let initial_capacity = initial_texture_capacity.max(INITIAL_TEXTURE_POOL_SIZE);
        let texture_pool = Self::create_texture_pool(&device, initial_capacity)?;

        Ok(Self {
            scene_sets: PerFrame::new(|slot| sets[slot.index()]),
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
            retired: Mutex::new(Retirement {
                queue: DeletionQueue::new(FRAMES_IN_FLIGHT as u64),
                frame: 0,
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

    /// The scene descriptor set (set 0) of one frame in flight.
    pub fn scene_set(&self, slot: FrameSlot) -> vk::DescriptorSet {
        self.scene_sets[slot]
    }

    /// Point one frame's scene UBO descriptor (set 0, binding 0) at a buffer.
    pub fn update_scene_ubo(&self, slot: FrameSlot, buffer: &ManagedBuffer, size: vk::DeviceSize) {
        self.write_ubo_binding(self.scene_sets[slot], 0, buffer, size);
    }

    /// Point one frame's light UBO descriptor (set 0, binding 1) at a buffer.
    pub fn update_light_ubo(&self, slot: FrameSlot, buffer: &ManagedBuffer, size: vk::DeviceSize) {
        self.write_ubo_binding(self.scene_sets[slot], 1, buffer, size);
    }

    /// Point one frame's surface table descriptor (set 0, binding 3) at a
    /// buffer.
    ///
    /// Written once at startup. The table is sized for the worst frame and
    /// never reallocated, precisely so this descriptor never has to be
    /// rewritten while a frame is in flight.
    pub fn update_surface_table(
        &self,
        slot: FrameSlot,
        buffer: &ManagedBuffer,
        size: vk::DeviceSize,
    ) {
        let buffer_info = [vk::DescriptorBufferInfo::default()
            .buffer(buffer.buffer)
            .offset(0)
            .range(size)];

        let write = [vk::WriteDescriptorSet::default()
            .dst_set(self.scene_sets[slot])
            .dst_binding(3)
            .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
            .buffer_info(&buffer_info)];

        unsafe {
            self.device.device.update_descriptor_sets(&write, &[]);
        }
    }

    /// Point every frame's grain descriptor (set 0, binding 4) at the shared
    /// grain texture.
    ///
    /// Written once at startup. The fragment shader statically samples this
    /// binding, so it must be valid before any draw, whether or not a material
    /// asks for grain — see `ResourceManager::create_texture_manager`.
    pub fn update_grain_texture(&self, image_view: vk::ImageView, sampler: vk::Sampler) {
        let image_info = [vk::DescriptorImageInfo::default()
            .image_layout(vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL)
            .image_view(image_view)
            .sampler(sampler)];

        let writes: Vec<_> = self
            .scene_sets
            .iter()
            .map(|&set| {
                vk::WriteDescriptorSet::default()
                    .dst_set(set)
                    .dst_binding(4)
                    .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                    .image_info(&image_info)
            })
            .collect();

        unsafe {
            self.device.device.update_descriptor_sets(&writes, &[]);
        }
    }

    /// Point every frame's shadow map descriptor (set 0, binding 2) at a depth
    /// image. There is one map: frames take turns with it, ordered on the GPU
    /// by the shadow pass's own dependencies.
    ///
    /// `layout` is the layout the image is in when it is sampled, which for a
    /// depth attachment is not the colour path's `SHADER_READ_ONLY_OPTIMAL`.
    ///
    /// No sampler is supplied: the binding carries an immutable one, declared
    /// in the descriptor set layout, and a sampler written here would be
    /// ignored.
    pub fn update_shadow_map(&self, image_view: vk::ImageView, layout: vk::ImageLayout) {
        let image_info = [vk::DescriptorImageInfo::default()
            .image_layout(layout)
            .image_view(image_view)];

        let writes: Vec<_> = self
            .scene_sets
            .iter()
            .map(|&set| {
                vk::WriteDescriptorSet::default()
                    .dst_set(set)
                    .dst_binding(2)
                    .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                    .image_info(&image_info)
            })
            .collect();

        unsafe {
            self.device.device.update_descriptor_sets(&writes, &[]);
        }
    }

    /// Point one binding of the scene descriptor set at a uniform buffer.
    ///
    /// Descriptor writes only rebind the buffer; per-frame *contents* are
    /// written through the mapped buffer, so this runs once at startup.
    fn write_ubo_binding(
        &self,
        set: vk::DescriptorSet,
        binding: u32,
        buffer: &ManagedBuffer,
        size: vk::DeviceSize,
    ) {
        let buffer_info = [vk::DescriptorBufferInfo::default()
            .buffer(buffer.buffer)
            .offset(0)
            .range(size)];

        let write = [vk::WriteDescriptorSet::default()
            .dst_set(set)
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

    /// Hand over a texture whose last handle has gone.
    ///
    /// A frame recorded before the release may still be on the GPU, sampling
    /// the image through the set, so neither is freed until every frame begun
    /// by then has finished — see [`Self::begin_frame`].
    pub fn retire_texture(&self, set: Option<vk::DescriptorSet>, texture: Arc<ManagedTexture>) {
        let mut retired = self.retired.lock().unwrap();
        let frame = retired.frame;
        retired.queue.queue(
            RetiredTexture {
                set,
                _texture: texture,
            },
            frame,
        );
    }

    /// Start a frame: free the textures retired `FRAMES_IN_FLIGHT` frames ago.
    ///
    /// Call once per frame, after waiting on that frame's fence — which is
    /// what guarantees the frame that last used them has finished.
    pub fn begin_frame(&self) {
        let ready = {
            let mut retired = self.retired.lock().unwrap();
            retired.frame += 1;
            let frame = retired.frame;
            retired.queue.take_ready(frame)
        };
        for texture in ready {
            if let Some(set) = texture.set {
                let _ = self.free_texture_set(set);
            }
        }
    }

    /// Free a texture descriptor set back to the pool that allocated it.
    fn free_texture_set(&self, set: vk::DescriptorSet) -> EngineResult<()> {
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
