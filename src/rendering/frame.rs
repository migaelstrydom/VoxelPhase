//! Per-frame data management.
//!
//! This module handles mutable per-frame resources like vertex/index buffers
//! and uniform buffers that change every frame.
//!
//! # Buffer Lifecycle
//!
//! When buffers need to be resized during rendering, we cannot destroy the old
//! buffer immediately because it may still be referenced by in-flight GPU commands.
//! Instead, old buffers are queued for deferred deletion via [`DeletionQueue`].
//!
//! The deletion queue should be flushed at frame start, after the fence wait
//! ensures the GPU has finished with previous frames.

use std::mem;
use std::sync::Arc;

use ash::vk;
use nalgebra::{Matrix4, Vector3};

use crate::core::device::ManagedDevice;
use crate::core::error::{BufferOperation, EngineError, EngineResult};
use crate::core::vulkan_context::find_memorytype_index;
use crate::lighting::{ActiveLight, ActiveLights, PointLight, MAX_ACTIVE_LIGHTS};
use crate::rendering::colour::Colour;
use crate::rendering::deletion_queue::DeletionQueue;
use crate::rendering::vertex::Vertex;

/// Uniform buffer object for per-frame scene data.
/// Model matrix is now passed via push constants per draw call.
///
/// Layout must match the `SceneUbo` block in shader/scene.glsl. All members
/// after the matrices are vec4-sized so std140 alignment needs no padding.
#[derive(Clone, Debug, Copy)]
#[repr(C)]
pub struct SceneUbo {
    pub view: Matrix4<f32>,
    pub proj: Matrix4<f32>,

    /// xyz = camera world position, w unused. Needed for specular response.
    pub camera_pos: [f32; 4],

    /// xyz = normalized direction from surface towards the sun, w = intensity.
    pub sun_direction: [f32; 4],

    /// rgb = linear sun colour, w unused.
    pub sun_colour: [f32; 4],

    /// rgb = linear ambient fill colour, w unused.
    pub ambient_colour: [f32; 4],
}

/// A single point light as the GPU sees it.
///
/// Two `vec4`s, which is exactly std140's array-element stride for this data —
/// no per-element padding is needed. Layout must match `PointLight` in
/// shader/lights.glsl.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct GpuPointLight {
    /// xyz = world position, w = range (distance at which contribution is zero).
    pub position_range: [f32; 4],

    /// rgb = linear colour (authored magnitude), w = scale bringing that
    /// colour to the authored intensity luminance. `shadePoint` in
    /// shader/lighting.glsl multiplies the two, so it is agnostic to which
    /// convention produced `w`.
    pub colour_intensity: [f32; 4],
}

impl GpuPointLight {
    /// Pack a collected light. Alpha of the colour is dropped; the light array
    /// has no use for it.
    ///
    /// The intensity-to-scale conversion (`PointLight::radiance_scale`)
    /// happens here, at the CPU/GPU boundary, mirroring how
    /// `Material::surface_params` converts `Emission::strength`. `ActiveLight`
    /// itself keeps carrying the authored `colour` and `intensity` unchanged.
    fn from_active(light: &ActiveLight) -> Self {
        Self {
            position_range: [
                light.position.x,
                light.position.y,
                light.position.z,
                light.range,
            ],
            colour_intensity: [
                light.colour.r,
                light.colour.g,
                light.colour.b,
                PointLight::new(light.colour, light.intensity, light.range).radiance_scale(),
            ],
        }
    }
}

/// Uniform buffer object for the per-frame point light set (set 0, binding 1).
///
/// Layout must match the `LightUbo` block in shader/lights.glsl. std140 aligns
/// the array to 16 bytes, so the `u32` count needs an explicit 12-byte tail
/// before `lights` begins — see the layout assertions in this module's tests.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct LightUbo {
    /// Number of entries in `lights` that are live. The shader loop is bounded
    /// by this rather than by `MAX_ACTIVE_LIGHTS`.
    pub count: u32,

    /// Padding to push `lights` to the 16-byte alignment std140 requires for
    /// an array. Never read by the shader.
    pub _padding: [u32; 3],

    /// The light array. Entries at or beyond `count` are zeroed rather than
    /// stale, so a shader that ignores `count` dims rather than corrupts.
    pub lights: [GpuPointLight; MAX_ACTIVE_LIGHTS],
}

impl LightUbo {
    /// Pack the collected light set for upload.
    ///
    /// Pure — no Vulkan involved — so the packing can be tested without a
    /// device. Lights beyond `MAX_ACTIVE_LIGHTS` cannot occur (the collector
    /// caps at that), but are truncated defensively rather than panicking.
    pub fn from_active_lights(active: &ActiveLights) -> Self {
        let mut ubo = Self::default();
        let lights = active.lights();
        let count = lights.len().min(MAX_ACTIVE_LIGHTS);

        for (slot, light) in ubo.lights.iter_mut().zip(&lights[..count]) {
            *slot = GpuPointLight::from_active(light);
        }
        ubo.count = count as u32;
        ubo
    }
}

impl Default for LightUbo {
    fn default() -> Self {
        Self {
            count: 0,
            _padding: [0; 3],
            lights: [GpuPointLight::default(); MAX_ACTIVE_LIGHTS],
        }
    }
}

/// Per-frame lighting environment shared by every lit surface.
#[derive(Clone, Copy, Debug)]
pub struct SceneLighting {
    /// Normalized direction from a surface towards the sun.
    pub sun_direction: Vector3<f32>,

    /// Linear sun colour.
    pub sun_colour: Colour,

    /// Sun brightness multiplier.
    pub sun_intensity: f32,

    /// Linear ambient fill applied to all surfaces.
    pub ambient_colour: Colour,
}

impl Default for SceneLighting {
    fn default() -> Self {
        Self {
            sun_direction: Vector3::new(0.5, 0.7, 0.5).normalize(),
            sun_colour: Colour::new(1.0, 0.97, 0.9, 1.0),
            // Roughly five times the fill the sky provides, so the sun reads as
            // a key light — it is the ratio between them that gives a surface
            // its modelling, and a sun that merely matches the fill renders
            // flat. The absolute level is held near where it was before the sky
            // became a light source, because level content is authored against
            // it: the albedos in use are saturated and already close to full
            // value, so raising total illumination pushes them up the tonemap's
            // shoulder and they go pale and electric rather than bright.
            sun_intensity: 1.2,
            // Near black on purpose. Sky and ground irradiance are what fill
            // unlit surfaces now, and they do it directionally; this remains as
            // an author's dial for lifting a scene without moving the sky, and
            // as a floor stopping anything reaching pure black.
            ambient_colour: Colour::new(0.03, 0.035, 0.045, 1.0),
        }
    }
}

/// A GPU buffer with RAII memory management.
pub struct ManagedBuffer {
    pub buffer: vk::Buffer,
    pub memory: vk::DeviceMemory,
    pub size: vk::DeviceSize,
    device: Arc<ManagedDevice>,
}

impl std::fmt::Debug for ManagedBuffer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ManagedBuffer")
            .field("buffer", &self.buffer)
            .field("memory", &self.memory)
            .field("size", &self.size)
            .finish()
    }
}

impl ManagedBuffer {
    pub fn new(
        device: Arc<ManagedDevice>,
        size: vk::DeviceSize,
        usage: vk::BufferUsageFlags,
        memory_properties: vk::MemoryPropertyFlags,
    ) -> EngineResult<Self> {
        unsafe {
            let buffer_info = vk::BufferCreateInfo::default()
                .size(size)
                .usage(usage)
                .sharing_mode(vk::SharingMode::EXCLUSIVE);

            let buffer = device
                .device
                .create_buffer(&buffer_info, None)
                .map_err(|e| EngineError::Buffer {
                    operation: BufferOperation::Create,
                    size,
                    reason: format!("{:?}", e),
                })?;

            let memory_req = device.device.get_buffer_memory_requirements(buffer);
            let memory_type_index = find_memorytype_index(
                &memory_req,
                &device.device_memory_properties,
                memory_properties,
            )
            .ok_or_else(|| EngineError::Buffer {
                operation: BufferOperation::Create,
                size,
                reason: "no suitable memory type".to_string(),
            })?;

            let allocate_info = vk::MemoryAllocateInfo::default()
                .allocation_size(memory_req.size)
                .memory_type_index(memory_type_index);

            let memory = device
                .device
                .allocate_memory(&allocate_info, None)
                .map_err(|e| EngineError::Buffer {
                    operation: BufferOperation::Create,
                    size,
                    reason: format!("memory allocation: {:?}", e),
                })?;

            device
                .device
                .bind_buffer_memory(buffer, memory, 0)
                .map_err(|e| EngineError::Buffer {
                    operation: BufferOperation::Bind,
                    size,
                    reason: format!("{:?}", e),
                })?;

            Ok(Self {
                buffer,
                memory,
                size,
                device,
            })
        }
    }

    /// Map buffer memory for CPU access.
    ///
    /// # Safety
    /// Caller must ensure proper usage of the returned pointer and call unmap_memory when done.
    pub unsafe fn map_memory(
        &self,
        offset: vk::DeviceSize,
        flags: vk::MemoryMapFlags,
    ) -> EngineResult<*mut std::ffi::c_void> {
        self.device
            .device
            .map_memory(self.memory, offset, self.size, flags)
            .map_err(|e| EngineError::Buffer {
                operation: BufferOperation::Map,
                size: self.size,
                reason: format!("{:?}", e),
            })
    }

    /// Unmap previously mapped buffer memory.
    ///
    /// # Safety
    /// Must only be called after a successful map_memory call.
    pub unsafe fn unmap_memory(&self) {
        self.device.device.unmap_memory(self.memory);
    }
}

impl Drop for ManagedBuffer {
    fn drop(&mut self) {
        unsafe {
            if self.buffer != vk::Buffer::null() {
                self.device.device.destroy_buffer(self.buffer, None);
            }
            if self.memory != vk::DeviceMemory::null() {
                self.device.device.free_memory(self.memory, None);
            }
        }
    }
}

/// Information needed to issue a draw call for a mesh.
#[derive(Debug, Clone, Copy)]
pub struct DrawInfo {
    pub index_count: u32,
    pub first_index: u32,
    pub vertex_offset: i32,
}

/// Manages per-frame mutable data: vertex buffer, index buffer, UBO.
///
/// Meshes are accumulated in the buffer during frame recording, then
/// the buffer is reset at the start of the next frame.
///
/// Handles buffer resizing safely by deferring destruction of old buffers
/// until the GPU is no longer using them.
pub struct FrameData {
    pub vertex_buffer: ManagedBuffer,
    pub index_buffer: ManagedBuffer,
    pub scene_ubo_buffer: ManagedBuffer,
    pub light_ubo_buffer: ManagedBuffer,
    device: Arc<ManagedDevice>,
    /// Buffers pending deletion (deferred until GPU is done with them).
    buffer_deletion_queue: DeletionQueue<ManagedBuffer>,
    /// Current frame number for deletion queue tracking.
    frame_number: u64,
    /// Current write position in vertex buffer (in vertices, not bytes).
    current_vertex_count: u32,
    /// Current write position in index buffer (in indices, not bytes).
    current_index_count: u32,
}

impl FrameData {
    /// Number of frames to wait before destroying old buffers.
    /// This should match your frames-in-flight count.
    const FRAMES_IN_FLIGHT: u64 = 2;

    /// Create frame data with initial buffer sizes.
    pub fn new(device: Arc<ManagedDevice>) -> EngineResult<Self> {
        // Modest initial sizes - buffers will grow as needed via deferred resize.
        let initial_vertex_size = mem::size_of::<Vertex>() as u64 * 1024;
        let initial_index_size = mem::size_of::<u32>() as u64 * 4096;
        let ubo_size = mem::size_of::<SceneUbo>() as u64;
        let light_ubo_size = mem::size_of::<LightUbo>() as u64;

        let vertex_buffer = ManagedBuffer::new(
            Arc::clone(&device),
            initial_vertex_size,
            vk::BufferUsageFlags::VERTEX_BUFFER,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )?;

        let index_buffer = ManagedBuffer::new(
            Arc::clone(&device),
            initial_index_size,
            vk::BufferUsageFlags::INDEX_BUFFER,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )?;

        let scene_ubo_buffer = ManagedBuffer::new(
            Arc::clone(&device),
            ubo_size,
            vk::BufferUsageFlags::UNIFORM_BUFFER,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )?;

        let light_ubo_buffer = ManagedBuffer::new(
            Arc::clone(&device),
            light_ubo_size,
            vk::BufferUsageFlags::UNIFORM_BUFFER,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )?;

        // Vulkan does not guarantee zeroed memory on allocation. Write a zeroed
        // `LightUbo` immediately so any renderer client that draws lit geometry
        // before its first `update_light_ubo` call reads `count = 0` rather than
        // whatever bytes the host-visible page happened to hold — an
        // uninitialised count could otherwise drive a near-unbounded shader loop.
        unsafe {
            let ptr = light_ubo_buffer.map_memory(0, vk::MemoryMapFlags::empty())?;
            let slice = std::slice::from_raw_parts_mut(ptr as *mut LightUbo, 1);
            slice[0] = LightUbo::default();
            light_ubo_buffer.unmap_memory();
        }

        Ok(Self {
            vertex_buffer,
            index_buffer,
            scene_ubo_buffer,
            light_ubo_buffer,
            device,
            buffer_deletion_queue: DeletionQueue::new(Self::FRAMES_IN_FLIGHT),
            frame_number: 0,
            current_vertex_count: 0,
            current_index_count: 0,
        })
    }

    /// Signal that a new frame is starting.
    ///
    /// Call this at frame start, AFTER waiting for the frame fence.
    /// This flushes old buffers that are safe to delete, advances the frame counter,
    /// and resets the mesh accumulation offsets.
    pub fn begin_frame(&mut self) {
        self.buffer_deletion_queue.flush(self.frame_number);
        self.frame_number += 1;
        // Reset for new frame - meshes will be accumulated fresh
        self.current_vertex_count = 0;
        self.current_index_count = 0;
    }

    /// Force cleanup of all pending deletions.
    ///
    /// Call this during shutdown after `device_wait_idle()`.
    pub fn cleanup(&mut self) {
        self.buffer_deletion_queue.flush_all();
    }

    /// Append mesh data to the frame's vertex/index buffers.
    ///
    /// Returns draw information (offsets and counts) needed for the draw call.
    /// Multiple meshes can be appended per frame; they accumulate in the buffer.
    ///
    /// If buffers need to be resized, the old buffers are queued for deferred
    /// deletion to avoid destroying them while the GPU may still be using them.
    pub fn append_mesh_data(
        &mut self,
        vertices: &[Vertex],
        indices: &[u32],
    ) -> EngineResult<DrawInfo> {
        let new_vertex_count = self.current_vertex_count + vertices.len() as u32;
        let new_index_count = self.current_index_count + indices.len() as u32;

        let required_vertex_size =
            (mem::size_of::<Vertex>() as u32 * new_vertex_count) as vk::DeviceSize;
        let required_index_size =
            (mem::size_of::<u32>() as u32 * new_index_count) as vk::DeviceSize;

        // Resize vertex buffer if needed (with deferred deletion of old buffer)
        if self.vertex_buffer.size < required_vertex_size {
            let old_size = self.vertex_buffer.size;
            // Double the size to reduce resize frequency
            let new_size = required_vertex_size.max(old_size * 2);
            let new_buffer = ManagedBuffer::new(
                Arc::clone(&self.device),
                new_size,
                vk::BufferUsageFlags::VERTEX_BUFFER,
                vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
            )?;
            // Swap in new buffer, queue old one for deferred deletion
            let old_buffer = mem::replace(&mut self.vertex_buffer, new_buffer);
            self.buffer_deletion_queue
                .queue(old_buffer, self.frame_number);
            log::debug!("Vertex buffer resized: {} -> {} bytes", old_size, new_size);
        }

        // Resize index buffer if needed (with deferred deletion of old buffer)
        if self.index_buffer.size < required_index_size {
            let old_size = self.index_buffer.size;
            // Double the size to reduce resize frequency
            let new_size = required_index_size.max(old_size * 2);
            let new_buffer = ManagedBuffer::new(
                Arc::clone(&self.device),
                new_size,
                vk::BufferUsageFlags::INDEX_BUFFER,
                vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
            )?;
            // Swap in new buffer, queue old one for deferred deletion
            let old_buffer = mem::replace(&mut self.index_buffer, new_buffer);
            self.buffer_deletion_queue
                .queue(old_buffer, self.frame_number);
            log::debug!("Index buffer resized: {} -> {} bytes", old_size, new_size);
        }

        // Calculate byte offsets for appending
        let vertex_byte_offset = (self.current_vertex_count as usize) * mem::size_of::<Vertex>();
        let index_byte_offset = (self.current_index_count as usize) * mem::size_of::<u32>();
        let vertex_size = mem::size_of::<Vertex>() * vertices.len();
        let index_size = mem::size_of::<u32>() * indices.len();

        // Append vertex data at current offset
        unsafe {
            let ptr = self
                .vertex_buffer
                .map_memory(0, vk::MemoryMapFlags::empty())?;
            let dst = (ptr as *mut u8).add(vertex_byte_offset);
            std::ptr::copy_nonoverlapping(vertices.as_ptr() as *const u8, dst, vertex_size);
            self.vertex_buffer.unmap_memory();
        }

        // Append index data at current offset
        unsafe {
            let ptr = self
                .index_buffer
                .map_memory(0, vk::MemoryMapFlags::empty())?;
            let dst = (ptr as *mut u8).add(index_byte_offset);
            std::ptr::copy_nonoverlapping(indices.as_ptr() as *const u8, dst, index_size);
            self.index_buffer.unmap_memory();
        }

        // Build draw info before updating offsets
        let draw_info = DrawInfo {
            index_count: indices.len() as u32,
            first_index: self.current_index_count,
            vertex_offset: self.current_vertex_count as i32,
        };

        // Update offsets for next mesh
        self.current_vertex_count = new_vertex_count;
        self.current_index_count = new_index_count;

        Ok(draw_info)
    }

    /// Update the scene uniform buffer with per-frame camera and lighting data.
    /// Call this once per frame, not per draw call.
    pub fn update_scene_ubo(
        &mut self,
        view: &Matrix4<f32>,
        proj: &Matrix4<f32>,
        camera_pos: &Vector3<f32>,
        lighting: &SceneLighting,
    ) -> EngineResult<()> {
        let sun = lighting.sun_direction.normalize();
        let ubo = SceneUbo {
            view: *view,
            proj: *proj,
            camera_pos: [camera_pos.x, camera_pos.y, camera_pos.z, 0.0],
            sun_direction: [sun.x, sun.y, sun.z, lighting.sun_intensity],
            sun_colour: [
                lighting.sun_colour.r,
                lighting.sun_colour.g,
                lighting.sun_colour.b,
                0.0,
            ],
            ambient_colour: [
                lighting.ambient_colour.r,
                lighting.ambient_colour.g,
                lighting.ambient_colour.b,
                0.0,
            ],
        };

        unsafe {
            let ptr = self
                .scene_ubo_buffer
                .map_memory(0, vk::MemoryMapFlags::empty())?;
            let slice = std::slice::from_raw_parts_mut(ptr as *mut SceneUbo, 1);
            slice[0] = ubo;
            self.scene_ubo_buffer.unmap_memory();
        }

        Ok(())
    }

    /// Upload the frame's point light set.
    ///
    /// Separate from the scene UBO because the two have different update
    /// triggers: the scene block changes whenever the camera moves, the light
    /// block only when the collected set changes.
    pub fn update_light_ubo(&mut self, active: &ActiveLights) -> EngineResult<()> {
        let ubo = LightUbo::from_active_lights(active);

        unsafe {
            let ptr = self
                .light_ubo_buffer
                .map_memory(0, vk::MemoryMapFlags::empty())?;
            let slice = std::slice::from_raw_parts_mut(ptr as *mut LightUbo, 1);
            slice[0] = ubo;
            self.light_ubo_buffer.unmap_memory();
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use nalgebra::Vector3;

    use crate::lighting::{LightCandidate, LightCollector, LightId, PointLight};

    /// Build an `ActiveLights` holding `n` distinguishable lights by running the
    /// real collector, so the tests exercise the same path the renderer does.
    fn active_lights(n: usize) -> ActiveLights {
        let candidates: Vec<_> = (0..n)
            .map(|i| {
                LightCandidate::new(
                    LightId(i as u32),
                    Vector3::new(i as f32, 0.0, 0.0),
                    PointLight::new(Colour::rgb(1.0, 0.5, 0.25), 2.0, 100.0),
                )
            })
            .collect();

        let mut active = ActiveLights::default();
        LightCollector::default().collect(Vector3::zeros(), candidates, &mut active);
        active
    }

    #[test]
    fn gpu_point_light_is_two_vec4s() {
        assert_eq!(mem::size_of::<GpuPointLight>(), 32);
        assert_eq!(mem::align_of::<GpuPointLight>(), 4);
    }

    #[test]
    fn light_array_starts_at_a_std140_aligned_offset() {
        // std140 requires an array of vec4-sized elements to begin on a 16-byte
        // boundary. The count occupies the first 4 bytes, so the padding must
        // carry it to 16 — if this fails the whole array is shifted and every
        // light reads garbage.
        let ubo = LightUbo::default();
        let base = &ubo as *const LightUbo as usize;
        let lights = &ubo.lights as *const _ as usize;
        assert_eq!(lights - base, 16);
    }

    #[test]
    fn light_ubo_size_matches_std140_expectation() {
        // 16-byte prologue + MAX_ACTIVE_LIGHTS * 32.
        assert_eq!(
            mem::size_of::<LightUbo>(),
            16 + MAX_ACTIVE_LIGHTS * 32,
            "LightUbo size drifted from the std140 layout the shader assumes"
        );
    }

    #[test]
    fn packs_empty_light_set() {
        let ubo = LightUbo::from_active_lights(&ActiveLights::default());
        assert_eq!(ubo.count, 0);
        assert!(ubo.lights.iter().all(|l| *l == GpuPointLight::default()));
    }

    #[test]
    fn packs_partial_fill_with_a_zeroed_tail() {
        let active = active_lights(3);
        let ubo = LightUbo::from_active_lights(&active);

        assert_eq!(ubo.count, 3);
        for (slot, light) in ubo.lights.iter().zip(active.lights()) {
            assert_eq!(slot.position_range[3], light.range);
        }
        // The tail is defined, not leftover garbage.
        assert!(ubo.lights[3..]
            .iter()
            .all(|l| *l == GpuPointLight::default()));
    }

    #[test]
    fn packs_a_full_set() {
        let active = active_lights(MAX_ACTIVE_LIGHTS);
        let ubo = LightUbo::from_active_lights(&active);

        assert_eq!(ubo.count, MAX_ACTIVE_LIGHTS as u32);
        // The authored intensity (2.0) is luminance, not the packed scale, so
        // check the property that matters: colour * scale has that luminance.
        assert!(ubo.lights.iter().all(|l| {
            let packed = l.colour_intensity;
            let radiance = Colour::new(
                packed[0] * packed[3],
                packed[1] * packed[3],
                packed[2] * packed[3],
                1.0,
            );
            (radiance.luminance() - 2.0).abs() < 1e-4
        }));
    }

    #[test]
    fn packs_position_and_colour_into_the_right_lanes() {
        let light = PointLight::new(Colour::rgb(0.1, 0.2, 0.3), 4.0, 12.0)
            .with_offset(Vector3::new(0.0, 5.0, 0.0));
        let mut active = ActiveLights::default();
        LightCollector::default().collect(
            Vector3::zeros(),
            vec![LightCandidate::new(
                LightId(0),
                Vector3::new(1.0, 0.0, -2.0),
                light,
            )],
            &mut active,
        );

        let ubo = LightUbo::from_active_lights(&active);
        assert_eq!(ubo.count, 1);
        // World position is entity + offset, range in w.
        assert_eq!(ubo.lights[0].position_range, [1.0, 5.0, -2.0, 12.0]);
        // Colour lanes carry the authored magnitude unchanged; w is the scale
        // that brings colour * w to the authored intensity (4.0) luminance,
        // not the intensity itself.
        let packed = ubo.lights[0].colour_intensity;
        assert_eq!(&packed[0..3], &[0.1, 0.2, 0.3]);
        let radiance = Colour::new(
            packed[0] * packed[3],
            packed[1] * packed[3],
            packed[2] * packed[3],
            1.0,
        );
        assert!((radiance.luminance() - 4.0).abs() < 1e-4);
    }

    #[test]
    fn glsl_light_count_matches_the_rust_constant() {
        // The build consumes pre-compiled .spv and never reads the GLSL, so
        // nothing else would catch a drift here. A mismatch silently corrupts
        // the tail of the light array at runtime.
        let source = include_str!("../../shader/lights.glsl");
        let declared = source
            .lines()
            .find_map(|line| {
                let line = line.trim();
                let rest = line.strip_prefix("const int MAX_ACTIVE_LIGHTS")?;
                let value = rest.split('=').nth(1)?;
                value
                    .trim()
                    .trim_end_matches(';')
                    .trim()
                    .parse::<usize>()
                    .ok()
            })
            .expect(
                "shader/lights.glsl must declare `const int MAX_ACTIVE_LIGHTS = <n>;` \
                 on a single line so this check can parse it",
            );

        assert_eq!(
            declared, MAX_ACTIVE_LIGHTS,
            "shader/lights.glsl declares MAX_ACTIVE_LIGHTS = {} but Rust has {}. \
             Update shader/lights.glsl to match, then recompile the shaders \
             (see CLAUDE.md) — the .spv in the tree is stale until you do.",
            declared, MAX_ACTIVE_LIGHTS
        );
    }
}
