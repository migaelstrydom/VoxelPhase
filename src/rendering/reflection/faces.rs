use std::sync::Arc;

use ash::vk;
use nalgebra::{Matrix4, Vector3};

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult};
use crate::rendering::frame::ManagedBuffer;
use crate::rendering::in_flight::{FrameSlot, PerFrame, FRAMES_IN_FLIGHT};

/// The most faces one frame can capture. Bounds the per-frame uniform buffer,
/// and so how many newcomers a frame admits beyond its refresh budget; also
/// the length of the layer list the mip filter takes as push constants,
/// which `probe_mips.comp` must agree with.
pub const MAX_FACES_PER_FRAME: u32 = 24;

/// Bytes between consecutive faces in the buffer.
///
/// A dynamic offset must be a multiple of the device's
/// `minUniformBufferOffsetAlignment`, which the spec caps at 256. Spacing the
/// faces that far apart satisfies every device without asking this one.
const FACE_STRIDE: vk::DeviceSize = 256;

/// One face's uniforms, as `probe_face.glsl` reads them.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct GpuProbeFace {
    /// World space to the face's clip space, column-major.
    view_proj: [f32; 16],
    /// The probe's centre; w unused.
    eye: [f32; 4],
}

impl GpuProbeFace {
    pub fn new(view_proj: &Matrix4<f32>, eye: &Vector3<f32>) -> Self {
        let mut columns = [0.0; 16];
        columns.copy_from_slice(view_proj.as_slice());
        Self {
            view_proj: columns,
            eye: [eye.x, eye.y, eye.z, 0.0],
        }
    }
}

/// The frame's face uniforms: one buffer per frame in flight, each holding up
/// to [`MAX_FACES_PER_FRAME`] faces, reached through one descriptor set
/// whose dynamic offset picks the face.
///
/// Per frame in flight because the CPU writes it while recording, like every
/// other buffer a frame fills; see `rendering::in_flight`.
pub struct FaceUniforms {
    /// Only ever reached through `mapped` and the descriptor sets; held so
    /// the buffers live as long as they do.
    #[allow(dead_code)]
    buffers: PerFrame<ManagedBuffer>,
    /// Persistent mappings of `buffers`. Host-coherent, so writes need no
    /// flush.
    mapped: PerFrame<*mut u8>,
    sets: PerFrame<vk::DescriptorSet>,
    pool: vk::DescriptorPool,
    device: Arc<ManagedDevice>,
}

// The mappings are owned exclusively by this struct and written only through
// `&mut self`, so moving or sharing it across threads is as safe as doing so
// with the buffers themselves. The same reasoning as `SurfaceBuffer`.
unsafe impl Send for FaceUniforms {}
unsafe impl Sync for FaceUniforms {}

impl FaceUniforms {
    pub fn new(
        device: Arc<ManagedDevice>,
        set_layout: vk::DescriptorSetLayout,
    ) -> EngineResult<Self> {
        let size = FACE_STRIDE * MAX_FACES_PER_FRAME as vk::DeviceSize;
        let buffers = PerFrame::try_new(|_| {
            ManagedBuffer::new(
                Arc::clone(&device),
                size,
                vk::BufferUsageFlags::UNIFORM_BUFFER,
                vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
            )
        })?;
        let mapped = PerFrame::try_new(|slot| unsafe {
            Ok(buffers[slot].map_memory(0, vk::MemoryMapFlags::empty())? as *mut u8)
        })?;

        let frames = FRAMES_IN_FLIGHT as u32;
        let pool_sizes = [vk::DescriptorPoolSize::default()
            .ty(vk::DescriptorType::UNIFORM_BUFFER_DYNAMIC)
            .descriptor_count(frames)];
        let pool_info = vk::DescriptorPoolCreateInfo::default()
            .max_sets(frames)
            .pool_sizes(&pool_sizes);
        let pool = unsafe { device.device.create_descriptor_pool(&pool_info, None) }
            .map_err(|e| EngineError::Descriptor(format!("probe face pool: {:?}", e)))?;

        let layouts = [set_layout; FRAMES_IN_FLIGHT];
        let alloc_info = vk::DescriptorSetAllocateInfo::default()
            .descriptor_pool(pool)
            .set_layouts(&layouts);
        let allocated = unsafe { device.device.allocate_descriptor_sets(&alloc_info) }
            .map_err(|e| EngineError::Descriptor(format!("probe face sets: {:?}", e)))?;
        let sets = PerFrame::new(|slot| allocated[slot.index()]);

        for slot in FrameSlot::all() {
            let buffer_info = [vk::DescriptorBufferInfo::default()
                .buffer(buffers[slot].buffer)
                .offset(0)
                .range(std::mem::size_of::<GpuProbeFace>() as vk::DeviceSize)];
            let write = vk::WriteDescriptorSet::default()
                .dst_set(sets[slot])
                .dst_binding(0)
                .descriptor_type(vk::DescriptorType::UNIFORM_BUFFER_DYNAMIC)
                .buffer_info(&buffer_info);
            unsafe { device.device.update_descriptor_sets(&[write], &[]) };
        }

        Ok(Self {
            buffers,
            mapped,
            sets,
            pool,
            device,
        })
    }

    /// Write the `index`th face of the frame in `slot`. Only after that
    /// slot's fence has been waited on.
    pub fn write(&mut self, slot: FrameSlot, index: u32, face: GpuProbeFace) {
        assert!(
            index < MAX_FACES_PER_FRAME,
            "probe face {index} past the frame's buffer"
        );
        unsafe {
            let at = self.mapped[slot].add((FACE_STRIDE * index as vk::DeviceSize) as usize);
            std::ptr::write_unaligned(at.cast::<GpuProbeFace>(), face);
        }
    }

    /// The descriptor set of the frame in `slot`.
    pub fn set(&self, slot: FrameSlot) -> vk::DescriptorSet {
        self.sets[slot]
    }

    /// The dynamic offset that selects the `index`th face.
    pub fn offset(index: u32) -> u32 {
        (FACE_STRIDE * index as vk::DeviceSize) as u32
    }
}

impl Drop for FaceUniforms {
    fn drop(&mut self) {
        // The buffers stay mapped until they are freed, which is legal.
        unsafe {
            self.device.device.destroy_descriptor_pool(self.pool, None);
        }
    }
}
