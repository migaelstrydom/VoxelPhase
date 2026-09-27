use ash::vk;

/// Which buffer pair a draw's geometry lives in.
///
/// A draw carries this rather than the buffer handles themselves: the frame's
/// buffers can be replaced by bigger ones partway through recording a frame,
/// and the handles a draw must bind are the ones current when it is
/// *recorded*, not when it was issued.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeshSource {
    /// The frame's streamed buffers, rewritten every frame: geometry that
    /// changes from one frame to the next, such as a posed character.
    Frame,
    /// A block of the resident arena, written once per mesh and kept:
    /// models and terrain. Holds the block's index.
    Resident(u32),
}

/// One vertex buffer and the index buffer that goes with it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MeshBuffers {
    pub vertex: vk::Buffer,
    pub index: vk::Buffer,
}

impl MeshBuffers {
    /// Bind both at offset zero.
    pub fn bind(&self, device: &ash::Device, cb: vk::CommandBuffer) {
        unsafe {
            device.cmd_bind_vertex_buffers(cb, 0, &[self.vertex], &[0]);
            device.cmd_bind_index_buffer(cb, self.index, 0, vk::IndexType::UINT32);
        }
    }
}

/// Every buffer pair a frame's geometry draws can read from.
#[derive(Clone, Debug)]
pub struct MeshBindings {
    /// The frame's streamed buffers.
    pub frame: MeshBuffers,
    /// The resident arena's blocks, indexed as `MeshSource::Resident` names
    /// them.
    pub resident: Vec<MeshBuffers>,
}

impl MeshBindings {
    /// The pair a draw from `source` binds.
    pub fn of(&self, source: MeshSource) -> MeshBuffers {
        match source {
            MeshSource::Frame => self.frame,
            MeshSource::Resident(block) => self.resident[block as usize],
        }
    }
}
