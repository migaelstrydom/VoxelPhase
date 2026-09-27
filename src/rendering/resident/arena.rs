use std::mem;
use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::EngineResult;
use crate::rendering::deletion_queue::DeletionQueue;
use crate::rendering::frame::{DrawInfo, ManagedBuffer};
use crate::rendering::mesh_source::{MeshBuffers, MeshSource};
use crate::rendering::resident::range_allocator::RangeAllocator;
use crate::rendering::vertex::Vertex;

/// Frames a released mesh is held before its ranges can be reused: every
/// frame that might still draw it has to have finished on the GPU first.
const FRAMES_IN_FLIGHT: u64 = 2;

/// Size of a shared block, which many small meshes are packed into.
const SHARED_BLOCK_VERTICES: u64 = 256 * 1024;
const SHARED_BLOCK_INDICES: u64 = 4 * SHARED_BLOCK_VERTICES;

/// A mesh needing more than this share of a shared block gets a block of its
/// own instead.
const DEDICATED_FRACTION: u64 = 4;

/// Room a dedicated block leaves over its first mesh, as a fraction of it.
/// Terrain gains geometry with every blast, and a replacement that still fits
/// can reuse the block its predecessor held.
const DEDICATED_SLACK: f64 = 0.25;

/// Where one mesh sits in the arena.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ResidentMesh {
    /// The block holding it.
    block: u32,
    /// First vertex, in vertices.
    vertex_offset: u64,
    vertex_count: u64,
    /// First index, in indices. The indices are the mesh's own, counted from
    /// its first vertex.
    index_offset: u64,
    index_count: u64,
}

impl ResidentMesh {
    /// What a draw of this mesh records.
    pub fn draw_info(&self) -> DrawInfo {
        DrawInfo {
            index_count: self.index_count as u32,
            first_index: self.index_offset as u32,
            vertex_offset: self.vertex_offset as i32,
            source: MeshSource::Resident(self.block),
        }
    }

    /// Whether there is anything to draw.
    pub fn is_empty(&self) -> bool {
        self.index_count == 0 || self.vertex_count == 0
    }
}

/// What the arena did since it was last asked.
#[derive(Clone, Copy, Debug, Default)]
pub struct UploadTally {
    /// Mesh bytes written.
    pub bytes: u64,
    /// Blocks allocated to make room.
    pub growths: u32,
}

/// Whether a block packs many meshes or holds one big one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BlockKind {
    Shared,
    Dedicated,
}

/// One vertex buffer and one index buffer, and the bookkeeping of what in
/// them is taken.
struct Block {
    kind: BlockKind,
    vertex_buffer: ManagedBuffer,
    index_buffer: ManagedBuffer,
    /// Counted in vertices.
    vertices: RangeAllocator,
    /// Counted in indices.
    indices: RangeAllocator,
}

impl Block {
    fn new(
        device: &Arc<ManagedDevice>,
        kind: BlockKind,
        vertices: u64,
        indices: u64,
    ) -> EngineResult<Self> {
        Ok(Self {
            kind,
            vertex_buffer: resident_buffer(
                device,
                vertices * mem::size_of::<Vertex>() as u64,
                vk::BufferUsageFlags::VERTEX_BUFFER,
            )?,
            index_buffer: resident_buffer(
                device,
                indices * mem::size_of::<u32>() as u64,
                vk::BufferUsageFlags::INDEX_BUFFER,
            )?,
            vertices: RangeAllocator::new(vertices),
            indices: RangeAllocator::new(indices),
        })
    }

    /// Take room for a mesh, both halves or neither.
    fn allocate(&mut self, vertices: u64, indices: u64) -> Option<(u64, u64)> {
        let vertex_offset = self.vertices.allocate(vertices)?;
        match self.indices.allocate(indices) {
            Some(index_offset) => Some((vertex_offset, index_offset)),
            None => {
                self.vertices.free(vertex_offset, vertices);
                None
            }
        }
    }

    fn is_empty(&self) -> bool {
        self.vertices.used() == 0 && self.indices.used() == 0
    }

    fn buffers(&self) -> MeshBuffers {
        MeshBuffers {
            vertex: self.vertex_buffer.buffer,
            index: self.index_buffer.buffer,
        }
    }
}

/// Buffers that meshes are written into once and drawn from for as long as
/// they live.
///
/// ```text
///   upload ─┬─ small ─▶ first shared block with room ──(none)──▶ new shared block
///           └─ big ───▶ empty dedicated block it fits ──(none)──▶ new dedicated block
///                            │
///                            ▼
///                 write once ─▶ drawn every frame ─▶ release ─▶ held FRAMES_IN_FLIGHT
///                                                                ─▶ ranges free
/// ```
///
/// Room is made by adding a block, never by growing one, so nothing already
/// written is ever copied. A big mesh — the terrain — gets a block to itself:
/// replaced whole on every edit, it alternates between two blocks, and the
/// one it leaves is kept empty for the next edit to write into.
///
/// Nothing is written into a range a frame in flight can read: a range only
/// returns to its block once every frame that might have drawn it has
/// finished, so uploading needs no synchronisation with the GPU.
///
/// The buffers are host-visible and written directly. On a unified-memory GPU
/// that *is* device memory, and device-local memory is asked for first so a
/// discrete GPU with host-visible VRAM gets it too; one without would want a
/// staging copy instead.
pub struct MeshArena {
    device: Arc<ManagedDevice>,
    /// Indexed by `ResidentMesh::block`. A slot is emptied when its block is
    /// destroyed, so the indices of the others never move.
    blocks: Vec<Option<Block>>,
    /// Meshes released, waiting until no frame in flight can read them.
    released: DeletionQueue<ResidentMesh>,
    /// The frame being recorded.
    frame: u64,
    tally: UploadTally,
}

impl MeshArena {
    pub fn new(device: Arc<ManagedDevice>) -> Self {
        Self {
            device,
            blocks: Vec::new(),
            released: DeletionQueue::new(FRAMES_IN_FLIGHT),
            frame: 0,
            tally: UploadTally::default(),
        }
    }

    /// Start frame `frame`: return to their blocks the ranges the finished
    /// frames can no longer read. Only after the fence of the slot being
    /// reused is waited on.
    pub fn begin_frame(&mut self, frame: u64) {
        self.frame = frame;
        for mesh in self.released.take_ready(frame) {
            if let Some(block) = self.blocks[mesh.block as usize].as_mut() {
                block.vertices.free(mesh.vertex_offset, mesh.vertex_count);
                block.indices.free(mesh.index_offset, mesh.index_count);
            }
        }
        self.drop_spare_dedicated_blocks();
    }

    /// The buffers of every block, indexed as `MeshSource::Resident` names
    /// them.
    pub fn buffers(&self) -> Vec<MeshBuffers> {
        self.blocks
            .iter()
            .map(|block| match block {
                Some(block) => block.buffers(),
                None => MeshBuffers {
                    vertex: vk::Buffer::null(),
                    index: vk::Buffer::null(),
                },
            })
            .collect()
    }

    /// Write a mesh in, adding a block if none has room.
    pub fn upload(&mut self, vertices: &[Vertex], indices: &[u32]) -> EngineResult<ResidentMesh> {
        let vertex_count = vertices.len() as u64;
        let index_count = indices.len() as u64;

        let (block, vertex_offset, index_offset) = match self.find_room(vertex_count, index_count) {
            Some(found) => found,
            None => self.add_block(vertex_count, index_count)?,
        };

        let written = self.blocks[block as usize]
            .as_ref()
            .expect("a block just allocated from exists");
        write_at(&written.vertex_buffer, vertex_offset, vertices)?;
        write_at(&written.index_buffer, index_offset, indices)?;
        self.tally.bytes += (mem::size_of_val(vertices) + mem::size_of_val(indices)) as u64;

        Ok(ResidentMesh {
            block,
            vertex_offset,
            vertex_count,
            index_offset,
            index_count,
        })
    }

    /// Give a mesh's ranges back, once no frame in flight can still draw it.
    pub fn release(&mut self, mesh: ResidentMesh) {
        self.released.queue(mesh, self.frame);
    }

    /// What was written since the last call.
    pub fn take_tally(&mut self) -> UploadTally {
        mem::take(&mut self.tally)
    }

    /// Whether a mesh this size is given a block of its own.
    fn is_dedicated(vertices: u64, indices: u64) -> bool {
        vertices > SHARED_BLOCK_VERTICES / DEDICATED_FRACTION
            || indices > SHARED_BLOCK_INDICES / DEDICATED_FRACTION
    }

    /// Room for a mesh in a block that already exists.
    fn find_room(&mut self, vertices: u64, indices: u64) -> Option<(u32, u64, u64)> {
        let dedicated = Self::is_dedicated(vertices, indices);
        self.blocks
            .iter_mut()
            .enumerate()
            .filter_map(|(index, block)| Some((index, block.as_mut()?)))
            .filter(|(_, block)| match block.kind {
                BlockKind::Shared => !dedicated,
                BlockKind::Dedicated => dedicated && block.is_empty(),
            })
            .find_map(|(index, block)| {
                let (vertex_offset, index_offset) = block.allocate(vertices, indices)?;
                Some((index as u32, vertex_offset, index_offset))
            })
    }

    /// Allocate a block for a mesh nothing had room for, and place it.
    fn add_block(&mut self, vertices: u64, indices: u64) -> EngineResult<(u32, u64, u64)> {
        let mut block = if Self::is_dedicated(vertices, indices) {
            let slack = |n: u64| n + (n as f64 * DEDICATED_SLACK) as u64;
            Block::new(
                &self.device,
                BlockKind::Dedicated,
                slack(vertices),
                slack(indices),
            )?
        } else {
            Block::new(
                &self.device,
                BlockKind::Shared,
                SHARED_BLOCK_VERTICES,
                SHARED_BLOCK_INDICES,
            )?
        };
        let (vertex_offset, index_offset) = block
            .allocate(vertices, indices)
            .expect("a new block has room for the mesh it was made for");
        self.tally.growths += 1;

        let index = match self.blocks.iter().position(Option::is_none) {
            Some(free) => {
                self.blocks[free] = Some(block);
                free
            }
            None => {
                self.blocks.push(Some(block));
                self.blocks.len() - 1
            }
        };
        Ok((index as u32, vertex_offset, index_offset))
    }

    /// Keep one empty dedicated block, the biggest, for the next big mesh to
    /// reuse, and destroy the rest.
    ///
    /// Safe to destroy outright: a block is only empty once every range in it
    /// has been held past the frames in flight.
    fn drop_spare_dedicated_blocks(&mut self) {
        let spare = |block: &Option<Block>| {
            block
                .as_ref()
                .filter(|b| b.kind == BlockKind::Dedicated && b.is_empty())
                .map(|b| b.vertices.capacity())
        };
        let Some(keep) = self
            .blocks
            .iter()
            .enumerate()
            .filter_map(|(index, block)| Some((index, spare(block)?)))
            .max_by_key(|&(_, capacity)| capacity)
            .map(|(index, _)| index)
        else {
            return;
        };
        for (index, block) in self.blocks.iter_mut().enumerate() {
            if index != keep && spare(block).is_some() {
                *block = None;
            }
        }
    }
}

/// A host-visible buffer, in device-local memory where there is any that the
/// host can also see.
fn resident_buffer(
    device: &Arc<ManagedDevice>,
    size: vk::DeviceSize,
    usage: vk::BufferUsageFlags,
) -> EngineResult<ManagedBuffer> {
    let host = vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT;
    ManagedBuffer::new(
        Arc::clone(device),
        size.max(1),
        usage,
        host | vk::MemoryPropertyFlags::DEVICE_LOCAL,
    )
    .or_else(|_| ManagedBuffer::new(Arc::clone(device), size.max(1), usage, host))
}

/// Write `items` into `buffer` starting at element `offset`.
fn write_at<T: Copy>(buffer: &ManagedBuffer, offset: u64, items: &[T]) -> EngineResult<()> {
    if items.is_empty() {
        return Ok(());
    }
    unsafe {
        let ptr = buffer.map_memory(0, vk::MemoryMapFlags::empty())?;
        let dst = (ptr as *mut u8).add(offset as usize * mem::size_of::<T>());
        std::ptr::copy_nonoverlapping(items.as_ptr() as *const u8, dst, mem::size_of_val(items));
        buffer.unmap_memory();
    }
    Ok(())
}
