//! Water surface rendering.
//!
//! Meshes are static between topology changes (see [`basin_mesher`]): the
//! renderer rebuilds its mesh only when a basin's region changes, uploads it
//! once into each frame slot's buffers, and otherwise only pushes each
//! body's level and swell per draw. A tile whose ripples are awake is drawn
//! instead from a fine grid displaced out of that frame's ripple storage.
//!
//! [`basin_mesher`]: super::basin_mesher

use std::sync::Arc;

use ash::vk;
use nalgebra::{Matrix4, Vector2, Vector3};
use rustc_hash::FxHashMap;

use super::basin_mesher::{MeshKey, WaterMesh, WaterScene};
use super::fall_mesher::{FallKey, FallMesh, SPREAD as FALL_SPREAD};
use super::pipeline::{WaterPipeline, BODY_PUSH_OFFSET, FRAGMENT_PUSH_OFFSET};
use super::reach_mesher::RiverMesh;
use super::vertex::FineVertex;
use crate::core::device::ManagedDevice;
use crate::core::error::EngineResult;
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::frame::ManagedBuffer;
use crate::rendering::in_flight::{FrameSlot, PerFrame, StreamedMesh};
use crate::water::geometry::{CHUNK_COLUMNS, COLUMNS_PER_CHUNK, COLUMN_SIZE};
use crate::water::ids::StoreId;
use crate::water::surface::{RippleConfig, PADDED_CELLS_PER_TILE, TILE_CELLS, TILE_CORNERS};

/// Ripple tiles one frame can draw: the ripple budget.
fn ripple_layers() -> usize {
    RippleConfig::default().max_active
}

/// Floats per ripple tile in the storage buffer: its heights with their
/// apron, the floor under each column, then the floor at each column corner.
/// Mirrors `RIPPLE_TILE_STRIDE` in `ripple.glsl`.
pub const RIPPLE_TILE_STRIDE: usize =
    PADDED_CELLS_PER_TILE + COLUMNS_PER_CHUNK + TILE_CORNERS * TILE_CORNERS;

/// Vertices along each side of the fine grid: one per ripple cell corner.
const FINE_SIDE: usize = TILE_CELLS + 1;

/// One frame slot's ripple storage, and the descriptor set pointing at it.
struct RippleSlot {
    buffer: ManagedBuffer,
    set: vk::DescriptorSet,
}

/// One frame slot's copy of the mesh.
struct SlotMesh {
    buffers: Option<StreamedMesh>,
    /// Vertices and indices the buffers have room for.
    capacity: (usize, usize),
    /// The mesh version these buffers hold.
    version: u64,
}

/// Renders every body of water from its static mesh.
pub struct WaterRenderer {
    device: Arc<ManagedDevice>,
    pipeline: WaterPipeline,
    mesh: WaterMesh,
    /// What `mesh` was built from.
    key: MeshKey,
    /// Bumped on every rebuild of `mesh`.
    version: u64,
    slots: PerFrame<SlotMesh>,
    /// The sea, kept apart so a basin's re-flood does not rebuild it.
    ocean: WaterMesh,
    ocean_key: MeshKey,
    ocean_version: u64,
    ocean_slots: PerFrame<SlotMesh>,
    rivers: RiverMesh,
    river_key: MeshKey,
    river_version: u64,
    river_slots: PerFrame<SlotMesh>,
    falls: FallMesh,
    fall_key: FallKey,
    fall_version: u64,
    fall_slots: PerFrame<SlotMesh>,
    ripple_slots: PerFrame<RippleSlot>,
    /// The fine grid every awake ripple tile is drawn with. Never changes.
    fine_grid: StreamedMesh,
    fine_indices: u32,
    /// The frame being recorded, as [`Self::begin_frame`] was told.
    slot: FrameSlot,
}

impl WaterRenderer {
    pub fn new(
        vulkan_context: Arc<VulkanContext>,
        render_pass: vk::RenderPass,
        depth_view: vk::ImageView,
        color_view: vk::ImageView,
    ) -> EngineResult<Self> {
        let device = Arc::clone(&vulkan_context.device);
        let pipeline =
            WaterPipeline::new(Arc::clone(&device), render_pass, depth_view, color_view)?;
        let host = vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT;
        let ripple_bytes =
            (ripple_layers() * RIPPLE_TILE_STRIDE * std::mem::size_of::<f32>()) as vk::DeviceSize;
        let ripple_slots = PerFrame::try_new(|_| {
            let buffer = ManagedBuffer::new(
                Arc::clone(&device),
                ripple_bytes,
                vk::BufferUsageFlags::STORAGE_BUFFER,
                host,
            )?;
            let set = pipeline.allocate_ripple_set(buffer.buffer, ripple_bytes)?;
            Ok(RippleSlot { buffer, set })
        })?;
        let (grid_vertices, grid_indices) = fine_grid();
        let fine_grid = StreamedMesh::new(
            &device,
            std::mem::size_of_val(grid_vertices.as_slice()) as vk::DeviceSize,
            std::mem::size_of_val(grid_indices.as_slice()) as vk::DeviceSize,
        )?;
        fine_grid.upload(&grid_vertices, &grid_indices)?;
        Ok(Self {
            device,
            pipeline,
            ripple_slots,
            fine_grid,
            fine_indices: grid_indices.len() as u32,
            mesh: WaterMesh::default(),
            key: MeshKey::new(),
            version: 0,
            slots: PerFrame::new(|_| SlotMesh {
                buffers: None,
                capacity: (0, 0),
                version: 0,
            }),
            ocean: WaterMesh::default(),
            ocean_key: MeshKey::new(),
            ocean_version: 0,
            ocean_slots: PerFrame::new(|_| SlotMesh {
                buffers: None,
                capacity: (0, 0),
                version: 0,
            }),
            rivers: RiverMesh::default(),
            river_key: MeshKey::new(),
            river_version: 0,
            river_slots: PerFrame::new(|_| SlotMesh {
                buffers: None,
                capacity: (0, 0),
                version: 0,
            }),
            falls: FallMesh::default(),
            fall_key: FallKey::new(),
            fall_version: 0,
            fall_slots: PerFrame::new(|_| SlotMesh {
                buffers: None,
                capacity: (0, 0),
                version: 0,
            }),
            slot: FrameSlot::default(),
        })
    }

    /// Point uploads at this frame's buffers.
    pub fn begin_frame(&mut self, slot: FrameSlot) {
        self.slot = slot;
    }

    /// Bring the mesh up to date with the water's topology, and this frame
    /// slot's buffers up to date with the mesh.
    fn sync(&mut self, water: &dyn WaterScene) -> EngineResult<()> {
        let key = water.mesh_key();
        if key != self.key {
            self.mesh = water.build_mesh();
            self.key = key;
            self.version += 1;
        }
        let key = water.ocean_key();
        if key != self.ocean_key {
            self.ocean = water.build_ocean();
            self.ocean_key = key;
            self.ocean_version += 1;
        }
        upload_slot(
            &self.device,
            &mut self.ocean_slots[self.slot],
            self.ocean_version,
            &self.ocean.vertices,
            &self.ocean.indices,
        )?;
        let key = water.river_key();
        if key != self.river_key {
            self.rivers = water.build_rivers();
            self.river_key = key;
            self.river_version += 1;
        }
        upload_slot(
            &self.device,
            &mut self.river_slots[self.slot],
            self.river_version,
            &self.rivers.vertices,
            &self.rivers.indices,
        )?;
        let key = water.fall_key();
        if key != self.fall_key {
            self.falls = water.build_falls();
            self.fall_key = key;
            self.fall_version += 1;
        }
        upload_slot(
            &self.device,
            &mut self.fall_slots[self.slot],
            self.fall_version,
            &self.falls.vertices,
            &self.falls.indices,
        )?;

        upload_slot(
            &self.device,
            &mut self.slots[self.slot],
            self.version,
            &self.mesh.vertices,
            &self.mesh.indices,
        )
    }

    /// Draw every body of water.
    pub fn render(
        &mut self,
        cb: vk::CommandBuffer,
        water: &dyn WaterScene,
        view_matrix: &Matrix4<f32>,
        proj_matrix: &Matrix4<f32>,
        camera_pos: &Vector3<f32>,
        sun_dir: &Vector3<f32>,
        time: f32,
        screen_width: f32,
        screen_height: f32,
        hue_preservation: f32,
        exposure: f32,
    ) -> EngineResult<()> {
        self.sync(water)?;
        if self.mesh.draws.is_empty()
            && self.ocean.draws.is_empty()
            && self.rivers.draws.is_empty()
            && self.falls.draws.is_empty()
        {
            return Ok(());
        }
        let layers = self.upload_ripples(water)?;
        let mesh = self.slots[self.slot].buffers.as_ref();
        let clock = water.clock();
        // Bind pipeline and draw
        unsafe {
            self.device.device.cmd_bind_pipeline(
                cb,
                vk::PipelineBindPoint::GRAPHICS,
                self.pipeline.pipeline(),
            );

            // Push view and projection matrices (vertex stage, offset 0)
            let view_bytes: &[u8] = bytemuck_cast_slice(view_matrix.as_slice());
            let proj_bytes: &[u8] = bytemuck_cast_slice(proj_matrix.as_slice());

            let mut vertex_push_data = [0u8; 128];
            vertex_push_data[0..64].copy_from_slice(view_bytes);
            vertex_push_data[64..128].copy_from_slice(proj_bytes);

            self.device.device.cmd_push_constants(
                cb,
                self.pipeline.layout(),
                vk::ShaderStageFlags::VERTEX,
                0,
                &vertex_push_data,
            );

            // Extract near/far from the projection matrix.
            // For a Vulkan perspective projection (depth [0,1]):
            //   proj[2][2] = far / (near - far)
            //   proj[3][2] = (near * far) / (near - far)
            // So: near = proj[3][2] / proj[2][2]
            //     far  = proj[3][2] / (proj[2][2] + 1)
            let p22 = proj_matrix[(2, 2)];
            let p32 = proj_matrix[(3, 2)];
            let near = p32 / p22;
            let far = p32 / (p22 + 1.0);

            // Push camera_pos, sun_dir, proj params, and screen params (fragment stage)
            let frag_push_data: [f32; 16] = [
                camera_pos.x,
                camera_pos.y,
                camera_pos.z,
                0.0, // padding
                sun_dir.x,
                sun_dir.y,
                sun_dir.z,
                0.0, // padding
                near,
                far,
                time,
                0.0, // padding
                screen_width,
                screen_height,
                // Both must match the composite pass, or refracted scene colour
                // resolves differently from the pixels beside it.
                hue_preservation,
                exposure,
            ];

            self.device.device.cmd_push_constants(
                cb,
                self.pipeline.layout(),
                vk::ShaderStageFlags::FRAGMENT,
                FRAGMENT_PUSH_OFFSET,
                bytemuck_cast_slice(&frag_push_data),
            );

            // Set 0: the opaque scene's colour and depth; set 1: this slot's
            // ripple tiles.
            self.device.device.cmd_bind_descriptor_sets(
                cb,
                vk::PipelineBindPoint::GRAPHICS,
                self.pipeline.layout(),
                0,
                &[
                    self.pipeline.descriptor_set(),
                    self.ripple_slots[self.slot].set,
                ],
                &[],
            );

            let meshes = [
                (mesh, &self.mesh),
                (self.ocean_slots[self.slot].buffers.as_ref(), &self.ocean),
            ];
            for (buffers, built) in meshes {
                let Some(buffers) = buffers else {
                    continue;
                };
                buffers.bind(&self.device.device, cb);
                for draw in &built.draws {
                    // A tile whose ripples are awake is drawn fine, below.
                    if layers.contains_key(&(draw.tile.x, draw.tile.z, draw.body)) {
                        continue;
                    }
                    let Some(level) = water.level(draw.body) else {
                        continue;
                    };
                    self.push_draw(cb, water, draw.body, level, clock, [0.0; 4]);
                    self.device.device.cmd_draw_indexed(
                        cb,
                        draw.index_count,
                        1,
                        draw.first_index,
                        0,
                        0,
                    );
                }
            }

            if let Some(rivers) = self.river_slots[self.slot].buffers.as_ref() {
                if !self.rivers.draws.is_empty() {
                    self.device.device.cmd_bind_pipeline(
                        cb,
                        vk::PipelineBindPoint::GRAPHICS,
                        self.pipeline.river_pipeline(),
                    );
                    rivers.bind(&self.device.device, cb);
                    for draw in &self.rivers.draws {
                        let Some(state) = water.river_state(draw.reach) else {
                            continue;
                        };
                        if state.front <= state.tail {
                            continue;
                        }
                        let constants: [f32; 8] = [
                            state.depth_scale,
                            state.tail,
                            state.front,
                            clock,
                            state.speed_scale,
                            0.0,
                            0.0,
                            0.0,
                        ];
                        self.device.device.cmd_push_constants(
                            cb,
                            self.pipeline.layout(),
                            vk::ShaderStageFlags::VERTEX,
                            BODY_PUSH_OFFSET,
                            bytemuck_cast_slice(&constants),
                        );
                        self.device.device.cmd_draw_indexed(
                            cb,
                            draw.index_count,
                            1,
                            draw.first_index,
                            0,
                            0,
                        );
                    }
                }
            }

            if !layers.is_empty() {
                self.device.device.cmd_bind_pipeline(
                    cb,
                    vk::PipelineBindPoint::GRAPHICS,
                    self.pipeline.fine_pipeline(),
                );
                self.fine_grid.bind(&self.device.device, cb);
                let mut tiles: Vec<(&(i32, i32, StoreId), &(usize, u32))> = layers.iter().collect();
                tiles.sort();
                let extent = CHUNK_COLUMNS as f32 * COLUMN_SIZE;
                for (&(tx, tz, body), &(layer, sealed)) in tiles {
                    let Some(level) = water.level(body) else {
                        continue;
                    };
                    let tile = [
                        tx as f32 * extent,
                        tz as f32 * extent,
                        layer as f32,
                        sealed as f32,
                    ];
                    self.push_draw(cb, water, body, level, clock, tile);
                    self.device
                        .device
                        .cmd_draw_indexed(cb, self.fine_indices, 1, 0, 0, 0);
                }
            }

            // Falls last: a sheet reads the scene behind it, water included
            // only as far as the opaque pass drew it.
            if let Some(falls) = self.fall_slots[self.slot].buffers.as_ref() {
                if !self.falls.draws.is_empty() {
                    self.device.device.cmd_bind_pipeline(
                        cb,
                        vk::PipelineBindPoint::GRAPHICS,
                        self.pipeline.fall_pipeline(),
                    );
                    falls.bind(&self.device.device, cb);
                    for draw in &self.falls.draws {
                        let Some(state) = water.fall_state(draw.link) else {
                            continue;
                        };
                        let constants: [f32; 8] = [
                            state.half_width,
                            state.strength,
                            FALL_SPREAD,
                            clock,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        ];
                        self.device.device.cmd_push_constants(
                            cb,
                            self.pipeline.layout(),
                            vk::ShaderStageFlags::VERTEX,
                            BODY_PUSH_OFFSET,
                            bytemuck_cast_slice(&constants),
                        );
                        self.device.device.cmd_draw_indexed(
                            cb,
                            draw.index_count,
                            1,
                            draw.first_index,
                            0,
                            0,
                        );
                    }
                }
            }
        }

        Ok(())
    }
}

impl WaterRenderer {
    /// Push one draw's body and tile constants.
    fn push_draw(
        &self,
        cb: vk::CommandBuffer,
        water: &dyn WaterScene,
        body: StoreId,
        level: f32,
        clock: f32,
        tile: [f32; 4],
    ) {
        let swell = water.swell(body);
        let constants: [f32; 8] = [
            level,
            swell.amplitude,
            swell.phase,
            clock,
            tile[0],
            tile[1],
            tile[2],
            tile[3],
        ];
        unsafe {
            self.device.device.cmd_push_constants(
                cb,
                self.pipeline.layout(),
                vk::ShaderStageFlags::VERTEX,
                BODY_PUSH_OFFSET,
                bytemuck_cast_slice(&constants),
            );
        }
    }

    /// Write this frame's awake ripple tiles into the slot's storage buffer.
    /// Returns each tile's layer and sealed edges, keyed by (tile x, tile z,
    /// body).
    fn upload_ripples(
        &self,
        water: &dyn WaterScene,
    ) -> EngineResult<FxHashMap<(i32, i32, StoreId), (usize, u32)>> {
        let mut layers = FxHashMap::default();
        let tiles = water.ripple_tiles();
        if tiles.is_empty() {
            return Ok(layers);
        }
        let buffer = &self.ripple_slots[self.slot].buffer;
        unsafe {
            let ptr = buffer.map_memory(0, vk::MemoryMapFlags::empty())? as *mut f32;
            for (layer, tile) in tiles.iter().take(ripple_layers()).enumerate() {
                let base = ptr.add(layer * RIPPLE_TILE_STRIDE);
                tile.write_heights(std::slice::from_raw_parts_mut(base, PADDED_CELLS_PER_TILE));
                std::ptr::copy_nonoverlapping(
                    tile.floors.as_ptr(),
                    base.add(PADDED_CELLS_PER_TILE),
                    COLUMNS_PER_CHUNK,
                );
                std::ptr::copy_nonoverlapping(
                    tile.corners.as_ptr(),
                    base.add(PADDED_CELLS_PER_TILE + COLUMNS_PER_CHUNK),
                    TILE_CORNERS * TILE_CORNERS,
                );
                layers.insert((tile.tile.x, tile.tile.z, tile.body), (layer, tile.sealed));
            }
            buffer.unmap_memory();
        }
        Ok(layers)
    }
}

/// Bring a frame slot's buffers up to a mesh version, growing them with
/// headroom when the mesh has outgrown them.
fn upload_slot<V: Copy>(
    device: &Arc<ManagedDevice>,
    slot: &mut SlotMesh,
    version: u64,
    vertices: &[V],
    indices: &[u32],
) -> EngineResult<()> {
    if slot.version == version {
        return Ok(());
    }
    let needed = (vertices.len(), indices.len());
    if slot.buffers.is_none() || needed.0 > slot.capacity.0 || needed.1 > slot.capacity.1 {
        let capacity = (
            (needed.0 + needed.0 / 4).max(1024),
            (needed.1 + needed.1 / 4).max(1536),
        );
        slot.buffers = Some(StreamedMesh::new(
            device,
            (capacity.0 * std::mem::size_of::<V>()) as vk::DeviceSize,
            (capacity.1 * std::mem::size_of::<u32>()) as vk::DeviceSize,
        )?);
        slot.capacity = capacity;
    }
    if let Some(buffers) = &slot.buffers {
        buffers.upload(vertices, indices)?;
    }
    slot.version = version;
    Ok(())
}

/// The fine grid over one tile: 65 × 65 vertices across 8 m.
fn fine_grid() -> (Vec<FineVertex>, Vec<u32>) {
    let extent = CHUNK_COLUMNS as f32 * COLUMN_SIZE;
    let step = extent / (FINE_SIDE - 1) as f32;
    let mut vertices = Vec::with_capacity(FINE_SIDE * FINE_SIDE);
    for k in 0..FINE_SIDE {
        for i in 0..FINE_SIDE {
            vertices.push(FineVertex {
                local: Vector2::new(i as f32 * step, k as f32 * step),
            });
        }
    }
    let at = |i: usize, k: usize| (k * FINE_SIDE + i) as u32;
    let mut indices = Vec::with_capacity((FINE_SIDE - 1) * (FINE_SIDE - 1) * 6);
    for k in 0..FINE_SIDE - 1 {
        for i in 0..FINE_SIDE - 1 {
            let (a, b, c, d) = (at(i, k), at(i + 1, k), at(i + 1, k + 1), at(i, k + 1));
            indices.extend_from_slice(&[a, d, c, a, c, b]);
        }
    }
    (vertices, indices)
}

/// Helper to cast a slice of f32 to bytes.
fn bytemuck_cast_slice(slice: &[f32]) -> &[u8] {
    unsafe {
        std::slice::from_raw_parts(
            slice.as_ptr() as *const u8,
            slice.len() * std::mem::size_of::<f32>(),
        )
    }
}
