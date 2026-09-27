//! Water surface rendering.
//!
//! Meshes are static between topology changes (see [`basin_mesher`]): the
//! renderer rebuilds its mesh only when a basin's region changes, uploads it
//! once into each frame slot's buffers, and otherwise only pushes each
//! body's level and swell per draw. A tile whose ripples are awake is drawn
//! instead from a fine grid displaced out of that frame's ripple storage.
//!
//! The water is drawn inside the HDR scene pass, over a copy of the scene
//! taken once everything beyond the surface is down, so a frame is prepared
//! and recorded apart:
//!
//! ```text
//!   prepare ──▶ meshes synced, ripples uploaded, draws planned ──▶ patches
//!   record  ──▶ the plan, after the refraction copy
//! ```
//!
//! [`basin_mesher`]: super::basin_mesher

use std::sync::Arc;

use ash::vk;
use nalgebra::{Matrix4, Vector2, Vector3};
use rustc_hash::FxHashMap;

use super::basin_mesher::{MeshKey, WaterMesh, WaterScene};
use super::divide::WaterPatch;
use super::fall_mesher::{FallKey, FallMesh, SPREAD as FALL_SPREAD};
use super::footprint::ScreenFootprint;
use super::pipeline::{WaterPipeline, BODY_PUSH_OFFSET, FRAGMENT_PUSH_OFFSET};
use super::reach_mesher::RiverMesh;
use super::vertex::FineVertex;
use crate::core::device::ManagedDevice;
use crate::core::error::EngineResult;
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::frame::ManagedBuffer;
use crate::rendering::in_flight::{FrameSlot, PerFrame, StreamedMesh};
use crate::rendering::reflection::BoundingSphere;
use crate::rendering::target::RefractionCopy;
use crate::rendering::view_volume::ViewVolume;
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

/// How far above and below its level a tile's surface may be drawn, over its
/// swell: ripples and a reach easing into the basin, m.
const TILE_HEIGHT_PAD: f32 = 0.5;

/// What a frame's prepared water tells the renderer.
#[derive(Debug, Clone, Default)]
pub struct WaterFrame {
    /// Every tile of still water, in view or not.
    pub patches: Vec<WaterPatch>,
    /// The part of the screen the planned draws can read.
    pub footprint: ScreenFootprint,
}

/// Which of the water's meshes a draw comes from; each has its own buffers
/// and pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WaterSurface {
    Basins,
    Ocean,
    Rivers,
    Ripples,
    Falls,
}

/// One draw of the frame's plan, with every constant it pushes.
#[derive(Debug, Clone, Copy)]
struct PlannedDraw {
    surface: WaterSurface,
    /// The body (or reach, or fall) constants at [`BODY_PUSH_OFFSET`].
    constants: [f32; 8],
    first_index: u32,
    index_count: u32,
}

/// What the water is seen through this frame.
#[derive(Debug, Clone, Copy)]
pub struct WaterView {
    pub view: Matrix4<f32>,
    pub projection: Matrix4<f32>,
    pub camera: Vector3<f32>,
    /// Towards the sun.
    pub sun: Vector3<f32>,
    /// Seconds, for the surface detail's drift.
    pub time: f32,
    /// The render target's size in pixels.
    pub screen: Vector2<f32>,
}

impl WaterView {
    /// The fragment stage's constants: camera, sun, near and far planes and
    /// time, screen size.
    fn fragment_constants(&self) -> [f32; 16] {
        // For a Vulkan perspective projection (depth [0,1]):
        //   proj[2][2] = far / (near - far)
        //   proj[3][2] = (near * far) / (near - far)
        // So: near = proj[3][2] / proj[2][2]
        //     far  = proj[3][2] / (proj[2][2] + 1)
        let p22 = self.projection[(2, 2)];
        let p32 = self.projection[(3, 2)];
        let near = p32 / p22;
        let far = p32 / (p22 + 1.0);
        [
            self.camera.x,
            self.camera.y,
            self.camera.z,
            0.0,
            self.sun.x,
            self.sun.y,
            self.sun.z,
            0.0,
            near,
            far,
            self.time,
            0.0,
            self.screen.x,
            self.screen.y,
            0.0,
            0.0,
        ]
    }
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
    /// This frame's draws, as [`Self::prepare`] planned them.
    plan: Vec<PlannedDraw>,
    /// What this frame's water is seen through.
    view: Option<WaterView>,
}

impl WaterRenderer {
    pub fn new(
        vulkan_context: Arc<VulkanContext>,
        render_pass: vk::RenderPass,
        refraction: &RefractionCopy,
    ) -> EngineResult<Self> {
        let device = Arc::clone(&vulkan_context.device);
        let pipeline = WaterPipeline::new(
            Arc::clone(&device),
            render_pass,
            refraction.depth_view(),
            refraction.colour_view(),
        )?;
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
            plan: Vec::new(),
            view: None,
        })
    }

    /// Point uploads at this frame's buffers, and forget last frame's plan.
    pub fn begin_frame(&mut self, slot: FrameSlot) {
        self.slot = slot;
        self.plan.clear();
        self.view = None;
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

    /// Bring the meshes and this frame's ripple storage up to date, and plan
    /// the frame's draws. Records nothing: [`Self::record`] replays the plan
    /// inside the scene pass, once everything drawn beyond the water is down.
    ///
    /// Draws out of view are left out of the plan. Returns the still water
    /// for the renderer to divide the scene's blended surfaces by, and the
    /// part of the screen the planned draws can read.
    pub fn prepare(
        &mut self,
        water: &dyn WaterScene,
        view: &WaterView,
    ) -> EngineResult<WaterFrame> {
        self.plan.clear();
        self.sync(water)?;
        let mut patches = Vec::new();
        let mut footprint = ScreenFootprint::EMPTY;
        let clip = view.projection * view.view;
        let volume = ViewVolume::new(&view.view, &view.projection, 1.0);
        let extent = CHUNK_COLUMNS as f32 * COLUMN_SIZE;
        // Whether a tile of water at `level` is in view, adding it to the
        // footprint if so.
        let in_view =
            |footprint: &mut ScreenFootprint, tile: (i32, i32), level: f32, swell: f32| {
                let pad = swell + TILE_HEIGHT_PAD;
                let min = Vector3::new(tile.0 as f32 * extent, level - pad, tile.1 as f32 * extent);
                let max = min + Vector3::new(extent, 2.0 * pad, extent);
                let sphere = BoundingSphere {
                    centre: (min + max) * 0.5,
                    radius: (max - min).norm() * 0.5,
                };
                let visible = volume.contains(&sphere);
                if visible {
                    footprint.add_box(&clip, min, max);
                }
                visible
            };
        if self.mesh.draws.is_empty()
            && self.ocean.draws.is_empty()
            && self.rivers.draws.is_empty()
            && self.falls.draws.is_empty()
        {
            return Ok(WaterFrame::default());
        }
        let layers = self.upload_ripples(water)?;
        let clock = water.clock();
        let body = |id: StoreId, level: f32, tile: [f32; 4]| {
            let swell = water.swell(id);
            [
                level,
                swell.amplitude,
                swell.phase,
                clock,
                tile[0],
                tile[1],
                tile[2],
                tile[3],
            ]
        };

        for (surface, built) in [
            (WaterSurface::Basins, &self.mesh),
            (WaterSurface::Ocean, &self.ocean),
        ] {
            for draw in &built.draws {
                let Some(level) = water.level(draw.body) else {
                    continue;
                };
                patches.push(WaterPatch::tile(draw.tile.x, draw.tile.z, level));
                // A tile whose ripples are awake is drawn fine, below.
                if layers.contains_key(&(draw.tile.x, draw.tile.z, draw.body)) {
                    continue;
                }
                if !in_view(
                    &mut footprint,
                    (draw.tile.x, draw.tile.z),
                    level,
                    water.swell(draw.body).amplitude,
                ) {
                    continue;
                }
                self.plan.push(PlannedDraw {
                    surface,
                    constants: body(draw.body, level, [0.0; 4]),
                    first_index: draw.first_index,
                    index_count: draw.index_count,
                });
            }
        }

        for draw in &self.rivers.draws {
            let Some(state) = water.river_state(draw.reach) else {
                continue;
            };
            if state.front <= state.tail {
                continue;
            }
            // A reach's strip has no bounds of its own to place it by.
            footprint.cover_all();
            self.plan.push(PlannedDraw {
                surface: WaterSurface::Rivers,
                constants: [
                    state.depth_scale,
                    state.tail,
                    state.front,
                    clock,
                    state.speed_scale,
                    state.ends.upstream,
                    state.ends.downstream,
                    state.length,
                ],
                first_index: draw.first_index,
                index_count: draw.index_count,
            });
        }

        let mut tiles: Vec<(&(i32, i32, StoreId), &(usize, u32))> = layers.iter().collect();
        tiles.sort();
        for (&(tx, tz, id), &(layer, sealed)) in tiles {
            let Some(level) = water.level(id) else {
                continue;
            };
            if !in_view(&mut footprint, (tx, tz), level, water.swell(id).amplitude) {
                continue;
            }
            let tile = [
                tx as f32 * extent,
                tz as f32 * extent,
                layer as f32,
                sealed as f32,
            ];
            self.plan.push(PlannedDraw {
                surface: WaterSurface::Ripples,
                constants: body(id, level, tile),
                first_index: 0,
                index_count: self.fine_indices,
            });
        }

        // Falls last: a sheet reads the scene behind it, which holds no
        // water.
        for draw in &self.falls.draws {
            let Some(state) = water.fall_state(draw.link, draw.back) else {
                continue;
            };
            footprint.cover_all();
            self.plan.push(PlannedDraw {
                surface: WaterSurface::Falls,
                constants: [
                    state.half_width,
                    state.strength,
                    FALL_SPREAD,
                    clock,
                    state.lift,
                    state.top,
                    state.cut,
                    state.aeration,
                ],
                first_index: draw.first_index,
                index_count: draw.index_count,
            });
        }

        self.view = Some(*view);
        Ok(WaterFrame { patches, footprint })
    }

    /// Whether [`Self::prepare`] left anything to draw this frame.
    pub fn is_empty(&self) -> bool {
        self.plan.is_empty()
    }

    /// Record the planned draws. They sample the refraction copy through
    /// set 0, which must hold this frame's scene by now.
    pub fn record(&self, cb: vk::CommandBuffer) {
        let Some(view) = self.view.filter(|_| !self.plan.is_empty()) else {
            return;
        };
        let device = &self.device.device;
        unsafe {
            let mut vertex_push_data = [0u8; 128];
            vertex_push_data[0..64].copy_from_slice(bytemuck_cast_slice(view.view.as_slice()));
            vertex_push_data[64..128]
                .copy_from_slice(bytemuck_cast_slice(view.projection.as_slice()));
            device.cmd_push_constants(
                cb,
                self.pipeline.layout(),
                vk::ShaderStageFlags::VERTEX,
                0,
                &vertex_push_data,
            );
            device.cmd_push_constants(
                cb,
                self.pipeline.layout(),
                vk::ShaderStageFlags::FRAGMENT,
                FRAGMENT_PUSH_OFFSET,
                bytemuck_cast_slice(&view.fragment_constants()),
            );

            // Set 0: the refraction copy's colour and depth; set 1: this
            // slot's ripple tiles.
            device.cmd_bind_descriptor_sets(
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

            let mut bound = None;
            for draw in &self.plan {
                if bound != Some(draw.surface) {
                    let Some(buffers) = self.buffers(draw.surface) else {
                        continue;
                    };
                    device.cmd_bind_pipeline(
                        cb,
                        vk::PipelineBindPoint::GRAPHICS,
                        self.pipeline_for(draw.surface),
                    );
                    buffers.bind(device, cb);
                    bound = Some(draw.surface);
                }
                device.cmd_push_constants(
                    cb,
                    self.pipeline.layout(),
                    vk::ShaderStageFlags::VERTEX,
                    BODY_PUSH_OFFSET,
                    bytemuck_cast_slice(&draw.constants),
                );
                device.cmd_draw_indexed(cb, draw.index_count, 1, draw.first_index, 0, 0);
            }
        }
    }

    /// The buffers a surface's draws index into, in this frame's slot.
    fn buffers(&self, surface: WaterSurface) -> Option<&StreamedMesh> {
        match surface {
            WaterSurface::Basins => self.slots[self.slot].buffers.as_ref(),
            WaterSurface::Ocean => self.ocean_slots[self.slot].buffers.as_ref(),
            WaterSurface::Rivers => self.river_slots[self.slot].buffers.as_ref(),
            WaterSurface::Ripples => Some(&self.fine_grid),
            WaterSurface::Falls => self.fall_slots[self.slot].buffers.as_ref(),
        }
    }

    fn pipeline_for(&self, surface: WaterSurface) -> vk::Pipeline {
        match surface {
            WaterSurface::Basins | WaterSurface::Ocean => self.pipeline.pipeline(),
            WaterSurface::Rivers => self.pipeline.river_pipeline(),
            WaterSurface::Ripples => self.pipeline.fine_pipeline(),
            WaterSurface::Falls => self.pipeline.fall_pipeline(),
        }
    }
}

impl WaterRenderer {
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
