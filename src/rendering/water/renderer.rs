//! Water surface rendering.
//!
//! A still surface's mesh is static between changes of its key: the renderer
//! builds it again only then, uploads it once into each frame slot's
//! buffers, and otherwise only pushes each body's level and swell per draw.
//! A tile whose ripples are awake is drawn instead from a fine grid displaced
//! out of that frame's ripple storage. River surfaces (the field's chunks)
//! carry their own heights, and are read again or built again as their water
//! moves (`RiverParts`).
//!
//! The water is drawn inside the HDR scene pass, over a copy of the scene
//! taken once everything beyond the surface is down, so a frame is prepared
//! and recorded apart:
//!
//! ```text
//!   prepare ──▶ meshes synced, ripples uploaded, draws planned ──▶ patches
//!   record  ──▶ the plan, after the refraction copy
//! ```

use std::sync::Arc;

use ash::vk;
use nalgebra::{Matrix4, Vector2, Vector3};
use rustc_hash::FxHashMap;

use super::divide::WaterPatch;
use super::footprint::ScreenFootprint;
use super::ocean_ring::RING_TILE;
use super::pipeline::{WaterPipeline, BODY_PUSH_OFFSET, FRAGMENT_PUSH_OFFSET};
use super::river_parts::RiverParts;
use super::scene::{MeshKey, WaterMesh, WaterScene};
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
use crate::water::ids::SurfaceId;
use crate::water::surface::{RippleConfig, Swell, PADDED_CELLS_PER_TILE, TILE_CELLS, TILE_CORNERS};
use crate::water::waves::SEA_BAND;

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
/// swell, for its ripples, m.
const TILE_HEIGHT_PAD: f32 = 0.5;

/// One awake ripple tile, as uploaded for this frame.
#[derive(Debug, Clone, Copy)]
struct RippleDraw {
    /// Its layer in the storage buffer.
    layer: usize,
    /// Edges beside coarse water.
    sealed: u32,
    level: f32,
    swell: Swell,
    /// The river surface it ripples, which draws it.
    surface: Option<SurfaceId>,
}

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
    /// The falling sheets of the river surfaces, drawn last and blended.
    Sheets,
}

/// One draw of the frame's plan, with every constant it pushes.
#[derive(Debug, Clone, Copy)]
struct PlannedDraw {
    surface: WaterSurface,
    /// The body (or river surface) constants at [`BODY_PUSH_OFFSET`].
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
    /// The sea's ring, kept apart from the still surfaces.
    ocean: WaterMesh,
    ocean_key: MeshKey,
    ocean_version: u64,
    ocean_slots: PerFrame<SlotMesh>,
    rivers: RiverParts,
    river_version: u64,
    river_slots: PerFrame<SlotMesh>,
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
            rivers: RiverParts::default(),
            river_version: 0,
            river_slots: PerFrame::new(|_| SlotMesh {
                buffers: None,
                capacity: (0, 0),
                version: 0,
            }),
            slot: FrameSlot::default(),
            plan: Vec::new(),
            view: None,
        })
    }

    /// Most river surfaces built again per frame; `None` builds every one
    /// that moved, as a tool drawing single frames needs.
    pub fn set_river_budget(&mut self, budget: Option<usize>) {
        self.rivers.set_budget(budget);
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
        if self.rivers.sync(water) {
            self.river_version += 1;
        }
        upload_slot(
            &self.device,
            &mut self.river_slots[self.slot],
            self.river_version,
            &self.rivers.mesh().vertices,
            self.rivers.index_buffer(),
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
            && self.rivers.mesh().draws.is_empty()
        {
            return Ok(WaterFrame::default());
        }
        patches.extend(water.still_water());
        let layers = self.upload_ripples(water)?;
        let clock = water.clock();
        let body = |id: SurfaceId, level: f32, tile: [f32; 4]| {
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
                if draw.tile == RING_TILE {
                    // The sea past the map's edges is no tile: it reaches the
                    // horizon all round, so it is always drawn and can cover
                    // any part of the screen.
                    footprint.cover_all();
                } else {
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
                }
                self.plan.push(PlannedDraw {
                    surface,
                    constants: body(draw.body, level, [0.0; 4]),
                    first_index: draw.first_index,
                    index_count: draw.index_count,
                });
            }
        }

        if !self.rivers.mesh().draws.is_empty() {
            // The surfaces are not tiles a level places: they may be drawn
            // anywhere on the screen.
            footprint.cover_all();
        }
        // A surface whose chunk's ripples are awake reads them from its
        // tile's layer, about the tile's corner.
        let rippling: FxHashMap<SurfaceId, (usize, i32, i32)> = layers
            .iter()
            .filter_map(|(&(tx, tz, _), t)| t.surface.map(|id| (id, (t.layer, tx, tz))))
            .collect();
        let sea = water.sea();
        let (swell, sea_level) = sea.map_or((Swell::default(), 0.0), |s| (s.swell, s.level));
        for draw in self
            .rivers
            .mesh()
            .draws
            .iter()
            .filter(|d| d.index_count > 0)
        {
            let (layer, tx, tz) = rippling
                .get(&draw.id)
                .map_or((-1.0, 0, 0), |&(layer, tx, tz)| (layer as f32, tx, tz));
            self.plan.push(PlannedDraw {
                surface: WaterSurface::Rivers,
                constants: [
                    layer,
                    swell.amplitude,
                    swell.phase,
                    clock,
                    sea_level,
                    SEA_BAND,
                    tx as f32 * extent,
                    tz as f32 * extent,
                ],
                first_index: draw.first_index,
                index_count: draw.index_count,
            });
        }

        let mut tiles: Vec<(&(i32, i32, SurfaceId), &RippleDraw)> = layers.iter().collect();
        tiles.sort_by_key(|(key, _)| **key);
        for (&(tx, tz, _), ripple) in tiles {
            if ripple.surface.is_some() {
                continue;
            }
            if !in_view(
                &mut footprint,
                (tx, tz),
                ripple.level,
                ripple.swell.amplitude,
            ) {
                continue;
            }
            let tile = [
                tx as f32 * extent,
                tz as f32 * extent,
                ripple.layer as f32,
                ripple.sealed as f32,
            ];
            self.plan.push(PlannedDraw {
                surface: WaterSurface::Ripples,
                constants: [
                    ripple.level,
                    ripple.swell.amplitude,
                    ripple.swell.phase,
                    clock,
                    tile[0],
                    tile[1],
                    tile[2],
                    tile[3],
                ],
                first_index: 0,
                index_count: self.fine_indices,
            });
        }

        // Falling sheets are drawn over every surface, so they blend over
        // the pool they fall into: they write no depth to hide it by.
        let sheet_base = self.rivers.sheet_base();
        for draw in &self.rivers.mesh().draws {
            if draw.sheet_index_count == 0 {
                continue;
            }
            self.plan.push(PlannedDraw {
                surface: WaterSurface::Sheets,
                constants: [
                    -1.0,
                    swell.amplitude,
                    swell.phase,
                    clock,
                    sea_level,
                    SEA_BAND,
                    0.0,
                    0.0,
                ],
                first_index: sheet_base + draw.first_sheet_index,
                index_count: draw.sheet_index_count,
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
            WaterSurface::Rivers | WaterSurface::Sheets => {
                self.river_slots[self.slot].buffers.as_ref()
            }
            WaterSurface::Ripples => Some(&self.fine_grid),
        }
    }

    fn pipeline_for(&self, surface: WaterSurface) -> vk::Pipeline {
        match surface {
            WaterSurface::Basins | WaterSurface::Ocean => self.pipeline.pipeline(),
            WaterSurface::Rivers => self.pipeline.river_pipeline(),
            WaterSurface::Ripples => self.pipeline.fine_pipeline(),
            WaterSurface::Sheets => self.pipeline.sheet_pipeline(),
        }
    }
}

impl WaterRenderer {
    /// Write this frame's awake ripple tiles into the slot's storage buffer.
    /// Returns how to draw each, keyed by (tile x, tile z, body).
    fn upload_ripples(
        &self,
        water: &dyn WaterScene,
    ) -> EngineResult<FxHashMap<(i32, i32, SurfaceId), RippleDraw>> {
        let mut layers = FxHashMap::default();
        let tiles = water.ripple_tiles();
        if tiles.is_empty() {
            return Ok(layers);
        }
        // A tile left undrawn would be drawn coarse while its neighbours
        // leave their edges open to its ripples.
        debug_assert!(
            tiles.len() <= ripple_layers(),
            "{} ripple tiles awake, storage for {}",
            tiles.len(),
            ripple_layers()
        );
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
                layers.insert(
                    (tile.tile.x, tile.tile.z, tile.body),
                    RippleDraw {
                        layer,
                        sealed: tile.sealed,
                        level: tile.level,
                        swell: tile.swell,
                        surface: tile.surface,
                    },
                );
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
