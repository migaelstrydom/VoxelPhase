//! Water surface rendering.
//!
//! Meshes are static between topology changes (see [`basin_mesher`]): the
//! renderer rebuilds its mesh only when a basin's region changes, uploads it
//! once into each frame slot's buffers, and otherwise only pushes each
//! body's level per draw.
//!
//! [`basin_mesher`]: super::basin_mesher

use std::sync::Arc;

use ash::vk;
use nalgebra::{Matrix4, Vector3};

use super::basin_mesher::{MeshKey, WaterMesh, WaterScene};
use super::pipeline::{WaterPipeline, BODY_PUSH_OFFSET, FRAGMENT_PUSH_OFFSET};
use super::vertex::BasinVertex;
use crate::core::device::ManagedDevice;
use crate::core::error::EngineResult;
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::in_flight::{FrameSlot, PerFrame, StreamedMesh};

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
        Ok(Self {
            device,
            pipeline,
            mesh: WaterMesh::default(),
            key: MeshKey::new(),
            version: 0,
            slots: PerFrame::new(|_| SlotMesh {
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
        let slot = &mut self.slots[self.slot];
        if slot.version == self.version {
            return Ok(());
        }
        let needed = (self.mesh.vertices.len(), self.mesh.indices.len());
        if slot.buffers.is_none() || needed.0 > slot.capacity.0 || needed.1 > slot.capacity.1 {
            // Grow with headroom, so a re-region that adds a few columns does
            // not reallocate.
            let capacity = (
                (needed.0 + needed.0 / 4).max(1024),
                (needed.1 + needed.1 / 4).max(1536),
            );
            slot.buffers = Some(StreamedMesh::new(
                &self.device,
                (capacity.0 * std::mem::size_of::<BasinVertex>()) as vk::DeviceSize,
                (capacity.1 * std::mem::size_of::<u32>()) as vk::DeviceSize,
            )?);
            slot.capacity = capacity;
        }
        if let Some(buffers) = &slot.buffers {
            buffers.upload(&self.mesh.vertices, &self.mesh.indices)?;
        }
        slot.version = self.version;
        Ok(())
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
        if self.mesh.draws.is_empty() {
            return Ok(());
        }
        let Some(mesh) = self.slots[self.slot].buffers.as_ref() else {
            return Ok(());
        };
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

            // Bind depth input attachment descriptor set
            self.device.device.cmd_bind_descriptor_sets(
                cb,
                vk::PipelineBindPoint::GRAPHICS,
                self.pipeline.layout(),
                0,
                &[self.pipeline.descriptor_set()],
                &[],
            );

            mesh.bind(&self.device.device, cb);

            for draw in &self.mesh.draws {
                let Some(level) = water.level(draw.body) else {
                    continue;
                };
                let body: [f32; 4] = [level, 0.0, 0.0, 0.0];
                self.device.device.cmd_push_constants(
                    cb,
                    self.pipeline.layout(),
                    vk::ShaderStageFlags::VERTEX,
                    BODY_PUSH_OFFSET,
                    bytemuck_cast_slice(&body),
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

        Ok(())
    }
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
