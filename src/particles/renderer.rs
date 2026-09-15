//! Particle rendering module.
//!
//! Renders particles as camera-facing billboards using instanced geometry.
//!
//! Particles are blended scene geometry, so they are drawn in the scene pass
//! alongside glass and ice rather than after the HDR resolve, and recording is
//! split in two to let the renderer interleave them with that geometry:
//!
//! ```text
//!   prepare ──▶ sorted far-to-near, uploaded ──▶ bind ──▶ draw_range × n
//! ```
//!
//! `prepare` orders the whole pool once; the renderer then asks
//! [`ParticleRenderer::count_beyond`] where a blended mesh falls in that order
//! and emits the particles behind it before recording the mesh.
//!
//! Drawing into the scene target means the composite's exposure and tonemap now
//! stand between a particle and the screen. The shader takes that into account
//! (see `particle.frag`) so that what an effect resolves to is what it resolved
//! to when it was tuned.

use std::sync::Arc;

use ash::vk;
use nalgebra::{Matrix4, Vector2, Vector3, Vector4};

use crate::core::device::ManagedDevice;
use crate::core::error::EngineResult;
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::frame::ManagedBuffer;

use super::particle::{Particle, ParticlePool};
use super::pipeline::ParticlePipeline;
use super::vertex::ParticleVertex;

/// Maximum particles that can be rendered in a single draw call.
const MAX_PARTICLES: usize = 10000;

/// Particle renderer for visual effects.
///
/// Renders particles as camera-facing billboards using a dedicated pipeline
/// with additive blending.
pub struct ParticleRenderer {
    device: Arc<ManagedDevice>,
    pipeline: ParticlePipeline,
    vertex_buffer: ManagedBuffer,
    index_buffer: ManagedBuffer,
    /// Particle indices ordered far-to-near for this frame, paired with the
    /// squared distance from the camera that ordered them. Kept between frames
    /// so the sort does not allocate every time.
    draw_order: Vec<(f32, usize)>,

    /// View and projection matrices `prepare` was given, pushed at bind time.
    matrices: Option<(Matrix4<f32>, Matrix4<f32>)>,
}

impl ParticleRenderer {
    /// Create a new particle renderer.
    pub fn new(
        vulkan_context: Arc<VulkanContext>,
        render_pass: vk::RenderPass,
    ) -> EngineResult<Self> {
        let device = Arc::clone(&vulkan_context.device);

        let pipeline = ParticlePipeline::new(Arc::clone(&device), render_pass)?;

        // Buffer sizes for MAX_PARTICLES quads (4 verts, 6 indices each)
        let vertex_buffer_size =
            (MAX_PARTICLES * 4 * std::mem::size_of::<ParticleVertex>()) as vk::DeviceSize;
        let index_buffer_size = (MAX_PARTICLES * 6 * std::mem::size_of::<u32>()) as vk::DeviceSize;

        let vertex_buffer = ManagedBuffer::new(
            Arc::clone(&device),
            vertex_buffer_size,
            vk::BufferUsageFlags::VERTEX_BUFFER,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )?;

        let index_buffer = ManagedBuffer::new(
            Arc::clone(&device),
            index_buffer_size,
            vk::BufferUsageFlags::INDEX_BUFFER,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )?;

        Ok(Self {
            device,
            pipeline,
            vertex_buffer,
            index_buffer,
            draw_order: Vec::with_capacity(MAX_PARTICLES),
            matrices: None,
        })
    }

    /// Drop last frame's prepared particles.
    ///
    /// Called from the renderer's own `begin_frame`, so a frame that never
    /// submits a pool — a tool that drives the renderer without particles —
    /// records nothing rather than replaying the last order it was given.
    pub fn begin_frame(&mut self) {
        self.draw_order.clear();
        self.matrices = None;
    }

    /// Sort the frame's particles far to near and upload their billboards.
    ///
    /// Records nothing. Call once per frame before the scene pass begins
    /// recording blended geometry; [`Self::bind`] and [`Self::draw_range`] then
    /// emit whatever slices of this stream the interleaving asks for.
    ///
    /// Returns the number of particles prepared, which is the pool's count
    /// capped at [`MAX_PARTICLES`].
    pub fn prepare(
        &mut self,
        pool: &ParticlePool,
        view_matrix: &Matrix4<f32>,
        proj_matrix: &Matrix4<f32>,
        camera_pos: &Vector3<f32>,
    ) -> EngineResult<usize> {
        self.draw_order.clear();
        self.matrices = None;

        if pool.is_empty() {
            return Ok(0);
        }

        let particles = pool.particles();
        self.sort_far_to_near(particles, camera_pos);
        let particle_count = self.draw_order.len();

        // Generate billboard vertices for each particle
        let mut vertices = Vec::with_capacity(particle_count * 4);
        let mut indices = Vec::with_capacity(particle_count * 6);

        // Billboard corner offsets
        let corners = [
            Vector2::new(-1.0, -1.0), // Bottom-left
            Vector2::new(1.0, -1.0),  // Bottom-right
            Vector2::new(1.0, 1.0),   // Top-right
            Vector2::new(-1.0, 1.0),  // Top-left
        ];

        for (i, &(_, particle_index)) in self.draw_order.iter().enumerate() {
            let particle = &particles[particle_index];
            let base_index = (i * 4) as u32;
            let life_normalized = particle.normalized_age();
            let motion = particle.velocity * particle.stretch;
            let shape = Vector4::new(
                particle.angle,
                particle.additive,
                particle.billow,
                particle.seed,
            );
            let size = particle.drawn_size();

            // Create 4 vertices for this particle's billboard
            for corner in &corners {
                vertices.push(ParticleVertex {
                    center: particle.position,
                    corner: *corner,
                    size,
                    color: particle.colour,
                    life: life_normalized,
                    motion,
                    shape,
                });
            }

            // Two triangles: 0-1-2, 0-2-3
            indices.push(base_index);
            indices.push(base_index + 1);
            indices.push(base_index + 2);
            indices.push(base_index);
            indices.push(base_index + 2);
            indices.push(base_index + 3);
        }

        // Upload vertex data
        unsafe {
            let vertex_size = std::mem::size_of::<ParticleVertex>() * vertices.len();
            let ptr = self
                .vertex_buffer
                .map_memory(0, vk::MemoryMapFlags::empty())?;
            std::ptr::copy_nonoverlapping(
                vertices.as_ptr() as *const u8,
                ptr as *mut u8,
                vertex_size,
            );
            self.vertex_buffer.unmap_memory();

            let index_size = std::mem::size_of::<u32>() * indices.len();
            let ptr = self
                .index_buffer
                .map_memory(0, vk::MemoryMapFlags::empty())?;
            std::ptr::copy_nonoverlapping(
                indices.as_ptr() as *const u8,
                ptr as *mut u8,
                index_size,
            );
            self.index_buffer.unmap_memory();
        }

        self.matrices = Some((*view_matrix, *proj_matrix));

        Ok(particle_count)
    }

    /// Whether `prepare` left any particles to draw this frame.
    pub fn is_empty(&self) -> bool {
        self.draw_order.is_empty()
    }

    /// How many of the prepared particles lie at or beyond `distance_sq` from
    /// the camera — that is, the length of the prefix of the draw order that
    /// must be recorded before something at that distance.
    ///
    /// The order is far to near, so this is a binary search on a descending
    /// key rather than a scan.
    pub fn count_beyond(&self, distance_sq: f32) -> usize {
        self.draw_order
            .partition_point(|&(key, _)| key >= distance_sq)
    }

    /// Bind the particle pipeline and this frame's buffers and matrices.
    ///
    /// Call once before the first [`Self::draw_range`], and again after any
    /// other pipeline has been bound in between.
    pub fn bind(&self, cb: vk::CommandBuffer, exposure: f32) {
        let Some((view_matrix, proj_matrix)) = self.matrices else {
            return;
        };

        unsafe {
            self.device.device.cmd_bind_pipeline(
                cb,
                vk::PipelineBindPoint::GRAPHICS,
                self.pipeline.pipeline(),
            );

            // Push view and projection matrices
            let view_bytes: &[u8] = bytemuck_cast_slice(view_matrix.as_slice());
            let proj_bytes: &[u8] = bytemuck_cast_slice(proj_matrix.as_slice());

            let mut push_data = [0u8; 128];
            push_data[0..64].copy_from_slice(view_bytes);
            push_data[64..128].copy_from_slice(proj_bytes);

            self.device.device.cmd_push_constants(
                cb,
                self.pipeline.layout(),
                vk::ShaderStageFlags::VERTEX,
                0,
                &push_data,
            );

            self.device.device.cmd_push_constants(
                cb,
                self.pipeline.layout(),
                vk::ShaderStageFlags::FRAGMENT,
                128,
                &exposure.to_ne_bytes(),
            );

            // Bind vertex and index buffers
            self.device
                .device
                .cmd_bind_vertex_buffers(cb, 0, &[self.vertex_buffer.buffer], &[0]);

            self.device.device.cmd_bind_index_buffer(
                cb,
                self.index_buffer.buffer,
                0,
                vk::IndexType::UINT32,
            );
        }
    }

    /// Draw the prepared particles in `first..end`, indices into the far-to-near
    /// draw order. An empty or out-of-range slice records nothing.
    pub fn draw_range(&self, cb: vk::CommandBuffer, first: usize, end: usize) {
        let end = end.min(self.draw_order.len());
        if first >= end {
            return;
        }

        // Six indices per particle, laid down in draw order by `prepare`, so a
        // contiguous slice of the order is a contiguous slice of the buffer.
        const INDICES_PER_PARTICLE: u32 = 6;
        let first_index = first as u32 * INDICES_PER_PARTICLE;
        let index_count = (end - first) as u32 * INDICES_PER_PARTICLE;

        unsafe {
            self.device
                .device
                .cmd_draw_indexed(cb, index_count, 1, first_index, 0, 0);
        }
    }

    /// Fill `draw_order` with particle indices, furthest from the camera first.
    ///
    /// Blended particles composite in draw order, so an unsorted pool lets a
    /// near smoke puff be drawn before a far one and darken it — visible as
    /// patches of a cloud flickering as the camera moves. Sorting by distance
    /// is O(n log n) on a few thousand floats, which is cheaper than any of the
    /// alternatives that avoid the problem in the blend state.
    ///
    /// Distance from the camera rather than view-space depth because the same
    /// key orders the blended geometry these draws are interleaved with; two
    /// streams sorted by different measures of "far" cannot be merged.
    ///
    /// The pool is capped at [`MAX_PARTICLES`] here rather than at spawn time,
    /// so an overfull pool drops its most distant particles — the ones covering
    /// the fewest pixels — instead of an arbitrary slice of them.
    fn sort_far_to_near(&mut self, particles: &[Particle], camera_pos: &Vector3<f32>) {
        self.draw_order.clear();
        self.draw_order.extend(
            particles
                .iter()
                .enumerate()
                .map(|(index, particle)| ((particle.position - camera_pos).norm_squared(), index)),
        );

        // Furthest first: the largest distance is drawn before the smallest.
        self.draw_order
            .sort_unstable_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

        // Trimmed from the front, which is the far end: an overfull pool loses
        // the particles covering the fewest pixels.
        if self.draw_order.len() > MAX_PARTICLES {
            let excess = self.draw_order.len() - MAX_PARTICLES;
            self.draw_order.drain(..excess);
        }
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
