//! Particle rendering module.
//!
//! Renders particles as camera-facing billboards using instanced geometry.

use std::sync::Arc;

use ash::vk;
use nalgebra::{Matrix4, Vector2, Vector4};

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
    /// Particle indices ordered far-to-near for this frame, kept between
    /// frames so the sort does not allocate every time.
    draw_order: Vec<(f32, usize)>,
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
        })
    }

    /// Render all particles in the pool.
    ///
    /// # Arguments
    /// * `cb` - Command buffer to record draw commands into
    /// * `pool` - Particle pool containing active particles
    /// * `view_matrix` - Camera view matrix
    /// * `proj_matrix` - Camera projection matrix
    pub fn render(
        &mut self,
        cb: vk::CommandBuffer,
        pool: &ParticlePool,
        view_matrix: &Matrix4<f32>,
        proj_matrix: &Matrix4<f32>,
    ) -> EngineResult<()> {
        if pool.is_empty() {
            return Ok(());
        }

        let particles = pool.particles();
        self.sort_far_to_near(particles, view_matrix);
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

        // Bind pipeline
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

            // Draw indexed
            self.device
                .device
                .cmd_draw_indexed(cb, indices.len() as u32, 1, 0, 0, 0);
        }

        Ok(())
    }

    /// Fill `draw_order` with particle indices, furthest from the camera first.
    ///
    /// Blended particles composite in draw order, so an unsorted pool lets a
    /// near smoke puff be drawn before a far one and darken it — visible as
    /// patches of a cloud flickering as the camera moves. Sorting by view depth
    /// is O(n log n) on a few thousand floats, which is cheaper than any of the
    /// alternatives that avoid the problem in the blend state.
    ///
    /// The pool is capped at [`MAX_PARTICLES`] here rather than at spawn time,
    /// so an overfull pool drops its most distant particles — the ones covering
    /// the fewest pixels — instead of an arbitrary slice of them.
    fn sort_far_to_near(&mut self, particles: &[Particle], view_matrix: &Matrix4<f32>) {
        // Third row of the view matrix: the only part needed to get view-space
        // z, and the translation term is shared by every particle so it cannot
        // change the ordering.
        let forward = view_matrix.row(2);

        self.draw_order.clear();
        self.draw_order
            .extend(particles.iter().enumerate().map(|(index, particle)| {
                let position = particle.position;
                let depth =
                    forward[0] * position.x + forward[1] * position.y + forward[2] * position.z;
                (depth, index)
            }));

        // View space looks down -z, so the furthest particles have the smallest
        // depth and must be drawn first.
        self.draw_order
            .sort_unstable_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

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
