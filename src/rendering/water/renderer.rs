//! Water surface mesh generation and rendering.

use std::sync::Arc;

use ash::vk;
use nalgebra::{Matrix4, Vector3};

use crate::core::device::ManagedDevice;
use crate::core::error::EngineResult;
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::frame::ManagedBuffer;
use crate::water::{WaterGrid, WaveGrid};

use super::pipeline::WaterPipeline;
use super::vertex::WaterVertex;

/// Maximum water quads that can be rendered in a single draw call.
/// At wave resolution (~10cm cells), a 25m pond produces ~62,500 quads.
const MAX_WATER_QUADS: usize = 65536;

/// Renders the water surface mesh generated from the wave grid.
///
/// Each frame, iterates wet wave cells and emits quads at wave resolution.
/// The per-vertex Y position is `bulk_level + wave_displacement`, producing
/// smooth surfaces with visible ripples.
pub struct WaterRenderer {
    device: Arc<ManagedDevice>,
    pipeline: WaterPipeline,
    vertex_buffer: ManagedBuffer,
    index_buffer: ManagedBuffer,
}

impl WaterRenderer {
    pub fn new(
        vulkan_context: Arc<VulkanContext>,
        render_pass: vk::RenderPass,
    ) -> EngineResult<Self> {
        let device = Arc::clone(&vulkan_context.device);
        let pipeline = WaterPipeline::new(Arc::clone(&device), render_pass)?;

        let vertex_buffer_size =
            (MAX_WATER_QUADS * 4 * std::mem::size_of::<WaterVertex>()) as vk::DeviceSize;
        let index_buffer_size =
            (MAX_WATER_QUADS * 6 * std::mem::size_of::<u32>()) as vk::DeviceSize;

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
        })
    }

    /// Generate the water mesh from both grids and render it.
    pub fn render(
        &mut self,
        cb: vk::CommandBuffer,
        flow_grid: &WaterGrid,
        wave_grid: &WaveGrid,
        view_matrix: &Matrix4<f32>,
        proj_matrix: &Matrix4<f32>,
    ) -> EngineResult<()> {
        let (vertices, indices) = Self::generate_mesh(flow_grid, wave_grid);
        if indices.is_empty() {
            return Ok(());
        }

        // Upload vertex data
        unsafe {
            let vertex_size = std::mem::size_of::<WaterVertex>() * vertices.len();
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

        // Bind pipeline and draw
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

            self.device
                .device
                .cmd_bind_vertex_buffers(cb, 0, &[self.vertex_buffer.buffer], &[0]);

            self.device.device.cmd_bind_index_buffer(
                cb,
                self.index_buffer.buffer,
                0,
                vk::IndexType::UINT32,
            );

            self.device
                .device
                .cmd_draw_indexed(cb, indices.len() as u32, 1, 0, 0, 0);
        }

        Ok(())
    }

    /// Generate water surface mesh at wave grid resolution.
    ///
    /// Iterates wet flow cells and emits quads from their wave cells. For each
    /// wave cell quad where all four corners are wet, a quad is emitted with
    /// per-vertex Y = `bulk_level + displacement`. Normals are computed from
    /// the heightfield gradient for smooth lighting.
    fn generate_mesh(flow_grid: &WaterGrid, wave_grid: &WaveGrid) -> (Vec<WaterVertex>, Vec<u32>) {
        let flow_dims = flow_grid.dims();
        let wave_dims = wave_grid.dims();
        let n = wave_grid.cells_per_flow_cell();
        let cell_area = flow_grid.cell_area();
        let wave_cs = wave_grid.cell_size();
        let origin = wave_grid.origin();

        if wave_dims.0 < 2 || wave_dims.1 < 2 {
            return (Vec::new(), Vec::new());
        }

        let mut vertices = Vec::with_capacity(MAX_WATER_QUADS * 4);
        let mut indices = Vec::with_capacity(MAX_WATER_QUADS * 6);
        let mut quad_count = 0;

        // Iterate flow cells; for each wet flow cell, emit wave-resolution quads.
        for fj in 0..flow_dims.1 {
            for fi in 0..flow_dims.0 {
                let flow_cell = flow_grid.cell(fi, fj);
                if flow_cell.volume <= 0.0 {
                    continue;
                }

                let w_start_i = fi * n;
                let w_start_j = fj * n;
                let w_end_i = ((fi + 1) * n).min(wave_dims.0 - 1);
                let w_end_j = ((fj + 1) * n).min(wave_dims.1 - 1);

                for wj in w_start_j..w_end_j {
                    for wi in w_start_i..w_end_i {
                        if quad_count >= MAX_WATER_QUADS {
                            return (vertices, indices);
                        }

                        // Check all 4 corners are wet.
                        if !wave_grid.is_wet(wi, wj, flow_grid)
                            || !wave_grid.is_wet(wi + 1, wj, flow_grid)
                            || !wave_grid.is_wet(wi, wj + 1, flow_grid)
                            || !wave_grid.is_wet(wi + 1, wj + 1, flow_grid)
                        {
                            continue;
                        }

                        let x0 = origin.x + wi as f32 * wave_cs;
                        let x1 = origin.x + (wi + 1) as f32 * wave_cs;
                        let z0 = origin.z + wj as f32 * wave_cs;
                        let z1 = origin.z + (wj + 1) as f32 * wave_cs;

                        // Get bulk levels for each corner (may span flow cells).
                        let bl00 = bulk_level_at(flow_grid, wave_grid, wi, wj, cell_area);
                        let bl10 = bulk_level_at(flow_grid, wave_grid, wi + 1, wj, cell_area);
                        let bl01 = bulk_level_at(flow_grid, wave_grid, wi, wj + 1, cell_area);
                        let bl11 = bulk_level_at(flow_grid, wave_grid, wi + 1, wj + 1, cell_area);

                        let y00 = bl00 + wave_grid.cell(wi, wj).displacement;
                        let y10 = bl10 + wave_grid.cell(wi + 1, wj).displacement;
                        let y01 = bl01 + wave_grid.cell(wi, wj + 1).displacement;
                        let y11 = bl11 + wave_grid.cell(wi + 1, wj + 1).displacement;

                        // Compute normals from the heightfield gradient at each corner.
                        let n00 = heightfield_normal(flow_grid, wave_grid, wi, wj, cell_area);
                        let n10 = heightfield_normal(flow_grid, wave_grid, wi + 1, wj, cell_area);
                        let n01 = heightfield_normal(flow_grid, wave_grid, wi, wj + 1, cell_area);
                        let n11 =
                            heightfield_normal(flow_grid, wave_grid, wi + 1, wj + 1, cell_area);

                        let base = vertices.len() as u32;

                        vertices.push(WaterVertex {
                            position: Vector3::new(x0, y00, z0),
                            normal: n00,
                        });
                        vertices.push(WaterVertex {
                            position: Vector3::new(x1, y10, z0),
                            normal: n10,
                        });
                        vertices.push(WaterVertex {
                            position: Vector3::new(x1, y11, z1),
                            normal: n11,
                        });
                        vertices.push(WaterVertex {
                            position: Vector3::new(x0, y01, z1),
                            normal: n01,
                        });

                        // Two triangles: 0-1-2, 0-2-3
                        indices.push(base);
                        indices.push(base + 1);
                        indices.push(base + 2);
                        indices.push(base);
                        indices.push(base + 2);
                        indices.push(base + 3);

                        quad_count += 1;
                    }
                }
            }
        }

        (vertices, indices)
    }
}

/// Get the smoothed bulk water level at a wave cell position.
#[inline]
fn bulk_level_at(
    flow_grid: &WaterGrid,
    wave_grid: &WaveGrid,
    wi: usize,
    wj: usize,
    cell_area: f32,
) -> f32 {
    let x = wave_grid.origin().x + wi as f32 * wave_grid.cell_size();
    let z = wave_grid.origin().z + wj as f32 * wave_grid.cell_size();
    smoothed_bulk_level_at(flow_grid, x, z, cell_area)
}

/// Sample a smoothed bulk surface from the coarse flow grid.
///
/// The flow simulation remains cell-based, but rendering bilinearly blends the
/// coarse surface levels so the visible water mesh does not inherit the raw
/// stair-step profile of the authoritative flow grid.
#[inline]
fn smoothed_bulk_level_at(flow_grid: &WaterGrid, x: f32, z: f32, cell_area: f32) -> f32 {
    let flow_dims = flow_grid.dims();
    if flow_dims.0 == 0 || flow_dims.1 == 0 {
        return 0.0;
    }

    let flow_cs = flow_grid.cell_size();
    let origin = flow_grid.origin();

    let u = (x - origin.x) / flow_cs - 0.5;
    let v = (z - origin.z) / flow_cs - 0.5;
    let i0 = u.floor() as i32;
    let j0 = v.floor() as i32;
    let tx = u - i0 as f32;
    let tz = v - j0 as f32;

    let samples = [
        (i0, j0, (1.0 - tx) * (1.0 - tz)),
        (i0 + 1, j0, tx * (1.0 - tz)),
        (i0, j0 + 1, (1.0 - tx) * tz),
        (i0 + 1, j0 + 1, tx * tz),
    ];

    let mut weighted_sum = 0.0;
    let mut total_weight = 0.0;

    for (i, j, weight) in samples {
        if weight <= 0.0 {
            continue;
        }

        if let Some(surface) = flow_surface_level(flow_grid, i, j, cell_area) {
            weighted_sum += surface * weight;
            total_weight += weight;
        }
    }

    if total_weight > 0.0 {
        return weighted_sum / total_weight;
    }

    let nearest_i = u.round() as i32;
    let nearest_j = v.round() as i32;
    flow_surface_level(flow_grid, nearest_i, nearest_j, cell_area).unwrap_or(0.0)
}

#[inline]
fn flow_surface_level(flow_grid: &WaterGrid, i: i32, j: i32, cell_area: f32) -> Option<f32> {
    let flow_dims = flow_grid.dims();
    if i < 0 || j < 0 || i >= flow_dims.0 as i32 || j >= flow_dims.1 as i32 {
        return None;
    }

    let cell = flow_grid.cell(i as usize, j as usize);
    if cell.volume <= 0.0 {
        return None;
    }

    Some(cell.surface_level(cell_area))
}

/// Compute the heightfield normal at a wave cell using central differences.
///
/// Uses the rendered surface level (`bulk_level + displacement`) of
/// neighboring cells to compute the gradient.
#[inline]
fn heightfield_normal(
    flow_grid: &WaterGrid,
    wave_grid: &WaveGrid,
    wi: usize,
    wj: usize,
    cell_area: f32,
) -> Vector3<f32> {
    let wave_dims = wave_grid.dims();
    let cs = wave_grid.cell_size();

    let h_center = rendered_level(flow_grid, wave_grid, wi, wj, cell_area);

    // Central differences (fall back to forward/backward at boundaries).
    let dh_dx = if wi > 0 && wi + 1 < wave_dims.0 {
        let h_left = rendered_level(flow_grid, wave_grid, wi - 1, wj, cell_area);
        let h_right = rendered_level(flow_grid, wave_grid, wi + 1, wj, cell_area);
        (h_right - h_left) / (2.0 * cs)
    } else if wi + 1 < wave_dims.0 {
        let h_right = rendered_level(flow_grid, wave_grid, wi + 1, wj, cell_area);
        (h_right - h_center) / cs
    } else if wi > 0 {
        let h_left = rendered_level(flow_grid, wave_grid, wi - 1, wj, cell_area);
        (h_center - h_left) / cs
    } else {
        0.0
    };

    let dh_dz = if wj > 0 && wj + 1 < wave_dims.1 {
        let h_back = rendered_level(flow_grid, wave_grid, wi, wj - 1, cell_area);
        let h_front = rendered_level(flow_grid, wave_grid, wi, wj + 1, cell_area);
        (h_front - h_back) / (2.0 * cs)
    } else if wj + 1 < wave_dims.1 {
        let h_front = rendered_level(flow_grid, wave_grid, wi, wj + 1, cell_area);
        (h_front - h_center) / cs
    } else if wj > 0 {
        let h_back = rendered_level(flow_grid, wave_grid, wi, wj - 1, cell_area);
        (h_center - h_back) / cs
    } else {
        0.0
    };

    Vector3::new(-dh_dx, 1.0, -dh_dz).normalize()
}

/// Rendered surface level at a wave cell: bulk_level + displacement.
#[inline]
fn rendered_level(
    flow_grid: &WaterGrid,
    wave_grid: &WaveGrid,
    wi: usize,
    wj: usize,
    cell_area: f32,
) -> f32 {
    bulk_level_at(flow_grid, wave_grid, wi, wj, cell_area)
        + wave_grid.cell(wi, wj).displacement
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::{WaterGridConfig, WaveGridConfig};

    fn make_flow_grid() -> WaterGrid {
        WaterGrid::new(WaterGridConfig {
            cell_size: 2.0,
            dims: (2, 2),
            origin: Vector3::new(0.0, 0.0, 0.0),
            ocean_level: None,
            flow_rate: 4.0,
            ..Default::default()
        })
    }

    #[test]
    fn smoothed_bulk_level_blends_adjacent_flow_cells() {
        let mut flow_grid = make_flow_grid();
        let cell_area = flow_grid.cell_area();

        flow_grid.add_water(0, 0, 2.0 * cell_area, 0.0);
        flow_grid.add_water(1, 0, 6.0 * cell_area, 0.0);
        flow_grid.add_water(0, 1, 10.0 * cell_area, 0.0);
        flow_grid.add_water(1, 1, 14.0 * cell_area, 0.0);

        let sample = smoothed_bulk_level_at(&flow_grid, 2.0, 2.0, cell_area);

        assert!(
            (sample - 8.0).abs() < 1e-4,
            "Expected bilinear average at the cell-center junction, got {sample}"
        );
    }

    #[test]
    fn bulk_level_at_uses_smoothed_sampling() {
        let mut flow_grid = make_flow_grid();
        let cell_area = flow_grid.cell_area();

        flow_grid.add_water(0, 0, 2.0 * cell_area, 0.0);
        flow_grid.add_water(1, 0, 6.0 * cell_area, 0.0);
        flow_grid.add_water(0, 1, 10.0 * cell_area, 0.0);
        flow_grid.add_water(1, 1, 14.0 * cell_area, 0.0);

        let wave_grid = WaveGrid::new(WaveGridConfig {
            cell_size: 1.0,
            dims: (4, 4),
            origin: Vector3::new(0.0, 0.0, 0.0),
            wave_speed: 4.0,
            wave_damping: 2.0,
            cells_per_flow_cell: 2,
        });

        let sample = bulk_level_at(&flow_grid, &wave_grid, 2, 2, cell_area);

        assert!(
            (sample - 8.0).abs() < 1e-4,
            "Expected smoothed bulk level at shared wave-grid corner, got {sample}"
        );
    }
}
