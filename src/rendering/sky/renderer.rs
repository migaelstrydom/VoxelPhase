//! Sky renderer module.
//!
//! Renders a procedural sky with atmospheric scattering, clouds, and sun.

use std::sync::Arc;

use ash::vk;
use nalgebra::{Matrix4, Vector3};

use crate::core::device::ManagedDevice;
use crate::core::error::EngineResult;
use crate::core::vulkan_context::VulkanContext;

use super::pipeline::SkyPipeline;

/// Push constant data for sky rendering.
#[repr(C)]
struct SkyPushConstants {
    /// Inverse view matrix (for transforming view-space rays to world-space)
    inv_view: Matrix4<f32>,
    /// Sun direction (xyz) and time (w)
    sun_direction_time: [f32; 4],
    /// tan(fov/2)*aspect, tan(fov/2), unused, unused
    tan_fov: [f32; 4],
}

/// Sky renderer for procedural atmospheric effects.
///
/// Renders a fullscreen sky with:
/// - Atmospheric scattering (Rayleigh/Mie)
/// - Procedural wispy clouds
/// - Sun disk with glow
/// - Horizon haze
pub struct SkyRenderer {
    device: Arc<ManagedDevice>,
    pipeline: SkyPipeline,
    sun_direction: Vector3<f32>,
    time: f32,
}

impl SkyRenderer {
    /// Create a new sky renderer.
    pub fn new(
        vulkan_context: Arc<VulkanContext>,
        render_pass: vk::RenderPass,
    ) -> EngineResult<Self> {
        let device = Arc::clone(&vulkan_context.device);
        let pipeline = SkyPipeline::new(Arc::clone(&device), render_pass)?;

        // Default sun direction (mid-morning)
        let sun_direction = Vector3::new(0.5, 0.7, 0.5).normalize();

        Ok(Self {
            device,
            pipeline,
            sun_direction,
            time: 0.0,
        })
    }

    /// Set the sun direction (should be normalized).
    #[allow(dead_code)]
    pub fn set_sun_direction(&mut self, direction: Vector3<f32>) {
        self.sun_direction = direction.normalize();
    }

    /// Get the current sun direction.
    #[allow(dead_code)]
    pub fn sun_direction(&self) -> Vector3<f32> {
        self.sun_direction
    }

    /// Update time for cloud animation.
    pub fn update(&mut self, delta_time: f32) {
        self.time += delta_time;
    }

    /// Render the sky.
    ///
    /// Should be called at the start of the frame, before any geometry is drawn.
    /// The sky renders without depth testing, so it will be behind everything.
    ///
    /// # Arguments
    /// * `cb` - Command buffer to record draw commands into
    /// * `view_matrix` - Camera view matrix
    /// * `proj_matrix` - Camera projection matrix
    /// * `viewport` - Viewport for rendering
    /// * `scissor` - Scissor rect for rendering
    pub fn render(
        &self,
        cb: vk::CommandBuffer,
        view_matrix: &Matrix4<f32>,
        proj_matrix: &Matrix4<f32>,
        viewport: vk::Viewport,
        scissor: vk::Rect2D,
    ) -> EngineResult<()> {
        // Calculate inverse view matrix (for transforming view-space rays to world-space)
        let inv_view = view_matrix.try_inverse().unwrap_or_else(Matrix4::identity);

        // Extract projection parameters from the projection matrix
        // For a perspective projection: proj[0][0] = 1/(aspect * tan(fov/2))
        //                               proj[1][1] = 1/tan(fov/2)  (may be negative for Vulkan Y-flip)
        // So: tan(fov/2) = 1/|proj[1][1]|
        //     tan(fov/2) * aspect = 1/|proj[0][0]|
        let tan_half_fov_aspect = 1.0 / proj_matrix[(0, 0)].abs();
        let tan_half_fov = 1.0 / proj_matrix[(1, 1)].abs();

        // Prepare push constants
        let push_constants = SkyPushConstants {
            inv_view,
            sun_direction_time: [
                self.sun_direction.x,
                self.sun_direction.y,
                self.sun_direction.z,
                self.time,
            ],
            tan_fov: [tan_half_fov_aspect, tan_half_fov, 0.0, 0.0],
        };

        unsafe {
            // Bind sky pipeline
            self.device.device.cmd_bind_pipeline(
                cb,
                vk::PipelineBindPoint::GRAPHICS,
                self.pipeline.pipeline(),
            );

            // Set viewport and scissor
            self.device.device.cmd_set_viewport(cb, 0, &[viewport]);
            self.device.device.cmd_set_scissor(cb, 0, &[scissor]);

            // Push constants
            let push_bytes: &[u8] = std::slice::from_raw_parts(
                &push_constants as *const SkyPushConstants as *const u8,
                std::mem::size_of::<SkyPushConstants>(),
            );
            self.device.device.cmd_push_constants(
                cb,
                self.pipeline.layout(),
                vk::ShaderStageFlags::VERTEX | vk::ShaderStageFlags::FRAGMENT,
                0,
                push_bytes,
            );

            // Draw fullscreen triangle (3 vertices, generated in vertex shader)
            self.device.device.cmd_draw(cb, 3, 1, 0, 0);
        }

        Ok(())
    }
}
