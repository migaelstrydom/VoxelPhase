//! The sun shadow pass.
//!
//! ```text
//!   Renderer::draw_mesh_internal
//!            |
//!            +--> geometry draw ------> draw command buffer ---+
//!            |                                                 |
//!            +--> ShadowRenderer::record_caster                |  submitted
//!                       |                                      |  in order
//!                       v                                      |
//!                shadow command buffer ------------------------+
//! ```
//!
//! The pass needs every caster, but the callers that issue draws do so one at a
//! time straight into an already-open geometry pass — there is no point at
//! which the frame's geometry is known up front. Recording into a *second*
//! command buffer sidesteps that: both are filled during the same walk over the
//! scene, and the shadow buffer is submitted first. The alternative, reusing
//! last frame's map, lags visibly whenever anything moves.

use std::sync::Arc;

use ash::vk;
use nalgebra::{Matrix4, Vector3};

use crate::core::command_buffer::ManagedCommandBuffer;
use crate::core::error::EngineResult;
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::frame::{DrawInfo, ShadowUniforms};
use crate::rendering::profile::{GpuSpan, GpuTimer};
use crate::rendering::shadow::frustum::ViewFrustum;
use crate::rendering::shadow::map::ShadowMap;
use crate::rendering::shadow::pipeline::ShadowPipeline;
use crate::rendering::shadow::volume::{ShadowFraming, ShadowVolume};

/// Renders the frame's casters from the sun's point of view into a depth map.
pub struct ShadowRenderer {
    /// How the covered slab of world is framed. Public so a scene can widen or
    /// tighten it; changes take effect on the next frame.
    pub volume: ShadowVolume,

    /// Turns the pass off without tearing down its resources. The map is still
    /// cleared each frame, so a disabled pass leaves everything lit rather than
    /// leaving a stale map behind.
    pub enabled: bool,

    map: ShadowMap,
    pipeline: ShadowPipeline,
    command_buffer: ManagedCommandBuffer,
    layout: vk::PipelineLayout,
    /// The volume resolved against the frame's camera, set by [`Self::aim`].
    /// Everything the shader is told about the map comes from here, so the
    /// lookup can never disagree with the box the casters were rendered into.
    framing: ShadowFraming,
    /// Set once per frame by [`Self::aim`], read back into the scene UBO.
    light_view_proj: Matrix4<f32>,
}

impl ShadowRenderer {
    /// Build the pass over the main geometry pipeline's layout, which it shares
    /// so that casters can be drawn with the push constants already in flight.
    ///
    /// The map is passed in rather than built here because that layout is
    /// itself built around the map's comparison sampler — the map has to exist
    /// before the pipeline this pass renders with.
    pub fn new(
        vulkan_context: &VulkanContext,
        map: ShadowMap,
        layout: vk::PipelineLayout,
        volume: ShadowVolume,
    ) -> EngineResult<Self> {
        let pipeline =
            ShadowPipeline::new(Arc::clone(&vulkan_context.device), map.render_pass, layout)?;
        let command_buffer = vulkan_context
            .command_buffer_manager
            .create_primary_buffer()?;

        Ok(Self {
            volume,
            enabled: true,
            map,
            pipeline,
            command_buffer,
            layout,
            // A stand-in until the first `aim`, so the uniforms are never
            // read from an unfitted volume.
            framing: volume.fit(&ViewFrustum::default()),
            light_view_proj: Matrix4::identity(),
        })
    }

    /// The image view and sampler the geometry pass reads the map through.
    pub fn map(&self) -> &ShadowMap {
        &self.map
    }

    /// The command buffer to submit ahead of the geometry pass's.
    pub fn command_buffer(&self) -> &ManagedCommandBuffer {
        &self.command_buffer
    }

    /// How the map is currently framed. Valid from the first [`Self::aim`].
    pub fn framing(&self) -> &ShadowFraming {
        &self.framing
    }

    /// Shadow parameters for the scene UBO: PCF tap spacing, receiver-side
    /// normal offset, strength, and PCF kernel half-width.
    fn shader_params(&self) -> [f32; 4] {
        let strength = if self.enabled {
            self.volume.strength
        } else {
            0.0
        };
        [
            self.framing.texel_uv_size(),
            self.framing.normal_offset(),
            strength,
            self.volume.pcf_radius as f32,
        ]
    }

    /// Open the command buffer and start the pass.
    ///
    /// Called once per frame before any caster is recorded. The map is cleared
    /// even when the pass is disabled, so a stale map can never be sampled.
    ///
    /// Separate from [`Self::aim`] because the two are known at different
    /// points in a frame: recording can start immediately, but where the light
    /// is pointed is not settled until the camera has been handed over.
    ///
    /// This command buffer is the first the GPU runs each frame, so it is
    /// where the frame's GPU timer is rewound.
    pub fn begin_frame(&mut self, timer: &GpuTimer) -> EngineResult<()> {
        self.command_buffer
            .begin(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT)?;
        let cb = self.command_buffer.raw();
        timer.open_frame(cb);
        timer.begin(cb, GpuSpan::Shadow);
        self.map.begin_pass(cb);
        Ok(())
    }

    /// Frame the light's box around the camera.
    ///
    /// Only affects uniform data, never the command buffer, so it may run
    /// after casters have already been recorded — they are transformed by the
    /// matrix at draw time, not at record time.
    ///
    /// The box is fitted to `frustum` rather than held at a fixed size, so the
    /// map covers what the camera can actually see. Re-fitting every frame is
    /// arithmetic and picks up a scene changing the volume between shots; the
    /// fit depends only on the frustum's *shape*, so the radius it produces
    /// holds still while the camera turns.
    pub fn aim(
        &mut self,
        frustum: &ViewFrustum,
        camera_pos: &Vector3<f32>,
        camera_forward: &Vector3<f32>,
        sun_direction: &Vector3<f32>,
    ) {
        self.framing = self.volume.fit(frustum);
        self.light_view_proj =
            self.framing
                .light_view_proj(camera_pos, camera_forward, sun_direction);
    }

    /// The frame's shadow uniforms, for upload into the scene block.
    pub fn uniforms(&self) -> ShadowUniforms {
        ShadowUniforms {
            light_view_proj: self.light_view_proj,
            params: self.shader_params(),
        }
    }

    /// Record one caster, using mesh data already uploaded to the frame's
    /// vertex and index buffers by the geometry draw that owns it.
    pub fn record_caster(
        &self,
        device: &ash::Device,
        model: &Matrix4<f32>,
        draw: &DrawInfo,
        vertex_buffer: vk::Buffer,
        index_buffer: vk::Buffer,
        scene_set: vk::DescriptorSet,
    ) {
        if !self.enabled {
            return;
        }

        let cb = self.command_buffer.raw();

        unsafe {
            device.cmd_bind_pipeline(cb, vk::PipelineBindPoint::GRAPHICS, self.pipeline.pipeline);

            let model_bytes: &[u8] = std::slice::from_raw_parts(
                model.as_ptr() as *const u8,
                std::mem::size_of::<Matrix4<f32>>(),
            );
            device.cmd_push_constants(
                cb,
                self.layout,
                // The geometry layout declares one range spanning both stages
                // — the fragment shader reads the model rotation to place an
                // object-space grain — so the push must name both, even though
                // the shadow pass has no fragment shader that reads it.
                crate::rendering::renderer::PUSH_CONSTANT_STAGES,
                0,
                model_bytes,
            );

            // Only set 0 is bound: the depth-only shader samples nothing, so
            // the texture set the geometry pass binds has no counterpart here.
            device.cmd_bind_descriptor_sets(
                cb,
                vk::PipelineBindPoint::GRAPHICS,
                self.layout,
                0,
                &[scene_set],
                &[],
            );

            device.cmd_bind_vertex_buffers(cb, 0, &[vertex_buffer], &[0]);
            device.cmd_bind_index_buffer(cb, index_buffer, 0, vk::IndexType::UINT32);
            device.cmd_draw_indexed(
                cb,
                draw.index_count,
                1,
                draw.first_index,
                draw.vertex_offset,
                0,
            );
        }
    }

    /// Close the pass and the command buffer, ready for submission.
    pub fn end_frame(&self, timer: &GpuTimer) -> EngineResult<()> {
        let cb = self.command_buffer.raw();
        self.map.end_pass(cb);
        timer.end(cb, GpuSpan::Shadow);
        self.command_buffer.end()
    }
}
