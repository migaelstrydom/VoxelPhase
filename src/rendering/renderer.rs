//! Main renderer that orchestrates all rendering components.
//!
//! The Renderer is now a thin orchestration layer that delegates to:
//! - `GraphicsPipeline` - immutable pipeline state
//! - `Swapchain` - presentation and synchronization
//! - `FrameData` - per-frame mutable buffers
//! - `DescriptorManager` - descriptor set management

use std::sync::Arc;

use ash::vk;
use nalgebra::Matrix4;
use winit::window::Window;

use crate::core::error::EngineResult;
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::descriptors::DescriptorManager;
use crate::rendering::frame::{FrameData, SceneUbo};
use crate::rendering::pipeline::{GraphicsPipeline, GraphicsPipelineConfig};
use crate::rendering::swapchain::Swapchain;
use crate::rendering::vertex::Vertex;
use crate::resources::textures::{TextureHandle, TextureManager};

/// The main renderer that orchestrates frame rendering.
///
/// This is a thin layer that coordinates the pipeline, swapchain, frame data,
/// and descriptor management to render frames.
pub struct Renderer {
    pub pipeline: GraphicsPipeline,
    pub swapchain: Swapchain,
    pub frame_data: FrameData,
    pub descriptors: Arc<DescriptorManager>,
    pub vulkan_context: Arc<VulkanContext>,
}

impl Renderer {
    /// Create a new renderer.
    pub fn new(
        vulkan_context: Arc<VulkanContext>,
        window: &Window,
        window_width: u32,
        window_height: u32,
    ) -> EngineResult<Self> {
        // Create pipeline first (we need the render pass for swapchain framebuffers)
        let pipeline_config = GraphicsPipelineConfig {
            color_format: vk::Format::B8G8R8A8_UNORM, // Will be overridden by swapchain format
            depth_format: vk::Format::D16_UNORM,
            extent: vk::Extent2D {
                width: window_width,
                height: window_height,
            },
        };

        let pipeline = GraphicsPipeline::new(Arc::clone(&vulkan_context.device), &pipeline_config)?;

        // Create swapchain with framebuffers that reference the render pass
        let swapchain = Swapchain::new(
            Arc::clone(&vulkan_context),
            window,
            pipeline.renderpass,
            window_width,
            window_height,
        )?;

        // Create frame data (vertex/index/UBO buffers)
        let frame_data = FrameData::new(Arc::clone(&vulkan_context.device))?;

        // Create descriptor manager
        let descriptors = Arc::new(DescriptorManager::new(
            Arc::clone(&vulkan_context.device),
            pipeline.scene_ubo_descriptor_set_layout,
            pipeline.sampler_descriptor_set_layout,
            100, // initial texture descriptor capacity
        )?);

        // Initialize the UBO descriptor
        descriptors.update_scene_ubo(
            &frame_data.scene_ubo_buffer,
            std::mem::size_of::<SceneUbo>() as vk::DeviceSize,
        );

        Ok(Self {
            pipeline,
            swapchain,
            frame_data,
            descriptors,
            vulkan_context,
        })
    }

    /// Get the descriptor manager (for creating texture managers)
    pub fn descriptor_manager(&self) -> Arc<DescriptorManager> {
        Arc::clone(&self.descriptors)
    }

    /// Begin a new frame: wait for previous frame, acquire swapchain image.
    pub fn begin_frame(&self) -> EngineResult<(vk::CommandBuffer, u32)> {
        // Wait for previous frame to complete
        self.swapchain.sync.wait_and_reset()?;

        // Acquire next swapchain image
        let image_index = self.swapchain.acquire_next_image()?;

        // Begin command buffer recording
        self.swapchain
            .draw_command_buffer
            .begin(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT)?;

        let cb = self.swapchain.draw_command_buffer.raw();

        // Begin render pass
        let clear_values = [
            vk::ClearValue {
                color: vk::ClearColorValue {
                    float32: [0.0, 0.0, 0.0, 1.0],
                },
            },
            vk::ClearValue {
                depth_stencil: vk::ClearDepthStencilValue {
                    depth: 1.0,
                    stencil: 0,
                },
            },
        ];

        let render_pass_begin = vk::RenderPassBeginInfo::default()
            .render_pass(self.pipeline.renderpass)
            .framebuffer(self.swapchain.framebuffers[image_index as usize])
            .render_area(self.swapchain.extent.into())
            .clear_values(&clear_values);

        unsafe {
            self.vulkan_context.device().cmd_begin_render_pass(
                cb,
                &render_pass_begin,
                vk::SubpassContents::INLINE,
            );
        }

        Ok((cb, image_index))
    }

    /// Draw a mesh with the given transform and texture.
    pub fn draw_mesh(
        &mut self,
        cb: vk::CommandBuffer,
        vertices: &[Vertex],
        indices: &[u32],
        model: &Matrix4<f32>,
        view: &Matrix4<f32>,
        proj: &Matrix4<f32>,
        texture_handles: &[TextureHandle],
        texture_manager: &TextureManager,
    ) -> EngineResult<()> {
        // Update frame data buffers
        self.frame_data.update_mesh_data(vertices, indices)?;
        self.frame_data.update_transforms(model, view, proj)?;

        // Bind pipeline
        unsafe {
            self.vulkan_context.device().cmd_bind_pipeline(
                cb,
                vk::PipelineBindPoint::GRAPHICS,
                self.pipeline.pipeline,
            );

            // Set dynamic state
            let viewports = [vk::Viewport {
                x: 0.0,
                y: 0.0,
                width: self.swapchain.extent.width as f32,
                height: self.swapchain.extent.height as f32,
                min_depth: 0.0,
                max_depth: 1.0,
            }];
            let scissors = [self.swapchain.extent.into()];

            self.vulkan_context
                .device()
                .cmd_set_viewport(cb, 0, &viewports);
            self.vulkan_context
                .device()
                .cmd_set_scissor(cb, 0, &scissors);

            // Get texture descriptor set
            let texture_set = if !texture_handles.is_empty() {
                texture_manager
                    .get_or_create_descriptor_set(&texture_handles[0])
                    .map_err(|e| crate::core::error::EngineError::Texture {
                        path: None,
                        reason: format!("descriptor set creation: {}", e),
                    })?
            } else {
                return Err(crate::core::error::EngineError::Texture {
                    path: None,
                    reason: "no texture provided for mesh".to_string(),
                });
            };

            // Bind descriptor sets
            let descriptor_sets = [self.descriptors.scene_ubo_set, texture_set];
            self.vulkan_context.device().cmd_bind_descriptor_sets(
                cb,
                vk::PipelineBindPoint::GRAPHICS,
                self.pipeline.layout,
                0,
                &descriptor_sets,
                &[],
            );

            // Bind vertex and index buffers
            self.vulkan_context.device().cmd_bind_vertex_buffers(
                cb,
                0,
                &[self.frame_data.vertex_buffer.buffer],
                &[0],
            );
            self.vulkan_context.device().cmd_bind_index_buffer(
                cb,
                self.frame_data.index_buffer.buffer,
                0,
                vk::IndexType::UINT32,
            );

            // Draw
            self.vulkan_context
                .device()
                .cmd_draw_indexed(cb, self.frame_data.index_count, 1, 0, 0, 0);
        }

        Ok(())
    }

    /// End the frame: finish render pass, submit commands, present.
    pub fn end_frame(&self, cb: vk::CommandBuffer, image_index: u32) -> EngineResult<()> {
        unsafe {
            self.vulkan_context.device().cmd_end_render_pass(cb);
        }

        self.swapchain.draw_command_buffer.end()?;

        // Submit command buffer
        self.vulkan_context
            .command_buffer_manager
            .submit_recorded_graphics_commands_async(
                &self.swapchain.draw_command_buffer,
                self.swapchain.sync.draw_fence,
                &[self.swapchain.sync.present_complete],
                &[self.swapchain.sync.rendering_complete],
                &[vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT],
            )?;

        // Present
        self.swapchain.present(
            image_index,
            self.vulkan_context.command_buffer_manager.graphics_queue,
        )?;

        Ok(())
    }
}

impl Drop for Renderer {
    fn drop(&mut self) {
        unsafe {
            log::info!("Renderer::drop - waiting for device idle");
            let _ = self.vulkan_context.device().device_wait_idle();
        }
        // Components drop in reverse order due to struct field ordering
    }
}
