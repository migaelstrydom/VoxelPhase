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
use crate::model::{Model, Transform};
use crate::particles::{ParticlePool, ParticleRenderer};
use crate::rendering::descriptors::DescriptorManager;
use crate::rendering::frame::{FrameData, SceneUbo};
use crate::rendering::material::MaterialManager;
use crate::rendering::overlay::OverlayRenderer;
use crate::rendering::pipeline::{GraphicsPipeline, GraphicsPipelineConfig};
use crate::rendering::sky::SkyRenderer;
use crate::rendering::swapchain::{SurfaceInfo, Swapchain};
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
    pub overlay: OverlayRenderer,
    pub particle_renderer: ParticleRenderer,
    pub sky_renderer: SkyRenderer,
}

impl Renderer {
    /// Create a new renderer.
    pub fn new(
        vulkan_context: Arc<VulkanContext>,
        window: &Window,
        window_width: u32,
        window_height: u32,
    ) -> EngineResult<Self> {
        // Create surface and query its format (done once)
        let surface_info = SurfaceInfo::new(&vulkan_context, window)?;

        // Save format before moving surface_info
        let color_format = surface_info.format.format;

        // Create pipeline first (we need the render pass for swapchain framebuffers)
        let pipeline_config = GraphicsPipelineConfig {
            color_format,
            depth_format: vk::Format::D16_UNORM,
            extent: vk::Extent2D {
                width: window_width,
                height: window_height,
            },
        };

        let pipeline = GraphicsPipeline::new(Arc::clone(&vulkan_context.device), &pipeline_config)?;

        // Create swapchain with the existing surface info
        let swapchain = Swapchain::new(
            Arc::clone(&vulkan_context),
            surface_info,
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

        // Create overlay renderer for debug text (uses same render pass for compatibility)
        let overlay = OverlayRenderer::new(
            Arc::clone(&vulkan_context),
            pipeline.renderpass,
            window_width,
            window_height,
        )?;

        // Create particle renderer
        let particle_renderer =
            ParticleRenderer::new(Arc::clone(&vulkan_context), pipeline.renderpass)?;

        // Create sky renderer
        let sky_renderer =
            SkyRenderer::new(Arc::clone(&vulkan_context), pipeline.renderpass)?;

        Ok(Self {
            pipeline,
            swapchain,
            frame_data,
            descriptors,
            vulkan_context,
            overlay,
            particle_renderer,
            sky_renderer,
        })
    }

    /// Get the descriptor manager (for creating texture managers)
    pub fn descriptor_manager(&self) -> Arc<DescriptorManager> {
        Arc::clone(&self.descriptors)
    }

    /// Begin a new frame: wait for previous frame, acquire swapchain image.
    pub fn begin_frame(&mut self) -> EngineResult<(vk::CommandBuffer, u32)> {
        // Wait for previous frame to complete
        self.swapchain.sync.wait_and_reset()?;

        // Now that the GPU is done with previous frames, flush deferred deletions
        self.frame_data.begin_frame();

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
                    float32: [0.3, 0.5, 0.8, 1.0],
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

    /// Update per-frame scene data (view/projection matrices).
    /// Call this once at the start of each frame, before any draw calls.
    pub fn update_scene(&mut self, view: &Matrix4<f32>, proj: &Matrix4<f32>) -> EngineResult<()> {
        self.frame_data.update_scene_ubo(view, proj)
    }

    /// Update sky renderer with delta time for cloud animation.
    pub fn update_sky(&mut self, delta_time: f32) {
        self.sky_renderer.update(delta_time);
    }

    /// Render the procedural sky.
    ///
    /// Should be called immediately after begin_frame and update_scene,
    /// before any geometry is drawn. The sky renders without depth testing
    /// so it will appear behind all other objects.
    pub fn render_sky(
        &self,
        cb: vk::CommandBuffer,
        view_matrix: &Matrix4<f32>,
        proj_matrix: &Matrix4<f32>,
    ) -> EngineResult<()> {
        let extent = self.swapchain.extent;
        let viewport = vk::Viewport {
            x: 0.0,
            y: 0.0,
            width: extent.width as f32,
            height: extent.height as f32,
            min_depth: 0.0,
            max_depth: 1.0,
        };
        let scissor = vk::Rect2D {
            offset: vk::Offset2D { x: 0, y: 0 },
            extent,
        };

        self.sky_renderer
            .render(cb, view_matrix, proj_matrix, viewport, scissor)
    }

    /// Draw a complete model with per-part transforms applied.
    ///
    /// This iterates through all model parts and draws each primitive,
    /// applying the part's local transform combined with the world transform.
    pub fn draw_model(
        &mut self,
        cb: vk::CommandBuffer,
        model: &Model,
        world_transform: &Matrix4<f32>,
        part_transforms: &[Transform],
        material_manager: &MaterialManager,
        texture_manager: &TextureManager,
    ) -> EngineResult<()> {
        for (part_idx, part) in model.parts.iter().enumerate() {
            // Get the animated/modified transform for this part
            let part_transform = if part_idx < part_transforms.len() {
                part.local_transform.compose(&part_transforms[part_idx])
            } else {
                part.local_transform.clone()
            };

            let part_matrix = part_transform.to_matrix();
            let final_transform = world_transform * part_matrix;

            // Draw each primitive in this part
            for primitive in &part.primitives {
                let texture = material_manager.get_effective_texture(primitive.material);

                self.draw_mesh_with_texture(
                    cb,
                    &primitive.vertices,
                    &primitive.indices,
                    &final_transform,
                    texture,
                    texture_manager,
                )?;
            }
        }

        Ok(())
    }

    /// Draw a procedural mesh (like a skeleton character or terrain chunk) using vertex colors.
    /// Uses a default white texture so vertex colors show through.
    pub fn draw_procedural_mesh(
        &mut self,
        cb: vk::CommandBuffer,
        vertices: &[Vertex],
        indices: &[u32],
        world_transform: &Matrix4<f32>,
        material_manager: &MaterialManager,
        texture_manager: &TextureManager,
    ) -> EngineResult<()> {
        if vertices.is_empty() || indices.is_empty() {
            return Ok(());
        }

        // Use the fallback white texture so vertex colors show
        let white_texture = material_manager.fallback_texture();
        self.draw_mesh_with_texture(
            cb,
            vertices,
            indices,
            world_transform,
            white_texture,
            texture_manager,
        )
    }

    /// Draw a mesh with a specific texture handle.
    ///
    /// This is a lower-level method used by draw_model and terrain rendering.
    pub fn draw_mesh_with_texture(
        &mut self,
        cb: vk::CommandBuffer,
        vertices: &[Vertex],
        indices: &[u32],
        model: &Matrix4<f32>,
        texture: &TextureHandle,
        texture_manager: &TextureManager,
    ) -> EngineResult<()> {
        // Append mesh data to frame buffers and get draw offsets
        let draw_info = self.frame_data.append_mesh_data(vertices, indices)?;

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

            // Push model matrix (per-draw data)
            let model_bytes: &[u8] = std::slice::from_raw_parts(
                model.as_ptr() as *const u8,
                std::mem::size_of::<Matrix4<f32>>(),
            );
            self.vulkan_context.device().cmd_push_constants(
                cb,
                self.pipeline.layout,
                vk::ShaderStageFlags::VERTEX,
                0,
                model_bytes,
            );

            // Get texture descriptor set
            let texture_set = texture_manager
                .get_or_create_descriptor_set(texture)
                .map_err(|e| crate::core::error::EngineError::Texture {
                    path: None,
                    reason: format!("descriptor set creation: {}", e),
                })?;

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
            self.vulkan_context.device().cmd_draw_indexed(
                cb,
                draw_info.index_count,
                1,
                draw_info.first_index,
                draw_info.vertex_offset,
                0,
            );
        }

        Ok(())
    }

    /// Render particles from the particle pool.
    ///
    /// Should be called after drawing the 3D scene but before overlay.
    pub fn render_particles(
        &mut self,
        cb: vk::CommandBuffer,
        pool: &ParticlePool,
        view_matrix: &Matrix4<f32>,
        proj_matrix: &Matrix4<f32>,
    ) -> EngineResult<()> {
        let extent = self.swapchain.extent;
        let viewport = vk::Viewport {
            x: 0.0,
            y: 0.0,
            width: extent.width as f32,
            height: extent.height as f32,
            min_depth: 0.0,
            max_depth: 1.0,
        };
        let scissor = vk::Rect2D {
            offset: vk::Offset2D { x: 0, y: 0 },
            extent,
        };

        unsafe {
            self.vulkan_context
                .device()
                .cmd_set_viewport(cb, 0, &[viewport]);
            self.vulkan_context
                .device()
                .cmd_set_scissor(cb, 0, &[scissor]);
        }

        self.particle_renderer
            .render(cb, pool, view_matrix, proj_matrix)
    }

    /// Render debug overlay with the given debug line entries.
    ///
    /// Should be called after drawing the 3D scene but before end_frame.
    pub fn render_overlay<'a>(
        &mut self,
        cb: vk::CommandBuffer,
        entries: impl Iterator<Item = (&'a str, &'a str)>,
    ) -> EngineResult<()> {
        let extent = self.swapchain.extent;
        let viewport = vk::Viewport {
            x: 0.0,
            y: 0.0,
            width: extent.width as f32,
            height: extent.height as f32,
            min_depth: 0.0,
            max_depth: 1.0,
        };
        let scissor = vk::Rect2D {
            offset: vk::Offset2D { x: 0, y: 0 },
            extent,
        };

        unsafe {
            self.vulkan_context
                .device()
                .cmd_set_viewport(cb, 0, &[viewport]);
            self.vulkan_context
                .device()
                .cmd_set_scissor(cb, 0, &[scissor]);
        }

        self.overlay.render_debug_lines(cb, entries)
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
        // Flush any pending buffer deletions now that GPU is idle
        self.frame_data.cleanup();
        // Components drop in reverse order due to struct field ordering
    }
}
