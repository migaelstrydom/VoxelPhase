//! Main renderer that orchestrates all rendering components.
//!
//! The Renderer is now a thin orchestration layer that delegates to:
//! - `GraphicsPipeline` - immutable pipeline state
//! - `Swapchain` - presentation and synchronization
//! - `FrameData` - per-frame mutable buffers
//! - `DescriptorManager` - descriptor set management

use std::sync::Arc;

use ash::vk;
use nalgebra::{Matrix4, Vector3};
use winit::window::Window;

use crate::core::error::EngineResult;
use crate::core::vulkan_context::VulkanContext;
use crate::fire::renderer::{ActiveFire, FireRenderer};
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
use crate::rendering::water::WaterRenderer;
use crate::resources::textures::{TextureHandle, TextureManager};
use crate::water::{WaterGrid, WaveGrid};

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
    pub water_renderer: WaterRenderer,
    pub fire_renderer: FireRenderer,
    /// Active fire instances with their GPU resources. Keyed by entity index.
    pub active_fires: Vec<(specs::Entity, ActiveFire)>,
    /// When true, backfaces are rendered in wireframe with `wireframe_color`.
    pub debug_wireframe_backfaces: bool,
    /// The solid color used for wireframe backface rendering (RGBA, 0-1).
    pub wireframe_color: [f32; 4],
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
            pipeline.transparent_renderpass,
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

        // Create overlay renderer for debug text (transparent pass)
        let overlay = OverlayRenderer::new(
            Arc::clone(&vulkan_context),
            pipeline.transparent_renderpass,
            window_width,
            window_height,
        )?;

        // Create particle renderer (transparent pass)
        let particle_renderer =
            ParticleRenderer::new(Arc::clone(&vulkan_context), pipeline.transparent_renderpass)?;

        // Create sky renderer (opaque pass)
        let sky_renderer = SkyRenderer::new(Arc::clone(&vulkan_context), pipeline.renderpass)?;

        // Create water renderer (transparent pass, samples opaque color target for refraction)
        let water_renderer = WaterRenderer::new(
            Arc::clone(&vulkan_context),
            pipeline.transparent_renderpass,
            swapchain.depth_buffer.view,
            swapchain.color_target.view,
        )?;

        // Create fire renderer with shared sim pool
        let fire_renderer = FireRenderer::new(
            Arc::clone(&vulkan_context.device),
            pipeline.transparent_renderpass,
            swapchain.depth_buffer.view,
        )?;

        Ok(Self {
            pipeline,
            swapchain,
            frame_data,
            descriptors,
            vulkan_context,
            overlay,
            particle_renderer,
            sky_renderer,
            water_renderer,
            fire_renderer,
            active_fires: Vec::new(),
            debug_wireframe_backfaces: true,
            wireframe_color: [0.0, 0.0, 0.0, 1.0],
        })
    }

    /// Get the descriptor manager (for creating texture managers)
    pub fn descriptor_manager(&self) -> Arc<DescriptorManager> {
        Arc::clone(&self.descriptors)
    }

    /// Begin a new frame: wait for previous frame, acquire swapchain image,
    /// start the command buffer. Call `begin_opaque_pass()` after any pre-pass
    /// compute work (e.g. fire simulation) is recorded.
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

        Ok((cb, image_index))
    }

    /// Begin the opaque render pass. Call after `begin_frame()` and any
    /// pre-pass compute dispatches.
    pub fn begin_opaque_pass(&self, cb: vk::CommandBuffer) {
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
            .framebuffer(self.swapchain.opaque_framebuffer)
            .render_area(self.swapchain.extent.into())
            .clear_values(&clear_values);

        unsafe {
            self.vulkan_context.device().cmd_begin_render_pass(
                cb,
                &render_pass_begin,
                vk::SubpassContents::INLINE,
            );
        }
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

    /// Draw a procedural mesh (like a skeleton character or terrain chunk) using vertex colours.
    /// Uses a default white texture so vertex colours show through.
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

        let white_texture = material_manager.fallback_texture();
        self.draw_mesh_internal(
            cb,
            vertices,
            indices,
            world_transform,
            white_texture,
            texture_manager,
            self.pipeline.opaque,
            true,
        )
    }

    /// Draw a procedural mesh using the transparent (alpha-blended) pipeline.
    /// No wireframe overlay is applied.
    pub fn draw_procedural_mesh_transparent(
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

        let white_texture = material_manager.fallback_texture();
        self.draw_mesh_internal(
            cb,
            vertices,
            indices,
            world_transform,
            white_texture,
            texture_manager,
            self.pipeline.transparent,
            false,
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
        self.draw_mesh_internal(
            cb, vertices, indices, model, texture, texture_manager,
            self.pipeline.opaque, true,
        )
    }

    /// Record a mesh draw call with the specified pipeline.
    fn draw_mesh_internal(
        &mut self,
        cb: vk::CommandBuffer,
        vertices: &[Vertex],
        indices: &[u32],
        model: &Matrix4<f32>,
        texture: &TextureHandle,
        texture_manager: &TextureManager,
        pipeline: vk::Pipeline,
        wireframe_overlay: bool,
    ) -> EngineResult<()> {
        if vertices.is_empty() || indices.is_empty() {
            return Ok(());
        }

        // Append mesh data to frame buffers and get draw offsets
        let draw_info = self.frame_data.append_mesh_data(vertices, indices)?;

        unsafe {
            self.vulkan_context.device().cmd_bind_pipeline(
                cb,
                vk::PipelineBindPoint::GRAPHICS,
                pipeline,
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

            // Push color override (alpha=0 means use normal rendering)
            let no_override: [f32; 4] = [0.0, 0.0, 0.0, 0.0];
            let override_bytes: &[u8] = std::slice::from_raw_parts(
                no_override.as_ptr() as *const u8,
                std::mem::size_of::<[f32; 4]>(),
            );
            self.vulkan_context.device().cmd_push_constants(
                cb,
                self.pipeline.layout,
                vk::ShaderStageFlags::FRAGMENT,
                64,
                override_bytes,
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

            // Wireframe backface pass: re-draw with wireframe pipeline and solid colour
            if wireframe_overlay && self.debug_wireframe_backfaces {
                self.vulkan_context.device().cmd_bind_pipeline(
                    cb,
                    vk::PipelineBindPoint::GRAPHICS,
                    self.pipeline.wireframe_backface,
                );

                let color_bytes: &[u8] = std::slice::from_raw_parts(
                    self.wireframe_color.as_ptr() as *const u8,
                    std::mem::size_of::<[f32; 4]>(),
                );
                self.vulkan_context.device().cmd_push_constants(
                    cb,
                    self.pipeline.layout,
                    vk::ShaderStageFlags::FRAGMENT,
                    64,
                    color_bytes,
                );

                self.vulkan_context.device().cmd_draw_indexed(
                    cb,
                    draw_info.index_count,
                    1,
                    draw_info.first_index,
                    draw_info.vertex_offset,
                    0,
                );
            }
        }

        Ok(())
    }

    /// Transition from subpass 0 (opaque) to subpass 1 (transparent).
    ///
    /// Must be called after all opaque geometry is drawn and before
    /// water, particles, or overlay rendering.
    /// End the opaque render pass, blit the result to the swapchain image,
    /// transition the color target for sampling, and begin the transparent render pass.
    pub fn begin_transparent_pass(&self, cb: vk::CommandBuffer, image_index: u32) {
        let device = self.vulkan_context.device();
        let extent = self.swapchain.extent;
        let src_image = self.swapchain.color_target.image;
        let dst_image = self.swapchain.swapchain_images[image_index as usize];

        unsafe {
            // End opaque render pass. Color target is now TRANSFER_SRC_OPTIMAL.
            device.cmd_end_render_pass(cb);

            // Transition swapchain image: UNDEFINED → TRANSFER_DST_OPTIMAL.
            let barrier_to_dst = vk::ImageMemoryBarrier::default()
                .image(dst_image)
                .old_layout(vk::ImageLayout::UNDEFINED)
                .new_layout(vk::ImageLayout::TRANSFER_DST_OPTIMAL)
                .dst_access_mask(vk::AccessFlags::TRANSFER_WRITE)
                .subresource_range(
                    vk::ImageSubresourceRange::default()
                        .aspect_mask(vk::ImageAspectFlags::COLOR)
                        .level_count(1)
                        .layer_count(1),
                );

            device.cmd_pipeline_barrier(
                cb,
                vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT,
                vk::PipelineStageFlags::TRANSFER,
                vk::DependencyFlags::empty(),
                &[],
                &[],
                &[barrier_to_dst],
            );

            // Blit offscreen color target → swapchain image.
            let region = vk::ImageBlit {
                src_subresource: vk::ImageSubresourceLayers {
                    aspect_mask: vk::ImageAspectFlags::COLOR,
                    mip_level: 0,
                    base_array_layer: 0,
                    layer_count: 1,
                },
                src_offsets: [
                    vk::Offset3D { x: 0, y: 0, z: 0 },
                    vk::Offset3D {
                        x: extent.width as i32,
                        y: extent.height as i32,
                        z: 1,
                    },
                ],
                dst_subresource: vk::ImageSubresourceLayers {
                    aspect_mask: vk::ImageAspectFlags::COLOR,
                    mip_level: 0,
                    base_array_layer: 0,
                    layer_count: 1,
                },
                dst_offsets: [
                    vk::Offset3D { x: 0, y: 0, z: 0 },
                    vk::Offset3D {
                        x: extent.width as i32,
                        y: extent.height as i32,
                        z: 1,
                    },
                ],
            };

            device.cmd_blit_image(
                cb,
                src_image,
                vk::ImageLayout::TRANSFER_SRC_OPTIMAL,
                dst_image,
                vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                &[region],
                vk::Filter::NEAREST,
            );

            // Transition swapchain image: TRANSFER_DST → COLOR_ATTACHMENT_OPTIMAL
            // (ready for the transparent render pass to composite on top).
            let barrier_to_color = vk::ImageMemoryBarrier::default()
                .image(dst_image)
                .old_layout(vk::ImageLayout::TRANSFER_DST_OPTIMAL)
                .new_layout(vk::ImageLayout::COLOR_ATTACHMENT_OPTIMAL)
                .src_access_mask(vk::AccessFlags::TRANSFER_WRITE)
                .dst_access_mask(vk::AccessFlags::COLOR_ATTACHMENT_WRITE)
                .subresource_range(
                    vk::ImageSubresourceRange::default()
                        .aspect_mask(vk::ImageAspectFlags::COLOR)
                        .level_count(1)
                        .layer_count(1),
                );

            // Transition color target: TRANSFER_SRC → SHADER_READ_ONLY_OPTIMAL
            // (ready to be sampled by the water shader for refraction).
            let barrier_to_read = vk::ImageMemoryBarrier::default()
                .image(src_image)
                .old_layout(vk::ImageLayout::TRANSFER_SRC_OPTIMAL)
                .new_layout(vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL)
                .src_access_mask(vk::AccessFlags::TRANSFER_READ)
                .dst_access_mask(vk::AccessFlags::SHADER_READ)
                .subresource_range(
                    vk::ImageSubresourceRange::default()
                        .aspect_mask(vk::ImageAspectFlags::COLOR)
                        .level_count(1)
                        .layer_count(1),
                );

            device.cmd_pipeline_barrier(
                cb,
                vk::PipelineStageFlags::TRANSFER,
                vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT
                    | vk::PipelineStageFlags::FRAGMENT_SHADER,
                vk::DependencyFlags::empty(),
                &[],
                &[],
                &[barrier_to_color, barrier_to_read],
            );

            // Begin transparent render pass (loads existing color + depth).
            let render_pass_begin = vk::RenderPassBeginInfo::default()
                .render_pass(self.pipeline.transparent_renderpass)
                .framebuffer(self.swapchain.transparent_framebuffers[image_index as usize])
                .render_area(extent.into());

            device.cmd_begin_render_pass(cb, &render_pass_begin, vk::SubpassContents::INLINE);
        }
    }

    /// Render the water surface mesh from a `WaterGrid`.
    ///
    /// Should be called after next_subpass but before particles.
    pub fn render_water(
        &mut self,
        cb: vk::CommandBuffer,
        flow_grid: &WaterGrid,
        wave_grid: &WaveGrid,
        view_matrix: &Matrix4<f32>,
        proj_matrix: &Matrix4<f32>,
        camera_pos: &Vector3<f32>,
        time: f32,
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

        let sun_dir = self.sky_renderer.sun_direction();
        let extent = self.swapchain.extent;
        self.water_renderer.render(
            cb,
            flow_grid,
            wave_grid,
            view_matrix,
            proj_matrix,
            camera_pos,
            &sun_dir,
            time,
            extent.width as f32,
            extent.height as f32,
        )
    }

    /// Run fire simulation compute dispatches. Call after `begin_frame()`
    /// but before `begin_opaque_pass()`.
    pub fn simulate_fire(&mut self, cb: vk::CommandBuffer, dt: f32, total_time: f32) {
        if self.active_fires.is_empty() {
            return;
        }

        let fires: Vec<&ActiveFire> = self.active_fires.iter().map(|(_, f)| f).collect();
        self.fire_renderer.simulate(cb, &fires, dt, total_time);
    }

    /// Render fire volumes via raymarching. Call during transparent pass,
    /// after water and before particles.
    pub fn render_fire(
        &self,
        cb: vk::CommandBuffer,
        view_matrix: &Matrix4<f32>,
        proj_matrix: &Matrix4<f32>,
        camera_pos: &Vector3<f32>,
    ) {
        if self.active_fires.is_empty() {
            return;
        }

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

        let fires: Vec<&ActiveFire> = self.active_fires.iter().map(|(_, f)| f).collect();
        self.fire_renderer
            .render(cb, &fires, view_matrix, proj_matrix, camera_pos);
    }

    /// Create a lightweight fire entry assigned to the least-used sim slot.
    pub fn create_active_fire(
        &mut self,
        entity: specs::Entity,
        volume_to_world: Matrix4<f32>,
        initial_fuel: f32,
    ) {
        let slot = self.fire_renderer.assign_slot(&self.active_fires);
        self.active_fires.push((
            entity,
            ActiveFire {
                sim_slot: slot,
                volume_to_world,
                fuel_remaining: initial_fuel,
                initial_fuel,
                burn_time: 0.0,
            },
        ));
    }

    /// Remove the active fire associated with an entity.
    /// No GPU cleanup needed — sim slots are permanent and shared.
    pub fn remove_active_fire(&mut self, entity: specs::Entity) {
        self.active_fires.retain(|(e, _)| *e != entity);
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

    /// End the frame: finish transparent render pass, submit commands, present.
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
