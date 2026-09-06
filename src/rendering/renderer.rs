//! Main renderer that orchestrates all rendering components.
//!
//! The Renderer is a thin orchestration layer that delegates to:
//! - `GraphicsPipeline` - immutable pipeline state
//! - `FrameOutput` - where finished frames go (a window, or an image)
//! - `FrameTargets` - what frames are drawn into, plus synchronization
//! - `FrameData` - per-frame mutable buffers
//! - `DescriptorManager` - descriptor set management
//!
//! The renderer holds its output behind the `FrameOutput` trait, so it has no
//! idea whether it is driving a window or filling an image for readback. That
//! is what lets the visual bench exercise the real pipeline and the real
//! shaders rather than a stand-in.

use std::sync::Arc;

use ash::vk;
use nalgebra::{Matrix4, Vector3};
use winit::window::Window;

use crate::core::error::{EngineError, EngineResult};
use crate::core::vulkan_context::VulkanContext;
use crate::fire::renderer::{ActiveFire, FireRenderer};
use crate::lighting::ActiveLights;
use crate::model::{Model, Transform};
use crate::particles::{ParticlePool, ParticleRenderer};
use crate::rendering::descriptors::DescriptorManager;
use crate::rendering::frame::{DrawInfo, FrameData, LightUbo, SceneLighting, SceneUbo};
use crate::rendering::material::{
    MaterialManager, SurfaceModulation, SurfaceParams, SURFACE_INDEX_OFFSET,
};
use crate::rendering::overlay::OverlayRenderer;
use crate::rendering::pipeline::{GraphicsPipeline, GraphicsPipelineConfig};
use crate::rendering::post::PostProcessRenderer;
use crate::rendering::shadow::map::SHADOW_SAMPLED_LAYOUT;
use crate::rendering::shadow::{ShadowMap, ShadowRenderer, ShadowVolume, ViewFrustum};
use crate::rendering::sky::SkyRenderer;
use crate::rendering::surface_buffer::SurfaceBuffer;
use crate::rendering::target::frame_targets::DEPTH_FORMAT;
use crate::rendering::target::{
    AcquiredFrame, FrameOutput, FrameTargets, OffscreenOutput, SurfaceInfo, SwapchainOutput,
};
use crate::rendering::vertex::Vertex;
use crate::rendering::water::WaterRenderer;
use crate::resources::textures::{TextureHandle, TextureManager};
use crate::water::{WaterGrid, WaveGrid};

/// Format of the offscreen scene target. Floating point so that emissive
/// surfaces can carry radiance above 1.0 into the post-processing resolve,
/// where tonemapping brings it back into the displayable range.
const SCENE_HDR_FORMAT: vk::Format = vk::Format::R16G16B16A16_SFLOAT;

/// Which of the frame's two geometry passes a draw belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DrawPass {
    /// Depth-tested, depth-writing, no blending. Into the HDR scene target.
    Opaque,
    /// Alpha blended, no depth write. Into the composited output image.
    Transparent,
}

/// How a mesh draw participates in the frame beyond issuing its own triangles.
///
/// Grouped rather than passed as loose flags so that adding a pass does not
/// grow the signature of every draw entry point.
#[derive(Clone, Copy, Debug)]
struct DrawOptions {
    pass: DrawPass,
    /// Whether the debug backface wireframe is drawn over this mesh, when the
    /// renderer has that mode on.
    wireframe_overlay: bool,
    /// Whether the mesh is also recorded into the sun shadow pass.
    casts_shadow: bool,
}

impl DrawOptions {
    /// Ordinary solid geometry: terrain, props, characters.
    const OPAQUE: Self = Self {
        pass: DrawPass::Opaque,
        wireframe_overlay: true,
        casts_shadow: true,
    };

    /// Blended geometry. Casts nothing: a shadow map stores one depth per
    /// texel and has no way to express partial occlusion, so a translucent
    /// caster would throw the solid shadow it visibly does not have.
    const TRANSPARENT: Self = Self {
        pass: DrawPass::Transparent,
        wireframe_overlay: false,
        casts_shadow: false,
    };
}

/// The world-space direction the camera looks along, recovered from its view
/// matrix.
///
/// A right-handed view matrix's third row is the camera's backward axis, so
/// negating it gives forward. Taken from the matrix rather than plumbed through
/// separately, so the shadow pass cannot end up aimed at a different frame than
/// the one being drawn.
fn camera_forward(view: &Matrix4<f32>) -> Vector3<f32> {
    -Vector3::new(view[(2, 0)], view[(2, 1)], view[(2, 2)])
}

// The shadow pass also needs the *shape* of the view cone, which
// `ViewFrustum::from_projection` recovers from the projection matrix on the
// same principle as `camera_forward` above: taking both from the matrices the
// frame is drawn with means the shadow box cannot end up fitted to a different
// camera than the one being rendered. It lives beside the type it builds
// because that is where it is unit-tested against the matrix convention.

/// The main renderer that orchestrates frame rendering.
///
/// This is a thin layer that coordinates the pipeline, output, frame targets,
/// frame data and descriptor management to render frames.
pub struct Renderer {
    pub pipeline: GraphicsPipeline,
    /// Where finished frames go. Boxed rather than generic so that a caller
    /// choosing between a window and an offscreen image at runtime does not
    /// force the choice through every type that holds a renderer.
    pub output: Box<dyn FrameOutput>,
    pub targets: FrameTargets,
    pub frame_data: FrameData,
    /// This frame's surface parameters, one entry per draw. Bound to the scene
    /// descriptor set; draws carry only an index into it.
    surfaces: SurfaceBuffer,
    pub descriptors: Arc<DescriptorManager>,
    pub vulkan_context: Arc<VulkanContext>,
    pub overlay: OverlayRenderer,
    pub particle_renderer: ParticleRenderer,
    pub sky_renderer: SkyRenderer,
    pub water_renderer: WaterRenderer,
    pub fire_renderer: FireRenderer,
    /// HDR resolve: tonemapping and bloom between the opaque and transparent passes.
    pub post_process: PostProcessRenderer,
    /// Sun shadow map, filled from the same draws the opaque pass issues.
    pub shadow: ShadowRenderer,
    /// Active fire instances with their GPU resources. Keyed by entity index.
    pub active_fires: Vec<(specs::Entity, ActiveFire)>,
    /// Scene lighting environment uploaded to the scene UBO each frame.
    lighting: SceneLighting,
    /// When true, backfaces are rendered in wireframe with `wireframe_color`.
    pub debug_wireframe_backfaces: bool,
    /// The solid color used for wireframe backface rendering (RGBA, 0-1).
    pub wireframe_color: [f32; 4],
    /// The output image this frame is being rendered into, between
    /// `begin_frame` and `end_frame`.
    current_frame: Option<AcquiredFrame>,
}

impl Renderer {
    /// Create a renderer that presents to a window.
    pub fn for_window(
        vulkan_context: Arc<VulkanContext>,
        window: &Window,
        window_width: u32,
        window_height: u32,
    ) -> EngineResult<Self> {
        let surface_info = SurfaceInfo::new(&vulkan_context, window)?;
        let output =
            SwapchainOutput::new(&vulkan_context, surface_info, window_width, window_height)?;

        Self::new(vulkan_context, Box::new(output))
    }

    /// Create a renderer that draws into an engine-owned image, for readback.
    ///
    /// The returned renderer is identical to the windowed one in every respect
    /// that affects pixels — same pipeline, same shaders, same post chain — so
    /// what it produces can be trusted as what the game would show.
    pub fn offscreen(
        vulkan_context: Arc<VulkanContext>,
        width: u32,
        height: u32,
    ) -> EngineResult<Self> {
        let output = OffscreenOutput::new(Arc::clone(&vulkan_context), width, height)?;
        Self::new(vulkan_context, Box::new(output))
    }

    /// Create a renderer over an already-built output.
    pub fn new(
        vulkan_context: Arc<VulkanContext>,
        output: Box<dyn FrameOutput>,
    ) -> EngineResult<Self> {
        let extent = output.extent();

        // The shadow map comes before the pipeline, which needs its comparison
        // sampler to build the scene descriptor set layout around.
        let shadow_volume = ShadowVolume::default();
        let shadow_map = ShadowMap::new(&vulkan_context, shadow_volume.resolution)?;

        // The pipeline comes next: its render passes are what the framebuffers
        // in `FrameTargets` are built against.
        let pipeline_config = GraphicsPipelineConfig {
            scene_color_format: SCENE_HDR_FORMAT,
            swapchain_format: output.format(),
            depth_format: DEPTH_FORMAT,
            extent,
            shadow_sampler: shadow_map.sampler,
        };

        let pipeline = GraphicsPipeline::new(Arc::clone(&vulkan_context.device), &pipeline_config)?;

        let targets = FrameTargets::new(
            &vulkan_context,
            output.as_ref(),
            pipeline.renderpass,
            pipeline.transparent_renderpass,
            SCENE_HDR_FORMAT,
        )?;

        // Post-processing resolves the HDR scene target onto the output image.
        let post_process = PostProcessRenderer::new(
            &vulkan_context,
            targets.color_target.view,
            output.image_views(),
            extent,
            output.format(),
            output.final_layout(),
        )?;

        // The shadow pass shares the geometry pipeline's layout, so it can
        // replay the same draw calls with the same push constants.
        let shadow =
            ShadowRenderer::new(&vulkan_context, shadow_map, pipeline.layout, shadow_volume)?;

        // Create frame data (vertex/index/UBO buffers)
        let frame_data = FrameData::new(Arc::clone(&vulkan_context.device))?;

        // The table every draw's shading parameters go into.
        let surfaces = SurfaceBuffer::new(Arc::clone(&vulkan_context.device))?;

        // Create descriptor manager
        let descriptors = Arc::new(DescriptorManager::new(
            Arc::clone(&vulkan_context.device),
            pipeline.scene_ubo_descriptor_set_layout,
            pipeline.sampler_descriptor_set_layout,
            100, // initial texture descriptor capacity
        )?);

        // Initialize the UBO descriptors
        descriptors.update_scene_ubo(
            &frame_data.scene_ubo_buffer,
            std::mem::size_of::<SceneUbo>() as vk::DeviceSize,
        );
        descriptors.update_light_ubo(
            &frame_data.light_ubo_buffer,
            std::mem::size_of::<LightUbo>() as vk::DeviceSize,
        );
        descriptors.update_shadow_map(shadow.map().view, SHADOW_SAMPLED_LAYOUT);
        descriptors.update_surface_table(surfaces.buffer(), SurfaceBuffer::SIZE);

        // Create overlay renderer for debug text (transparent pass)
        let overlay = OverlayRenderer::new(
            Arc::clone(&vulkan_context),
            pipeline.transparent_renderpass,
            extent.width,
            extent.height,
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
            targets.depth_buffer.view,
            targets.color_target.view,
        )?;

        // Create fire renderer with shared sim pool
        let fire_renderer = FireRenderer::new(
            Arc::clone(&vulkan_context.device),
            pipeline.transparent_renderpass,
            targets.depth_buffer.view,
        )?;

        Ok(Self {
            pipeline,
            output,
            targets,
            frame_data,
            surfaces,
            descriptors,
            vulkan_context,
            overlay,
            particle_renderer,
            sky_renderer,
            water_renderer,
            fire_renderer,
            post_process,
            shadow,
            active_fires: Vec::new(),
            lighting: SceneLighting::default(),
            debug_wireframe_backfaces: false,
            wireframe_color: [0.0, 0.0, 0.0, 1.0],
            current_frame: None,
        })
    }

    /// The extent every pass renders at.
    pub fn extent(&self) -> vk::Extent2D {
        self.targets.extent
    }

    /// Get the descriptor manager (for creating texture managers)
    pub fn descriptor_manager(&self) -> Arc<DescriptorManager> {
        Arc::clone(&self.descriptors)
    }

    /// Begin a new frame: wait for the previous frame, acquire an output image,
    /// start the command buffer. Call `begin_opaque_pass()` after any pre-pass
    /// compute work (e.g. fire simulation) is recorded.
    pub fn begin_frame(&mut self) -> EngineResult<(vk::CommandBuffer, u32)> {
        // Wait for previous frame to complete. The fence is *not* reset here —
        // `end_frame` resets it immediately before the submit that re-signals
        // it, so a frame abandoned in between leaves it signalled rather than
        // stranding every later frame on a signal that never arrives.
        self.targets.sync.wait()?;

        // Now that the GPU is done with previous frames, flush deferred deletions
        self.frame_data.begin_frame();
        self.surfaces.begin_frame();

        // Everything fallible that costs nothing to redo goes first, so the
        // acquire is the last step that can fail. An acquired swapchain image
        // has to be handed back by a present; if a later step in here failed we
        // would be holding one with no way to return it, and after a few frames
        // the acquire would block on an exhausted pool.
        //
        // Both command buffers may still be recording from a frame that was
        // abandoned mid-flight. `begin` implicitly resets them (the pool is
        // created with RESET_COMMAND_BUFFER), so that state is self-healing.

        // The shadow pass records into its own command buffer, filled by the
        // same draw calls that fill the geometry one. Opening it here means a
        // caller cannot forget to.
        self.shadow.begin_frame()?;

        // Begin command buffer recording
        self.targets
            .draw_command_buffer
            .begin(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT)?;

        let frame = self.output.acquire(&self.targets.sync)?;
        let image_index = frame.index;
        self.current_frame = Some(frame);

        let cb = self.targets.draw_command_buffer.raw();

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
            .framebuffer(self.targets.opaque_framebuffer)
            .render_area(self.targets.extent.into())
            .clear_values(&clear_values);

        unsafe {
            self.vulkan_context.device().cmd_begin_render_pass(
                cb,
                &render_pass_begin,
                vk::SubpassContents::INLINE,
            );
        }
    }

    /// Update per-frame scene data (camera and lighting).
    /// Call this once at the start of each frame, before any draw calls.
    ///
    /// The sun direction is taken from the sky renderer so that shaded geometry
    /// and the visible sun disc always agree.
    ///
    /// Also aims the shadow pass, which needs the same camera and sun. Doing it
    /// here rather than in `begin_frame` costs nothing — the light matrix is
    /// uniform data, and casters are transformed by it at draw time, not when
    /// they were recorded.
    pub fn update_scene(
        &mut self,
        view: &Matrix4<f32>,
        proj: &Matrix4<f32>,
        camera_pos: &Vector3<f32>,
    ) -> EngineResult<()> {
        let sun_direction = self.sky_renderer.sun_direction();
        let lighting = SceneLighting {
            sun_direction,
            ..self.lighting
        };

        self.shadow.aim(
            &ViewFrustum::from_projection(proj),
            camera_pos,
            &camera_forward(view),
            &sun_direction,
        );

        self.frame_data
            .update_scene_ubo(view, proj, camera_pos, &lighting, &self.shadow.uniforms())
    }

    /// Upload the frame's point light set.
    ///
    /// Call once per frame alongside `update_scene`. The buffer is already
    /// bound to set 0, binding 1; this only refreshes its contents.
    pub fn update_lights(&mut self, active: &ActiveLights) -> EngineResult<()> {
        self.frame_data.update_light_ubo(active)
    }

    /// Mutable access to the scene lighting environment (sun colour, ambient,
    /// intensity). The sun *direction* is owned by the sky renderer.
    pub fn lighting_mut(&mut self) -> &mut SceneLighting {
        &mut self.lighting
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
        let extent = self.targets.extent;
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
        modulation: SurfaceModulation,
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
                let surface = material_manager
                    .get_surface_params(primitive.material)
                    .modulated(modulation);

                self.draw_mesh_with_texture(
                    cb,
                    &primitive.vertices,
                    &primitive.indices,
                    &final_transform,
                    texture,
                    surface,
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
            SurfaceParams::MATTE,
            texture_manager,
            DrawOptions::OPAQUE,
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
            SurfaceParams::MATTE,
            texture_manager,
            DrawOptions::TRANSPARENT,
        )
    }

    /// Draw a mesh with a specific texture handle and lighting parameters.
    ///
    /// This is a lower-level method used by draw_model and terrain rendering.
    pub fn draw_mesh_with_texture(
        &mut self,
        cb: vk::CommandBuffer,
        vertices: &[Vertex],
        indices: &[u32],
        model: &Matrix4<f32>,
        texture: &TextureHandle,
        surface: SurfaceParams,
        texture_manager: &TextureManager,
    ) -> EngineResult<()> {
        self.draw_mesh_internal(
            cb,
            vertices,
            indices,
            model,
            texture,
            surface,
            texture_manager,
            DrawOptions::OPAQUE,
        )
    }

    /// Record a mesh into the sun shadow pass.
    ///
    /// The mesh data is already in this frame's vertex and index buffers — the
    /// geometry draw that owns it put it there — so this costs one more draw
    /// call and no extra upload.
    fn record_shadow_caster(&self, model: &Matrix4<f32>, draw_info: &DrawInfo) {
        self.shadow.record_caster(
            self.vulkan_context.device(),
            model,
            draw_info,
            self.frame_data.vertex_buffer.buffer,
            self.frame_data.index_buffer.buffer,
            self.descriptors.scene_ubo_set,
        );
    }

    /// Record a mesh draw call with the specified pipeline.
    fn draw_mesh_internal(
        &mut self,
        cb: vk::CommandBuffer,
        vertices: &[Vertex],
        indices: &[u32],
        model: &Matrix4<f32>,
        texture: &TextureHandle,
        surface: SurfaceParams,
        texture_manager: &TextureManager,
        options: DrawOptions,
    ) -> EngineResult<()> {
        if vertices.is_empty() || indices.is_empty() {
            return Ok(());
        }

        // Append mesh data to frame buffers and get draw offsets
        let draw_info = self.frame_data.append_mesh_data(vertices, indices)?;

        // Park this draw's shading parameters in the frame's surface table.
        let surface_index = self.surfaces.push(surface.to_gpu());

        if options.casts_shadow {
            self.record_shadow_caster(model, &draw_info);
        }

        let pipeline = match options.pass {
            DrawPass::Opaque => self.pipeline.opaque,
            DrawPass::Transparent => self.pipeline.transparent,
        };

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
                width: self.targets.extent.width as f32,
                height: self.targets.extent.height as f32,
                min_depth: 0.0,
                max_depth: 1.0,
            }];
            let scissors = [self.targets.extent.into()];

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

            // Push where this draw's parameters landed in the surface table.
            // The parameters themselves went into the table above; only this
            // index travels through the push constants.
            self.vulkan_context.device().cmd_push_constants(
                cb,
                self.pipeline.layout,
                vk::ShaderStageFlags::FRAGMENT,
                SURFACE_INDEX_OFFSET,
                &surface_index.as_bytes(),
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
            if options.wireframe_overlay && self.debug_wireframe_backfaces {
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

    /// End the opaque render pass, resolve the HDR scene onto the output image
    /// (tonemap + bloom), and begin the transparent render pass.
    ///
    /// Must be called after all opaque geometry is drawn and before water,
    /// particles, or overlay rendering.
    ///
    /// The opaque render pass leaves the HDR colour target in
    /// `SHADER_READ_ONLY_OPTIMAL`, and the composite pass leaves the output
    /// image in `COLOR_ATTACHMENT_OPTIMAL`, so no manual barriers are needed
    /// between the three passes.
    pub fn begin_transparent_pass(&self, cb: vk::CommandBuffer, image_index: u32) {
        let device = self.vulkan_context.device();
        let extent = self.targets.extent;

        unsafe {
            device.cmd_end_render_pass(cb);

            self.post_process.resolve(cb, image_index, extent);

            // Begin transparent render pass (loads composited colour + depth).
            let render_pass_begin = vk::RenderPassBeginInfo::default()
                .render_pass(self.pipeline.transparent_renderpass)
                .framebuffer(self.targets.transparent_framebuffers[image_index as usize])
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
        let extent = self.targets.extent;
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
        let extent = self.targets.extent;
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
            self.post_process.config.hue_preservation,
            self.post_process.config.exposure,
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

        let extent = self.targets.extent;
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
        let extent = self.targets.extent;
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
        let extent = self.targets.extent;
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

    /// End the frame: finish the transparent render pass, submit, and hand the
    /// image to the output.
    pub fn end_frame(&mut self, cb: vk::CommandBuffer, image_index: u32) -> EngineResult<()> {
        let frame = self
            .current_frame
            .take()
            .ok_or_else(|| EngineError::Swapchain("end_frame without begin_frame".to_string()))?;

        unsafe {
            self.vulkan_context.device().cmd_end_render_pass(cb);
        }

        // Bloom goes on last so transparent surfaces cannot paint over a halo
        // that belongs in front of them. This is also the pass that transitions
        // the output image into the layout its consumer expects.
        self.post_process
            .apply_bloom(cb, image_index, self.targets.extent);

        self.targets.draw_command_buffer.end()?;

        // Closes the pass that has been collecting casters alongside every
        // opaque draw this frame.
        self.shadow.end_frame()?;

        // Only a swapchain acquire produces semaphores to synchronize against;
        // an engine-owned image is ready the moment it is asked for, and the
        // draw fence alone orders one frame against the next.
        let wait: Vec<vk::Semaphore> = frame.wait.into_iter().collect();
        let signal: Vec<vk::Semaphore> = frame.signal.into_iter().collect();
        let wait_stages = vec![vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT; wait.len()];

        self.targets.sync.reset()?;

        // The shadow map goes first: the geometry pass samples it, and the
        // ordering plus the shadow pass's own external dependency are what make
        // that read see this frame's contents rather than the last one's.
        self.vulkan_context
            .command_buffer_manager
            .submit_recorded_graphics_batch_async(
                &[
                    self.shadow.command_buffer(),
                    &self.targets.draw_command_buffer,
                ],
                self.targets.sync.draw_fence,
                &wait,
                &signal,
                &wait_stages,
            )?;

        self.output.release(
            &frame,
            self.vulkan_context.command_buffer_manager.graphics_queue,
        )
    }

    /// Block until the frame submitted by `end_frame` has finished on the GPU.
    ///
    /// Only meaningful for outputs that are read back rather than presented;
    /// the windowed path lets the next `begin_frame` do the waiting.
    pub fn wait_for_frame(&self) -> EngineResult<()> {
        self.targets.sync.wait()
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
