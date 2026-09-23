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
use std::time::Instant;

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
use crate::rendering::overlay::{OverlayGeometry, OverlayRenderer};
use crate::rendering::pipeline::{GraphicsPipeline, GraphicsPipelineConfig};
use crate::rendering::post::PostProcessRenderer;
use crate::rendering::profile::{GpuSpan, GpuTimer, RenderProfile, RenderStage};
use crate::rendering::shadow::map::SHADOW_SAMPLED_LAYOUT;
use crate::rendering::shadow::{ShadowMap, ShadowRenderer, ShadowVolume, ViewFrustum};
use crate::rendering::sky::SkyRenderer;
use crate::rendering::surface_buffer::SurfaceBuffer;
use crate::rendering::target::frame_targets::DEPTH_FORMAT;
use crate::rendering::target::{
    AcquiredFrame, FrameOutput, FrameTargets, OffscreenOutput, SurfaceInfo, SwapchainOutput,
};
use crate::rendering::transparency::{BlendedDraw, MeshBounds, TransparentQueue};
use crate::rendering::vertex::Vertex;
use crate::rendering::water::WaterRenderer;
use crate::resources::textures::{TextureHandle, TextureManager};
use crate::water::{WaterGrid, WaveGrid};

/// Format of the offscreen scene target. Floating point so that emissive
/// surfaces can carry radiance above 1.0 into the post-processing resolve,
/// where tonemapping brings it back into the displayable range.
const SCENE_HDR_FORMAT: vk::Format = vk::Format::R16G16B16A16_SFLOAT;

/// Stages the geometry pipeline's single push-constant range covers.
///
/// One range spanning both stages, so every `cmd_push_constants` into this
/// layout must name both — Vulkan requires a push to cover every stage of every
/// range it overlaps, whichever stage actually reads the bytes.
pub const PUSH_CONSTANT_STAGES: vk::ShaderStageFlags = vk::ShaderStageFlags::from_raw(
    vk::ShaderStageFlags::VERTEX.as_raw() | vk::ShaderStageFlags::FRAGMENT.as_raw(),
);

/// Which of the frame's geometry passes a draw belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DrawPass {
    /// Depth-tested, depth-writing, no blending. Into the HDR scene target.
    Opaque,
    /// Alpha blended, no depth write, into the HDR scene target after every
    /// opaque draw. Held back and sorted rather than recorded where it is
    /// issued; see `rendering::transparency`.
    ///
    /// Not a pass a caller asks for — a draw lands here because its *material*
    /// lets light through, which is the only thing that can decide it. The
    /// alternative would let a spawnable declare ice and still be drawn opaque
    /// by a call site that forgot.
    SceneBlended,
    /// Alpha blended, no depth write. Into the composited output image, after
    /// tonemapping, where debug overlays and particles live.
    Overlay,
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
    ///
    /// `casts_shadow` is what the *pass* asks for; a surface that lets too
    /// much light through overrides it, because a shadow map can only store
    /// a fully solid shadow. See `draw_mesh_internal`.
    const OPAQUE: Self = Self {
        pass: DrawPass::Opaque,
        wireframe_overlay: true,
        casts_shadow: true,
    };

    /// Debug overlay geometry, composited after tonemapping. Casts nothing:
    /// a shadow map stores one depth per texel and has no way to express
    /// partial occlusion, and a debug shape has no business in the scene's
    /// lighting anyway.
    const OVERLAY: Self = Self {
        pass: DrawPass::Overlay,
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
    /// Blended scene draws held back for the sorted flush at the end of the
    /// opaque pass.
    transparent_queue: TransparentQueue,
    /// Where the camera is this frame, as `update_scene` was told. Held
    /// because sorting blended draws needs it and a draw call has no reason
    /// to be handed it again.
    camera_pos: Vector3<f32>,
    /// Scene lighting environment uploaded to the scene UBO each frame.
    lighting: SceneLighting,
    /// When true, backfaces are rendered in wireframe with `wireframe_color`.
    pub debug_wireframe_backfaces: bool,
    /// The solid color used for wireframe backface rendering (RGBA, 0-1).
    pub wireframe_color: [f32; 4],
    /// The output image this frame is being rendered into, between
    /// `begin_frame` and `end_frame`.
    current_frame: Option<AcquiredFrame>,
    /// Timestamps the frame's GPU spans.
    gpu_timer: GpuTimer,
    /// The frame being recorded: stage times and counts so far.
    profile: RenderProfile,
    /// The last frame to finish on the GPU, complete with its GPU times.
    last_profile: RenderProfile,
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

        // Create particle renderer (scene pass, interleaved with blended geometry)
        let particle_renderer =
            ParticleRenderer::new(Arc::clone(&vulkan_context), pipeline.renderpass)?;

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

        let gpu_timer = GpuTimer::new(&vulkan_context)?;

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
            transparent_queue: TransparentQueue::new(),
            camera_pos: Vector3::zeros(),
            lighting: SceneLighting::default(),
            debug_wireframe_backfaces: false,
            wireframe_color: [0.0, 0.0, 0.0, 1.0],
            current_frame: None,
            gpu_timer,
            profile: RenderProfile::default(),
            last_profile: RenderProfile::default(),
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
        let fence_wait = Instant::now();
        self.targets.sync.wait()?;
        let fence_wait = fence_wait.elapsed();

        // The previous frame is now finished on both sides, so its GPU times
        // can be read without stalling and its profile is complete.
        self.profile.gpu = self.gpu_timer.collect();
        self.last_profile = std::mem::take(&mut self.profile);
        self.profile.record(RenderStage::FenceWait, fence_wait);

        // Now that the GPU is done with previous frames, flush deferred deletions
        self.frame_data.begin_frame();
        self.surfaces.begin_frame();
        // Must be rewound with the buffers it indexes into: a held-over entry
        // would point at geometry that is about to be overwritten.
        self.transparent_queue.begin_frame();
        self.particle_renderer.begin_frame();

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
        self.shadow.begin_frame(&self.gpu_timer)?;

        // Begin command buffer recording
        self.targets
            .draw_command_buffer
            .begin(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT)?;

        let acquire = Instant::now();
        let frame = self.output.acquire(&self.targets.sync)?;
        self.profile.record(RenderStage::Acquire, acquire.elapsed());
        let image_index = frame.index;
        self.current_frame = Some(frame);

        let cb = self.targets.draw_command_buffer.raw();
        self.gpu_timer.begin(cb, GpuSpan::FireSim);

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

        self.gpu_timer.end(cb, GpuSpan::FireSim);
        self.gpu_timer.begin(cb, GpuSpan::Scene);

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
        self.camera_pos = *camera_pos;

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
            DrawOptions::OVERLAY,
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

    /// Commit a mesh to the frame and record it, or hold it back.
    ///
    /// Everything a draw needs is committed here either way — the geometry
    /// into the frame's buffers, the shading parameters into its surface
    /// table, the caster into the shadow pass — because all three are
    /// order-independent. Only the *recording* of the colour draw depends on
    /// what else the frame contains, and only for blended surfaces, so only
    /// that part is deferred.
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
        let buffer_sizes = self.frame_data.mesh_buffer_sizes();
        let draw_info = self.frame_data.append_mesh_data(vertices, indices)?;
        self.profile.counters.record_upload(
            std::mem::size_of_val(vertices) + std::mem::size_of_val(indices),
            self.frame_data.mesh_buffer_sizes() != buffer_sizes,
        );

        // Park this draw's shading parameters in the frame's surface table.
        let surface_index = self.surfaces.push(surface.to_gpu());

        // A shadow map stores one depth per texel and has no way to express
        // partial occlusion, so a transmissive caster can only throw a fully
        // solid shadow. Whether that is better or worse than none is the
        // material's call, not the pass's — a cloudy block wants the shadow,
        // a window would only put a hard black bite behind itself.
        if options.casts_shadow && surface.transparency.casts_shadow() {
            self.record_shadow_caster(model, &draw_info);
            if self.shadow.enabled {
                self.profile.counters.shadow_casters += 1;
            }
        }

        let texture_set = texture_manager
            .get_or_create_descriptor_set(texture)
            .map_err(|e| crate::core::error::EngineError::Texture {
                path: None,
                reason: format!("descriptor set creation: {}", e),
            })?;

        let blended = BlendedDraw::new(*model, draw_info, surface_index, texture_set);

        // A material that lets light through overrides the scene pass it was
        // asked for. The call site does not know what it is drawing — a model
        // carries whatever material its spawnable gave it — so the decision
        // belongs to the surface.
        let pass = match options.pass {
            DrawPass::Opaque if surface.transparency.is_blended() => DrawPass::SceneBlended,
            other => other,
        };

        self.profile.counters.triangles += (indices.len() / 3) as u64;

        match pass {
            DrawPass::SceneBlended => {
                self.profile.counters.blended_draws += 1;
                self.transparent_queue
                    .push(blended.sorted_from(&self.camera_pos, &MeshBounds::of(vertices)));
            }
            DrawPass::Opaque => {
                self.profile.counters.opaque_draws += 1;
                self.record_geometry_draw(cb, self.pipeline.opaque, &blended);
                self.record_wireframe_overlay(cb, &blended, options);
            }
            DrawPass::Overlay => {
                self.profile.counters.overlay_draws += 1;
                self.record_geometry_draw(cb, self.pipeline.transparent, &blended);
                self.record_wireframe_overlay(cb, &blended, options);
            }
        }

        Ok(())
    }

    /// Set the dynamic viewport and scissor to the whole target.
    fn set_full_viewport(&self, cb: vk::CommandBuffer) {
        let extent = self.targets.extent;
        let viewport = vk::Viewport {
            x: 0.0,
            y: 0.0,
            width: extent.width as f32,
            height: extent.height as f32,
            min_depth: 0.0,
            max_depth: 1.0,
        };

        unsafe {
            let device = self.vulkan_context.device();
            device.cmd_set_viewport(cb, 0, &[viewport]);
            device.cmd_set_scissor(cb, 0, &[extent.into()]);
        }
    }

    /// Record the frame's blended surfaces, farthest first.
    ///
    /// Call once, after the last opaque draw and before the opaque pass ends —
    /// these draws go into the HDR scene target, so that glass is exposed,
    /// tonemapped and bloomed with everything behind it rather than pasted on
    /// after the resolve.
    ///
    /// Two streams are merged here, both ordered by distance from the camera:
    /// the blended meshes held back by the queue, and the frame's particles.
    ///
    /// ```text
    ///   geometry:  ────────■──────────────■────────────▶ near
    ///   particles: ░░░░░░░░ ░░░░░░░░░░░░░░ ░░░░░░░░░░░░
    ///              └ drawn ┘              └ drawn after the glass in front ┘
    /// ```
    ///
    /// Merging them is what lets an explosion read through an ice wall: a
    /// particle behind the glass is composited first and the glass tints it,
    /// and one in front is composited over the glass. Drawing all the particles
    /// on either side of the glass gets one of those two cases wrong.
    ///
    /// Each mesh is recorded twice, back faces then front faces. Sorting can
    /// only order whole draws, and a closed mesh contains its own far and near
    /// surfaces; splitting them by cull mode is what puts those two in order.
    fn flush_scene_transparency(&mut self, cb: vk::CommandBuffer) {
        if self.transparent_queue.is_empty() && self.particle_renderer.is_empty() {
            return;
        }

        // Taken out of the queue so the recording loop is not holding a borrow
        // of `self` through calls that need `&self` for the device.
        let draws: Vec<BlendedDraw> = self.transparent_queue.sorted().to_vec();
        let exposure = self.post_process.config.exposure;

        // The particle pipeline takes its viewport dynamically and the flush
        // may record a particle before any mesh has set one.
        self.set_full_viewport(cb);

        let mut particles_drawn = 0;
        for draw in &draws {
            // Everything behind this mesh goes down before it does.
            let behind = self.particle_renderer.count_beyond(draw.depth_key());
            if behind > particles_drawn {
                self.particle_renderer.bind(cb, exposure);
                self.particle_renderer
                    .draw_range(cb, particles_drawn, behind);
                particles_drawn = behind;
            }

            self.record_geometry_draw(cb, self.pipeline.scene_blended_back, draw);
            self.record_geometry_draw(cb, self.pipeline.scene_blended_front, draw);
        }

        if !self.particle_renderer.is_empty() {
            self.particle_renderer.bind(cb, exposure);
            self.particle_renderer
                .draw_range(cb, particles_drawn, usize::MAX);
        }
    }

    /// Bind the given pipeline and issue one mesh's draw call.
    ///
    /// The shared tail of every geometry draw, whichever pass and whenever it
    /// was decided: state that varies per draw is pushed here, and nothing
    /// about it depends on when the draw was committed.
    fn record_geometry_draw(
        &self,
        cb: vk::CommandBuffer,
        pipeline: vk::Pipeline,
        draw: &BlendedDraw,
    ) {
        let device = self.vulkan_context.device();

        unsafe {
            device.cmd_bind_pipeline(cb, vk::PipelineBindPoint::GRAPHICS, pipeline);

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

            device.cmd_set_viewport(cb, 0, &viewports);
            device.cmd_set_scissor(cb, 0, &scissors);

            // Push model matrix (per-draw data)
            let model_bytes: &[u8] = std::slice::from_raw_parts(
                draw.model.as_ptr() as *const u8,
                std::mem::size_of::<Matrix4<f32>>(),
            );
            device.cmd_push_constants(
                cb,
                self.pipeline.layout,
                // The range is declared for both stages — the fragment shader
                // reads the model rotation to place an object-space grain — so
                // the push must name both, even though only the vertex stage
                // uses it here.
                PUSH_CONSTANT_STAGES,
                0,
                model_bytes,
            );

            // Push color override (alpha=0 means use normal rendering)
            let no_override: [f32; 4] = [0.0, 0.0, 0.0, 0.0];
            let override_bytes: &[u8] = std::slice::from_raw_parts(
                no_override.as_ptr() as *const u8,
                std::mem::size_of::<[f32; 4]>(),
            );
            device.cmd_push_constants(
                cb,
                self.pipeline.layout,
                PUSH_CONSTANT_STAGES,
                64,
                override_bytes,
            );

            // Push where this draw's parameters landed in the surface table.
            // The parameters themselves went into the table above; only this
            // index travels through the push constants.
            device.cmd_push_constants(
                cb,
                self.pipeline.layout,
                PUSH_CONSTANT_STAGES,
                SURFACE_INDEX_OFFSET,
                &draw.surface_index.as_bytes(),
            );

            // Bind descriptor sets
            let descriptor_sets = [self.descriptors.scene_ubo_set, draw.texture_set];
            device.cmd_bind_descriptor_sets(
                cb,
                vk::PipelineBindPoint::GRAPHICS,
                self.pipeline.layout,
                0,
                &descriptor_sets,
                &[],
            );

            // Bind vertex and index buffers
            device.cmd_bind_vertex_buffers(cb, 0, &[self.frame_data.vertex_buffer.buffer], &[0]);
            device.cmd_bind_index_buffer(
                cb,
                self.frame_data.index_buffer.buffer,
                0,
                vk::IndexType::UINT32,
            );

            device.cmd_draw_indexed(
                cb,
                draw.draw.index_count,
                1,
                draw.draw.first_index,
                draw.draw.vertex_offset,
                0,
            );
        }
    }

    /// Re-draw a mesh's backfaces in wireframe, when that debug mode is on.
    ///
    /// Follows the mesh's own draw and reuses everything it just bound, so
    /// only the pipeline and the colour override change.
    fn record_wireframe_overlay(
        &self,
        cb: vk::CommandBuffer,
        draw: &BlendedDraw,
        options: DrawOptions,
    ) {
        if !options.wireframe_overlay || !self.debug_wireframe_backfaces {
            return;
        }

        let device = self.vulkan_context.device();

        unsafe {
            device.cmd_bind_pipeline(
                cb,
                vk::PipelineBindPoint::GRAPHICS,
                self.pipeline.wireframe_backface,
            );

            let color_bytes: &[u8] = std::slice::from_raw_parts(
                self.wireframe_color.as_ptr() as *const u8,
                std::mem::size_of::<[f32; 4]>(),
            );
            device.cmd_push_constants(
                cb,
                self.pipeline.layout,
                PUSH_CONSTANT_STAGES,
                64,
                color_bytes,
            );

            device.cmd_draw_indexed(
                cb,
                draw.draw.index_count,
                1,
                draw.draw.first_index,
                draw.draw.vertex_offset,
                0,
            );
        }
    }
    /// End the opaque render pass, resolve the HDR scene onto the output image
    /// (tonemap + bloom), and begin the transparent render pass.
    ///
    /// Must be called after all opaque geometry is drawn and before water,
    /// particles, or overlay rendering.
    ///
    /// The frame's blended scene geometry is recorded here, on the way out of
    /// the opaque pass: it belongs to the HDR target, and this is the one
    /// point every caller already passes through on leaving it, so no call
    /// site has to remember to flush the queue itself.
    ///
    /// The opaque render pass leaves the HDR colour target in
    /// `SHADER_READ_ONLY_OPTIMAL`, and the composite pass leaves the output
    /// image in `COLOR_ATTACHMENT_OPTIMAL`, so no manual barriers are needed
    /// between the three passes.
    pub fn begin_transparent_pass(&mut self, cb: vk::CommandBuffer, image_index: u32) {
        let flush = Instant::now();
        self.flush_scene_transparency(cb);
        self.profile
            .record(RenderStage::TransparencyFlush, flush.elapsed());

        let device = self.vulkan_context.device();
        let extent = self.targets.extent;

        unsafe {
            device.cmd_end_render_pass(cb);
            self.gpu_timer.end(cb, GpuSpan::Scene);

            self.gpu_timer.begin(cb, GpuSpan::Resolve);
            self.post_process.resolve(cb, image_index, extent);
            self.gpu_timer.end(cb, GpuSpan::Resolve);
            self.gpu_timer.begin(cb, GpuSpan::Composite);

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

    /// Hand the frame's particles to the renderer, to be recorded with the
    /// rest of the scene's blended geometry.
    ///
    /// Records nothing itself. Particles are blended surfaces in the scene, so
    /// they belong in the same sorted flush as glass and ice — see
    /// [`Self::flush_scene_transparency`] — and that flush cannot run until
    /// every blended draw of the frame is known.
    pub fn submit_particles(
        &mut self,
        pool: &ParticlePool,
        view_matrix: &Matrix4<f32>,
        proj_matrix: &Matrix4<f32>,
    ) -> EngineResult<()> {
        let camera_pos = self.camera_pos;
        let prepared =
            self.particle_renderer
                .prepare(pool, view_matrix, proj_matrix, &camera_pos)?;
        self.profile.counters.particles = prepared as u32;
        Ok(())
    }

    /// Draw a frame's overlay geometry — debug text and HUD together.
    ///
    /// Should be called after drawing the 3D scene but before end_frame.
    pub fn render_overlay(
        &mut self,
        cb: vk::CommandBuffer,
        geometry: &OverlayGeometry,
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

        self.overlay.render(cb, geometry)
    }

    /// End the frame: finish the transparent render pass, submit, and hand the
    /// image to the output.
    pub fn end_frame(&mut self, cb: vk::CommandBuffer, image_index: u32) -> EngineResult<()> {
        let frame = self
            .current_frame
            .take()
            .ok_or_else(|| EngineError::Swapchain("end_frame without begin_frame".to_string()))?;

        let submit = Instant::now();

        unsafe {
            self.vulkan_context.device().cmd_end_render_pass(cb);
        }
        self.gpu_timer.end(cb, GpuSpan::Composite);

        // Bloom goes on last so transparent surfaces cannot paint over a halo
        // that belongs in front of them. This is also the pass that transitions
        // the output image into the layout its consumer expects.
        self.gpu_timer.begin(cb, GpuSpan::Bloom);
        self.post_process
            .apply_bloom(cb, image_index, self.targets.extent);
        self.gpu_timer.end(cb, GpuSpan::Bloom);

        self.targets.draw_command_buffer.end()?;

        // Closes the pass that has been collecting casters alongside every
        // opaque draw this frame.
        self.shadow.end_frame(&self.gpu_timer)?;

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
        self.gpu_timer.mark_submitted();

        let released = self.output.release(
            &frame,
            self.vulkan_context.command_buffer_manager.graphics_queue,
        );
        self.profile.record(RenderStage::Submit, submit.elapsed());
        released
    }

    /// The last frame the GPU finished: CPU time per stage, GPU time per span,
    /// and what it drew.
    pub fn profile(&self) -> &RenderProfile {
        &self.last_profile
    }

    /// Add CPU time to a stage of the frame being recorded. For the stages a
    /// caller drives; the renderer times its own.
    pub fn record_stage(&mut self, stage: RenderStage, elapsed: std::time::Duration) {
        self.profile.record(stage, elapsed);
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
