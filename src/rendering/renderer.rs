//! Main renderer that orchestrates all rendering components.
//!
//! The Renderer is a thin orchestration layer that delegates to:
//! - `GraphicsPipeline` - immutable pipeline state
//! - `FrameOutput` - where finished frames go (a window, or an image)
//! - `FrameTargets` - what frames are drawn into
//! - `InFlightFrame` - per-frame mutable buffers, command buffer and fence,
//!   one per frame in flight
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
use crate::rendering::frame::{DrawInfo, LightUbo, SceneLighting, SceneUbo};
use crate::rendering::geometry_draw::{GeometryDraw, GeometryRecorder, SharedBindings};
use crate::rendering::in_flight::{FrameSlot, InFlightFrame, PerFrame};
use crate::rendering::material::{MaterialManager, SurfaceModulation, SurfaceParams};
use crate::rendering::mesh_source::{MeshBindings, MeshBuffers};
use crate::rendering::overlay::{OverlayGeometry, OverlayRenderer};
use crate::rendering::pipeline::{GraphicsPipeline, GraphicsPipelineConfig};
use crate::rendering::post::PostProcessRenderer;
use crate::rendering::profile::{GpuSpan, GpuTimer, RenderProfile, RenderStage};
use crate::rendering::reflection::{
    BoundingSphere, ProbeCaster, ProbeConfig, ProbeOwner, ProbeRenderer, Reach, Reflects,
};
use crate::rendering::resident::{ResidentGeometry, ResidentPrimitive, VersionedMeshId};
use crate::rendering::shadow::map::SHADOW_SAMPLED_LAYOUT;
use crate::rendering::shadow::{
    CasterBindings, ShadowMap, ShadowRenderer, ShadowVolume, ViewFrustum,
};
use crate::rendering::sky::SkyRenderer;
use crate::rendering::surface_buffer::SurfaceBuffer;
use crate::rendering::target::frame_targets::DEPTH_FORMAT;
use crate::rendering::target::{
    AcquiredFrame, FrameOutput, FrameTargets, OffscreenOutput, SurfaceInfo, SwapchainOutput,
};
use crate::rendering::transparency::{BlendedDraw, MeshBounds, TransparentQueue};
use crate::rendering::vertex::Vertex;
use crate::rendering::view_volume::ViewVolume;
use crate::rendering::water::WaterRenderer;
use crate::rendering::water::WaterScene;
use crate::resources::textures::{TextureHandle, TextureManager};

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

/// How much wider than the camera's view an object may be and still count as
/// in view for a reflection probe, as a factor on the tangent of each
/// half-angle.
///
/// A probe is handed out the frame after it is asked for, so an object that
/// waited to be on screen would show the sky for its first frame there. The
/// margin asks a little early, as it approaches the edge of the view.
const PROBE_VIEW_MARGIN: f32 = 1.25;

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

/// Whether, and how far out, a draw shows in the reflection probes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum InProbes {
    /// Not captured: debug geometry, which has no business in a reflection.
    Hidden,
    /// Captured by the probes whose reach its bounding sphere falls within.
    Bounded,
    /// Captured by every probe: the terrain, which surrounds them all and
    /// whose sphere would be the size of the level.
    Everywhere,
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
    /// Whether it is also recorded into the reflection probes that see it.
    /// Only ever an opaque draw: a probe captures solid surroundings.
    in_probes: InProbes,
    /// The object the mesh belongs to, which its own probe leaves out.
    owner: Option<ProbeOwner>,
}

impl DrawOptions {
    /// Ordinary solid geometry: terrain, props, characters.
    ///
    /// `casts_shadow` is what the *pass* asks for; a surface that lets too
    /// much light through overrides it, because a shadow map can only store
    /// a fully solid shadow. See `submit_draw`.
    const OPAQUE: Self = Self {
        pass: DrawPass::Opaque,
        wireframe_overlay: true,
        casts_shadow: true,
        in_probes: InProbes::Bounded,
        owner: None,
    };

    /// Debug overlay geometry, composited after tonemapping. Casts nothing:
    /// a shadow map stores one depth per texel and has no way to express
    /// partial occlusion, and a debug shape has no business in the scene's
    /// lighting anyway.
    const OVERLAY: Self = Self {
        pass: DrawPass::Overlay,
        wireframe_overlay: false,
        casts_shadow: false,
        in_probes: InProbes::Hidden,
        owner: None,
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
/// An opaque draw held back until the scene's draws are recorded, so that a
/// run of them shares one set of bindings.
#[derive(Clone, Copy, Debug)]
struct HeldDraw {
    /// The draw itself.
    geometry: GeometryDraw,
    /// Whether the debug backface wireframe goes over it.
    wireframe: bool,
}

pub struct Renderer {
    pub pipeline: GraphicsPipeline,
    /// Where finished frames go. Boxed rather than generic so that a caller
    /// choosing between a window and an offscreen image at runtime does not
    /// force the choice through every type that holds a renderer.
    pub output: Box<dyn FrameOutput>,
    pub targets: FrameTargets,
    /// What each frame in flight writes while it is recorded: buffers,
    /// surface table, command buffer, fence and GPU timer.
    frames: PerFrame<InFlightFrame>,
    /// The frame in flight being recorded.
    slot: FrameSlot,
    /// Models and terrain, kept on the GPU between frames and uploaded only
    /// when they change. Everything else is streamed into the frame's
    /// buffers each time it is drawn.
    resident: ResidentGeometry,
    /// Frames begun so far; the number of the one being recorded.
    frames_begun: u64,
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
    /// Reflection probes: small cube maps, captured from the same draws, of
    /// what surrounds the objects whose materials ask to reflect it.
    pub probes: ProbeRenderer,
    /// Active fire instances with their GPU resources. Keyed by entity index.
    pub active_fires: Vec<(specs::Entity, ActiveFire)>,
    /// Opaque scene draws, in the order they were issued, held back to be
    /// recorded together at the end of the opaque pass.
    opaque_draws: Vec<HeldDraw>,
    /// Blended scene draws held back for the sorted flush at the end of the
    /// opaque pass.
    transparent_queue: TransparentQueue,
    /// Where the camera is this frame, as `update_scene` was told. Held
    /// because sorting blended draws needs it and a draw call has no reason
    /// to be handed it again.
    camera_pos: Vector3<f32>,
    /// What the camera can see this frame, a little widened. Only objects in
    /// it ask for a reflection probe.
    probe_view: ViewVolume,
    /// Scene lighting environment uploaded to the scene UBO each frame.
    lighting: SceneLighting,
    /// When true, backfaces are rendered in wireframe with `wireframe_color`.
    pub debug_wireframe_backfaces: bool,
    /// The solid color used for wireframe backface rendering (RGBA, 0-1).
    pub wireframe_color: [f32; 4],
    /// The output image this frame is being rendered into, between
    /// `begin_frame` and `end_frame`.
    current_frame: Option<AcquiredFrame>,
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

        // Probes replay the same draws again, through a layout that repeats
        // the scene's descriptor sets.
        let probes = ProbeRenderer::new(
            &vulkan_context,
            ProbeConfig::default(),
            pipeline.scene_ubo_descriptor_set_layout,
            pipeline.sampler_descriptor_set_layout,
        )?;

        // Buffers, surface table, command buffer and fence for each frame in
        // flight.
        let frames = PerFrame::try_new(|_| InFlightFrame::new(&vulkan_context))?;
        let resident = ResidentGeometry::new(Arc::clone(&vulkan_context.device));

        // Create descriptor manager
        let descriptors = Arc::new(DescriptorManager::new(
            Arc::clone(&vulkan_context.device),
            pipeline.scene_ubo_descriptor_set_layout,
            pipeline.sampler_descriptor_set_layout,
            100, // initial texture descriptor capacity
        )?);

        // Point each frame's scene set at that frame's own buffers.
        for slot in FrameSlot::all() {
            let frame = &frames[slot];
            descriptors.update_scene_ubo(
                slot,
                &frame.data.scene_ubo_buffer,
                std::mem::size_of::<SceneUbo>() as vk::DeviceSize,
            );
            descriptors.update_light_ubo(
                slot,
                &frame.data.light_ubo_buffer,
                std::mem::size_of::<LightUbo>() as vk::DeviceSize,
            );
            descriptors.update_surface_table(slot, frame.surfaces.buffer(), SurfaceBuffer::SIZE);
        }
        descriptors.update_shadow_map(shadow.map().view, SHADOW_SAMPLED_LAYOUT);
        descriptors.update_reflection_probes(probes.atlas().view, probes.atlas().sampler);

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

        Ok(Self {
            pipeline,
            output,
            targets,
            frames,
            slot: FrameSlot::default(),
            resident,
            frames_begun: 0,
            descriptors,
            vulkan_context,
            overlay,
            particle_renderer,
            sky_renderer,
            water_renderer,
            fire_renderer,
            post_process,
            shadow,
            probes,
            active_fires: Vec::new(),
            opaque_draws: Vec::new(),
            transparent_queue: TransparentQueue::new(),
            camera_pos: Vector3::zeros(),
            probe_view: ViewVolume::EVERYTHING,
            lighting: SceneLighting::default(),
            debug_wireframe_backfaces: false,
            wireframe_color: [0.0, 0.0, 0.0, 1.0],
            current_frame: None,
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

    /// Begin a new frame: wait for the frame that last used this slot, acquire
    /// an output image, start the command buffer. Call `begin_opaque_pass()`
    /// after any pre-pass compute work (e.g. fire simulation) is recorded.
    ///
    /// The wait is for the frame before last, not the last one: that one may
    /// still be drawing while this one is recorded.
    pub fn begin_frame(&mut self) -> EngineResult<(vk::CommandBuffer, u32)> {
        self.slot = self.slot.next();
        self.frames_begun += 1;
        let slot = self.slot;

        // The fence is *not* reset here — `end_frame` resets it immediately
        // before the submit that re-signals it, so a frame abandoned in
        // between leaves it signalled rather than stranding every later frame
        // on a signal that never arrives.
        let fence_wait = Instant::now();
        let finished = self.frames[slot].wait()?;
        let fence_wait = fence_wait.elapsed();

        // The frame this slot last carried is now finished on both sides, so
        // its profile is complete.
        if let Some(finished) = finished {
            self.last_profile = finished;
        }
        self.profile = RenderProfile::for_frame(self.frames_begun);
        self.profile.record(RenderStage::FenceWait, fence_wait);

        // Now that the GPU is done with this slot, its buffers can be rewound
        // and whatever was retired while it was in flight freed.
        self.descriptors.begin_frame();
        self.frames[slot].rewind();
        self.resident.begin_frame(self.frames_begun);
        // Must be rewound with the buffers it indexes into: a held-over entry
        // would point at geometry that is about to be overwritten.
        self.transparent_queue.begin_frame();
        self.opaque_draws.clear();
        self.particle_renderer.begin_frame(slot);
        self.overlay.begin_frame(slot);
        self.water_renderer.begin_frame(slot);

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
        let frame = &self.frames[slot];
        self.shadow.begin_frame(slot, &frame.timer)?;
        self.probes.begin_frame(slot, &frame.timer)?;

        frame
            .command_buffer
            .begin(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT)?;

        let acquire = Instant::now();
        let acquired = self.output.acquire(&frame.sync)?;
        self.profile.record(RenderStage::Acquire, acquire.elapsed());
        let image_index = acquired.index;
        self.current_frame = Some(acquired);

        let cb = self.frames[slot].command_buffer.raw();
        self.timer().begin(cb, GpuSpan::FireSim);

        Ok((cb, image_index))
    }

    /// The frame in flight being recorded.
    fn frame(&self) -> &InFlightFrame {
        &self.frames[self.slot]
    }

    fn frame_mut(&mut self) -> &mut InFlightFrame {
        &mut self.frames[self.slot]
    }

    /// The GPU timer of the frame being recorded.
    fn timer(&self) -> &GpuTimer {
        &self.frame().timer
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

        self.timer().end(cb, GpuSpan::FireSim);
        self.timer().begin(cb, GpuSpan::Scene);

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
        self.probe_view = ViewVolume::new(view, proj, PROBE_VIEW_MARGIN);

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

        let shadow = self.shadow.uniforms();
        self.frame_mut()
            .data
            .update_scene_ubo(view, proj, camera_pos, &lighting, &shadow)
    }

    /// Upload the frame's point light set.
    ///
    /// Call once per frame alongside `update_scene`. The buffer is already
    /// bound to set 0, binding 1; this only refreshes its contents.
    pub fn update_lights(&mut self, active: &ActiveLights) -> EngineResult<()> {
        self.frame_mut().data.update_light_ubo(active)
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
    /// The model's meshes are resident: uploaded the first time this `Arc`
    /// is drawn, and drawn from the GPU's copy every time after, until the
    /// last `Arc` to it is dropped. A model that changes shape — a compound
    /// losing a piece — is a new `Arc`, and so a new upload.
    ///
    /// `owner` names the object across frames. A model with a material that
    /// reflects its surroundings needs one to hold a reflection probe, centred
    /// on `world_transform`'s origin; without one it reflects the sky. Only
    /// a model in view, or nearly, asks for one: an object off screen has no
    /// reflection to show.
    pub fn draw_model(
        &mut self,
        cb: vk::CommandBuffer,
        model: &Arc<Model>,
        world_transform: &Matrix4<f32>,
        part_transforms: &[Transform],
        material_manager: &MaterialManager,
        texture_manager: &TextureManager,
        modulation: SurfaceModulation,
        owner: Option<ProbeOwner>,
    ) -> EngineResult<()> {
        let primitives = self.resident.model(model)?.to_vec();
        let mut resident = primitives.iter().copied();

        let reflects_surroundings =
            model
                .parts
                .iter()
                .flat_map(|part| &part.primitives)
                .any(|primitive| {
                    material_manager.get(primitive.material).reflects == Reflects::Surroundings
                });
        let probe = match owner {
            Some(owner)
                if reflects_surroundings
                    && self.in_probe_view(model, &primitives, world_transform) =>
            {
                let centre = world_transform.fixed_view::<3, 1>(0, 3).into_owned();
                self.probes.request(owner, centre, &self.camera_pos)
            }
            _ => None,
        };
        let options = DrawOptions {
            owner,
            ..DrawOptions::OPAQUE
        };

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
                let Some(uploaded) = resident.next() else {
                    break;
                };
                if uploaded.mesh.is_empty() {
                    continue;
                }
                let texture = material_manager.get_effective_texture(primitive.material);
                let mut surface = material_manager
                    .get_surface_params(primitive.material)
                    .modulated(modulation);
                if surface.reflects == Reflects::Surroundings {
                    surface = surface.with_probe(probe);
                }

                self.submit_draw(
                    cb,
                    uploaded.mesh.draw_info(),
                    || uploaded.bounds,
                    &final_transform,
                    texture,
                    surface,
                    texture_manager,
                    options,
                )?;
            }
        }

        Ok(())
    }

    /// Whether any primitive of `model`, drawn at `world_transform`, is in
    /// the widened view that decides who asks for a reflection probe.
    ///
    /// Each part is placed by its resting transform; the per-frame part
    /// transforms `draw_model` takes are animation's, and nudge a part far
    /// less than the view's margin.
    fn in_probe_view(
        &self,
        model: &Model,
        primitives: &[ResidentPrimitive],
        world_transform: &Matrix4<f32>,
    ) -> bool {
        let placements = model.parts.iter().flat_map(|part| {
            let placed = world_transform * part.local_transform.to_matrix();
            part.primitives.iter().map(move |_| placed)
        });
        placements.zip(primitives).any(|(placed, primitive)| {
            self.probe_view
                .contains(&BoundingSphere::around(&primitive.bounds, &placed))
        })
    }

    /// Draw a mesh whose owner publishes a version that changes whenever the
    /// mesh does, such as the terrain.
    ///
    /// Uploaded when `version` differs from the one resident under `id`, and
    /// drawn from the GPU's copy otherwise, so the vertices are only read on
    /// the frames they changed.
    pub fn draw_versioned_mesh(
        &mut self,
        cb: vk::CommandBuffer,
        id: VersionedMeshId,
        version: u64,
        vertices: &[Vertex],
        indices: &[u32],
        model: &Matrix4<f32>,
        texture: &TextureHandle,
        surface: SurfaceParams,
        texture_manager: &TextureManager,
    ) -> EngineResult<()> {
        if vertices.is_empty() || indices.is_empty() {
            return Ok(());
        }
        let uploaded = self.resident.versioned(id, version, vertices, indices)?;
        // Seen by every probe, however far off: it is the ground they all
        // stand on, and walking its vertices for a sphere every frame would
        // cost more than recording it.
        let options = DrawOptions {
            in_probes: InProbes::Everywhere,
            ..DrawOptions::OPAQUE
        };
        self.submit_draw(
            cb,
            uploaded.draw_info(),
            || MeshBounds::of(vertices),
            model,
            texture,
            surface,
            texture_manager,
            options,
        )
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

    /// Stream a mesh into the frame's buffers, then draw it.
    ///
    /// For geometry that changes from one frame to the next. Anything that
    /// holds still between frames belongs in the resident arena instead —
    /// see [`Self::draw_model`] and [`Self::draw_versioned_mesh`].
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

        let data = &mut self.frame_mut().data;
        let buffer_sizes = data.mesh_buffer_sizes();
        let draw_info = data.append_mesh_data(vertices, indices)?;
        let grew = data.mesh_buffer_sizes() != buffer_sizes;
        self.profile.counters.record_upload(
            std::mem::size_of_val(vertices) + std::mem::size_of_val(indices),
            grew,
        );

        self.submit_draw(
            cb,
            draw_info,
            || MeshBounds::of(vertices),
            model,
            texture,
            surface,
            texture_manager,
            options,
        )
    }

    /// Draw geometry already in a buffer, and hold the draw back or record
    /// it.
    ///
    /// Everything else a draw needs is committed here — the shading
    /// parameters into the frame's surface table, the caster into the shadow
    /// pass. Scene draws are then held back and recorded together at the end
    /// of the opaque pass: blended ones because their order depends on what
    /// else the frame contains, opaque ones so a run of them shares one set
    /// of bindings. Overlay draws land after the scene is resolved and are
    /// recorded where they are issued.
    ///
    /// `bounds` is only asked for when the draw turns out to be blended,
    /// which sorts by it, or opaque and bounded while any reflection probe
    /// is live, which culls by it.
    fn submit_draw(
        &mut self,
        cb: vk::CommandBuffer,
        draw_info: DrawInfo,
        bounds: impl FnOnce() -> MeshBounds,
        model: &Matrix4<f32>,
        texture: &TextureHandle,
        surface: SurfaceParams,
        texture_manager: &TextureManager,
        options: DrawOptions,
    ) -> EngineResult<()> {
        // Park this draw's shading parameters in the frame's surface table.
        let surface_index = self.frame_mut().surfaces.push(surface.to_gpu());

        // A shadow map stores one depth per texel and has no way to express
        // partial occlusion, so a transmissive caster can only throw a fully
        // solid shadow. Whether that is better or worse than none is the
        // material's call, not the pass's — a cloudy block wants the shadow,
        // a window would only put a hard black bite behind itself.
        if options.casts_shadow && surface.transparency.casts_shadow() && self.shadow.enabled {
            self.shadow.add_caster(model, &draw_info);
            self.profile.counters.shadow_casters += 1;
        }

        let texture_set = texture_manager
            .get_or_create_descriptor_set(texture)
            .map_err(|e| crate::core::error::EngineError::Texture {
                path: None,
                reason: format!("descriptor set creation: {}", e),
            })?;

        let geometry = GeometryDraw {
            model: *model,
            draw: draw_info,
            surface_index,
            texture_set,
        };
        let wireframe = options.wireframe_overlay && self.debug_wireframe_backfaces;

        // A material that lets light through overrides the scene pass it was
        // asked for. The call site does not know what it is drawing — a model
        // carries whatever material its spawnable gave it — so the decision
        // belongs to the surface.
        let pass = match options.pass {
            DrawPass::Opaque if surface.transparency.is_blended() => DrawPass::SceneBlended,
            other => other,
        };

        self.profile.counters.triangles += (draw_info.index_count / 3) as u64;

        match pass {
            DrawPass::SceneBlended => {
                self.profile.counters.blended_draws += 1;
                self.transparent_queue
                    .push(BlendedDraw::new(geometry).sorted_from(&self.camera_pos, &bounds()));
            }
            DrawPass::Opaque => {
                self.profile.counters.opaque_draws += 1;
                if self.probes.collecting() {
                    let reach = match options.in_probes {
                        InProbes::Hidden => None,
                        InProbes::Everywhere => Some(Reach::Everywhere),
                        InProbes::Bounded => {
                            Some(Reach::Within(BoundingSphere::around(&bounds(), model)))
                        }
                    };
                    if let Some(reach) = reach {
                        self.probes.add_caster(ProbeCaster {
                            geometry,
                            reach,
                            owner: options.owner,
                        });
                    }
                }
                self.opaque_draws.push(HeldDraw {
                    geometry,
                    wireframe,
                });
            }
            DrawPass::Overlay => {
                self.profile.counters.overlay_draws += 1;
                let mut recorder = self.geometry_recorder(cb);
                recorder.draw(self.pipeline.transparent, &geometry);
                if wireframe {
                    recorder.overdraw(
                        self.pipeline.wireframe_backface,
                        self.wireframe_color,
                        &geometry,
                    );
                }
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

    /// The buffer pairs this frame's draws read from, as they stand now.
    ///
    /// Read when the draws are recorded, not when they are issued, so bindings
    /// taken after the last mesh is committed name buffers that hold every
    /// mesh — a buffer that grows carries what it held across (see
    /// `FrameData` and `MeshArena`).
    fn mesh_bindings(&self) -> MeshBindings {
        let data = &self.frame().data;
        MeshBindings {
            frame: MeshBuffers {
                vertex: data.vertex_buffer.buffer,
                index: data.index_buffer.buffer,
            },
            resident: self.resident.buffers(),
        }
    }

    /// A recorder of geometry draws into `cb`, over this frame's bindings.
    fn geometry_recorder(&self, cb: vk::CommandBuffer) -> GeometryRecorder<'_> {
        GeometryRecorder::new(
            self.vulkan_context.device(),
            cb,
            self.pipeline.layout,
            SharedBindings {
                extent: self.targets.extent,
                scene_set: self.descriptors.scene_set(self.slot),
                meshes: self.mesh_bindings(),
            },
        )
    }

    /// Record the frame's held scene draws: the opaque ones in the order they
    /// were issued, then the blended ones farthest first.
    ///
    /// Call once, after the last opaque draw and before the opaque pass ends —
    /// these draws go into the HDR scene target, so that glass is exposed,
    /// tonemapped and bloomed with everything behind it rather than pasted on
    /// after the resolve.
    ///
    /// Two blended streams are merged here, both ordered by distance from the
    /// camera: the blended meshes held back by the queue, and the frame's
    /// particles.
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
    /// Each blended mesh is recorded twice, back faces then front faces.
    /// Sorting can only order whole draws, and a closed mesh contains its own
    /// far and near surfaces; splitting them by cull mode is what puts those
    /// two in order.
    fn record_scene_draws(&mut self, cb: vk::CommandBuffer) {
        // Taken out of the queue so the recording loop is not holding a borrow
        // of `self` through calls that need `&self` for the device.
        let blended: Vec<BlendedDraw> = self.transparent_queue.sorted().to_vec();
        let exposure = self.post_process.config.exposure;
        let pipeline = &self.pipeline;
        let mut recorder = self.geometry_recorder(cb);

        for held in &self.opaque_draws {
            recorder.draw(pipeline.opaque, &held.geometry);
            if held.wireframe {
                recorder.overdraw(
                    pipeline.wireframe_backface,
                    self.wireframe_color,
                    &held.geometry,
                );
            }
        }

        if blended.is_empty() && self.particle_renderer.is_empty() {
            return;
        }

        // The particle pipeline takes its viewport dynamically and the flush
        // may record a particle before any mesh has set one.
        self.set_full_viewport(cb);

        let mut particles_drawn = 0;
        for draw in &blended {
            // Everything behind this mesh goes down before it does.
            let behind = self.particle_renderer.count_beyond(draw.depth_key());
            if behind > particles_drawn {
                self.particle_renderer.bind(cb, exposure);
                self.particle_renderer
                    .draw_range(cb, particles_drawn, behind);
                particles_drawn = behind;
                recorder.interrupted();
            }

            recorder.draw(pipeline.scene_blended_back, &draw.geometry);
            recorder.draw(pipeline.scene_blended_front, &draw.geometry);
        }

        if !self.particle_renderer.is_empty() {
            self.particle_renderer.bind(cb, exposure);
            self.particle_renderer
                .draw_range(cb, particles_drawn, usize::MAX);
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
        let scene_draws = Instant::now();
        self.record_scene_draws(cb);
        self.profile
            .record(RenderStage::SceneDraws, scene_draws.elapsed());

        let device = self.vulkan_context.device();
        let extent = self.targets.extent;

        unsafe {
            device.cmd_end_render_pass(cb);
            self.timer().end(cb, GpuSpan::Scene);

            self.timer().begin(cb, GpuSpan::Resolve);
            self.post_process.resolve(cb, image_index, extent);
            self.timer().end(cb, GpuSpan::Resolve);
            self.timer().begin(cb, GpuSpan::Composite);

            // Begin transparent render pass (loads composited colour + depth).
            let render_pass_begin = vk::RenderPassBeginInfo::default()
                .render_pass(self.pipeline.transparent_renderpass)
                .framebuffer(self.targets.transparent_framebuffers[image_index as usize])
                .render_area(extent.into());

            device.cmd_begin_render_pass(cb, &render_pass_begin, vk::SubpassContents::INLINE);
        }
    }

    /// Render every body of water.
    ///
    /// Should be called after next_subpass but before particles.
    pub fn render_water(
        &mut self,
        cb: vk::CommandBuffer,
        water: &dyn WaterScene,
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
            water,
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
    /// [`Self::record_scene_draws`] — and that flush cannot run until
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
    ///
    /// Returns without waiting for the GPU. The frame's profile is parked on
    /// its slot and published once the slot's fence is next waited on.
    pub fn end_frame(&mut self, cb: vk::CommandBuffer, image_index: u32) -> EngineResult<()> {
        let result = self.submit_frame(cb, image_index);
        let uploaded = self.resident.take_tally();
        self.profile.counters.uploaded_mesh_bytes += uploaded.bytes;
        self.profile.counters.buffer_growths += uploaded.growths;
        let profile = std::mem::take(&mut self.profile);
        self.frame_mut().park(profile);
        result
    }

    fn submit_frame(&mut self, cb: vk::CommandBuffer, image_index: u32) -> EngineResult<()> {
        let frame = self
            .current_frame
            .take()
            .ok_or_else(|| EngineError::Swapchain("end_frame without begin_frame".to_string()))?;

        let submit = Instant::now();

        unsafe {
            self.vulkan_context.device().cmd_end_render_pass(cb);
        }
        self.timer().end(cb, GpuSpan::Composite);

        // Bloom goes on last so transparent surfaces cannot paint over a halo
        // that belongs in front of them. This is also the pass that transitions
        // the output image into the layout its consumer expects.
        self.timer().begin(cb, GpuSpan::Bloom);
        self.post_process
            .apply_bloom(cb, image_index, self.targets.extent);
        self.timer().end(cb, GpuSpan::Bloom);

        let meshes = self.mesh_bindings();
        let scene_set = self.descriptors.scene_set(self.slot);
        let in_flight = &self.frames[self.slot];
        in_flight.command_buffer.end()?;

        // Records the casters collected alongside every opaque draw this
        // frame, now that the buffers hold all of them, and closes the pass.
        self.shadow.end_frame(
            self.vulkan_context.device(),
            &in_flight.timer,
            CasterBindings {
                scene_set,
                meshes: meshes.clone(),
            },
        )?;
        let recording = Instant::now();
        let probes = self.probes.end_frame(&in_flight.timer, scene_set, meshes)?;
        let recording = recording.elapsed();
        self.profile.record(RenderStage::Probes, recording);
        self.profile.counters.probes = probes.probes;
        self.profile.counters.probe_faces = probes.faces;
        self.profile.counters.probe_draws = probes.draws;

        // Only a swapchain acquire produces semaphores to synchronize against;
        // an engine-owned image is ready the moment it is asked for, and the
        // draw fence alone orders one frame against the next.
        let wait: Vec<vk::Semaphore> = frame.wait.into_iter().collect();
        let signal: Vec<vk::Semaphore> = frame.signal.into_iter().collect();
        let wait_stages = vec![vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT; wait.len()];

        in_flight.sync.reset()?;

        // The shadow map goes first: the probes and the geometry pass sample
        // it, and the ordering plus the shadow pass's own external dependency
        // are what make that read see this frame's contents rather than the
        // last one's. The probes go next, on the same terms, for the geometry
        // pass that samples them.
        self.vulkan_context
            .command_buffer_manager
            .submit_recorded_graphics_batch_async(
                &[
                    self.shadow.command_buffer(),
                    self.probes.command_buffer(),
                    &in_flight.command_buffer,
                ],
                in_flight.sync.draw_fence,
                &wait,
                &signal,
                &wait_stages,
            )?;
        self.frame_mut().timer.mark_submitted();

        let released = self.output.release(
            &frame,
            self.vulkan_context.command_buffer_manager.graphics_queue,
        );
        self.profile.record(
            RenderStage::Submit,
            submit.elapsed().saturating_sub(recording),
        );
        released
    }

    /// The number of the frame most recently begun, counted from 1. Matches
    /// the `frame` of its profile once that is published.
    pub fn frame_number(&self) -> u64 {
        self.frames_begun
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

    /// Block until every frame submitted by `end_frame` has finished on the
    /// GPU.
    ///
    /// Only meaningful for outputs that are read back rather than presented;
    /// the windowed path lets `begin_frame` do the waiting, one slot at a time.
    pub fn wait_for_frame(&self) -> EngineResult<()> {
        self.frames.iter().try_for_each(|frame| frame.sync.wait())
    }
}

impl Drop for Renderer {
    fn drop(&mut self) {
        unsafe {
            log::info!("Renderer::drop - waiting for device idle");
            let _ = self.vulkan_context.device().device_wait_idle();
        }
        // Flush any pending buffer deletions now that GPU is idle
        for frame in self.frames.iter_mut() {
            frame.data.cleanup();
        }
        // Components drop in reverse order due to struct field ordering
    }
}
