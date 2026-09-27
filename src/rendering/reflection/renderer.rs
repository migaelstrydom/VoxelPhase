//! Reflection probes: small cube maps of what is around a glossy object.
//!
//! ```text
//!   Renderer::begin_frame ──▶ ProbeRenderer::begin_frame
//!                               pool hands out slots from last frame's requests
//!
//!   Renderer::draw_model ──material reflects Surroundings?──▶ request(owner)
//!        │                                                    └─▶ slot, if held
//!        └─ surface.probe = slot ──▶ scene draw samples it
//!   Renderer::submit_draw (opaque) ──▶ add_caster ──▶ held until end_frame
//!
//!   Renderer::end_frame ──▶ ProbeRenderer::end_frame
//!        FaceSchedule: new probes whole, then `faces_per_frame` round-robin
//!        each face: render the casters it can see ──▶ ProbeAtlas layer
//!        every face at once: box-filter its mips (ProbeMipFilter)
//!
//!   submitted: shadow cb ──▶ probe cb ──▶ scene cb
//! ```
//!
//! The pass mirrors the shadow pass: the casters are the frame's own opaque
//! draws, collected as they are issued and recorded into a command buffer of
//! its own once the frame's geometry is known. It is submitted after the
//! shadow pass, whose map it reads, and before the scene, which reads it.
//!
//! What asks for a probe is the material — see [`Reflects`] — so ice, glass,
//! polished stone or anything else opts in the same way the metal cubes do,
//! with nothing in the renderer knowing what they are.
//!
//! [`Reflects`]: crate::rendering::reflection::Reflects

use std::sync::Arc;

use ash::vk;
use nalgebra::Vector3;

use crate::core::command_buffer::ManagedCommandBuffer;
use crate::core::device::ManagedDevice;
use crate::core::error::EngineResult;
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::geometry_draw::{GeometryRecorder, SharedBindings};
use crate::rendering::in_flight::{FrameSlot, PerFrame};
use crate::rendering::mesh_source::MeshBindings;
use crate::rendering::profile::{GpuSpan, GpuTimer};
use crate::rendering::reflection::atlas::ProbeAtlas;
use crate::rendering::reflection::caster::ProbeCaster;
use crate::rendering::reflection::config::ProbeConfig;
use crate::rendering::reflection::face::CubeFace;
use crate::rendering::reflection::faces::{FaceUniforms, GpuProbeFace, MAX_FACES_PER_FRAME};
use crate::rendering::reflection::mip_filter::ProbeMipFilter;
use crate::rendering::reflection::owner::ProbeOwner;
use crate::rendering::reflection::pipeline::ProbePipeline;
use crate::rendering::reflection::pool::{ProbePool, ProbeSlot};
use crate::rendering::reflection::schedule::FaceSchedule;

/// What the probe pass did in one frame.
#[derive(Clone, Copy, Debug, Default)]
pub struct ProbeFrameStats {
    /// Probes live.
    pub probes: u32,
    /// Faces captured.
    pub faces: u32,
    /// Draws recorded, summed over the faces.
    pub draws: u32,
}

/// Captures and serves the reflection probes.
pub struct ProbeRenderer {
    /// Refresh rate and reach. Public so a scene can tune them; the atlas's
    /// capacity is fixed at creation and changing it here does nothing.
    pub config: ProbeConfig,
    /// Turns probes off: every object reflects the sky, and no face is
    /// captured.
    pub enabled: bool,
    atlas: ProbeAtlas,
    pipeline: ProbePipeline,
    mips: ProbeMipFilter,
    faces: FaceUniforms,
    pool: ProbePool,
    schedule: FaceSchedule,
    /// One per frame in flight, like the shadow pass's.
    command_buffers: PerFrame<ManagedCommandBuffer>,
    /// The frame being recorded.
    slot: FrameSlot,
    /// The frame's opaque draws so far, collected while any probe is live.
    casters: Vec<ProbeCaster>,
    device: Arc<ManagedDevice>,
}

impl ProbeRenderer {
    /// Build the pass over the scene's descriptor set layouts, which the
    /// capture pipeline's layout must repeat for a scene draw to be recorded
    /// into it unchanged.
    pub fn new(
        vulkan_context: &VulkanContext,
        config: ProbeConfig,
        scene_set_layout: vk::DescriptorSetLayout,
        texture_set_layout: vk::DescriptorSetLayout,
    ) -> EngineResult<Self> {
        let device = Arc::clone(&vulkan_context.device);
        let atlas = ProbeAtlas::new(vulkan_context, &config)?;
        let pipeline = ProbePipeline::new(
            Arc::clone(&device),
            atlas.render_pass,
            scene_set_layout,
            texture_set_layout,
        )?;
        let mips = ProbeMipFilter::new(Arc::clone(&device), &atlas)?;
        let faces = FaceUniforms::new(Arc::clone(&device), pipeline.face_set_layout)?;
        let command_buffers = PerFrame::try_new(|_| {
            vulkan_context
                .command_buffer_manager
                .create_primary_buffer()
        })?;

        Ok(Self {
            pool: ProbePool::new(config.capacity),
            config,
            enabled: true,
            atlas,
            pipeline,
            mips,
            faces,
            schedule: FaceSchedule::default(),
            command_buffers,
            slot: FrameSlot::default(),
            casters: Vec::new(),
            device,
        })
    }

    /// The cube array and sampler the scene reads the probes through.
    pub fn atlas(&self) -> &ProbeAtlas {
        &self.atlas
    }

    /// The command buffer to submit between the shadow pass's and the
    /// scene's.
    pub fn command_buffer(&self) -> &ManagedCommandBuffer {
        &self.command_buffers[self.slot]
    }

    /// Faces refreshed round-robin per frame, within what the frame's buffer
    /// holds.
    fn budget(&self) -> u32 {
        self.config.faces_per_frame.min(MAX_FACES_PER_FRAME)
    }

    /// Start frame `slot`: hand the probes out for it and open its command
    /// buffer. Before any draw of the frame is issued, so that whether any
    /// probe is live — and so whether draws are collected — holds for the
    /// whole frame.
    pub fn begin_frame(&mut self, slot: FrameSlot, timer: &GpuTimer) -> EngineResult<()> {
        self.slot = slot;
        self.casters.clear();
        if self.enabled {
            // Newcomers are captured whole outside the refresh budget; admit
            // no more than the rest of the frame's face buffer can hold.
            let spare = MAX_FACES_PER_FRAME - self.budget();
            self.pool
                .begin_frame((spare / CubeFace::COUNT as u32) as usize);
        } else {
            self.pool.clear();
        }

        let command_buffer = self.command_buffer();
        command_buffer.begin(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT)?;
        timer.begin(command_buffer.raw(), GpuSpan::Probes);
        Ok(())
    }

    /// Whether this frame's opaque draws are wanted: only while a probe is
    /// live to capture them.
    pub fn collecting(&self) -> bool {
        !self.pool.is_empty()
    }

    /// Ask for a probe for `owner`, an object whose origin is at `centre`.
    /// Returns the slot its draws sample, or `None` while it has none — it
    /// is then considered at the next frame, nearest the camera first.
    pub fn request(
        &mut self,
        owner: ProbeOwner,
        centre: Vector3<f32>,
        camera_pos: &Vector3<f32>,
    ) -> Option<ProbeSlot> {
        if !self.enabled {
            return None;
        }
        self.pool
            .request(owner, centre, (centre - camera_pos).norm())
    }

    /// Whether the frame being recorded left an object waiting for a probe
    /// the next frame will give it. A tool that renders a single still can
    /// repeat the frame until this clears, and so show every reflection a
    /// running game would.
    pub fn settling(&self) -> bool {
        self.enabled && self.pool.has_waiting()
    }

    /// Add one of the frame's opaque draws, to be recorded into every face
    /// that sees it.
    pub fn add_caster(&mut self, caster: ProbeCaster) {
        self.casters.push(caster);
    }

    /// Record this frame's faces and close the command buffer.
    ///
    /// `scene_set` and `meshes` are read now, after the last draw was
    /// committed, so the buffers they name hold every caster's mesh.
    pub fn end_frame(
        &mut self,
        timer: &GpuTimer,
        scene_set: vk::DescriptorSet,
        meshes: MeshBindings,
    ) -> EngineResult<ProbeFrameStats> {
        let plan = self
            .schedule
            .plan(&self.pool, self.config.capacity, self.budget());
        let cb = self.command_buffers[self.slot].raw();
        let device = &self.device.device;
        let mut stats = ProbeFrameStats {
            probes: self.pool.live().count() as u32,
            faces: plan.len() as u32,
            draws: 0,
        };

        let mut recorder = GeometryRecorder::new(
            device,
            cb,
            self.pipeline.layout,
            SharedBindings {
                extent: self.atlas.extent,
                scene_set,
                meshes,
            },
        );
        for (index, capture) in plan.iter().enumerate() {
            let Some(probe) = self.pool.get(capture.slot).copied() else {
                continue;
            };
            let index = index as u32;
            let view_proj =
                capture
                    .face
                    .view_proj(&probe.centre, self.config.near, self.config.far);
            self.faces.write(
                self.slot,
                index,
                GpuProbeFace::new(&view_proj, &probe.centre),
            );

            self.atlas
                .begin_face(cb, ProbeAtlas::layer(capture.slot, capture.face));
            unsafe {
                device.cmd_bind_descriptor_sets(
                    cb,
                    vk::PipelineBindPoint::GRAPHICS,
                    self.pipeline.layout,
                    2,
                    &[self.faces.set(self.slot)],
                    &[FaceUniforms::offset(index)],
                );
            }
            let seen = self.casters.iter().filter(|caster| {
                caster.seen_by(
                    probe.owner,
                    &probe.centre,
                    capture.face,
                    self.config.reach,
                    self.config.near,
                )
            });
            for caster in seen {
                recorder.draw(self.pipeline.pipeline, &caster.geometry);
                stats.draws += 1;
            }
            self.atlas.end_face(cb);
        }

        let layers: Vec<u32> = plan
            .iter()
            .map(|capture| ProbeAtlas::layer(capture.slot, capture.face))
            .collect();
        self.mips.record(cb, &layers);
        for capture in &plan {
            self.pool.mark_captured(capture.slot);
        }

        timer.end(cb, GpuSpan::Probes);
        self.command_buffers[self.slot].end()?;
        Ok(stats)
    }
}
