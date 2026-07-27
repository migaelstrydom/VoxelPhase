//! HDR resolve with bloom.
//!
//! Replaces what used to be a straight blit from the offscreen colour target to
//! the swapchain. The scene is now rendered into a floating-point target, so
//! emissive surfaces can carry radiance above 1.0; this module turns that into
//! displayable pixels.
//!
//! ```text
//!  scene HDR ─┬─────────────────────────────┐
//!             │                             │
//!             ▼                             ▼
//!        bright pass ──► blur H ──► blur V ─┴─► composite ──► swapchain
//!         (¼ res)        (ping)     (pong)      (tonemap)
//! ```
//!
//! The blur pair runs `BLUR_ITERATIONS` times, ping-ponging between two
//! quarter-resolution targets to widen the kernel without extra taps.

use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult};
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::post::pipeline::PostPipelines;
use crate::rendering::post::target::PostTarget;

/// Bloom is computed at this fraction of the screen resolution in each axis.
/// Quarter resolution is invisible after blurring and cuts the cost 16x.
const BLOOM_DOWNSCALE: u32 = 4;

/// Horizontal+vertical blur pairs. Each pass roughly doubles the effective radius.
const BLUR_ITERATIONS: usize = 3;

/// Bloom target format. Must stay floating-point: the whole point is to carry
/// values above 1.0 through the blur.
const BLOOM_FORMAT: vk::Format = vk::Format::R16G16B16A16_SFLOAT;

/// Tuning for the HDR resolve.
#[derive(Clone, Copy, Debug)]
pub struct PostProcessConfig {
    /// Luminance above which a pixel starts contributing to bloom.
    pub bloom_threshold: f32,

    /// Width of the soft ramp either side of the threshold.
    pub bloom_knee: f32,

    /// How strongly the blurred bloom is added back over the scene.
    pub bloom_intensity: f32,

    /// Linear exposure multiplier applied before the filmic curve.
    ///
    /// Below 1.0 because the test scene is built from light-coloured surfaces
    /// (bright grass, near-white stone) that otherwise sit in the shoulder of
    /// the filmic curve, washing the whole frame out. This is a display
    /// setting: it dims the scene and the glow together, and deliberately does
    /// not change which pixels pass the bloom threshold — that test is on
    /// pre-exposure scene radiance.
    pub exposure: f32,

    /// How strongly the tonemap preserves hue, in [0, 1].
    ///
    /// 0 is the per-channel ACES curve, which desaturates bright colours
    /// towards white by design. 1 curves luminance alone and rescales chroma to
    /// match, so a saturated emissive surface keeps its colour instead of
    /// bleaching. Intermediate values blend the two.
    ///
    /// Shared by every stage that resolves HDR — composite, bloom overlay and
    /// the water surface — so they cannot disagree about how a colour resolves.
    pub hue_preservation: f32,
}

impl Default for PostProcessConfig {
    fn default() -> Self {
        Self {
            // A white matte surface in full sun reaches ~1.0, so the ramp is
            // placed above that: only genuinely emissive things bloom.
            bloom_threshold: 1.3,
            bloom_knee: 0.4,
            bloom_intensity: 0.7,
            exposure: 0.7,
            hue_preservation: 1.0,
        }
    }
}

/// Owns the bloom targets, descriptors and per-frame recording of the chain.
pub struct PostProcessRenderer {
    device: Arc<ManagedDevice>,
    pipelines: PostPipelines,

    /// Quarter-res ping-pong pair. `ping` holds the final bloom after `resolve`.
    ping: PostTarget,
    pong: PostTarget,

    sampler: vk::Sampler,
    descriptor_pool: vk::DescriptorPool,

    /// Samples the full-resolution HDR scene (used by the bright pass).
    scene_set: vk::DescriptorSet,
    /// Samples `ping` (used when blurring ping → pong).
    ping_set: vk::DescriptorSet,
    /// Samples `pong` (used when blurring pong → ping).
    pong_set: vk::DescriptorSet,

    /// One per swapchain image, compatible with the composite render pass.
    composite_framebuffers: Vec<vk::Framebuffer>,

    pub config: PostProcessConfig,
}

impl PostProcessRenderer {
    /// `scene_view` must be the HDR colour target the opaque pass writes to;
    /// `swapchain_views` are the images the composite resolves into.
    pub fn new(
        vulkan_context: &VulkanContext,
        scene_view: vk::ImageView,
        swapchain_views: &[vk::ImageView],
        screen_extent: vk::Extent2D,
        swapchain_format: vk::Format,
    ) -> EngineResult<Self> {
        let device = Arc::clone(&vulkan_context.device);
        let pipelines = PostPipelines::new(Arc::clone(&device), BLOOM_FORMAT, swapchain_format)?;

        let bloom_extent = vk::Extent2D {
            width: (screen_extent.width / BLOOM_DOWNSCALE).max(1),
            height: (screen_extent.height / BLOOM_DOWNSCALE).max(1),
        };

        let ping = PostTarget::new(
            vulkan_context,
            bloom_extent,
            BLOOM_FORMAT,
            pipelines.offscreen_pass,
        )?;
        let pong = PostTarget::new(
            vulkan_context,
            bloom_extent,
            BLOOM_FORMAT,
            pipelines.offscreen_pass,
        )?;

        let sampler = Self::create_sampler(&device)?;
        let descriptor_pool = Self::create_descriptor_pool(&device)?;

        let scene_set =
            Self::allocate_set(&device, descriptor_pool, pipelines.single_sampler_layout)?;
        let ping_set =
            Self::allocate_set(&device, descriptor_pool, pipelines.single_sampler_layout)?;
        let pong_set =
            Self::allocate_set(&device, descriptor_pool, pipelines.single_sampler_layout)?;

        // The images never change, so the descriptor sets are written once.
        // `scene_set` is shared by the bright pass and the composite, and
        // `ping_set` by the blur and the bloom overlay — every stage samples a
        // single texture through the same layout.
        Self::write_set(&device, scene_set, 0, sampler, scene_view);
        Self::write_set(&device, ping_set, 0, sampler, ping.view);
        Self::write_set(&device, pong_set, 0, sampler, pong.view);

        let composite_framebuffers = swapchain_views
            .iter()
            .map(|view| {
                let create_info = vk::FramebufferCreateInfo::default()
                    .render_pass(pipelines.composite_pass)
                    .attachments(std::slice::from_ref(view))
                    .width(screen_extent.width)
                    .height(screen_extent.height)
                    .layers(1);

                unsafe { device.device.create_framebuffer(&create_info, None) }
                    .map_err(|e| EngineError::Pipeline(format!("composite framebuffer: {:?}", e)))
            })
            .collect::<EngineResult<Vec<_>>>()?;

        Ok(Self {
            device,
            pipelines,
            composite_framebuffers,
            ping,
            pong,
            sampler,
            descriptor_pool,
            scene_set,
            ping_set,
            pong_set,
            config: PostProcessConfig::default(),
        })
    }

    fn create_sampler(device: &ManagedDevice) -> EngineResult<vk::Sampler> {
        // Linear filtering with clamped edges: the blur relies on bilinear taps,
        // and clamping stops bright pixels wrapping to the opposite edge.
        let create_info = vk::SamplerCreateInfo::default()
            .mag_filter(vk::Filter::LINEAR)
            .min_filter(vk::Filter::LINEAR)
            .mipmap_mode(vk::SamplerMipmapMode::LINEAR)
            .address_mode_u(vk::SamplerAddressMode::CLAMP_TO_EDGE)
            .address_mode_v(vk::SamplerAddressMode::CLAMP_TO_EDGE)
            .address_mode_w(vk::SamplerAddressMode::CLAMP_TO_EDGE)
            .max_lod(1.0);

        unsafe { device.device.create_sampler(&create_info, None) }
            .map_err(|e| EngineError::Pipeline(format!("post sampler: {:?}", e)))
    }

    fn create_descriptor_pool(device: &ManagedDevice) -> EngineResult<vk::DescriptorPool> {
        // One set each for the scene, ping and pong textures.
        let pool_size = vk::DescriptorPoolSize {
            ty: vk::DescriptorType::COMBINED_IMAGE_SAMPLER,
            descriptor_count: 3,
        };

        let create_info = vk::DescriptorPoolCreateInfo::default()
            .pool_sizes(std::slice::from_ref(&pool_size))
            .max_sets(3);

        unsafe { device.device.create_descriptor_pool(&create_info, None) }
            .map_err(|e| EngineError::Pipeline(format!("post descriptor pool: {:?}", e)))
    }

    fn allocate_set(
        device: &ManagedDevice,
        pool: vk::DescriptorPool,
        layout: vk::DescriptorSetLayout,
    ) -> EngineResult<vk::DescriptorSet> {
        let alloc_info = vk::DescriptorSetAllocateInfo::default()
            .descriptor_pool(pool)
            .set_layouts(std::slice::from_ref(&layout));

        let sets = unsafe { device.device.allocate_descriptor_sets(&alloc_info) }
            .map_err(|e| EngineError::Pipeline(format!("post descriptor set: {:?}", e)))?;

        Ok(sets[0])
    }

    fn write_set(
        device: &ManagedDevice,
        set: vk::DescriptorSet,
        binding: u32,
        sampler: vk::Sampler,
        view: vk::ImageView,
    ) {
        let image_info = vk::DescriptorImageInfo {
            sampler,
            image_view: view,
            image_layout: vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
        };

        let write = vk::WriteDescriptorSet::default()
            .dst_set(set)
            .dst_binding(binding)
            .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
            .image_info(std::slice::from_ref(&image_info));

        unsafe {
            device.device.update_descriptor_sets(&[write], &[]);
        }
    }

    /// Record the full HDR resolve: bloom extraction, blur, and tonemapped
    /// composite into `swapchain_framebuffer`.
    ///
    /// Must be called after the opaque pass has ended (its colour target is
    /// expected to be in `SHADER_READ_ONLY_OPTIMAL`) and before the transparent
    /// pass begins.
    pub fn resolve(&self, cb: vk::CommandBuffer, image_index: u32, screen_extent: vk::Extent2D) {
        let bloom_extent = self.ping.extent;

        // Bright pass: full-res scene → quarter-res ping.
        let bright_params = [
            self.config.bloom_threshold,
            self.config.bloom_knee,
            0.0,
            0.0,
        ];
        self.record_stage(
            cb,
            self.pipelines.offscreen_pass,
            self.ping.framebuffer,
            bloom_extent,
            self.pipelines.bright_pass,
            self.pipelines.bright_pass_layout,
            self.scene_set,
            &bright_params,
        );

        // Separable blur, ping-ponging. A horizontal pass then a vertical pass
        // leaves the result back in `ping` for the next iteration.
        let texel = (
            1.0 / bloom_extent.width as f32,
            1.0 / bloom_extent.height as f32,
        );

        for _ in 0..BLUR_ITERATIONS {
            self.record_stage(
                cb,
                self.pipelines.offscreen_pass,
                self.pong.framebuffer,
                bloom_extent,
                self.pipelines.blur,
                self.pipelines.blur_layout,
                self.ping_set,
                &[texel.0, 0.0, 0.0, 0.0],
            );

            self.record_stage(
                cb,
                self.pipelines.offscreen_pass,
                self.ping.framebuffer,
                bloom_extent,
                self.pipelines.blur,
                self.pipelines.blur_layout,
                self.pong_set,
                &[0.0, texel.1, 0.0, 0.0],
            );
        }

        // Composite: tonemap the scene into the swapchain image. Bloom is added
        // later, by `apply_bloom`.
        self.record_stage(
            cb,
            self.pipelines.composite_pass,
            self.composite_framebuffers[image_index as usize],
            screen_extent,
            self.pipelines.composite,
            self.pipelines.composite_layout,
            self.scene_set,
            &self.tuning_params(),
        );
    }

    /// Add the bloom computed by `resolve` over the finished frame, and leave
    /// the swapchain image ready to present.
    ///
    /// Runs *after* the transparent pass rather than as part of the composite:
    /// bloom is a screen-space bleed with no depth, so compositing it earlier
    /// let opaque-writing transparent geometry (the water surface) paint over
    /// halos that belong in front of it.
    pub fn apply_bloom(
        &self,
        cb: vk::CommandBuffer,
        image_index: u32,
        screen_extent: vk::Extent2D,
    ) {
        self.record_stage(
            cb,
            self.pipelines.bloom_overlay_pass,
            self.composite_framebuffers[image_index as usize],
            screen_extent,
            self.pipelines.bloom_overlay,
            self.pipelines.bloom_overlay_layout,
            self.ping_set,
            &self.tuning_params(),
        );
    }

    /// Push-constant payload shared by the composite and bloom overlay stages.
    fn tuning_params(&self) -> [f32; 4] {
        [
            self.config.bloom_intensity,
            self.config.exposure,
            self.config.hue_preservation,
            0.0,
        ]
    }

    /// Record one fullscreen pass: begin render pass, bind, draw 3 vertices, end.
    #[allow(clippy::too_many_arguments)]
    fn record_stage(
        &self,
        cb: vk::CommandBuffer,
        render_pass: vk::RenderPass,
        framebuffer: vk::Framebuffer,
        extent: vk::Extent2D,
        pipeline: vk::Pipeline,
        layout: vk::PipelineLayout,
        descriptor_set: vk::DescriptorSet,
        params: &[f32; 4],
    ) {
        let device = &self.device.device;

        unsafe {
            let begin_info = vk::RenderPassBeginInfo::default()
                .render_pass(render_pass)
                .framebuffer(framebuffer)
                .render_area(extent.into());

            device.cmd_begin_render_pass(cb, &begin_info, vk::SubpassContents::INLINE);

            let viewport = vk::Viewport {
                x: 0.0,
                y: 0.0,
                width: extent.width as f32,
                height: extent.height as f32,
                min_depth: 0.0,
                max_depth: 1.0,
            };
            device.cmd_set_viewport(cb, 0, &[viewport]);
            device.cmd_set_scissor(cb, 0, &[extent.into()]);

            device.cmd_bind_pipeline(cb, vk::PipelineBindPoint::GRAPHICS, pipeline);
            device.cmd_bind_descriptor_sets(
                cb,
                vk::PipelineBindPoint::GRAPHICS,
                layout,
                0,
                &[descriptor_set],
                &[],
            );

            let bytes = std::slice::from_raw_parts(
                params.as_ptr() as *const u8,
                std::mem::size_of::<[f32; 4]>(),
            );
            device.cmd_push_constants(cb, layout, vk::ShaderStageFlags::FRAGMENT, 0, bytes);

            device.cmd_draw(cb, 3, 1, 0, 0);
            device.cmd_end_render_pass(cb);
        }
    }
}

impl Drop for PostProcessRenderer {
    fn drop(&mut self) {
        unsafe {
            for framebuffer in &self.composite_framebuffers {
                self.device.device.destroy_framebuffer(*framebuffer, None);
            }
            self.device
                .device
                .destroy_descriptor_pool(self.descriptor_pool, None);
            self.device.device.destroy_sampler(self.sampler, None);
        }
    }
}
