//! Renders scenes headlessly through the real Vulkan pipeline.

use std::sync::Arc;

use image::RgbaImage;

use crate::core::error::{EngineError, EngineResult};
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::colour::Colour;
use crate::rendering::material::{MaterialManager, MaterialManagerBuilder};
use crate::rendering::renderer::Renderer;
use crate::rendering::visual_bench::scene::{SceneContext, SceneShot};
use crate::resources::manager::ResourceManager;
use crate::resources::textures::TextureManager;

/// Time handed to the water shader. Fixed, because a shot that is compared
/// against its own past has to see the same ripples every run.
const WATER_TIME: f32 = 0.0;

/// A headless renderer plus the resource managers a scene needs to draw.
///
/// Owns the same objects the game's world builder wires together, so a scene is
/// drawn with the same materials and textures the game would use.
pub struct VisualBench {
    renderer: Renderer,
    texture_manager: TextureManager,
    material_manager: MaterialManager,

    /// Keeps the transfer service alive for the texture manager's lifetime.
    _resources: ResourceManager,
}

impl VisualBench {
    /// Build a bench rendering at the given resolution.
    ///
    /// Creates its own Vulkan context with no windowing-system connection, so
    /// this works from a plain shell with no display attached.
    pub fn new(width: u32, height: u32) -> EngineResult<Self> {
        let vulkan_context = Arc::new(VulkanContext::headless()?);
        let mut renderer = Renderer::offscreen(Arc::clone(&vulkan_context), width, height)?;

        let resources = ResourceManager::new(Arc::clone(&vulkan_context))?;
        let texture_manager = resources.create_texture_manager(renderer.descriptor_manager())?;

        let fallback_white = texture_manager
            .create_solid_colour(Colour::WHITE)
            .map_err(|e| {
                EngineError::InvalidState(format!("visual bench fallback texture: {}", e))
            })?;
        let material_manager = MaterialManagerBuilder::new().build(fallback_white);

        // Nothing in the bench needs the wireframe debug pass, and leaving it
        // on would put black lines through every shot.
        renderer.debug_wireframe_backfaces = false;

        Ok(Self {
            renderer,
            texture_manager,
            material_manager,
            _resources: resources,
        })
    }

    /// Resources a scene may use while building its shots.
    pub fn context(&self) -> SceneContext<'_> {
        SceneContext {
            textures: &self.texture_manager,
            materials: &self.material_manager,
        }
    }

    /// Render one shot and read the result back as an image.
    ///
    /// Follows exactly the frame sequence `RenderSystem` uses, minus the passes
    /// no scene currently populates (fire, particles) — those go through the
    /// same transparent pass and can be added when a scene needs them.
    pub fn render(&mut self, shot: &SceneShot) -> EngineResult<RgbaImage> {
        let extent = self.renderer.extent();
        let aspect = extent.width as f32 / extent.height as f32;

        let view = shot.camera.view();
        let projection = shot.camera.projection(aspect);
        let camera_pos = shot.camera.eye.coords;

        // The sun lives on the sky renderer so that the shaded geometry and the
        // visible sun disc cannot disagree; the rest of the environment goes
        // straight onto the renderer.
        self.renderer
            .sky_renderer
            .set_sun_direction(shot.environment.lighting.sun_direction);
        *self.renderer.lighting_mut() = shot.environment.lighting;
        self.renderer.post_process.config = shot.environment.post;

        // The map was allocated at the renderer's resolution and cannot be
        // resized per shot, so that one field stays as built.
        let resolution = self.renderer.shadow.volume.resolution;
        self.renderer.shadow.volume = shot.environment.shadow;
        self.renderer.shadow.volume.resolution = resolution;

        let (cb, image_index) = self.renderer.begin_frame()?;

        self.renderer.begin_opaque_pass(cb);
        self.renderer
            .update_scene(&view, &projection, &camera_pos)?;
        self.renderer
            .update_lights(&shot.environment.active_lights())?;
        self.renderer.render_sky(cb, &view, &projection)?;

        // Destructured so the draw can hold `&mut renderer` while the texture
        // reference borrows the material manager.
        let Self {
            renderer,
            texture_manager,
            material_manager,
            ..
        } = self;

        for mesh in &shot.meshes {
            let texture = mesh
                .texture
                .as_ref()
                .unwrap_or_else(|| material_manager.fallback_texture());

            renderer.draw_mesh_with_texture(
                cb,
                &mesh.vertices,
                &mesh.indices,
                &mesh.transform,
                texture,
                mesh.surface,
                texture_manager,
            )?;
        }

        // Particles are blended scene surfaces, submitted before the scene
        // pass closes so they are sorted in with the blended meshes — the same
        // place and the same order as in a level.
        if let Some(particles) = &shot.particles {
            renderer.submit_particles(particles, &view, &projection)?;
        }

        renderer.begin_transparent_pass(cb, image_index);

        // In the transparent pass, against the depth the scene left behind —
        // the same place and the same order as in a level.
        if let Some(pool) = &shot.water {
            renderer.render_water(cb, pool, &view, &projection, &camera_pos, WATER_TIME)?;
        }

        renderer.end_frame(cb, image_index)?;

        // Readback reads the image directly, so the frame has to be finished
        // rather than merely submitted.
        renderer.wait_for_frame()?;
        let pixels = renderer.output.read_pixels()?;

        RgbaImage::from_raw(extent.width, extent.height, pixels).ok_or_else(|| {
            EngineError::InvalidState("readback produced the wrong number of pixels".to_string())
        })
    }
}
