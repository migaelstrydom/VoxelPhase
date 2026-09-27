//! Renders a level file headlessly, through the pipeline the game uses.
//!
//! `level_check` answers every question about a level that can be put as a
//! number, and a level can pass all of them and still be wrong in ways that are
//! obvious the moment anyone looks at it. An object floating a quarter of a
//! metre off the ground is legal — deliberate drops are ordinary authoring — and
//! it is unmistakable in a single frame. So is a wall facing the wrong way, a
//! jump that reads as impossible, a landmark hidden behind a bench.
//!
//! ```text
//!   level.ron ─load──▶ Level ─create_level_terrain──▶ TerrainWorld ─┐
//!                       │                                           │
//!                       ├─create_level_materials──▶ LevelMaterials  │
//!                       │                              │            │
//!                       │                              ▼            ▼
//!                       ├────────spawn_objects────▶  ECS world (models, positions)
//!                       └─create_level_water──▶ WaterWorld  │
//!                                                        │
//!                                       Renderer::offscreen ──▶ RgbaImage
//! ```
//!
//! Nothing here is a stand-in. The terrain is meshed by the real generator, the
//! objects are built by the real spawnables with the real materials, and the
//! frame goes through the real shaders — which is the whole point, because a
//! blockout made of proxy boxes would answer a different question from the one
//! being asked.
//!
//! What it is *not* is a running game: no systems are dispatched, so nothing has
//! settled under gravity and nothing has moved. That is a feature. What it shows
//! is the level exactly as authored, which is the thing being checked. The one
//! exception is asked for: [`LevelViewer::stir_water`] blasts and splashes the
//! water and runs it on, so ripples and their seams can be seen.

use std::sync::Arc;

use image::RgbaImage;
use nalgebra::{Matrix4, Point3, Vector3};
use specs::{Join, World, WorldExt};

use crate::animation::critter::CritterAnimator;
use crate::animation::peeper::PeeperAnimator;
use crate::app::world_builder::WorldBuilder;
use crate::components::{ModelInstance, Orientation, Position, Renderable, Rotation};
use crate::core::error::{EngineError, EngineResult};
use crate::core::vulkan_context::VulkanContext;
use crate::level::{
    create_level_materials, create_level_terrain, create_level_water, spawn_objects, Level,
};
use crate::model::Transform;
use crate::rendering::colour::Colour;
use crate::rendering::material::{MaterialManager, MaterialManagerBuilder, SurfaceModulation};
use crate::rendering::renderer::Renderer;
use crate::rendering::resident::VersionedMeshId;
use crate::resources::manager::ResourceManager;
use crate::resources::textures::TextureManager;
use crate::systems::probe_owner;
use crate::terrain::{self, BlastConfig, TerrainWorld};
use crate::water::{Disturbance, WaterWorld};

use super::shots::ViewerShot;

/// The most frames a still is drawn over while its reflection probes are
/// handed out. Each admits a few; this covers a probe atlas's worth.
const MAX_PROBE_SETTLING_FRAMES: usize = 8;

/// A level loaded onto the GPU, ready to be photographed from anywhere.
///
/// Loading is the expensive part — terrain generation dominates — so the viewer
/// is built once and rendered from many times.
pub struct LevelViewer {
    renderer: Renderer,
    texture_manager: TextureManager,
    material_manager: MaterialManager,

    /// Holds the spawned objects, the terrain and the water. Never
    /// dispatched.
    world: World,

    /// Keeps the transfer service alive for the texture manager's lifetime.
    _resources: ResourceManager,
}

impl LevelViewer {
    /// Generate, spawn and upload a level at the given render resolution.
    ///
    /// Creates its own Vulkan context with no windowing-system connection, so
    /// this works from a plain shell with no display attached — which is the
    /// only reason it is useful, since the game's own window cannot be opened
    /// from one.
    pub fn open(level: &Level, width: u32, height: u32) -> EngineResult<Self> {
        let vulkan_context = Arc::new(VulkanContext::headless()?);
        let mut renderer = Renderer::offscreen(Arc::clone(&vulkan_context), width, height)?;

        let resources = ResourceManager::new(Arc::clone(&vulkan_context))?;
        let texture_manager = resources.create_texture_manager(renderer.descriptor_manager())?;

        // Order matters and is not negotiable: every object's materials have to
        // be registered while the builder is still mutable, so the terrain and
        // the props are generated before the manager is sealed.
        let terrain = create_level_terrain(level, &texture_manager)?;

        let mut material_builder = MaterialManagerBuilder::new();
        let materials = create_level_materials(level, &texture_manager, &mut material_builder)?;
        let fallback_white = texture_manager
            .create_solid_colour(Colour::WHITE)
            .map_err(|e| {
                EngineError::InvalidState(format!("level viewer fallback texture: {e}"))
            })?;
        let material_manager = material_builder.build(fallback_white);

        // Terrain-anchored objects resolve their height from the world as they
        // spawn, so it goes in first.
        let water = create_level_water(level, &terrain);
        let mut world = WorldBuilder::new().with_default_resources().build()?;
        world.insert(terrain);
        if let Some(water) = water {
            world.insert(water);
        }
        spawn_objects(&mut world, level, &materials);

        // The wireframe pass would put black lines through every shot.
        renderer.debug_wireframe_backfaces = false;

        Ok(Self {
            renderer,
            texture_manager,
            material_manager,
            world,
            _resources: resources,
        })
    }

    /// The level's meshed terrain, for framing shots against what was actually
    /// generated rather than against what was authored.
    pub fn terrain(&self) -> specs::shred::Fetch<'_, TerrainWorld> {
        self.world.read_resource::<TerrainWorld>()
    }

    /// Swap in a terrain and water from outside, and back out on the next
    /// call: how an offline harness that edits and simulates its own world
    /// photographs it without copying either.
    pub fn swap_state(&mut self, terrain: &mut TerrainWorld, water: &mut WaterWorld) {
        std::mem::swap(&mut *self.world.write_resource::<TerrainWorld>(), terrain);
        if !self.world.has_value::<WaterWorld>() {
            let (empty, _) = WaterWorld::from_config(&Default::default(), terrain);
            self.world.insert(empty);
        }
        std::mem::swap(&mut *self.world.write_resource::<WaterWorld>(), water);
    }

    /// Set off each blast, drop a splash at each point, then run the water
    /// on for `seconds` of 60 Hz frames. The blasts' frame lasts `blast_dt`:
    /// a blast frame in the game is a long one.
    pub fn stir_water(
        &mut self,
        blasts: &[Point3<f32>],
        splashes: &[Point3<f32>],
        blast_dt: f32,
        seconds: f32,
    ) {
        let mut terrain = self.world.write_resource::<TerrainWorld>();
        let Some(mut water) = self.world.try_fetch_mut::<WaterWorld>() else {
            return;
        };
        if !blasts.is_empty() {
            for &at in blasts {
                terrain.detonate(at, &BlastConfig::default());
            }
            terrain.update();
            water.on_terrain_update(&terrain);
            for &at in blasts {
                water.disturb(at, 2.0, Disturbance::Velocity(-6.0));
            }
            water.step(blast_dt);
        }
        for &at in splashes {
            water.disturb(at, 0.5, Disturbance::Velocity(-3.0));
        }
        let frame = 1.0 / 60.0;
        for _ in 0..(seconds / frame).round() as usize {
            water.step(frame);
        }
    }

    /// Render one view and read the result back as an image.
    ///
    /// Mirrors `RenderSystem`'s frame sequence, minus the passes a static level
    /// has nothing to put in: no particles and no fire.
    pub fn render(&mut self, shot: &ViewerShot) -> EngineResult<RgbaImage> {
        let extent = self.renderer.extent();
        let aspect = extent.width as f32 / extent.height as f32;

        let view = shot.camera.view();
        let projection = shot.camera.projection(aspect);
        let camera_pos = shot.camera.eye.coords;

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

        // A reflection probe is handed out on the frame after its object first
        // asks for one, and a frame admits only a few, so the shot is drawn
        // until every probe it wants is live. The game gets there within a
        // few frames of a level loading, so the still shows what it shows.
        for _ in 0..MAX_PROBE_SETTLING_FRAMES {
            self.draw_frame(&view, &projection, &camera_pos, shot)?;
            if !self.renderer.probes.settling() {
                break;
            }
        }

        let pixels = self.renderer.output.read_pixels()?;

        RgbaImage::from_raw(extent.width, extent.height, pixels).ok_or_else(|| {
            EngineError::InvalidState("readback produced the wrong number of pixels".to_string())
        })
    }

    /// Record, submit and finish one frame of `shot`.
    fn draw_frame(
        &mut self,
        view: &Matrix4<f32>,
        projection: &Matrix4<f32>,
        camera_pos: &Vector3<f32>,
        shot: &ViewerShot,
    ) -> EngineResult<()> {
        let (view, projection, camera_pos) = (*view, *projection, *camera_pos);
        let (cb, image_index) = self.renderer.begin_frame()?;

        self.renderer.begin_opaque_pass(cb);
        self.renderer
            .update_scene(&view, &projection, &camera_pos)?;
        self.renderer
            .update_lights(&shot.environment.active_lights())?;
        self.renderer.render_sky(cb, &view, &projection)?;

        self.draw_terrain(cb)?;
        self.draw_objects(cb)?;

        if let Some(water) = self.world.try_fetch::<WaterWorld>() {
            self.renderer
                .submit_water(&*water, &view, &projection, &camera_pos, 0.0)?;
        }
        self.renderer.begin_transparent_pass(cb, image_index);
        self.renderer.end_frame(cb, image_index)?;

        // Readback reads the image directly, so the frame has to be finished
        // rather than merely submitted.
        self.renderer.wait_for_frame()
    }

    fn draw_terrain(&mut self, cb: ash::vk::CommandBuffer) -> EngineResult<()> {
        let terrain = self.world.read_resource::<TerrainWorld>();
        if !terrain.has_geometry() {
            return Ok(());
        }

        let texture = terrain
            .texture()
            .unwrap_or_else(|| self.material_manager.fallback_texture());

        self.renderer.draw_versioned_mesh(
            cb,
            VersionedMeshId::TERRAIN,
            terrain.render_version(),
            terrain.render_vertices(),
            terrain.render_indices(),
            &Matrix4::identity(),
            texture,
            terrain::surface::surface_params(),
            &self.texture_manager,
        )
    }

    /// Draw every spawned object, exactly as `RenderSystem` would.
    ///
    /// Part transforms are the identity throughout: those are animation's job,
    /// and nothing here is animated.
    fn draw_objects(&mut self, cb: ash::vk::CommandBuffer) -> EngineResult<()> {
        {
            let entities = self.world.entities();
            let models = self.world.read_storage::<ModelInstance>();
            let positions = self.world.read_storage::<Position>();
            let renderables = self.world.read_storage::<Renderable>();
            let orientations = self.world.read_storage::<Orientation>();
            let rotations = self.world.read_storage::<Rotation>();

            for (entity, model, position, _) in
                (&entities, &models, &positions, &renderables).join()
            {
                let rotation = if let Some(orientation) = orientations.get(entity) {
                    orientation.0.to_homogeneous()
                } else if let Some(rotation) = rotations.get(entity) {
                    Matrix4::from_axis_angle(&Vector3::y_axis(), rotation.0)
                } else {
                    Matrix4::identity()
                };

                let world_matrix = Matrix4::new_translation(&position.0) * rotation;
                let part_transforms = vec![Transform::default(); model.model.parts.len()];

                self.renderer.draw_model(
                    cb,
                    &model.model,
                    &world_matrix,
                    &part_transforms,
                    &self.material_manager,
                    &self.texture_manager,
                    SurfaceModulation::IDENTITY,
                    Some(probe_owner(entity)),
                )?;
            }
        }

        self.draw_rigs(cb)
    }

    /// Draw the procedurally-rigged creatures, which carry no model.
    ///
    /// Their pose is whatever the rig was built at, since nothing here
    /// animates — but a critter standing in its rest pose is still the
    /// difference between an author seeing where their creatures landed
    /// and seeing an empty field.
    fn draw_rigs(&mut self, cb: ash::vk::CommandBuffer) -> EngineResult<()> {
        let mut meshes = Vec::new();
        {
            let entities = self.world.entities();
            let renderables = self.world.read_storage::<Renderable>();
            let mut critters = self.world.write_storage::<CritterAnimator>();
            let mut peepers = self.world.write_storage::<PeeperAnimator>();

            for (_, animator, _) in (&entities, &mut critters, &renderables).join() {
                let (vertices, indices) = animator.mesh();
                meshes.push((vertices.to_vec(), indices.to_vec()));
            }
            for (_, animator, _) in (&entities, &mut peepers, &renderables).join() {
                let (vertices, indices) = animator.mesh();
                meshes.push((vertices.to_vec(), indices.to_vec()));
            }
        }

        for (vertices, indices) in meshes {
            self.renderer.draw_procedural_mesh(
                cb,
                &vertices,
                &indices,
                &Matrix4::identity(),
                &self.material_manager,
                &self.texture_manager,
            )?;
        }

        Ok(())
    }
}
