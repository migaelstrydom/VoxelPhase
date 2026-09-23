use std::sync::Arc;
use std::time::Duration;

use image::RgbaImage;
use nalgebra::{Point3, Vector3};
use specs::{Builder, Dispatcher, Entity, World, WorldExt};

use crate::app::{run_frame, FrameTiming, GameWorld};
use crate::components::CameraComponent;
use crate::core::error::{EngineError, EngineResult};
use crate::core::vulkan_context::VulkanContext;
use crate::level::Level;
use crate::projectile::{spawn_grenade, GrenadeConfig, GrenadeModelResource};
use crate::rendering::camera::Camera;
use crate::rendering::profile::RenderProfile;
use crate::rendering::renderer::Renderer;
use crate::resources::manager::ResourceManager;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;
use crate::time::Time;

/// Vertical field of view of the bench camera, matching the game's.
const FIELD_OF_VIEW: f32 = std::f32::consts::FRAC_PI_4;

/// What one call to [`GameHarness::step`] measured.
#[derive(Debug, Clone)]
pub struct FrameSample {
    /// Wall-clock time of the frame just run, split into its two halves.
    pub timing: FrameTiming,
    /// The physics world's own account of the frame just run.
    pub physics: Duration,
    /// The *previous* frame's render profile. A frame's GPU times can only be
    /// read once its fence has signalled, which is at the start of the next
    /// frame, so the renderer publishes each profile one frame late.
    pub previous_render: RenderProfile,
}

/// The real game, run headlessly against an offscreen renderer.
///
/// ```text
///   Level ─▶ GameWorld::assemble(Renderer::offscreen) ─▶ World + Dispatcher
///                                                          │
///   step: Time += dt ─▶ run_frame (the game's own frame) ◀─┘
/// ```
///
/// Nothing is stood in for: the systems are the game's, the dispatcher is the
/// game's, and `run_frame` is the very sequence `App` runs, so what this
/// measures is what the game costs. The only differences are the clock, which
/// advances by a fixed step rather than the wall, and the camera, which is
/// placed rather than following the player.
pub struct GameHarness<'a, 'b> {
    world: World,
    dispatcher: Dispatcher<'a, 'b>,
    /// The fixed camera, once placed.
    camera: Option<Entity>,
    /// Width over height of the render target.
    aspect: f32,
    _context: Arc<VulkanContext>,
}

impl<'a, 'b> GameHarness<'a, 'b> {
    /// Load `level` into a game rendering at `width` × `height`, stepping
    /// `frame_dt` seconds a frame.
    pub fn open(level: &Level, width: u32, height: u32, frame_dt: f32) -> EngineResult<Self> {
        let context = Arc::new(VulkanContext::headless()?);
        let renderer = Renderer::offscreen(Arc::clone(&context), width, height)?;
        let resources = ResourceManager::new(Arc::clone(&context))?;
        let textures = resources.create_texture_manager(renderer.descriptor_manager())?;

        let GameWorld {
            mut world,
            dispatcher,
            ..
        } = GameWorld::assemble(renderer, resources, textures, level)?;
        world.insert(Time::fixed(frame_dt));

        Ok(Self {
            world,
            dispatcher,
            camera: None,
            aspect: width as f32 / height as f32,
            _context: context,
        })
    }

    /// Look from `eye` at `target` for every frame from now on.
    ///
    /// The camera carries no follow target, so the camera system leaves it
    /// where it is put.
    pub fn place_camera(&mut self, eye: Point3<f32>, target: Point3<f32>) {
        let camera = CameraComponent(Camera::new(
            eye,
            target,
            Vector3::y(),
            FIELD_OF_VIEW,
            self.aspect,
            0.1,
            500.0,
        ));
        match self.camera {
            Some(entity) => {
                let mut cameras = self.world.write_storage::<CameraComponent>();
                if let Some(existing) = cameras.get_mut(entity) {
                    *existing = camera;
                }
            }
            None => self.camera = Some(self.world.create_entity().with(camera).build()),
        }
    }

    /// Put a live grenade at `position`, at rest, fuse lit.
    pub fn drop_grenade(&mut self, position: Point3<f32>) -> EngineResult<Entity> {
        let model = self
            .world
            .read_resource::<GrenadeModelResource>()
            .model
            .clone()
            .ok_or_else(|| EngineError::InvalidState("no grenade model".to_string()))?;
        let config = self.world.read_resource::<GrenadeConfig>().clone();
        let mut physics = self.world.write_resource::<PhysicsResource>();

        Ok(spawn_grenade(
            self.world.create_entity_unchecked(),
            &mut physics.world,
            &config,
            model,
            position,
            Vector3::zeros(),
        ))
    }

    /// Whether `entity` still exists — a grenade stops existing when it goes off.
    pub fn is_alive(&self, entity: Entity) -> bool {
        self.world.is_alive(entity)
    }

    /// The top of the meshed terrain at world column (x, z).
    pub fn surface_at(&self, x: f32, z: f32) -> Option<Point3<f32>> {
        self.world
            .read_resource::<TerrainWorld>()
            .mesh_surface_height_at(x, z)
            .map(|y| Point3::new(x, y, z))
    }

    /// Simulated seconds since the harness opened.
    pub fn sim_time(&self) -> f32 {
        self.world.read_resource::<Time>().total_seconds()
    }

    /// Run one frame of the game.
    pub fn step(&mut self) -> FrameSample {
        self.world.write_resource::<Time>().advance_fixed();
        let timing = run_frame(&mut self.world, &mut self.dispatcher);

        let physics = self
            .world
            .read_resource::<PhysicsResource>()
            .world
            .frame_profile()
            .total();
        let previous_render = self.world.read_resource::<Renderer>().profile().clone();

        FrameSample {
            timing,
            physics,
            previous_render,
        }
    }

    /// The last frame drawn, as an image. Waits for the GPU, so take it after
    /// the frames being timed, never between them.
    pub fn snapshot(&self) -> EngineResult<RgbaImage> {
        let renderer = self.world.read_resource::<Renderer>();
        renderer.wait_for_frame()?;
        let extent = renderer.extent();
        let pixels = renderer.output.read_pixels()?;
        RgbaImage::from_raw(extent.width, extent.height, pixels).ok_or_else(|| {
            EngineError::InvalidState("readback produced the wrong number of pixels".to_string())
        })
    }
}

impl Drop for GameHarness<'_, '_> {
    fn drop(&mut self) {
        // The world owns GPU resources the last frame may still be using.
        let renderer = self.world.read_resource::<Renderer>();
        let _ = renderer.wait_for_frame();
    }
}
