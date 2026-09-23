use std::path::Path;
use std::sync::Arc;

use winit::{
    dpi::LogicalSize,
    event::Event,
    event_loop::{ControlFlow, EventLoop},
    window::{Window, WindowBuilder},
};

use specs::{Dispatcher, World, WorldExt};

use crate::core::error::{EngineError, EngineResult};
use crate::core::vulkan_context::VulkanContext;
use crate::debug::DebugConfig;
use crate::level::load_level;
use crate::rendering::renderer::Renderer;
use crate::resources::manager::ResourceManager;
use crate::systems::FrameStart;
use crate::time::Time;

use super::event_handler::{set_mouse_captured, EventHandler, EventResult};
use super::frame::run_frame;
use super::game_world::GameWorld;
use super::spawners::spawn_camera;

pub struct App<'a, 'b> {
    event_loop: Option<EventLoop<()>>,
    window: Window,
    world: World,
    dispatcher: Dispatcher<'a, 'b>,
}

impl<'a, 'b> App<'a, 'b> {
    pub fn new(
        window_width: u32,
        window_height: u32,
        app_title: &str,
        level_path: &Path,
    ) -> EngineResult<Self> {
        let (event_loop, window) = Self::create_window(window_width, window_height, app_title)?;
        let (_vulkan_context, renderer, resource_manager, texture_manager) =
            Self::create_rendering_context(&window, window_width, window_height)?;

        let level = load_level(level_path)
            .map_err(|e| EngineError::InvalidState(format!("Failed to load level: {}", e)))?;

        let GameWorld {
            mut world,
            dispatcher,
            player,
        } = GameWorld::assemble(renderer, resource_manager, texture_manager, &level)?;
        spawn_camera(&mut world, player, window_width, window_height);

        Ok(Self {
            event_loop: Some(event_loop),
            window,
            world,
            dispatcher,
        })
    }

    fn create_window(
        width: u32,
        height: u32,
        title: &str,
    ) -> EngineResult<(EventLoop<()>, Window)> {
        let event_loop = EventLoop::new()
            .map_err(|e| EngineError::Window(format!("Failed to create event loop: {}", e)))?;

        let window = WindowBuilder::new()
            .with_title(title)
            .with_inner_size(LogicalSize::new(f64::from(width), f64::from(height)))
            .build(&event_loop)
            .map_err(|e| EngineError::Window(format!("Failed to create window: {}", e)))?;

        Ok((event_loop, window))
    }

    fn create_rendering_context(
        window: &Window,
        width: u32,
        height: u32,
    ) -> EngineResult<(
        Arc<VulkanContext>,
        Renderer,
        ResourceManager,
        crate::resources::textures::TextureManager,
    )> {
        let vulkan_context = Arc::new(VulkanContext::new(window)?);

        let renderer = Renderer::for_window(Arc::clone(&vulkan_context), window, width, height)
            .map_err(|e| EngineError::InvalidState(format!("Failed to create renderer: {}", e)))?;

        let resource_manager = ResourceManager::new(Arc::clone(&vulkan_context))?;
        let descriptor_manager = renderer.descriptor_manager();
        let texture_manager = resource_manager.create_texture_manager(descriptor_manager)?;

        Ok((vulkan_context, renderer, resource_manager, texture_manager))
    }

    pub fn run(&mut self) -> EngineResult<()> {
        let event_loop = self
            .event_loop
            .take()
            .ok_or_else(|| EngineError::InvalidState("Event loop already consumed".to_string()))?;

        set_mouse_captured(&self.window, &mut self.world, true);

        let window = &self.window;
        let world = &mut self.world;
        let dispatcher = &mut self.dispatcher;

        let run_result = event_loop.run(|event, elwt| {
            elwt.set_control_flow(ControlFlow::Poll);

            let is_about_to_wait = matches!(event, Event::AboutToWait);

            match EventHandler::handle(event, window, world, elwt) {
                EventResult::Exit => return,
                EventResult::Continue => {}
            }

            if is_about_to_wait {
                {
                    let mut time = world.write_resource::<Time>();
                    time.update();
                }
                {
                    let show_cpu_ms = world.read_resource::<DebugConfig>().show_cpu_ms;
                    let mut frame_start = world.write_resource::<FrameStart>();
                    frame_start.0 = show_cpu_ms.then(std::time::Instant::now);
                }

                run_frame(world, dispatcher);
            }
        });

        {
            let renderer = self.world.read_resource::<Renderer>();
            unsafe {
                renderer
                    .vulkan_context
                    .device()
                    .device_wait_idle()
                    .expect("Failed to wait for device idle");
            }
        }

        log::info!("App::run - Event loop finished. App will now drop.");

        run_result.map_err(|e| EngineError::Window(format!("Event loop error: {}", e)))?;
        Ok(())
    }
}

#[cfg(test)]
mod frame_order {
    use specs::{
        Builder, Component, DispatcherBuilder, Entities, Join, LazyUpdate, Read, ReadStorage,
        System, VecStorage, World, WorldExt,
    };
    use std::sync::{Arc, Mutex};

    #[derive(Component)]
    #[storage(VecStorage)]
    struct Spawned;

    /// Stands in for `FractureSystem`: a parallel system that puts a new
    /// entity into the world through `LazyUpdate`.
    struct Spawner {
        done: bool,
    }

    impl<'a> System<'a> for Spawner {
        type SystemData = (Entities<'a>, Read<'a, LazyUpdate>);

        fn run(&mut self, (entities, lazy): Self::SystemData) {
            if self.done {
                return;
            }
            self.done = true;
            lazy.create_entity(&entities).with(Spawned).build();
        }
    }

    /// Stands in for `RenderSystem`: thread-local, and sees the world only
    /// through a join, exactly as the draw loop does.
    struct Drawer {
        seen: Arc<Mutex<Vec<usize>>>,
    }

    impl<'a> System<'a> for Drawer {
        type SystemData = (Entities<'a>, ReadStorage<'a, Spawned>);

        fn run(&mut self, (entities, spawned): Self::SystemData) {
            let count = (&entities, &spawned).join().count();
            self.seen.lock().unwrap().push(count);
        }
    }

    fn world_and_log() -> (World, Arc<Mutex<Vec<usize>>>) {
        let mut world = World::new();
        world.register::<Spawned>();
        (world, Arc::new(Mutex::new(Vec::new())))
    }

    /// The frame loop must draw once, and only after `maintain` has put this
    /// frame's lazily created entities into their storages.
    ///
    /// Getting this wrong is what made a fractured piece blink: the compound's
    /// model drops the piece the instant it breaks off, so a draw that happens
    /// before the piece's own entity exists shows neither.
    #[test]
    fn the_frame_draws_once_and_only_after_the_world_is_maintained() {
        let (mut world, seen) = world_and_log();
        let mut dispatcher = DispatcherBuilder::new()
            .with(Spawner { done: false }, "spawn", &[])
            .with_thread_local(Drawer { seen: seen.clone() })
            .build();

        // Exactly the sequence in `App::run`.
        dispatcher.dispatch_par(&world);
        world.maintain();
        dispatcher.dispatch_thread_local(&world);
        world.maintain();

        let seen = seen.lock().unwrap();
        assert_eq!(
            seen.len(),
            1,
            "the frame drew {} times, not once",
            seen.len()
        );
        assert_eq!(
            seen[0], 1,
            "the draw did not see the entity created this frame"
        );
    }

    /// Why the loop calls `dispatch_par` and not `dispatch`: `dispatch` runs
    /// the thread-local systems itself, so pairing it with an explicit
    /// `dispatch_thread_local` draws the frame twice — and the first of those
    /// draws happens before `maintain`, with the hole described above.
    #[test]
    fn dispatch_would_run_the_thread_local_draw_as_well() {
        let (world, seen) = world_and_log();
        let mut dispatcher = DispatcherBuilder::new()
            .with(Spawner { done: false }, "spawn", &[])
            .with_thread_local(Drawer { seen: seen.clone() })
            .build();

        dispatcher.dispatch(&world);

        let seen = seen.lock().unwrap();
        assert_eq!(
            seen.len(),
            1,
            "dispatch no longer runs thread-local systems; the frame loop's \
             split into dispatch_par + dispatch_thread_local can be revisited"
        );
        assert_eq!(
            seen[0], 0,
            "the draw inside dispatch ran before any maintain, so it cannot \
             see this frame's lazily created entity"
        );
    }
}
