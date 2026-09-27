use crate::aim::AimState;
use crate::animation::critter::CritterAnimator;
use crate::animation::peeper::PeeperAnimator;
use crate::animation::CharacterAnimator;
use crate::components::{
    CameraComponent, MaterialModulation, ModelInstance, Orientation, Position, Renderable,
    RigidBodyComponent, Rotation,
};
use crate::core::error::{EngineError, EngineResult};
use crate::debug::{DebugConfig, DebugLines, DebugLog, DebugOverlays};
use crate::fire::components::OnFire;
use crate::hud::{Hud, HudContext};
use crate::lighting::ActiveLights;
use crate::model::Transform;
use crate::particles::ParticlePool;
use crate::rendering::debug_render::{
    render_debug_overlays_opaque, render_debug_overlays_transparent,
};
use crate::rendering::material::{MaterialManager, SurfaceModulation};
use crate::rendering::profile::{RenderProfile, RenderStage};
use crate::rendering::reflection::ProbeOwner;
use crate::rendering::renderer::Renderer;
use crate::rendering::resident::VersionedMeshId;
use crate::rendering::vertex::Vertex;
use crate::resources::textures::TextureManager;
use crate::terrain::{self, TerrainWorld};
use crate::water::WaterWorld;
use nalgebra::{Matrix4, Vector3};
use specs::shred::ResourceId;
use specs::{
    Entities, Entity, Join, Read, ReadExpect, ReadStorage, System, SystemData, World, Write,
    WriteExpect, WriteStorage,
};
use std::time::{Duration, Instant};

/// Every procedurally-rigged character in the world.
///
/// Grouped rather than listed alongside the rest of the system's data
/// because each new rig is one more storage, and the flat tuple is already
/// at the limit specs implements. Each of these draws the same way — a
/// world-space mesh the animator regenerates — so the group also says
/// something true about them.
#[derive(SystemData)]
pub struct RigAnimators<'a> {
    humanoid: WriteStorage<'a, CharacterAnimator>,
    critter: WriteStorage<'a, CritterAnimator>,
    peeper: WriteStorage<'a, PeeperAnimator>,
}

/// The debug resources the render system reads toggles from and reports into.
///
/// Grouped for the same reason as [`RigAnimators`]: the system's flat tuple is
/// at the limit specs implements.
#[derive(SystemData)]
pub struct DebugChannels<'a> {
    config: Read<'a, DebugConfig>,
    lines: Write<'a, DebugLines>,
    overlays: Read<'a, DebugOverlays>,
    log: Write<'a, DebugLog>,
}

#[derive(Default)]
pub struct RenderSystem {
    cpu_ms_ema: f32,
    /// Smoothed whole-frame GPU time, as the timestamps last reported it.
    gpu_ms_ema: f32,
    /// The game's HUD. Lives here rather than in a resource because its only
    /// reader is this system and its state is per-frame animation, not
    /// anything another system should be able to reach into.
    hud: Hud,
}

/// Marks the start of a frame's CPU work. Set by `app.rs` immediately before
/// the dispatcher runs; read by `RenderSystem` to compute total per-frame CPU
/// time excluding the vsync wait.
#[derive(Default)]
pub struct FrameStart(pub Option<std::time::Instant>);

/// Compute the fire volume scale from an entity's collider bounding radius.
fn fire_volume_scale(
    physics_world: &crate::physics::PhysicsWorld,
    rb: &RigidBodyComponent,
) -> Vector3<f32> {
    let default = Vector3::new(1.0, 1.5, 1.0);
    let body = match physics_world.body(rb.0) {
        Some(b) => b,
        None => return default,
    };
    let ch = match body.colliders().first() {
        Some(&c) => c,
        None => return default,
    };
    match physics_world.collider(ch) {
        Some(collider) => {
            let r = collider.shape().bounding_radius();
            Vector3::new(r * 2.0, r * 3.0, r * 2.0)
        }
        None => default,
    }
}

/// Build the volume-to-world matrix: translates so the volume base is centered
/// on the entity position, then scales to the fire volume dimensions.
fn fire_volume_to_world(pos: &Vector3<f32>, scale: &Vector3<f32>) -> Matrix4<f32> {
    let offset = Vector3::new(-scale.x * 0.5, 0.0, -scale.z * 0.5);
    Matrix4::new_translation(&(pos + offset)) * Matrix4::new_nonuniform_scaling(scale)
}

/// The name an entity's reflection probe is held under: its index and its
/// generation, so an entity that reuses a freed index is a new owner.
pub fn probe_owner(entity: Entity) -> ProbeOwner {
    ProbeOwner((entity.gen().id() as u32 as u64) << 32 | entity.id() as u64)
}

fn millis(time: Duration) -> f32 {
    time.as_secs_f32() * 1000.0
}

/// Exponentially smoothed frame time for the on-screen readout, seeded by
/// the first sample rather than climbing from zero.
fn smoothed(ema: f32, sample: f32) -> f32 {
    const ALPHA: f32 = 0.1;
    if ema == 0.0 {
        sample
    } else {
        ema * (1.0 - ALPHA) + sample * ALPHA
    }
}

/// Where the last finished frame's rendering time went, on both processors,
/// and what it drew — so a slow frame in the game can be read span by span.
fn log_render_profile(profile: &RenderProfile, debug_log: &mut DebugLog) {
    for (stage, time) in profile.iter() {
        debug_log.add(
            format!("Render/Cpu/{}", stage.label()),
            format!("{:.3} ms", millis(time)),
        );
    }
    debug_log.add(
        "Render/Cpu/total_work",
        format!("{:.3} ms", millis(profile.cpu_work())),
    );

    match profile.gpu {
        Some(gpu) => {
            for (span, time) in gpu.iter() {
                let text = time.map_or("-".to_string(), |t| format!("{:.3} ms", millis(t)));
                debug_log.add(format!("Render/Gpu/{}", span.label()), text);
            }
            let total = gpu
                .total
                .map_or("-".to_string(), |t| format!("{:.3} ms", millis(t)));
            debug_log.add("Render/Gpu/total", total);
        }
        None => debug_log.add("Render/Gpu/total", "unavailable"),
    }

    let counters = &profile.counters;
    debug_log.add(
        "Render/Count/draws",
        format!(
            "{} ({} opaque, {} blended, {} overlay)",
            counters.mesh_draws(),
            counters.opaque_draws,
            counters.blended_draws,
            counters.overlay_draws
        ),
    );
    debug_log.add("Render/Count/triangles", counters.triangles.to_string());
    debug_log.add(
        "Render/Count/shadow_casters",
        counters.shadow_casters.to_string(),
    );
    debug_log.add("Render/Count/particles", counters.particles.to_string());
    debug_log.add(
        "Render/Count/probes",
        format!(
            "{} ({} faces, {} draws)",
            counters.probes, counters.probe_faces, counters.probe_draws
        ),
    );
    debug_log.add(
        "Render/Count/uploaded_mesh",
        format!(
            "{:.2} MB",
            counters.uploaded_mesh_bytes as f64 / (1024.0 * 1024.0)
        ),
    );
    debug_log.add(
        "Render/Count/buffer_growths",
        counters.buffer_growths.to_string(),
    );
}

impl<'a> System<'a> for RenderSystem {
    type SystemData = (
        Entities<'a>,
        WriteExpect<'a, Renderer>,
        ReadExpect<'a, TextureManager>,
        ReadExpect<'a, MaterialManager>,
        Read<'a, crate::time::Time>,
        DebugChannels<'a>,
        Read<'a, ParticlePool>,
        Option<Read<'a, TerrainWorld>>,
        Option<Read<'a, WaterWorld>>,
        ReadStorage<'a, ModelInstance>,
        ReadStorage<'a, MaterialModulation>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, Orientation>,
        ReadStorage<'a, Renderable>,
        ReadStorage<'a, CameraComponent>,
        RigAnimators<'a>,
        ReadStorage<'a, OnFire>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadExpect<'a, super::PhysicsResource>,
        Read<'a, FrameStart>,
        Read<'a, ActiveLights>,
        Read<'a, AimState>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            entities,
            mut renderer,
            texture_manager,
            material_manager,
            time,
            mut debug,
            particle_pool,
            terrain_manager_opt,
            water_opt,
            model_instances,
            material_modulations,
            positions,
            rotations,
            orientations,
            renderables,
            camera_components,
            mut rigs,
            on_fires,
            rigid_bodies,
            physics_resource,
            frame_start,
            active_lights,
            aim_state,
        ) = data;

        let camera = camera_components.join().next();
        if camera.is_none() {
            log::error!("RenderSystem: No CameraComponent found in world!");
            return;
        }
        let camera_data = &camera.unwrap().0;
        let view_matrix = camera_data.get_view_matrix();
        let proj_matrix = camera_data.get_projection_matrix();

        // Sync fire GPU resources with OnFire ECS components
        {
            let physics_world = &physics_resource.world;

            // Remove fires for entities that are no longer on fire
            let to_remove: Vec<specs::Entity> = renderer
                .active_fires
                .iter()
                .filter(|(e, _)| !on_fires.contains(*e))
                .map(|(e, _)| *e)
                .collect();
            for entity in to_remove {
                renderer.remove_active_fire(entity);
            }

            // Create fires for newly ignited entities
            for (entity, on_fire, pos, rb) in
                (&entities, &on_fires, &positions, &rigid_bodies).join()
            {
                if renderer.active_fires.iter().any(|(e, _)| *e == entity) {
                    continue;
                }

                let scale = fire_volume_scale(physics_world, rb);
                let volume_to_world = fire_volume_to_world(&pos.0, &scale);

                renderer.create_active_fire(entity, volume_to_world, on_fire.fuel_remaining);
            }

            // Update fuel and position on existing fires
            for (entity, on_fire, pos, rb) in
                (&entities, &on_fires, &positions, &rigid_bodies).join()
            {
                if let Some((_, fire)) =
                    renderer.active_fires.iter_mut().find(|(e, _)| *e == entity)
                {
                    fire.fuel_remaining = on_fire.fuel_remaining;
                    let scale = fire_volume_scale(physics_world, rb);
                    fire.volume_to_world = fire_volume_to_world(&pos.0, &scale);
                }
            }
        }

        let begin_frame_start = std::time::Instant::now();
        let begin_frame_result = renderer.begin_frame();
        let vsync_wait = begin_frame_start.elapsed();

        match begin_frame_result {
            Ok((draw_cb, present_index)) => {
                // Recording the frame is separated from ending it. Anything in
                // here may bail out, but `end_frame` must still run either way:
                // its submit is what re-signals the draw fence and hands the
                // swapchain image back to be presented. A bare `return` from
                // the middle strands both and freezes rendering for good.
                let recorded = (|| -> EngineResult<()> {
                    // Run fire simulation compute passes before the render pass
                    let lap = Instant::now();
                    renderer.simulate_fire(draw_cb, time.delta_seconds(), time.total_seconds());
                    renderer.record_stage(RenderStage::Fire, lap.elapsed());

                    // Begin the opaque render pass
                    renderer.begin_opaque_pass(draw_cb);

                    let lap = Instant::now();

                    // Update per-frame scene data (view/projection) once
                    let camera_world_pos = Vector3::new(
                        camera_data.position.x,
                        camera_data.position.y,
                        camera_data.position.z,
                    );
                    renderer.update_scene(&view_matrix, &proj_matrix, &camera_world_pos)?;

                    renderer.update_lights(&active_lights)?;

                    // Update and render sky (before any geometry)
                    renderer.update_sky(time.delta_seconds());
                    if let Err(e) = renderer.render_sky(draw_cb, &view_matrix, &proj_matrix) {
                        log::error!("RenderSystem: Failed to render sky: {}", e);
                    }
                    renderer.record_stage(RenderStage::Setup, lap.elapsed());

                    // Draw terrain
                    let lap = Instant::now();
                    if let Some(ref terrain_manager) = terrain_manager_opt {
                        if terrain_manager.has_geometry() {
                            let identity = Matrix4::identity();

                            // Use terrain's texture if set, otherwise fallback to white
                            let texture = terrain_manager
                                .texture()
                                .unwrap_or(material_manager.fallback_texture());

                            if let Err(e) = renderer.draw_versioned_mesh(
                                draw_cb,
                                VersionedMeshId::TERRAIN,
                                terrain_manager.render_version(),
                                terrain_manager.render_vertices(),
                                terrain_manager.render_indices(),
                                &identity,
                                texture,
                                terrain::surface::surface_params(),
                                &texture_manager,
                            ) {
                                log::error!("RenderSystem: Failed to draw terrain: {}", e);
                            }
                        }
                    }
                    renderer.record_stage(RenderStage::Terrain, lap.elapsed());

                    // Draw all model instances (grenades, beach balls, etc.)
                    let lap = Instant::now();
                    for (entity, model_instance, pos, _renderable) in
                        (&entities, &model_instances, &positions, &renderables).join()
                    {
                        // Prefer 3D orientation (quaternion) if available, fall back to Y-axis rotation
                        let rotation_matrix = if let Some(orient) = orientations.get(entity) {
                            orient.0.to_homogeneous()
                        } else if let Some(rot) = rotations.get(entity) {
                            Matrix4::from_axis_angle(&Vector3::y_axis(), rot.0)
                        } else {
                            Matrix4::identity()
                        };

                        let world_matrix = Matrix4::new_translation(&pos.0) * rotation_matrix;

                        // No animation, use identity transforms
                        let part_transforms: Vec<Transform> =
                            vec![Transform::default(); model_instance.model.parts.len()];

                        let modulation = material_modulations
                            .get(entity)
                            .map(|m| m.0)
                            .unwrap_or(SurfaceModulation::IDENTITY);

                        if let Err(e) = renderer.draw_model(
                            draw_cb,
                            &model_instance.model,
                            &world_matrix,
                            &part_transforms,
                            &material_manager,
                            &texture_manager,
                            modulation,
                            Some(probe_owner(entity)),
                        ) {
                            log::error!("RenderSystem: Failed to draw model: {}", e);
                        }
                    }
                    renderer.record_stage(RenderStage::Models, lap.elapsed());

                    // Draw every procedurally-rigged character.
                    //
                    // Their vertices are already in world space — a rig
                    // poses its own joints there — so the transform is
                    // identity and the only thing that differs between rigs
                    // is which storage the mesh comes out of.
                    let lap = Instant::now();
                    let identity = Matrix4::identity();
                    let mut rig_meshes: Vec<(&[Vertex], &[u32])> = Vec::new();
                    for (controller, _pos, _rot, _renderable) in
                        (&mut rigs.humanoid, &positions, &rotations, &renderables).join()
                    {
                        rig_meshes.push(controller.mesh());
                    }
                    for (animator, _pos, _rot, _renderable) in
                        (&mut rigs.critter, &positions, &rotations, &renderables).join()
                    {
                        rig_meshes.push(animator.mesh());
                    }
                    for (animator, _pos, _rot, _renderable) in
                        (&mut rigs.peeper, &positions, &rotations, &renderables).join()
                    {
                        rig_meshes.push(animator.mesh());
                    }
                    for (vertices, indices) in rig_meshes {
                        if let Err(e) = renderer.draw_procedural_mesh(
                            draw_cb,
                            vertices,
                            indices,
                            &identity,
                            &material_manager,
                            &texture_manager,
                        ) {
                            log::error!("RenderSystem: Failed to draw a rigged character: {}", e);
                        }
                    }
                    renderer.record_stage(RenderStage::Rigs, lap.elapsed());

                    // Render opaque debug overlay shapes (spheres, lines)
                    let lap = Instant::now();
                    if let Err(e) = render_debug_overlays_opaque(
                        &mut renderer,
                        draw_cb,
                        &debug.overlays,
                        &material_manager,
                        &texture_manager,
                    ) {
                        log::error!("RenderSystem: Failed to draw debug overlays: {}", e);
                    }
                    renderer.record_stage(RenderStage::DebugShapes, lap.elapsed());

                    // Hand the water over before the particles: it is drawn inside
                    // the scene pass, and the particles are sorted by which side
                    // of its surface they are on.
                    let lap = Instant::now();
                    if let Some(ref water) = water_opt {
                        let camera_pos = Vector3::new(
                            camera_data.position.x,
                            camera_data.position.y,
                            camera_data.position.z,
                        );
                        if let Err(e) = renderer.submit_water(
                            &**water,
                            &view_matrix,
                            &proj_matrix,
                            &camera_pos,
                            time.total_seconds(),
                        ) {
                            log::error!("RenderSystem: Failed to prepare water: {}", e);
                        }
                    }
                    renderer.record_stage(RenderStage::Water, lap.elapsed());

                    // Hand the frame's particles over before the scene pass
                    // closes: they are blended scene surfaces and are recorded
                    // in order with the glass and ice, not painted on after the
                    // HDR resolve.
                    let lap = Instant::now();
                    if let Err(e) =
                        renderer.submit_particles(&particle_pool, &view_matrix, &proj_matrix)
                    {
                        log::error!("RenderSystem: Failed to prepare particles: {}", e);
                    }
                    renderer.record_stage(RenderStage::Particles, lap.elapsed());

                    // End opaque pass, blit to swapchain, begin transparent pass.
                    renderer.begin_transparent_pass(draw_cb, present_index);

                    // Render fire volumes
                    let lap = Instant::now();
                    {
                        let camera_pos = Vector3::new(
                            camera_data.position.x,
                            camera_data.position.y,
                            camera_data.position.z,
                        );
                        renderer.render_fire(draw_cb, &view_matrix, &proj_matrix, &camera_pos);
                    }
                    renderer.record_stage(RenderStage::Fire, lap.elapsed());

                    // Render transparent debug overlay shapes (triangles)
                    let lap = Instant::now();
                    if let Err(e) = render_debug_overlays_transparent(
                        &mut renderer,
                        draw_cb,
                        &debug.overlays,
                        &material_manager,
                        &texture_manager,
                    ) {
                        log::error!(
                            "RenderSystem: Failed to draw transparent debug overlays: {}",
                            e
                        );
                    }
                    renderer.record_stage(RenderStage::DebugShapes, lap.elapsed());

                    // Add FPS and fire count to debug lines
                    if debug.config.show_fps {
                        let fps = 1.0 / time.delta_seconds();
                        debug.lines.add("FPS", format!("{:.0}", fps));
                    }
                    if debug.config.show_cpu_ms {
                        debug.lines.add("CPU ms", format!("{:.2}", self.cpu_ms_ema));
                    }
                    if debug.config.show_gpu_ms {
                        if let Some(gpu) = renderer.profile().gpu.and_then(|g| g.total) {
                            self.gpu_ms_ema = smoothed(self.gpu_ms_ema, millis(gpu));
                            debug.lines.add("GPU ms", format!("{:.2}", self.gpu_ms_ema));
                        }
                    }
                    log_render_profile(renderer.profile(), &mut debug.log);
                    // if !renderer.active_fires.is_empty() {
                    //     let fire_count = renderer.active_fires.len();
                    //     let mut slot_counts = [0usize; crate::fire::renderer::SIM_POOL_SIZE];
                    //     for (_, f) in &renderer.active_fires {
                    //         slot_counts[f.sim_slot] += 1;
                    //     }
                    //     let active_slots = slot_counts.iter().filter(|&&c| c > 0).count();
                    //     debug_lines.add("Fires", format!("{} ({} slots)", fire_count, active_slots));
                    // }

                    // Debug text and HUD go out as one batch: the overlay
                    // owns a single vertex buffer, so a second upload before
                    // the first draw executes would redraw the first batch
                    // with the second's contents.
                    let lap = Instant::now();
                    let mut overlay = renderer.overlay.layout_debug_lines(debug.lines.iter());
                    let hud_context = HudContext::new(
                        renderer.overlay.screen_size(),
                        time.delta_seconds(),
                        proj_matrix * view_matrix,
                        &aim_state,
                    );
                    let solid_uv = renderer.overlay.solid_uv();
                    overlay.append(&self.hud.render(&hud_context, solid_uv));

                    if let Err(e) = renderer.render_overlay(draw_cb, &overlay) {
                        log::error!("RenderSystem: Failed to render overlay: {}", e);
                    }
                    renderer.record_stage(RenderStage::Overlay, lap.elapsed());

                    Ok(())
                })();

                if let Err(e) = recorded {
                    log::error!("RenderSystem: frame recording aborted: {}", e);
                }

                // Runs whether or not recording completed.
                match renderer.end_frame(draw_cb, present_index) {
                    Ok(()) => {}
                    // A resize or display change invalidated the swapchain
                    // mid-frame. Expected, and recovered by the rebuild on the
                    // next acquire — not worth an error line.
                    Err(EngineError::SwapchainOutOfDate) => {
                        log::debug!("RenderSystem: swapchain out of date on present");
                    }
                    Err(e) => log::error!("RenderSystem: Failed to end_frame: {}", e),
                }

                if let Some(start) = frame_start.0 {
                    let total = start.elapsed();
                    let cpu_work = total.saturating_sub(vsync_wait);
                    self.cpu_ms_ema = smoothed(self.cpu_ms_ema, millis(cpu_work));
                }
            }
            Err(EngineError::SwapchainOutOfDate) => {
                log::debug!("RenderSystem: swapchain out of date on acquire, skipping frame");
            }
            Err(e) => {
                log::error!("RenderSystem: Failed to begin_frame: {}", e);
            }
        }
    }
}
