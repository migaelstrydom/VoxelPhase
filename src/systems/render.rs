use crate::animation::critter::CritterAnimator;
use crate::animation::CharacterAnimator;
use crate::components::{
    CameraComponent, MaterialModulation, ModelInstance, Orientation, Position, Renderable,
    RigidBodyComponent, Rotation,
};
use crate::core::error::{EngineError, EngineResult};
use crate::debug::{DebugConfig, DebugLines, DebugOverlays};
use crate::fire::components::OnFire;
use crate::lighting::ActiveLights;
use crate::model::Transform;
use crate::particles::ParticlePool;
use crate::rendering::debug_render::{
    render_debug_overlays_opaque, render_debug_overlays_transparent,
};
use crate::rendering::material::{MaterialManager, SurfaceModulation};
use crate::rendering::renderer::Renderer;
use crate::rendering::vertex::Vertex;
use crate::resources::textures::TextureManager;
use crate::terrain::{self, TerrainWorld};
use crate::water::{WaterGrid, WaveGrid};
use nalgebra::{Matrix4, Vector3};
use specs::{
    Entities, Join, Read, ReadExpect, ReadStorage, System, Write, WriteExpect, WriteStorage,
};

#[derive(Default)]
pub struct RenderSystem {
    cpu_ms_ema: f32,
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

impl<'a> System<'a> for RenderSystem {
    type SystemData = (
        Entities<'a>,
        WriteExpect<'a, Renderer>,
        ReadExpect<'a, TextureManager>,
        ReadExpect<'a, MaterialManager>,
        Read<'a, crate::time::Time>,
        Write<'a, DebugLines>,
        Read<'a, DebugOverlays>,
        Read<'a, ParticlePool>,
        Option<Read<'a, TerrainWorld>>,
        Option<Read<'a, WaterGrid>>,
        Option<Read<'a, WaveGrid>>,
        ReadStorage<'a, ModelInstance>,
        ReadStorage<'a, MaterialModulation>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, Orientation>,
        ReadStorage<'a, Renderable>,
        ReadStorage<'a, CameraComponent>,
        WriteStorage<'a, CharacterAnimator>,
        WriteStorage<'a, CritterAnimator>,
        ReadStorage<'a, OnFire>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadExpect<'a, super::PhysicsResource>,
        Read<'a, FrameStart>,
        Read<'a, DebugConfig>,
        Read<'a, ActiveLights>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            entities,
            mut renderer,
            texture_manager,
            material_manager,
            time,
            mut debug_lines,
            debug_overlays,
            particle_pool,
            terrain_manager_opt,
            water_grid_opt,
            wave_grid_opt,
            model_instances,
            material_modulations,
            positions,
            rotations,
            orientations,
            renderables,
            camera_components,
            mut biped_controllers,
            mut critter_animators,
            on_fires,
            rigid_bodies,
            physics_resource,
            frame_start,
            debug_config,
            active_lights,
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
                    renderer.simulate_fire(draw_cb, time.delta_seconds(), time.total_seconds());

                    // Begin the opaque render pass
                    renderer.begin_opaque_pass(draw_cb);

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

                    // Draw terrain
                    if let Some(ref terrain_manager) = terrain_manager_opt {
                        if terrain_manager.has_geometry() {
                            let identity = Matrix4::identity();

                            // Use terrain's texture if set, otherwise fallback to white
                            let texture = terrain_manager
                                .texture()
                                .unwrap_or(material_manager.fallback_texture());

                            if let Err(e) = renderer.draw_mesh_with_texture(
                                draw_cb,
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

                    // Draw all model instances (grenades, beach balls, etc.)
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
                        ) {
                            log::error!("RenderSystem: Failed to draw model: {}", e);
                        }
                    }

                    // Draw every procedurally-rigged character.
                    //
                    // Their vertices are already in world space — a rig
                    // poses its own joints there — so the transform is
                    // identity and the only thing that differs between rigs
                    // is which storage the mesh comes out of.
                    let identity = Matrix4::identity();
                    let mut rig_meshes: Vec<(&[Vertex], &[u32])> = Vec::new();
                    for (controller, _pos, _rot, _renderable) in
                        (&mut biped_controllers, &positions, &rotations, &renderables).join()
                    {
                        rig_meshes.push(controller.mesh());
                    }
                    for (animator, _pos, _rot, _renderable) in
                        (&mut critter_animators, &positions, &rotations, &renderables).join()
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

                    // Render opaque debug overlay shapes (spheres, lines)
                    if let Err(e) = render_debug_overlays_opaque(
                        &mut renderer,
                        draw_cb,
                        &debug_overlays,
                        &material_manager,
                        &texture_manager,
                    ) {
                        log::error!("RenderSystem: Failed to draw debug overlays: {}", e);
                    }

                    // End opaque pass, blit to swapchain, begin transparent pass.
                    renderer.begin_transparent_pass(draw_cb, present_index);

                    // Render water surface (after geometry, before particles)
                    if let (Some(ref water_grid), Some(ref wave_grid)) =
                        (&water_grid_opt, &wave_grid_opt)
                    {
                        let camera_pos = Vector3::new(
                            camera_data.position.x,
                            camera_data.position.y,
                            camera_data.position.z,
                        );
                        if let Err(e) = renderer.render_water(
                            draw_cb,
                            water_grid,
                            wave_grid,
                            &view_matrix,
                            &proj_matrix,
                            &camera_pos,
                            time.total_seconds(),
                        ) {
                            log::error!("RenderSystem: Failed to render water: {}", e);
                        }
                    }

                    // Render fire volumes (after water, before particles)
                    {
                        let camera_pos = Vector3::new(
                            camera_data.position.x,
                            camera_data.position.y,
                            camera_data.position.z,
                        );
                        renderer.render_fire(draw_cb, &view_matrix, &proj_matrix, &camera_pos);
                    }

                    // Render transparent debug overlay shapes (triangles)
                    if let Err(e) = render_debug_overlays_transparent(
                        &mut renderer,
                        draw_cb,
                        &debug_overlays,
                        &material_manager,
                        &texture_manager,
                    ) {
                        log::error!(
                            "RenderSystem: Failed to draw transparent debug overlays: {}",
                            e
                        );
                    }

                    // Render particles (after models, before overlay)
                    if let Err(e) = renderer.render_particles(
                        draw_cb,
                        &particle_pool,
                        &view_matrix,
                        &proj_matrix,
                    ) {
                        log::error!("RenderSystem: Failed to render particles: {}", e);
                    }

                    // Add FPS and fire count to debug lines
                    if debug_config.show_fps {
                        let fps = 1.0 / time.delta_seconds();
                        debug_lines.add("FPS", format!("{:.0}", fps));
                    }
                    if debug_config.show_cpu_ms {
                        debug_lines.add("CPU ms", format!("{:.2}", self.cpu_ms_ema));
                    }
                    // if !renderer.active_fires.is_empty() {
                    //     let fire_count = renderer.active_fires.len();
                    //     let mut slot_counts = [0usize; crate::fire::renderer::SIM_POOL_SIZE];
                    //     for (_, f) in &renderer.active_fires {
                    //         slot_counts[f.sim_slot] += 1;
                    //     }
                    //     let active_slots = slot_counts.iter().filter(|&&c| c > 0).count();
                    //     debug_lines.add("Fires", format!("{} ({} slots)", fire_count, active_slots));
                    // }

                    // Render debug overlay (cleared in app.rs after all systems complete)
                    if let Err(e) = renderer.render_overlay(draw_cb, debug_lines.iter()) {
                        log::error!("RenderSystem: Failed to render overlay: {}", e);
                    }

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
                    let cpu_ms = cpu_work.as_secs_f32() * 1000.0;
                    let alpha = 0.1;
                    self.cpu_ms_ema = if self.cpu_ms_ema == 0.0 {
                        cpu_ms
                    } else {
                        self.cpu_ms_ema * (1.0 - alpha) + cpu_ms * alpha
                    };
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
