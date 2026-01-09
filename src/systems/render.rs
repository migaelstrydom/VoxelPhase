use crate::components::{CameraComponent, ModelInstance, Position, Renderable, Rotation};
use crate::model::Transform;
use crate::player::PlayerAnimationState;
use crate::rendering::material::MaterialManager;
use crate::rendering::renderer::Renderer;
use crate::resources::textures::TextureManager;
use nalgebra::{Matrix4, Vector3};
use specs::{Join, LendJoin, ReadExpect, ReadStorage, System, WriteExpect};

pub struct RenderSystem;

impl<'a> System<'a> for RenderSystem {
    type SystemData = (
        WriteExpect<'a, Renderer>,
        ReadExpect<'a, TextureManager>,
        ReadExpect<'a, MaterialManager>,
        ReadStorage<'a, ModelInstance>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, Renderable>,
        ReadStorage<'a, CameraComponent>,
        ReadStorage<'a, PlayerAnimationState>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            mut renderer,
            texture_manager,
            material_manager,
            model_instances,
            positions,
            rotations,
            renderables,
            camera_components,
            player_animations,
        ) = data;

        let camera = camera_components.join().next();
        if camera.is_none() {
            log::error!("RenderSystem: No CameraComponent found in world!");
            return;
        }
        let camera_data = &camera.unwrap().0;
        let view_matrix = camera_data.get_view_matrix();
        let proj_matrix = camera_data.get_projection_matrix();

        match renderer.begin_frame() {
            Ok((draw_cb, present_index)) => {
                // Update per-frame scene data (view/projection) once
                if let Err(e) = renderer.update_scene(&view_matrix, &proj_matrix) {
                    log::error!("RenderSystem: Failed to update scene UBO: {}", e);
                    return;
                }

                // Draw all model instances
                for (model_instance, pos, rot, _renderable, player_anim) in (
                    &model_instances,
                    &positions,
                    &rotations,
                    &renderables,
                    (&player_animations).maybe(),
                )
                    .join()
                {
                    let world_matrix = Matrix4::new_translation(&pos.0)
                        * Matrix4::from_axis_angle(&Vector3::y_axis(), rot.0);

                    // Collect part transforms from animation state
                    let part_transforms: Vec<Transform> = if let Some(anim) = player_anim {
                        // Player-specific animation
                        use crate::player::PlayerPart;
                        PlayerPart::all()
                            .iter()
                            .map(|part| anim.part_transform(*part))
                            .collect()
                    } else {
                        // No animation, use identity transforms
                        vec![Transform::default(); model_instance.model.parts.len()]
                    };

                    if let Err(e) = renderer.draw_model(
                        draw_cb,
                        &model_instance.model,
                        &world_matrix,
                        &part_transforms,
                        &material_manager,
                        &texture_manager,
                    ) {
                        log::error!("RenderSystem: Failed to draw model: {}", e);
                    }
                }

                if let Err(e) = renderer.end_frame(draw_cb, present_index) {
                    log::error!("RenderSystem: Failed to end_frame: {}", e);
                }
            }
            Err(e) => {
                log::error!("RenderSystem: Failed to begin_frame: {}", e);
            }
        }
    }
}
