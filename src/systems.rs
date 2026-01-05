use crate::components::{Rotation, SpinSpeed};
use specs::{Join, ReadExpect, ReadStorage, System, WriteStorage};

// New imports for RenderSystem
use crate::components::{CameraComponent, Mesh, Position, Renderable};
use crate::rendering::renderer::Renderer;
use crate::resources::textures::TextureManager;
use nalgebra::{Matrix4, Vector3};
use specs::WriteExpect;

pub struct SpinningSystem;

impl<'a> System<'a> for SpinningSystem {
    type SystemData = (WriteStorage<'a, Rotation>, ReadStorage<'a, SpinSpeed>);

    fn run(&mut self, (mut rotations, speeds): Self::SystemData) {
        for (rotation, speed) in (&mut rotations, &speeds).join() {
            rotation.0 += speed.0; // Simple increment, assumes fixed delta time for now
        }
    }
}

pub struct RenderSystem;

impl<'a> System<'a> for RenderSystem {
    type SystemData = (
        WriteExpect<'a, Renderer>,
        ReadExpect<'a, TextureManager>,
        ReadStorage<'a, Mesh>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, Renderable>,
        ReadStorage<'a, CameraComponent>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            mut renderer,
            texture_manager,
            meshes,
            positions,
            rotations,
            renderables,
            camera_components,
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
                for (mesh, pos, rot, _renderable) in
                    (&meshes, &positions, &rotations, &renderables).join()
                {
                    let model_matrix = Matrix4::new_translation(&pos.0)
                        * Matrix4::from_axis_angle(&Vector3::y_axis(), rot.0);

                    if let Err(e) = renderer.draw_mesh(
                        draw_cb,
                        &mesh.vertices[..],
                        &mesh.indices[..],
                        &model_matrix,
                        &view_matrix,
                        &proj_matrix,
                        &mesh.texture_handles,
                        &texture_manager,
                    ) {
                        log::error!("RenderSystem: Failed to draw mesh data: {}", e);
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
