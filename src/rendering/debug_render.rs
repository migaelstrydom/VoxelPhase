//! Standalone debug overlay rendering functions.
//!
//! These functions draw debug shapes (spheres, lines) using the renderer's
//! procedural mesh pipeline. They are decoupled from the ECS so they can be
//! used both by `RenderSystem` and by standalone tools like the bench viewer.

use ash::vk;

use crate::core::error::EngineResult;
use crate::debug::DebugOverlays;
use crate::geometry::{generate_cylinder, generate_sphere_indices, generate_sphere_vertices};
use crate::rendering::material::MaterialManager;
use crate::rendering::renderer::Renderer;
use crate::resources::textures::TextureManager;

use nalgebra::Matrix4;

/// Render all debug overlay shapes (spheres and lines) into the current frame.
///
/// This is the main entry point for debug shape rendering. It draws all spheres
/// as procedural UV-sphere meshes and all lines as thin cylinders.
pub fn render_debug_overlays(
    renderer: &mut Renderer,
    cb: vk::CommandBuffer,
    overlays: &DebugOverlays,
    material_manager: &MaterialManager,
    texture_manager: &TextureManager,
) -> EngineResult<()> {
    render_debug_spheres(renderer, cb, overlays, material_manager, texture_manager)?;
    render_debug_lines(renderer, cb, overlays, material_manager, texture_manager)?;
    Ok(())
}

/// Render debug overlay spheres as procedural meshes.
fn render_debug_spheres(
    renderer: &mut Renderer,
    cb: vk::CommandBuffer,
    overlays: &DebugOverlays,
    material_manager: &MaterialManager,
    texture_manager: &TextureManager,
) -> EngineResult<()> {
    if overlays.spheres().is_empty() {
        return Ok(());
    }

    let segments = 12;
    let rings = 8;
    let indices = generate_sphere_indices(segments, rings);

    for sphere in overlays.spheres() {
        let vertices = generate_sphere_vertices(sphere.radius, segments, rings, sphere.colour);
        let transform = Matrix4::new_translation(&sphere.position.coords);
        renderer.draw_procedural_mesh(
            cb,
            &vertices,
            &indices,
            &transform,
            material_manager,
            texture_manager,
        )?;
    }

    Ok(())
}

/// Render debug overlay lines as thin cylinders.
fn render_debug_lines(
    renderer: &mut Renderer,
    cb: vk::CommandBuffer,
    overlays: &DebugOverlays,
    material_manager: &MaterialManager,
    texture_manager: &TextureManager,
) -> EngineResult<()> {
    let identity = Matrix4::identity();

    for line in overlays.lines() {
        let (vertices, indices) =
            generate_cylinder(line.start, line.end, line.radius, 6, line.colour);
        if !vertices.is_empty() {
            renderer.draw_procedural_mesh(
                cb,
                &vertices,
                &indices,
                &identity,
                material_manager,
                texture_manager,
            )?;
        }
    }

    Ok(())
}
