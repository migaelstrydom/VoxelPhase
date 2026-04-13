//! Standalone debug overlay rendering functions.
//!
//! Draws debug shapes (spheres, lines, triangles) using the renderer's
//! procedural mesh pipeline. Decoupled from the ECS so both `RenderSystem`
//! and standalone tools like the bench viewer can use it.
//!
//! Shapes are stored as lightweight [`DebugShape`] descriptors on
//! [`DebugOverlays`]. Tessellation into renderable vertices is deferred
//! to these render functions, keeping the data model allocation-free.

use nalgebra::{Matrix4, Vector2, Vector4};

use crate::core::error::EngineResult;
use crate::debug::{DebugOverlays, DebugShape};
use crate::geometry::{generate_cylinder, generate_sphere_indices, generate_sphere_vertices};
use crate::rendering::material::MaterialManager;
use crate::rendering::renderer::Renderer;
use crate::rendering::vertex::Vertex;
use crate::resources::textures::TextureManager;

/// Render opaque debug overlay shapes into the current frame.
///
/// Call during the opaque render pass, after all scene geometry.
pub fn render_debug_overlays_opaque(
    renderer: &mut Renderer,
    cb: ash::vk::CommandBuffer,
    overlays: &DebugOverlays,
    material_manager: &MaterialManager,
    texture_manager: &TextureManager,
) -> EngineResult<()> {
    for shape in overlays.opaque_shapes() {
        let (vertices, indices, transform) = tessellate(shape);
        if !vertices.is_empty() {
            renderer.draw_procedural_mesh(
                cb,
                &vertices,
                &indices,
                &transform,
                material_manager,
                texture_manager,
            )?;
        }
    }
    Ok(())
}

/// Render transparent debug overlay shapes into the current frame.
///
/// Call during the transparent render pass (after water/fire, before particles).
pub fn render_debug_overlays_transparent(
    renderer: &mut Renderer,
    cb: ash::vk::CommandBuffer,
    overlays: &DebugOverlays,
    material_manager: &MaterialManager,
    texture_manager: &TextureManager,
) -> EngineResult<()> {
    for shape in overlays.transparent_shapes() {
        let (vertices, indices, transform) = tessellate(shape);
        if !vertices.is_empty() {
            renderer.draw_procedural_mesh_transparent(
                cb,
                &vertices,
                &indices,
                &transform,
                material_manager,
                texture_manager,
            )?;
        }
    }
    Ok(())
}

/// Tessellate a debug shape into renderable vertices, indices, and a transform.
fn tessellate(shape: &DebugShape) -> (Vec<Vertex>, Vec<u32>, Matrix4<f32>) {
    match shape {
        DebugShape::Sphere {
            center,
            radius,
            colour,
        } => {
            let segments = 6;
            let rings = 6;
            let vertices = generate_sphere_vertices(*radius, segments, rings, *colour);
            let indices = generate_sphere_indices(segments, rings);
            let transform = Matrix4::new_translation(&center.coords);
            (vertices, indices, transform)
        }
        DebugShape::Line {
            start,
            end,
            radius,
            colour,
        } => {
            let (vertices, indices) = generate_cylinder(*start, *end, *radius, 4, *colour);
            (vertices, indices, Matrix4::identity())
        }
        DebugShape::Triangle { vertices, colour } => {
            let colour_vec = colour.to_vec4();
            let v0 = vertices[0];
            let v1 = vertices[1];
            let v2 = vertices[2];
            let edge1 = v1 - v0;
            let edge2 = v2 - v0;
            let normal = edge1.cross(&edge2).normalize();

            let tri_verts = vec![
                Vertex {
                    pos: Vector4::new(v0.x, v0.y, v0.z, 1.0),
                    color: colour_vec,
                    tex_coords: Vector2::new(0.0, 0.0),
                    normal,
                },
                Vertex {
                    pos: Vector4::new(v1.x, v1.y, v1.z, 1.0),
                    color: colour_vec,
                    tex_coords: Vector2::new(1.0, 0.0),
                    normal,
                },
                Vertex {
                    pos: Vector4::new(v2.x, v2.y, v2.z, 1.0),
                    color: colour_vec,
                    tex_coords: Vector2::new(0.0, 1.0),
                    normal,
                },
            ];
            (tri_verts, vec![0, 1, 2], Matrix4::identity())
        }
    }
}
