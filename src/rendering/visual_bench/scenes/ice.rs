//! Ice: the material, and the ordering apparatus that lets it be drawn.
//!
//! Two things to look at, and they fail differently.
//!
//! The *material* is judged on the single-cube shots: the block should read as
//! a solid volume you can see into, with edges that brighten and a highlight on
//! the bevels. The failure to watch for is a flat grey pane — that is what
//! transparency looks like when the Fresnel term is not reaching the shader.
//!
//! The *ordering* is judged on `three_deep` and `overlap`. Blended surfaces
//! composite in the order they are recorded, so a broken sort shows as a near
//! cube vanishing behind a far one, or as a cube's own back face painting over
//! its front. Both are unmistakable once you know to look; neither is visible
//! in a shot with only one cube in it, which is why those shots exist.
//!
//! The cubes are drawn with the game's own mesh and the game's own substance,
//! so what this sheet shows is what the ice cube in a level looks like.

use nalgebra::{Matrix4, Point3, Vector2, Vector3};

use crate::app::spawnables::ice_cube_mesh;
use crate::core::error::EngineResult;
use crate::geometry::{generate_sphere_indices, generate_sphere_vertices};
use crate::rendering::colour::Colour;
use crate::rendering::pattern;
use crate::rendering::substance;
use crate::rendering::vertex::Vertex;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};
use crate::resources::textures::TextureHandle;

/// Half-extent of every cube on the sheet.
const CUBE_HALF: f32 = 0.5;

/// Half-width of the ground plane.
const GROUND_HALF_EXTENT: f32 = 9.0;

const TEXTURE_SIZE: u32 = 256;

const SPHERE_SEGMENTS: u32 = 32;
const SPHERE_RINGS: u32 = 22;

pub struct Ice;

impl VisualScene for Ice {
    fn name(&self) -> &str {
        "ice"
    }

    fn description(&self) -> &str {
        "Ice cubes: the transmissive material, and the back-to-front sort behind it."
    }

    fn shots(&self, ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let texture = ctx.textures.create_from_rgba(
            TEXTURE_SIZE,
            TEXTURE_SIZE,
            &pattern::ICE.bake(TEXTURE_SIZE, &substance::ICE.palette, 7),
            true,
        )?;

        // Sun across the frame rather than behind the camera: a transmissive
        // surface shows almost nothing under a light that is directly at the
        // viewer's back, because the Fresnel gain and the highlight both live
        // away from head-on.
        let environment = SceneEnvironment::default().with_sun(Vector3::new(-0.55, 0.55, 0.63));

        let cube = |position: Vector3<f32>, yaw: f32| ice(&texture, position, yaw);

        let shots = vec![
            // One cube, close. The material shot: is this a volume or a pane?
            SceneShot::new(
                "single",
                SceneCamera::looking_at(
                    Point3::new(1.1, 1.0, 2.1),
                    Point3::new(0.0, CUBE_HALF, 0.0),
                )
                .with_fov(36.0),
            )
            .with_environment(environment.clone())
            .with_meshes([ground(), cube(Vector3::new(0.0, CUBE_HALF, 0.0), 22.0)]),
            // The same cube with something opaque behind it. If the blend is
            // working, the marker is visible through the ice and tinted by it.
            SceneShot::new(
                "against_a_marker",
                SceneCamera::looking_at(
                    Point3::new(0.0, 1.0, 2.6),
                    Point3::new(0.0, CUBE_HALF, 0.0),
                )
                .with_fov(38.0),
            )
            .with_environment(environment.clone())
            .with_meshes([
                ground(),
                marker(
                    Vector3::new(0.0, CUBE_HALF, -1.1),
                    Colour::new(0.85, 0.3, 0.2, 1.0),
                ),
                cube(Vector3::new(0.0, CUBE_HALF, 0.0), 18.0),
            ]),
            // Three cubes in a line away from the camera, each behind the last.
            // The sort test: all three must be visible, each one tinting the
            // ones behind it. A reversed sort erases the far two.
            SceneShot::new(
                "three_deep",
                SceneCamera::looking_at(
                    Point3::new(0.55, 1.05, 3.0),
                    Point3::new(0.0, CUBE_HALF, -1.2),
                )
                .with_fov(40.0),
            )
            .with_environment(environment.clone())
            .with_meshes([
                ground(),
                marker(
                    Vector3::new(0.0, CUBE_HALF, -3.4),
                    Colour::new(0.85, 0.3, 0.2, 1.0),
                ),
                // Deliberately pushed in the wrong order: nearest first, so the
                // shot fails unless the queue actually reorders them.
                cube(Vector3::new(-0.95, CUBE_HALF, 0.2), 0.0),
                cube(Vector3::new(0.0, CUBE_HALF, -1.2), 20.0),
                cube(Vector3::new(0.95, CUBE_HALF, -2.6), 40.0),
            ]),
            // Two cubes side by side and overlapping in screen space, at
            // different depths. Where they cross, the near one must be the one
            // on top.
            SceneShot::new(
                "overlap",
                SceneCamera::looking_at(
                    Point3::new(0.0, 1.1, 2.9),
                    Point3::new(0.0, CUBE_HALF, -0.6),
                )
                .with_fov(42.0),
            )
            .with_environment(environment.clone())
            .with_meshes([
                ground(),
                cube(Vector3::new(-0.35, CUBE_HALF, -1.4), 15.0),
                cube(Vector3::new(0.32, CUBE_HALF, -0.2), -15.0),
            ]),
            // Backlit. The extreme case for the Fresnel gain: with the sun
            // behind the block, the edges should go bright and the middle
            // should stay clear.
            SceneShot::new(
                "backlit",
                SceneCamera::looking_at(
                    Point3::new(0.0, 0.85, 2.0),
                    Point3::new(0.0, CUBE_HALF, 0.0),
                )
                .with_fov(36.0),
            )
            .with_environment(SceneEnvironment::default().with_sun(Vector3::new(0.05, 0.35, -0.93)))
            .with_meshes([ground(), cube(Vector3::new(0.0, CUBE_HALF, 0.0), 30.0)]),
        ];

        Ok(shots)
    }
}

/// One ice cube, drawn with the game's mesh and the game's substance.
fn ice(texture: &TextureHandle, position: Vector3<f32>, yaw_degrees: f32) -> SceneMesh {
    let (vertices, indices) = ice_cube_mesh(CUBE_HALF);
    let transform = Matrix4::new_translation(&position)
        * Matrix4::from_axis_angle(&Vector3::y_axis(), yaw_degrees.to_radians());

    SceneMesh::new(vertices, indices)
        .with_transform(transform)
        .with_texture(texture.clone())
        .with_surface(substance::ICE.material(texture.clone()).surface_params())
}

/// An opaque sphere, for putting behind the ice so that there is something to
/// see through it.
fn marker(position: Vector3<f32>, colour: Colour) -> SceneMesh {
    SceneMesh::new(
        generate_sphere_vertices(0.34, SPHERE_SEGMENTS, SPHERE_RINGS, colour),
        generate_sphere_indices(SPHERE_SEGMENTS, SPHERE_RINGS),
    )
    .at(position)
}

/// A matte quad at `y = 0`, so the cubes have somewhere to sit and something
/// to cast a shadow on.
fn ground() -> SceneMesh {
    let e = GROUND_HALF_EXTENT;
    let colour = Colour::new(0.34, 0.36, 0.34, 1.0).to_vec4();
    let normal = Vector3::y();

    let corners = [
        (Vector3::new(-e, 0.0, -e), [0.0, 0.0]),
        (Vector3::new(e, 0.0, -e), [1.0, 0.0]),
        (Vector3::new(e, 0.0, e), [1.0, 1.0]),
        (Vector3::new(-e, 0.0, e), [0.0, 1.0]),
    ];

    let vertices: Vec<Vertex> = corners
        .iter()
        .map(|(position, uv)| Vertex {
            pos: *position,
            color: colour,
            tex_coords: Vector2::new(uv[0], uv[1]),
            normal,
            ao: 1.0,
        })
        .collect();

    SceneMesh::new(vertices, vec![0, 1, 2, 0, 2, 3])
}
