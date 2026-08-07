//! The game's own palette, for judging exposure and saturation.
//!
//! The other scenes are built from muted, mid-value albedos, which is a
//! comfortable place for a renderer to sit and a misleading one to tune in: a
//! lighting change can multiply scene radiance several times over without any
//! of them looking wrong. The level content is not like that. It is saturated
//! primaries at high value — grass green, primary blue, sign yellow — and those
//! are what clip, and what go electric when a bright saturated colour is
//! tonemapped with hue preservation.
//!
//! So this scene deliberately picks the worst-behaved colours in the game and
//! stands them on the brightest surface in the game. If exposure and saturation
//! read correctly here they will read correctly on the muted scenes; the
//! reverse does not hold.

use nalgebra::{Matrix4, Point3, Vector2, Vector3, Vector4};

use crate::core::error::EngineResult;
use crate::geometry::{
    generate_cube_indices, generate_cube_vertices, generate_sphere_indices,
    generate_sphere_vertices,
};
use crate::rendering::colour::Colour;
use crate::rendering::material::SurfaceParams;
use crate::rendering::vertex::Vertex;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};

const SEGMENTS: u32 = 40;
const RINGS: u32 = 28;

/// Half-width of the ground plane. Large enough to reach the horizon at every
/// camera here, so the frame is dominated by the brightest albedo in the scene.
const GROUND_HALF_EXTENT: f32 = 60.0;

/// Terrain green, at the saturation and value the level content actually uses.
const GRASS: Colour = Colour::new(0.42, 0.82, 0.30, 1.0);

pub struct Palette;

impl VisualScene for Palette {
    fn name(&self) -> &str {
        "palette"
    }

    fn description(&self) -> &str {
        "Saturated level colours on bright terrain. The exposure and clipping check."
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let environment = SceneEnvironment::default().with_sun(Vector3::new(-0.42, 0.68, 0.6));

        // An eye-level view matching how the game is actually seen, and a
        // close one where a single saturated surface fills most of the frame —
        // the case where oversaturation is most obvious and least escapable.
        let cameras = [
            (
                "eye level",
                SceneCamera::looking_at(Point3::new(0.0, 2.6, 11.0), Point3::new(0.0, 1.2, 0.0)),
            ),
            (
                "close",
                SceneCamera::looking_at(Point3::new(-2.2, 1.4, 4.4), Point3::new(0.0, 1.0, 0.0))
                    .with_fov(40.0),
            ),
        ];

        Ok(cameras
            .into_iter()
            .map(|(label, camera)| {
                SceneShot::new(label, camera)
                    .with_environment(environment.clone())
                    .with_meshes(arrangement())
            })
            .collect())
    }
}

fn arrangement() -> Vec<SceneMesh> {
    let sphere_indices = generate_sphere_indices(SEGMENTS, RINGS);
    let matte = SurfaceParams::MATTE;
    let glossy = SurfaceParams {
        emissive: [0.0, 0.0, 0.0, 0.0],
        surface: [0.2, 0.0, 0.0, 3.0],
    };

    let block = |half: f32, colour: Colour, position: Vector3<f32>, surface: SurfaceParams| {
        SceneMesh::new(
            generate_cube_vertices(Vector3::new(half, half, half), colour),
            generate_cube_indices(),
        )
        .at(position)
        .with_surface(surface)
    };

    vec![
        ground(),
        // Near-white stone, the other thing that clips: a pale surface has no
        // headroom left once the sun is on it.
        block(
            1.6,
            Colour::new(0.88, 0.89, 0.9, 1.0),
            Vector3::new(-4.2, 1.6, -3.0),
            matte,
        ),
        block(
            0.8,
            Colour::new(1.0, 0.78, 0.08, 1.0),
            Vector3::new(1.8, 0.8, -1.0),
            matte,
        ),
        block(
            0.8,
            Colour::new(0.85, 0.15, 0.12, 1.0),
            Vector3::new(3.6, 0.8, 0.6),
            glossy,
        ),
        SceneMesh::new(
            generate_sphere_vertices(1.0, SEGMENTS, RINGS, Colour::new(0.18, 0.34, 0.92, 1.0)),
            sphere_indices,
        )
        .at(Vector3::new(-0.6, 1.0, 1.4))
        .with_surface(glossy),
    ]
}

fn ground() -> SceneMesh {
    let e = GROUND_HALF_EXTENT;
    let colour = GRASS.to_vec4();
    let normal = Vector3::new(0.0, 1.0, 0.0);

    let corners = [
        (Vector3::new(-e, 0.0, -e), [0.0, 0.0]),
        (Vector3::new(e, 0.0, -e), [1.0, 0.0]),
        (Vector3::new(e, 0.0, e), [1.0, 1.0]),
        (Vector3::new(-e, 0.0, e), [0.0, 1.0]),
    ];

    let vertices: Vec<Vertex> = corners
        .iter()
        .map(|(position, uv)| Vertex {
            pos: Vector4::new(position.x, position.y, position.z, 1.0),
            color: colour,
            tex_coords: Vector2::new(uv[0], uv[1]),
            normal,
        })
        .collect();

    SceneMesh::new(vertices, vec![0, 2, 1, 0, 3, 2])
        .with_transform(Matrix4::identity())
        .with_surface(SurfaceParams::MATTE)
}
