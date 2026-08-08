//! A composed tableau, for judging the look as a whole.
//!
//! The counterpart to the sweeps: a handful of objects with different finishes
//! sitting on a ground plane under one lighting setup, viewed from a few angles.
//! Sweeps answer "what does this parameter do"; this answers "does the frame
//! look right".

use nalgebra::{Matrix4, Point3, Vector3};

use crate::core::error::EngineResult;
use crate::geometry::{
    generate_cube_indices, generate_cube_vertices, generate_sphere_indices,
    generate_sphere_vertices,
};
use crate::lighting::{ActiveLight, LightId};
use crate::rendering::colour::Colour;
use crate::rendering::material::SurfaceParams;
use crate::rendering::vertex::Vertex;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};

const SEGMENTS: u32 = 40;
const RINGS: u32 = 28;

/// Half-width of the ground plane.
const GROUND_HALF_EXTENT: f32 = 14.0;

pub struct Props;

impl VisualScene for Props {
    fn name(&self) -> &str {
        "props"
    }

    fn description(&self) -> &str {
        "Mixed finishes on a ground plane, from three angles. The look-as-a-whole shot."
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let environment = SceneEnvironment::default()
            .with_sun(Vector3::new(-0.42, 0.68, 0.6))
            .with_point_light(ActiveLight {
                id: LightId(0),
                position: Vector3::new(2.4, 1.6, 2.0),
                range: 12.0,
                colour: Colour::new(0.4, 0.7, 1.0, 1.0),
                intensity: 6.0,
            });

        // Three angles round the same arrangement: a wide establishing view, a
        // low one that puts the horizon behind the props, and a near view that
        // fills the frame with surface.
        let cameras = [
            (
                "wide",
                SceneCamera::looking_at(Point3::new(0.0, 4.2, 9.5), Point3::new(0.0, 0.7, 0.0)),
            ),
            (
                "low",
                SceneCamera::looking_at(Point3::new(-6.0, 1.1, 6.5), Point3::new(0.0, 0.9, 0.0)),
            ),
            (
                "close",
                SceneCamera::looking_at(Point3::new(1.6, 1.8, 4.0), Point3::new(0.2, 0.9, 0.0))
                    .with_fov(38.0),
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

/// The objects, identical in every shot so the angles are comparable.
fn arrangement() -> Vec<SceneMesh> {
    let sphere_indices = generate_sphere_indices(SEGMENTS, RINGS);

    let polished = SurfaceParams {
        emissive: [0.0, 0.0, 0.0, 0.0],
        surface: [0.15, 0.0, 0.0, 3.0],
    };
    let matte = SurfaceParams::MATTE;
    let metal = SurfaceParams {
        emissive: [0.0, 0.0, 0.0, 0.0],
        surface: [0.25, 1.0, 0.0, 3.0],
    };
    // Bright enough to cross the default bloom threshold of 1.3.
    let glowing = SurfaceParams {
        emissive: [0.2, 0.9, 1.0, 3.0],
        surface: [0.2, 0.0, 0.8, 2.5],
    };

    vec![
        ground(),
        SceneMesh::new(
            generate_sphere_vertices(0.9, SEGMENTS, RINGS, Colour::new(0.75, 0.2, 0.18, 1.0)),
            sphere_indices.clone(),
        )
        .at(Vector3::new(-1.9, 0.9, 0.0))
        .with_surface(polished),
        SceneMesh::new(
            generate_sphere_vertices(0.9, SEGMENTS, RINGS, Colour::new(0.85, 0.83, 0.78, 1.0)),
            sphere_indices.clone(),
        )
        .at(Vector3::new(0.0, 0.9, 0.0))
        .with_surface(metal),
        SceneMesh::new(
            generate_cube_vertices(
                Vector3::new(0.75, 0.75, 0.75),
                Colour::new(0.35, 0.45, 0.3, 1.0),
            ),
            generate_cube_indices(),
        )
        .at(Vector3::new(2.0, 0.75, -0.4))
        .with_surface(matte),
        SceneMesh::new(
            generate_sphere_vertices(0.45, SEGMENTS, RINGS, Colour::new(0.2, 0.9, 1.0, 1.0)),
            sphere_indices,
        )
        .at(Vector3::new(1.0, 0.45, 1.9))
        .with_surface(glowing),
    ]
}

/// A large matte quad for everything to stand on.
///
/// Two triangles rather than a subdivided grid: with no shadows and no
/// per-vertex lighting there is nothing for extra vertices to carry, and a flat
/// plane makes the specular response of the props easier to read against.
fn ground() -> SceneMesh {
    let e = GROUND_HALF_EXTENT;
    let colour = Colour::new(0.32, 0.34, 0.3, 1.0).to_vec4();
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
            pos: Vector3::new(position.x, position.y, position.z),
            color: colour,
            tex_coords: nalgebra::Vector2::new(uv[0], uv[1]),
            normal,
            ao: 1.0,
        })
        .collect();

    SceneMesh::new(vertices, vec![0, 2, 1, 0, 3, 2])
        .with_transform(Matrix4::identity())
        .with_surface(SurfaceParams::MATTE)
}
