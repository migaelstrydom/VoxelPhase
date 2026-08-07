//! Stand-in level content, shared by the scenes that judge a whole frame.
//!
//! Saturated primaries at high value on bright terrain — the palette the game
//! actually uses, and the one that misbehaves. Kept in one place so that a
//! scene varying lighting and a scene varying grade are looking at identical
//! geometry and identical albedos, and any difference between their sheets is
//! the thing under test rather than the props.

use nalgebra::{Matrix4, Vector2, Vector3, Vector4};

use crate::geometry::{
    generate_cube_indices, generate_cube_vertices, generate_sphere_indices,
    generate_sphere_vertices,
};
use crate::rendering::colour::Colour;
use crate::rendering::material::SurfaceParams;
use crate::rendering::vertex::Vertex;
use crate::rendering::visual_bench::scene::SceneMesh;
use crate::terrain::VoxelMaterial;

const SEGMENTS: u32 = 40;
const RINGS: u32 = 28;

/// Half-width of the ground plane. Large enough to reach the horizon at every
/// camera that uses it, so the frame is dominated by the brightest albedo.
const GROUND_HALF_EXTENT: f32 = 60.0;

/// Terrain green, taken from the voxel material rather than copied.
///
/// A hand-written stand-in drifted lighter than the real thing once already,
/// which defeats the purpose of a scene whose whole job is to be honest about
/// what the game's albedos do under the lighting.
fn grass() -> Colour {
    let [r, g, b, a] = VoxelMaterial::Grass.color();
    Colour::new(r, g, b, a)
}

/// The full arrangement: ground plus a spread of the palette's worst offenders.
pub fn arrangement() -> Vec<SceneMesh> {
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

/// A large quad of terrain green for everything to stand on.
///
/// Wound so the upward face is the front face; the reverse is invisible from
/// above and easy to mistake for a floor, because the sky's below-horizon
/// colour sits behind it.
pub fn ground() -> SceneMesh {
    let e = GROUND_HALF_EXTENT;
    let colour = grass().to_vec4();
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
