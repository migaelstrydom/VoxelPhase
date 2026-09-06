//! Grain on props, at the strengths the library ships.
//!
//! Terrain has had detail normals for a while and they are most of why it reads
//! as rock. This sheet is the same question for everything else: does a stone
//! prop with grain read as stone, does a plank read as wood, and does the
//! difference survive at the distance a player actually sees them from.
//!
//! Each subject appears twice — with its grain and without — because the only
//! judgement that matters is the comparison. A rough surface looks plausible in
//! isolation whether or not the microstructure is doing anything.

use nalgebra::{Matrix4, Point3, Vector3};

use crate::core::error::EngineResult;
use crate::geometry::{
    generate_cube_indices, generate_cube_vertices, generate_sphere_indices,
    generate_sphere_vertices,
};
use crate::rendering::colour::Colour;
use crate::rendering::grain::GrainSpec;
use crate::rendering::material::SurfaceParams;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};

const SEGMENTS: u32 = 48;
const RINGS: u32 = 32;

/// Half-width of the ground plane.
const GROUND_HALF_EXTENT: f32 = 12.0;

/// The finish stone takes. Low enough roughness that the grain has a highlight
/// to break up — grain under a fully diffuse lobe is invisible, which is the
/// pairing `surface_character.glsl` documents for terrain.
const STONE_ROUGHNESS: f32 = 0.5;

/// The finish timber takes. Rougher than stone and glossier than chalk.
const WOOD_ROUGHNESS: f32 = 0.65;

pub struct PropGrain;

impl VisualScene for PropGrain {
    fn name(&self) -> &str {
        "prop_grain"
    }

    fn description(&self) -> &str {
        "Grain on props, each subject with it and without. Use --columns 2."
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        // A low sun. Grain is a normal perturbation, so it is only visible
        // where light grazes the surface; a sun overhead would hide the very
        // thing this sheet exists to show.
        let environment = SceneEnvironment::default().with_sun(Vector3::new(-0.75, 0.34, 0.42));

        let camera =
            SceneCamera::looking_at(Point3::new(0.0, 1.5, 4.2), Point3::new(0.0, 0.75, 0.0))
                .with_fov(40.0);

        let far = SceneCamera::looking_at(Point3::new(0.0, 3.0, 11.0), Point3::new(0.0, 0.7, 0.0))
            .with_fov(36.0);

        let subjects = [
            ("stone", GrainSpec::STONE, STONE_ROUGHNESS),
            ("dressed stone", GrainSpec::DRESSED_STONE, STONE_ROUGHNESS),
            ("concrete", GrainSpec::CONCRETE, STONE_ROUGHNESS),
        ];

        let mut shots = Vec::new();

        for (label, grain, roughness) in subjects {
            for (suffix, spec) in [("with grain", grain), ("smooth", GrainSpec::NONE)] {
                shots.push(
                    SceneShot::new(format!("{label} — {suffix}"), camera.clone())
                        .with_environment(environment.clone())
                        .with_meshes(stone_pair(spec, roughness)),
                );
            }
        }

        // Wood is addressed by UV rather than projected, so it gets its own
        // pair on a shape that has meaningful texture coordinates.
        for (suffix, spec) in [("with grain", GrainSpec::WOOD), ("smooth", GrainSpec::NONE)] {
            shots.push(
                SceneShot::new(format!("timber — {suffix}"), camera.clone())
                    .with_environment(environment.clone())
                    .with_meshes(timber(spec)),
            );
        }

        // The distance check. Grain must fade into gloss rather than sparkle,
        // which is what the mip chain on the atlas is for.
        for (suffix, spec) in [
            ("with grain", GrainSpec::STONE),
            ("smooth", GrainSpec::NONE),
        ] {
            shots.push(
                SceneShot::new(format!("at range — {suffix}"), far.clone())
                    .with_environment(environment.clone())
                    .with_meshes(stone_pair(spec, STONE_ROUGHNESS)),
            );
        }

        Ok(shots)
    }
}

/// A sphere and a block in the same stone, so the sheet shows the grain on both
/// a curved surface and a flat one — a triplanar projection can look right on
/// one and seam on the other.
fn stone_pair(grain: GrainSpec, roughness: f32) -> Vec<SceneMesh> {
    let stone = Colour::new(0.62, 0.60, 0.57, 1.0);
    let surface = SurfaceParams {
        surface: [roughness, 0.0, 0.0, 3.0],
        ..SurfaceParams::MATTE
    }
    .with_grain(grain);

    vec![
        ground(),
        SceneMesh::new(
            generate_sphere_vertices(0.75, SEGMENTS, RINGS, stone),
            generate_sphere_indices(SEGMENTS, RINGS),
        )
        .at(Vector3::new(-1.0, 0.75, 0.0))
        .with_surface(surface),
        SceneMesh::new(
            generate_cube_vertices(Vector3::new(0.7, 0.7, 0.7), stone),
            generate_cube_indices(),
        )
        .at(Vector3::new(1.0, 0.7, 0.0))
        .with_surface(surface),
    ]
}

/// A plank-proportioned block, to show fibre running along the mesh's v axis.
fn timber(grain: GrainSpec) -> Vec<SceneMesh> {
    let timber = Colour::new(0.55, 0.40, 0.26, 1.0);
    let surface = SurfaceParams {
        surface: [WOOD_ROUGHNESS, 0.0, 0.0, 3.0],
        ..SurfaceParams::MATTE
    }
    .with_uv_grain(grain);

    vec![
        ground(),
        SceneMesh::new(
            generate_cube_vertices(Vector3::new(1.5, 0.16, 0.55), timber),
            generate_cube_indices(),
        )
        .at(Vector3::new(0.0, 0.9, 0.0))
        .with_surface(surface),
        SceneMesh::new(
            generate_cube_vertices(Vector3::new(0.5, 0.5, 0.5), timber),
            generate_cube_indices(),
        )
        .at(Vector3::new(0.0, 0.5, -1.4))
        .with_surface(surface),
    ]
}

/// A plain matte ground plane, so nothing about it competes with the subjects.
fn ground() -> SceneMesh {
    SceneMesh::new(
        generate_cube_vertices(
            Vector3::new(GROUND_HALF_EXTENT, 0.5, GROUND_HALF_EXTENT),
            Colour::new(0.42, 0.44, 0.40, 1.0),
        ),
        generate_cube_indices(),
    )
    .with_transform(Matrix4::new_translation(&Vector3::new(0.0, -0.5, 0.0)))
}
