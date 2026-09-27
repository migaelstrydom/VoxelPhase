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
//! The *other pass* is judged on `smoke_behind` and `smoke_in_front`.
//! Particles are blended surfaces too, and they are sorted in with the ice
//! rather than drawn after it, so a burst behind the block must show *through*
//! it, tinted, and a burst in front of it must cover it. A burst that vanishes
//! entirely means the particles are being depth-tested against the glass from a
//! later pass; a burst that is visible but untinted in `smoke_behind` means
//! they are being drawn after it in the same pass.
//!
//! The *water* is judged on `in_water`, `smoke_over_water` and
//! `smoke_in_water`. Water is drawn in the scene pass between what lies beyond
//! its surface and what lies this side of it, so the far water must show
//! through the cube's dry half, the burst above the pool must cover the water,
//! and the sunken cube and burst must be seen through it, tinted.
//!
//! The cubes are drawn with the game's own mesh and the game's own substance,
//! so what this sheet shows is what the ice cube in a level looks like.

use nalgebra::{Matrix4, Point3, Vector2, Vector3};

use crate::app::spawnables::{ice_block_mesh, ice_texture_spread};
use crate::core::error::EngineResult;
use crate::geometry::{generate_sphere_indices, generate_sphere_vertices};
use crate::particles::{Particle, ParticleConfig, ParticleEffectType, ParticlePool};
use crate::rendering::colour::Colour;
use crate::rendering::pattern;
use crate::rendering::pattern::Spread;
use crate::rendering::substance;
use crate::rendering::vertex::Vertex;
use crate::rendering::visual_bench::pool::ScenePool;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};
use crate::resources::textures::TextureHandle;

/// Half-extent of every cube on the sheet.
const CUBE_HALF: f32 = 0.5;

/// Half-width of the ground plane.
const GROUND_HALF_EXTENT: f32 = 9.0;

const TEXTURE_SIZE: u32 = 256;

/// Height of the water surface in the `in_water` shot. Low enough that a cube
/// standing on the ground crosses it, which is the case that matters: the
/// surface then has to be in front of the cube's lower half and behind its
/// upper half within one frame.
const WATER_LEVEL: f32 = 0.3;

/// The floor the pool rests on. Below the ground the cubes stand on, so that
/// the water has a depth to shade by.
const WATER_FLOOR: f32 = -0.8;

/// A pool deep enough to sink a cube and a burst in.
const DEEP_WATER_LEVEL: f32 = 1.6;

const SPHERE_SEGMENTS: u32 = 32;
const SPHERE_RINGS: u32 = 22;

/// Puffs in a burst, how far the ring of them reaches, and how big each one is.
///
/// Small enough not to merge into one blown-out mass: the ring has to stay
/// legible as separate puffs for the colours along it to be judged, and for a
/// mis-sort among them to be visible at all.
const BURST_PUFFS: usize = 9;
const BURST_RADIUS: f32 = 0.6;
const BURST_PUFF_SIZE: f32 = 0.13;

/// Oldest the burst's puffs are sampled at, as a fraction of their life. The
/// ring runs from birth to here, so one shot carries the whole hot half of the
/// ramp: the white core the youngest puffs should be, and the orange the
/// oldest have cooled to.
const BURST_MAX_AGE: f32 = 0.35;

pub struct Ice;

impl VisualScene for Ice {
    fn name(&self) -> &str {
        "ice"
    }

    fn description(&self) -> &str {
        "Ice cubes: the transmissive material, and the back-to-front sort behind it."
    }

    fn shots(&self, ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let size = cube_spread().texture_size(TEXTURE_SIZE);
        let texture = ctx.textures.create_from_rgba(
            size,
            size,
            &pattern::ICE.bake_spread(TEXTURE_SIZE, &substance::ICE.palette, 7, cube_spread()),
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
            // Standing in water. The cube crosses the surface, so it is drawn
            // in two halves: the submerged half before the water, which tints
            // it, and the dry half after, over the water. Two failures are
            // unmistakable: the surface painting over the cube's dry half, and
            // the far water vanishing where it is seen through that half.
            SceneShot::new(
                "in_water",
                SceneCamera::looking_at(
                    Point3::new(1.5, 1.75, 2.6),
                    Point3::new(0.0, CUBE_HALF * 0.6, -0.4),
                )
                .with_fov(42.0),
            )
            .with_environment(environment.clone())
            .with_water(ScenePool::new(WATER_LEVEL, WATER_FLOOR, 7.0))
            .with_meshes([
                ground(),
                cube(Vector3::new(0.0, CUBE_HALF, 0.0), 24.0),
                cube(Vector3::new(-1.15, CUBE_HALF, -1.5), -10.0),
            ]),
            // A burst behind the ice. The explosion this whole apparatus is
            // for: the smoke must be visible through the block and tinted by
            // it, exactly as the opaque marker is two shots above.
            SceneShot::new(
                "smoke_behind",
                SceneCamera::looking_at(
                    Point3::new(0.0, 1.0, 2.6),
                    Point3::new(0.0, CUBE_HALF, 0.0),
                )
                .with_fov(38.0),
            )
            .with_environment(environment.clone())
            .with_particles(burst(Vector3::new(0.0, CUBE_HALF, -1.3)))
            .with_meshes([ground(), cube(Vector3::new(0.0, CUBE_HALF, 0.0), 18.0)]),
            // The same burst in front of the ice, which is the half of the
            // ordering the first shot cannot show: here the smoke covers the
            // block rather than being tinted by it.
            SceneShot::new(
                "smoke_in_front",
                SceneCamera::looking_at(
                    Point3::new(0.0, 1.0, 2.6),
                    Point3::new(0.0, CUBE_HALF, 0.0),
                )
                .with_fov(38.0),
            )
            .with_environment(environment.clone())
            .with_particles(burst(Vector3::new(0.0, CUBE_HALF, 1.2)))
            .with_meshes([ground(), cube(Vector3::new(0.0, CUBE_HALF, 0.0), 18.0)]),
            // A burst over the water, with the pool running on behind it.
            // The water is drawn in the scene pass before anything blended on
            // this side of its surface, so the smoke must cover the water;
            // water painted over the puffs means it is being drawn last again.
            SceneShot::new(
                "smoke_over_water",
                SceneCamera::looking_at(
                    Point3::new(0.0, 1.4, 2.6),
                    Point3::new(0.0, WATER_LEVEL, -1.5),
                )
                .with_fov(42.0),
            )
            .with_environment(environment.clone())
            .with_water(ScenePool::new(WATER_LEVEL, WATER_FLOOR, 7.0))
            .with_particles(burst(Vector3::new(0.0, 1.0, 0.6)))
            .with_meshes([ground(), cube(Vector3::new(-1.15, CUBE_HALF, -1.5), -10.0)]),
            // The same burst and a cube under deep water. Both lie beyond the
            // surface, so they are drawn before it and seen through it: tinted,
            // and shifted by the refraction, never pasted over the top.
            SceneShot::new(
                "smoke_in_water",
                SceneCamera::looking_at(Point3::new(0.0, 2.8, 2.6), Point3::new(0.0, 0.6, -0.5))
                    .with_fov(42.0),
            )
            .with_environment(environment.clone())
            .with_water(ScenePool::new(DEEP_WATER_LEVEL, WATER_FLOOR, 7.0))
            .with_particles(burst(Vector3::new(0.8, 0.8, -0.8)))
            .with_meshes([ground(), cube(Vector3::new(-0.6, CUBE_HALF, 0.0), 24.0)]),
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

/// The spread the game would give a block this size, so the sheet judges the
/// texture a real cube wears rather than a tile chosen for the bench.
fn cube_spread() -> Spread {
    ice_texture_spread(Vector3::repeat(CUBE_HALF))
}

/// One ice cube, drawn with the game's mesh and the game's substance.
fn ice(texture: &TextureHandle, position: Vector3<f32>, yaw_degrees: f32) -> SceneMesh {
    let (vertices, indices) = ice_block_mesh(Vector3::repeat(CUBE_HALF), cube_spread());
    let transform = Matrix4::new_translation(&position)
        * Matrix4::from_axis_angle(&Vector3::y_axis(), yaw_degrees.to_radians());

    SceneMesh::new(vertices, indices)
        .with_transform(transform)
        .with_texture(texture.clone())
        .with_surface(substance::ICE.material(texture.clone()).surface_params())
}

/// A ring of fireball puffs centred on `centre`, placed rather than simulated.
///
/// Every value is fixed — the ring's angles, the ages the ramp is sampled at —
/// because a shot that is compared against its own past cannot contain a random
/// burst. The spec is the game's own, so the puffs are the colour and the
/// silhouette an explosion actually draws.
fn burst(centre: Vector3<f32>) -> ParticlePool {
    let spec = ParticleConfig::new()
        .spec(ParticleEffectType::Fireball)
        .clone();

    let mut pool = ParticlePool::new(BURST_PUFFS);
    for index in 0..BURST_PUFFS {
        let angle = index as f32 / BURST_PUFFS as f32 * std::f32::consts::TAU;
        // The ring is tipped out of the camera plane so the puffs sit at a
        // spread of depths, which is what makes them sort against each other.
        let offset = Vector3::new(
            angle.cos() * BURST_RADIUS,
            angle.sin() * BURST_RADIUS * 0.7,
            (angle * 2.0).sin() * BURST_RADIUS * 0.5,
        );

        let age = index as f32 / (BURST_PUFFS - 1) as f32 * BURST_MAX_AGE;

        let mut puff = Particle::new(
            centre + offset,
            Vector3::zeros(),
            BURST_PUFF_SIZE,
            1.0,
            spec.ramp.clone(),
        );
        puff.life = 1.0 - age;
        puff.colour = spec.ramp.sample(age);
        puff.additive = spec.additive;
        puff.billow = spec.billow;
        puff.seed = index as f32 * 0.37;
        pool.spawn(puff);
    }

    pool
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
