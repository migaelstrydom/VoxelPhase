//! The sun shadow tuning sheet.
//!
//! Two things are being judged here, and they pull against each other:
//!
//! - **Contact.** An object's shadow must start where the object touches the
//!   ground. Too much depth bias detaches it and the object floats again,
//!   which is exactly the failure shadows were added to fix.
//! - **Acne.** A lit surface must not stripe itself. Too little bias and the
//!   map's depth quantisation makes a surface shadow itself in bands.
//!
//! Both are worst at a low sun, where a shadow texel covers a long stretch of
//! ground, so the sweep runs the sun down towards the horizon rather than
//! holding it at a flattering elevation. The last shot drops the shadow to
//! nothing as the before-and-after: it is the frame this whole feature exists
//! to improve on.

use nalgebra::{Matrix4, Point3, Vector3, Vector4};

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

/// Half-width of the ground plane. Comfortably larger than the shadow volume,
/// so the edge fade is visible rather than falling off the end of the world.
const GROUND_HALF_EXTENT: f32 = 40.0;

/// Sun elevations to sweep, in degrees above the horizon.
const ELEVATIONS: [f32; 5] = [60.0, 40.0, 25.0, 14.0, 7.0];

pub struct Shadows;

impl VisualScene for Shadows {
    fn name(&self) -> &str {
        "shadows"
    }

    fn description(&self) -> &str {
        "Sun elevation sweep over a shadow-casting arrangement. The contact-vs-acne sheet."
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        // Close enough to read a contact edge, high enough to see the shadows
        // lie across the ground rather than hide behind their casters.
        let camera =
            SceneCamera::looking_at(Point3::new(-5.5, 4.0, 8.5), Point3::new(0.0, 0.6, 0.0));

        let mut shots: Vec<SceneShot> = ELEVATIONS
            .iter()
            .map(|&degrees| {
                SceneShot::new(format!("{:.0}deg", degrees), camera)
                    .with_environment(SceneEnvironment::default().with_sun(sun_at(degrees)))
                    .with_meshes(arrangement())
            })
            .collect();

        shots.push(
            SceneShot::new("no shadows", camera)
                .with_environment(
                    SceneEnvironment::default()
                        .with_sun(sun_at(ELEVATIONS[2]))
                        .without_shadows(),
                )
                .with_meshes(arrangement()),
        );

        Ok(shots)
    }
}

/// Direction from a surface towards a sun at the given elevation, kept on a
/// fixed azimuth so shadows fall the same way across the whole sheet.
fn sun_at(elevation_degrees: f32) -> Vector3<f32> {
    let elevation = elevation_degrees.to_radians();
    let azimuth = (-35.0f32).to_radians();
    Vector3::new(
        elevation.cos() * azimuth.sin(),
        elevation.sin(),
        elevation.cos() * azimuth.cos(),
    )
}

/// Shapes chosen for what their shadows reveal rather than for their surfaces.
///
/// The sphere resting on the ground is the contact test; the box on stilts is
/// the detachment test (its shadow is far from its geometry, so peter-panning
/// shows as a gap); the leaning slab is the acne test, since a surface nearly
/// edge-on to the light is where slope bias earns its keep.
fn arrangement() -> Vec<SceneMesh> {
    let sphere_indices = generate_sphere_indices(SEGMENTS, RINGS);
    let matte = SurfaceParams::MATTE;

    vec![
        ground(),
        SceneMesh::new(
            generate_sphere_vertices(0.9, SEGMENTS, RINGS, Colour::new(0.75, 0.25, 0.2, 1.0)),
            sphere_indices,
        )
        .at(Vector3::new(-2.2, 0.9, 0.4))
        .with_surface(matte),
        SceneMesh::new(
            generate_cube_vertices(
                Vector3::new(0.9, 0.12, 0.9),
                Colour::new(0.4, 0.5, 0.35, 1.0),
            ),
            generate_cube_indices(),
        )
        .at(Vector3::new(0.6, 2.1, -0.6))
        .with_surface(matte),
        // The legs that hold the slab up, so the gap under it is a real
        // occlusion rather than a floating plate.
        leg(Vector3::new(-0.2, 1.0, -1.4)),
        leg(Vector3::new(1.4, 1.0, 0.2)),
        SceneMesh::new(
            generate_cube_vertices(
                Vector3::new(1.4, 0.08, 0.7),
                Colour::new(0.55, 0.5, 0.42, 1.0),
            ),
            generate_cube_indices(),
        )
        .with_transform(
            Matrix4::new_translation(&Vector3::new(3.4, 0.75, 1.2))
                * Matrix4::from_axis_angle(&Vector3::z_axis(), -0.9),
        )
        .with_surface(matte),
    ]
}

fn leg(position: Vector3<f32>) -> SceneMesh {
    SceneMesh::new(
        generate_cube_vertices(
            Vector3::new(0.1, 1.0, 0.1),
            Colour::new(0.3, 0.3, 0.32, 1.0),
        ),
        generate_cube_indices(),
    )
    .at(position)
    .with_surface(SurfaceParams::MATTE)
}

/// A large matte quad for everything to stand on and cast onto.
fn ground() -> SceneMesh {
    let e = GROUND_HALF_EXTENT;
    let colour = Colour::new(0.42, 0.44, 0.4, 1.0).to_vec4();
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
            tex_coords: nalgebra::Vector2::new(uv[0], uv[1]),
            normal,
        })
        .collect();

    SceneMesh::new(vertices, vec![0, 2, 1, 0, 3, 2]).with_surface(SurfaceParams::MATTE)
}
