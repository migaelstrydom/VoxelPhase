//! A floor for a rig to stand on, and a stand-in for the sensing that
//! finds it.
//!
//! Every rig scene needs the same two things: somewhere to put the feet,
//! and an answer to the foot probes. Both are the bench's job — everything
//! else in those scenes is the shipping code.

use nalgebra::{Vector2, Vector3};

use crate::rendering::colour::Colour;
use crate::rendering::vertex::Vertex;
use crate::rendering::visual_bench::scene::SceneMesh;
use crate::sensing::{ContactCandidate, Probe};

/// Half-width of the ground plane.
const GROUND_HALF_EXTENT: f32 = 4.0;

/// Answer each probe against the plane `y = 0`.
pub fn probe_flat_ground(probes: &[Probe]) -> Vec<ContactCandidate> {
    probes
        .iter()
        .filter_map(|probe| {
            if probe.direction.y >= -1e-4 {
                return None;
            }
            let distance = -probe.origin.y / probe.direction.y;
            (distance <= probe.length).then(|| ContactCandidate {
                tag: probe.tag,
                point: probe.origin + probe.direction * distance,
                normal: Vector3::y(),
                distance,
            })
        })
        .collect()
}

/// A matte quad at `y = 0`.
pub fn ground() -> SceneMesh {
    let e = GROUND_HALF_EXTENT;
    let colour = Colour::new(0.3, 0.33, 0.31, 1.0).to_vec4();
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
            pos: *position,
            color: colour,
            tex_coords: Vector2::new(uv[0], uv[1]),
            normal,
            ao: 1.0,
        })
        .collect();

    SceneMesh::new(vertices, vec![0, 1, 2, 0, 2, 3])
}
