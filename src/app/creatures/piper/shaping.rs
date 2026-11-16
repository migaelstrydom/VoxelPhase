//! Placing, projecting and painting: what every part of the piper's
//! sculpture is put together with.

use nalgebra::{UnitQuaternion, Vector3};

use crate::app::sculpt::Clay;
use crate::model::MeshPrimitive;
use crate::rendering::vertex::Vertex;

/// A point given as three numbers, as a vector.
pub(super) fn vector(point: (f32, f32, f32)) -> Vector3<f32> {
    Vector3::new(point.0, point.1, point.2)
}

/// A vector as the three numbers clay takes a point as.
pub(super) fn triple(point: Vector3<f32>) -> (f32, f32, f32) {
    (point.x, point.y, point.z)
}

/// 0 below `low`, 1 above `high`, eased between.
pub(super) fn ease(low: f32, high: f32, x: f32) -> f32 {
    let t = ((x - low) / (high - low)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// A mesh turned by `turn` and then moved by `shift`.
pub(super) fn placed(
    mut mesh: MeshPrimitive,
    turn: UnitQuaternion<f32>,
    shift: Vector3<f32>,
) -> MeshPrimitive {
    for vertex in &mut mesh.vertices {
        vertex.pos = turn * vertex.pos + shift;
        vertex.normal = turn * vertex.normal;
    }
    mesh
}

/// The turn that sets +Y along `up` and +Z as near `forward` as that
/// allows.
pub(super) fn turned_to(up: Vector3<f32>, forward: Vector3<f32>) -> UnitQuaternion<f32> {
    let up = up.normalize();
    let forward = (forward - up * forward.dot(&up))
        .try_normalize(1e-6)
        .unwrap_or_else(|| up.cross(&Vector3::x()).normalize());
    UnitQuaternion::from_basis_unchecked(&[up.cross(&forward), up, forward])
}

/// Where a ray from `from` (inside `form`) along `along` comes out of it,
/// and the way out of the form there.
pub(super) fn surface(
    form: &Clay,
    from: Vector3<f32>,
    along: Vector3<f32>,
) -> (Vector3<f32>, Vector3<f32>) {
    let along = along.normalize();
    let (mut inside, mut outside) = (0.0, 1.5);
    for _ in 0..24 {
        let middle = 0.5 * (inside + outside);
        if form.distance(from + along * middle) < 0.0 {
            inside = middle;
        } else {
            outside = middle;
        }
    }
    let point = from + along * inside;
    (point, way_out(form, point).unwrap_or(along))
}

/// The way out of `form` at `point`, if it has one there.
pub(super) fn way_out(form: &Clay, point: Vector3<f32>) -> Option<Vector3<f32>> {
    let slope = |axis: Vector3<f32>| {
        form.distance(point + axis * 0.004) - form.distance(point - axis * 0.004)
    };
    Vector3::new(
        slope(Vector3::x()),
        slope(Vector3::y()),
        slope(Vector3::z()),
    )
    .try_normalize(1e-9)
}

/// Multiply a mesh's paint by `tint`.
pub(super) fn wash(mesh: &mut MeshPrimitive, tint: impl Fn(&Vertex) -> Vector3<f32>) {
    for vertex in &mut mesh.vertices {
        let tint = tint(vertex);
        vertex.color.x *= tint.x;
        vertex.color.y *= tint.y;
        vertex.color.z *= tint.z;
    }
}

/// Paint a mesh's vertices outright.
pub(super) fn paint(mesh: &mut MeshPrimitive, colour: impl Fn(&Vertex) -> Vector3<f32>) {
    for vertex in &mut mesh.vertices {
        let colour = colour(vertex);
        vertex.color.x = colour.x;
        vertex.color.y = colour.y;
        vertex.color.z = colour.z;
    }
}
