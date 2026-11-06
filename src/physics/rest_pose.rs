//! The face a body can be set down on, and the least turn that puts it there.
//!
//! ```text
//!      authored pose              resting pose
//!          /\                       ____
//!         /  \     least turn      /    \
//!         \  /   ─────────────▶    \____/
//!          \/                      ══════  floor
//!     balanced on a vertex       on the face nearest below
//! ```
//!
//! A face is one a body can rest on when its centre of mass lies over it: set
//! down on that face, gravity holds it there rather than tipping it off. Of
//! those faces, the one whose outward normal already points nearest to `down`
//! is the least turn away, so the body keeps as much of its authored pose as
//! it can — a crate authored square stays exactly as it was, yaw included.
//!
//! Only a body made of one box or one convex hull has faces to rest on. A
//! sphere rests however it is turned, and a capsule or a compound body is left
//! as authored. A box's faces are read from the box itself rather than from a
//! hull built over it: a box may be far thinner than a hull is allowed to be,
//! as a pane of glass is.

use nalgebra::{Isometry3, Point3, UnitQuaternion, UnitVector3, Vector3};

use crate::collision::ConvexHull;

use super::collider::ColliderShape;
use super::handle::RigidBodyHandle;
use super::world::PhysicsWorld;

/// How far, as a cosine, a face may be from `down` and still count as already
/// resting: about a degree.
const ALREADY_DOWN_COS: f32 = 0.9998;

/// How far outside a face's edges, in metres, the centre of mass may project
/// and the face still count as one it rests on. Regular solids put it exactly
/// at the centre; this only absorbs rounding.
const OVER_FACE_TOLERANCE: f32 = 1e-4;

/// The least turn, about the body's centre of mass, that sets `body` down on a
/// face it can rest on, with `down` the direction it rests toward.
///
/// `None` when the body has no faces to rest on (see the module docs) or
/// already rests on one. Apply it as `turn * rotation`.
pub fn resting_turn(
    world: &PhysicsWorld,
    body: RigidBodyHandle,
    down: UnitVector3<f32>,
) -> Option<UnitQuaternion<f32>> {
    let rigid = world.body(body)?;
    let [collider] = rigid.colliders() else {
        return None;
    };
    let collider = world.collider(*collider)?;
    let faces = match collider.shape() {
        ColliderShape::ConvexHull { hull } => hull_faces(hull),
        ColliderShape::Box { half_extents } => box_faces(half_extents),
        ColliderShape::Sphere { .. } | ColliderShape::Capsule { .. } => return None,
    };
    let placement = collider.world_transform(rigid.position(), rigid.rotation());

    let normal = resting_face_normal(&faces, &placement, rigid.position(), down)?;
    if normal.dot(&down) >= ALREADY_DOWN_COS {
        return None;
    }
    UnitQuaternion::rotation_between(&normal, &down)
}

/// A flat face of a collider, in the collider's frame.
struct Face {
    /// Outward unit normal.
    normal: Vector3<f32>,
    /// Corners, wound counter-clockwise seen from the side `normal` points to.
    corners: Vec<Vector3<f32>>,
}

/// The faces of a convex hull.
fn hull_faces(hull: &ConvexHull) -> Vec<Face> {
    hull.faces
        .iter()
        .map(|face| Face {
            normal: face.normal,
            corners: face
                .vertex_indices
                .iter()
                .map(|&i| hull.vertices[i as usize])
                .collect(),
        })
        .collect()
}

/// The six faces of a box with `half_extents`, centred on the origin.
fn box_faces(half_extents: &Vector3<f32>) -> Vec<Face> {
    // Taken in this order round the axes `u` and `v` of a face whose normal is
    // `u × v`, its corners wind counter-clockwise seen from outside.
    const QUAD: [(f32, f32); 4] = [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)];
    let along = |axis: usize, sign: f32| Vector3::ith(axis, sign * half_extents[axis]);

    let mut faces = Vec::with_capacity(6);
    for axis in 0..3 {
        let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
        for sign in [1.0, -1.0] {
            let mut corners: Vec<Vector3<f32>> = QUAD
                .iter()
                .map(|&(su, sv)| along(axis, sign) + along(u, su) + along(v, sv))
                .collect();
            if sign < 0.0 {
                corners.reverse();
            }
            faces.push(Face {
                normal: Vector3::ith(axis, sign),
                corners,
            });
        }
    }
    faces
}

/// The outward world normal of the face, among those `centre_of_mass` lies
/// over, that points nearest to `down`.
fn resting_face_normal(
    faces: &[Face],
    placement: &Isometry3<f32>,
    centre_of_mass: Point3<f32>,
    down: UnitVector3<f32>,
) -> Option<Vector3<f32>> {
    faces
        .iter()
        .filter_map(|face| {
            let normal = placement.rotation * face.normal;
            let polygon: Vec<Point3<f32>> = face
                .corners
                .iter()
                .map(|&corner| placement * Point3::from(corner))
                .collect();
            lies_over(centre_of_mass, &polygon, &normal).then_some(normal)
        })
        .max_by(|a, b| a.dot(&down).total_cmp(&b.dot(&down)))
}

/// Whether `point`, projected along `normal`, falls inside the convex
/// `polygon`, wound counter-clockwise seen from the side `normal` points to.
fn lies_over(point: Point3<f32>, polygon: &[Point3<f32>], normal: &Vector3<f32>) -> bool {
    polygon.iter().enumerate().all(|(i, a)| {
        let b = polygon[(i + 1) % polygon.len()];
        let edge = b - a;
        // Inward, in the face's plane.
        let inward = normal.cross(&edge);
        let length = inward.magnitude();
        length < 1e-9 || inward.dot(&(point - a)) / length >= -OVER_FACE_TOLERANCE
    })
}

#[cfg(test)]
mod tests {
    use std::f32::consts::FRAC_PI_6;
    use std::sync::Arc;

    use super::*;
    use crate::collision::convex_hull::dodecahedron_hull;
    use crate::physics::{ColliderDesc, PhysicsConfig, RigidBodyDesc};

    fn down() -> UnitVector3<f32> {
        -Vector3::y_axis()
    }

    fn body_with(
        world: &mut PhysicsWorld,
        collider: ColliderDesc,
        rotation: UnitQuaternion<f32>,
    ) -> RigidBodyHandle {
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(0.0, 2.0, 0.0))
                .rotation(rotation),
        );
        world.attach_collider(body, collider);
        body
    }

    /// The face the body would sit on after the turn points straight down.
    fn lowest_face_after(world: &PhysicsWorld, body: RigidBodyHandle, hull: &ConvexHull) -> f32 {
        let turn = resting_turn(world, body, down()).unwrap_or_else(UnitQuaternion::identity);
        let rotation = turn * world.body(body).unwrap().rotation();
        hull.faces
            .iter()
            .map(|f| (rotation * f.normal).dot(&down()))
            .fold(f32::MIN, f32::max)
    }

    #[test]
    fn a_square_box_needs_no_turn_and_keeps_its_yaw() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let yaw = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.6);
        let body = body_with(
            &mut world,
            ColliderDesc::box_shape(Vector3::new(0.5, 0.3, 0.8)),
            yaw,
        );
        assert!(resting_turn(&world, body, down()).is_none());
    }

    /// A box tipped 30° about x turns back by 30° about x, onto its base.
    #[test]
    fn a_tipped_box_turns_back_onto_its_base() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let tip = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), FRAC_PI_6);
        let body = body_with(
            &mut world,
            ColliderDesc::box_shape(Vector3::new(0.5, 0.5, 0.5)),
            tip,
        );
        let turn = resting_turn(&world, body, down()).expect("tipped");
        assert!(
            (turn.angle() - FRAC_PI_6).abs() < 1e-3,
            "turned {}",
            turn.angle()
        );
    }

    /// An octahedron is built point-down; set down, it lies on a face.
    #[test]
    fn a_vertex_down_octahedron_turns_onto_a_face() {
        let s = 0.75;
        let vertices = vec![
            Vector3::new(s, 0.0, 0.0),
            Vector3::new(-s, 0.0, 0.0),
            Vector3::new(0.0, s, 0.0),
            Vector3::new(0.0, -s, 0.0),
            Vector3::new(0.0, 0.0, s),
            Vector3::new(0.0, 0.0, -s),
        ];
        let hull = Arc::new(octahedron(vertices));
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let body = body_with(
            &mut world,
            ColliderDesc::convex_hull(hull.clone()),
            UnitQuaternion::identity(),
        );

        let turn = resting_turn(&world, body, down()).expect("point-down");
        // Vertex to face centre on an octahedron: acos(1/√3).
        assert!(
            (turn.angle() - 0.9553).abs() < 1e-3,
            "turned {}",
            turn.angle()
        );
        assert!(lowest_face_after(&world, body, &hull) > 0.9999);
    }

    #[test]
    fn a_dodecahedron_comes_to_rest_on_a_face() {
        let hull = Arc::new(dodecahedron_hull(0.5));
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let body = body_with(
            &mut world,
            ColliderDesc::convex_hull(hull.clone()),
            UnitQuaternion::from_euler_angles(0.3, 0.2, 0.1),
        );
        assert!(lowest_face_after(&world, body, &hull) > 0.9999);
    }

    /// A box far thinner than any hull may be (a pane of glass lying flat)
    /// still has faces to rest on.
    #[test]
    fn a_thin_box_lying_flat_needs_no_turn() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let yaw = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 1.2);
        let body = body_with(
            &mut world,
            ColliderDesc::box_shape(Vector3::new(1.2, 0.0075, 0.9)),
            yaw,
        );
        assert!(resting_turn(&world, body, down()).is_none());
    }

    /// The same pane tipped less than halfway to its edge turns back to lie
    /// flat; tipped nearer its edge, it would stand on that.
    #[test]
    fn a_tipped_thin_box_turns_back_to_lie_flat() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let tip = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.5);
        let body = body_with(
            &mut world,
            ColliderDesc::box_shape(Vector3::new(1.2, 0.0075, 0.9)),
            tip,
        );
        let turn = resting_turn(&world, body, down()).expect("tipped");
        let rotation = turn * tip;
        assert!((rotation * Vector3::y()).y.abs() > 0.9999);
    }

    #[test]
    fn a_sphere_has_no_face_to_turn_to() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let body = body_with(
            &mut world,
            ColliderDesc::sphere(0.5),
            UnitQuaternion::from_euler_angles(0.3, 0.2, 0.1),
        );
        assert!(resting_turn(&world, body, down()).is_none());
    }

    fn octahedron(vertices: Vec<Vector3<f32>>) -> ConvexHull {
        use crate::collision::convex_hull::HullFace;
        let mut faces = Vec::new();
        for &x in &[0u16, 1] {
            for &y in &[2u16, 3] {
                for &z in &[4u16, 5] {
                    let (a, b, c) = (
                        vertices[x as usize],
                        vertices[y as usize],
                        vertices[z as usize],
                    );
                    let normal = (a + b + c).normalize();
                    let ccw = (b - a).cross(&(c - a)).dot(&normal) > 0.0;
                    let indices = if ccw { [x, y, z] } else { [x, z, y] };
                    faces.push(HullFace {
                        vertex_indices: indices.into_iter().collect(),
                        normal,
                    });
                }
            }
        }
        ConvexHull::new(vertices, faces)
    }
}
