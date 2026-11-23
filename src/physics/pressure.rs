//! Where a loading over a body's envelope centres: the point a blast's
//! shove acts at.
//!
//! ```text
//!   EnvelopePart ── area it shows the loading × its solidity ──▶ its share
//!        │  sampled across its bounds (two-point Gauss)
//!        ▼
//!   each sample weighed by how hard the loading is there
//!        │  summed over the parts
//!        ▼
//!   centre of pressure
//! ```
//!
//! The near side of a body is struck harder than the far, so a blast at one
//! end of a plank spins it, and one at a standing body's feet sweeps them.
//! A part shielded by another still draws the point to itself.

use nalgebra::{Isometry3, Point3, Vector3};
use smallvec::SmallVec;

use super::bulk::EnvelopePart;
use super::collider::ColliderShape;
use super::impulses::Loading;

/// Where across its bounds a part is sampled, in units of its half extent:
/// the two-point Gauss rule, which weighs a loading that changes linearly
/// across the part exactly.
const GAUSS_POINT: f32 = 0.577_350_3;

/// Where `loading` centres on a body posed at `pose` with its envelope
/// `parts`. Each part is weighed by the area it shows the loading and its
/// solidity, and sampled across its bounds, each sample by how hard it is
/// loaded there. `None` where it loads none of them.
pub fn centre_of_pressure<'a>(
    parts: impl IntoIterator<Item = EnvelopePart<'a>>,
    pose: &Isometry3<f32>,
    loading: impl Fn(Point3<f32>) -> Option<Loading>,
) -> Option<Point3<f32>> {
    let mut weighed = Vector3::zeros();
    let mut weight = 0.0;
    for part in parts {
        let placed = pose * part.offset;
        let Some(at_centre) = loading(placed * Point3::origin()) else {
            continue;
        };
        let area = part.solidity
            * part
                .shape
                .projected_area(placed.rotation.inverse() * at_centre.travel);
        let samples = pressure_samples(part.shape);
        let share = area / samples.len() as f32;
        for sample in samples {
            let at = placed * Point3::from(sample);
            let Some(there) = loading(at) else {
                continue;
            };
            let w = share * there.per_area.norm();
            weighed += at.coords * w;
            weight += w;
        }
    }
    (weight > 0.0).then(|| Point3::from(weighed / weight))
}

/// Where `shape` is sampled for the pressure on it, in its own frame: a
/// ball at its centre, since a blast's pressure on it all points through
/// its centre; a capsule along its axis; a box or hull at the Gauss points
/// of its bounds.
fn pressure_samples(shape: &ColliderShape) -> SmallVec<[Vector3<f32>; 8]> {
    let corners = |centre: Vector3<f32>, half: Vector3<f32>| {
        let half = half * GAUSS_POINT;
        (0..8)
            .map(|corner| {
                let sign = |bit: u32| if corner & (1 << bit) == 0 { -1.0 } else { 1.0 };
                centre + Vector3::new(sign(0) * half.x, sign(1) * half.y, sign(2) * half.z)
            })
            .collect()
    };
    match shape {
        ColliderShape::Sphere { .. } => SmallVec::from_slice(&[Vector3::zeros()]),
        ColliderShape::Capsule { half_height, .. } => {
            let along = Vector3::y() * (*half_height * GAUSS_POINT);
            SmallVec::from_slice(&[along, -along])
        }
        ColliderShape::Box { half_extents } => corners(Vector3::zeros(), *half_extents),
        ColliderShape::ConvexHull { hull } => corners(hull.box_center, hull.box_half_extents),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::{
        ColliderDesc, PhysicsImpulse, PhysicsWorld, RigidBodyDesc, RigidBodyHandle,
    };

    /// A body at `at` wearing `collider`, shoved by `blast` as a step would.
    fn blasted(
        collider: ColliderDesc,
        at: Point3<f32>,
        blast: &PhysicsImpulse,
    ) -> (PhysicsWorld, RigidBodyHandle) {
        let mut world = PhysicsWorld::default();
        let body = world.create_body(RigidBodyDesc::dynamic().position(at));
        world.attach_collider(body, collider);
        assert!(world.apply_blast(body, blast));
        (world, body)
    }

    /// A blast's pressure on a ball all points through its centre: struck
    /// off to one side, it flies without spinning.
    #[test]
    fn a_blast_does_not_spin_a_ball() {
        let blast = PhysicsImpulse::radial(Point3::new(-1.0, -0.8, 0.3), 4.0, 2000.0);
        let (world, ball) = blasted(ColliderDesc::sphere(0.4), Point3::origin(), &blast);
        let ball = world.body(ball).unwrap();
        assert!(ball.linear_velocity().norm() > 1.0);
        assert!(
            ball.angular_velocity().norm() < 1e-4,
            "{}",
            ball.angular_velocity()
        );
    }

    /// A plank lying across a blast's path is struck harder at the end
    /// nearer the blast, which is thrown the faster.
    #[test]
    fn a_blast_spins_a_plank_by_its_nearer_end() {
        let blast = PhysicsImpulse::radial(Point3::new(1.5, 0.0, -1.0), 4.0, 2000.0);
        let (world, plank) = blasted(
            ColliderDesc::box_shape(Vector3::new(1.5, 0.05, 0.2)),
            Point3::origin(),
            &blast,
        );
        let plank = world.body(plank).unwrap();
        let near = plank.velocity_at(Point3::new(1.5, 0.0, 0.0)).z;
        let far = plank.velocity_at(Point3::new(-1.5, 0.0, 0.0)).z;
        assert!(far > 0.0, "far end {far}");
        assert!(near > 1.5 * far, "near end {near}, far end {far}");
    }

    /// A blast on the ground beside a standing body sweeps its feet out
    /// from under it: they are thrown faster than its head.
    #[test]
    fn a_blast_at_the_feet_sweeps_them() {
        let blast = PhysicsImpulse::radial(Point3::new(-1.5, 0.0, 0.0), 5.0, 2000.0);
        let (world, body) = blasted(
            ColliderDesc::capsule(0.9, 0.25),
            Point3::new(0.0, 0.9, 0.0),
            &blast,
        );
        let body = world.body(body).unwrap();
        let feet = body.velocity_at(Point3::new(0.0, 0.1, 0.0)).x;
        let head = body.velocity_at(Point3::new(0.0, 1.7, 0.0)).x;
        assert!(head > 0.0, "head {head}");
        assert!(feet > head, "feet {feet}, head {head}");
    }
}
