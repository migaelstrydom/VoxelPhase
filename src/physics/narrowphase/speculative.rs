//! Speculative contacts: contacts generated where a pair *will* meet, carried
//! back to where it is now with the gap it still has to close.
//!
//! ```text
//!   pair now ──predict──▶ pose at meeting ──dispatch──▶ manifold there
//!                                                          │
//!   solver ◀── speculative manifold (gap per point) ◀──rewind
//! ```
//!
//! The solver reads each point's gap as a closing allowance: the pair may
//! approach by the gap and is arrested only beyond it, so it stops on arrival
//! rather than wherever it happened to be when the contact was generated.

use nalgebra::Vector3;

use crate::collision::contact::ContactManifold;
use crate::collision::continuous::gjk_raycast;

use super::collider_state::ColliderState;

/// Fraction of `frame_dt` at which two colliders, each holding its velocity and
/// orientation, first touch — `None` if they do not meet within it.
///
/// Translation only, as in the CCD body sweep: over the band's short horizon a
/// body's rotation moves its surface little compared to its translation.
///
/// Pairs whose bounding spheres cannot meet are answered without the search —
/// most of a blast's pairs, which are flying apart.
pub(super) fn time_of_impact(a: &ColliderState, b: &ColliderState, frame_dt: f32) -> Option<f32> {
    let relative = (a.velocity - b.velocity) * frame_dt;
    if !bounding_spheres_meet(a, b, relative) {
        return None;
    }
    gjk_raycast(&a.view(), &b.view(), relative, Vector3::zeros()).map(|hit| hit.t)
}

/// Whether the colliders' bounding spheres touch at any point while `a` moves
/// by `relative` against `b`.
///
/// A necessary condition for the shapes themselves to touch, so a `false` is
/// safe to act on whatever the shapes are: each bounding sphere encloses its
/// shape about the collider's centre, and the shapes, like the spheres, only
/// translate. Pairs whose spheres already overlap always pass; the test only
/// rules out pairs that are clear of each other and stay clear.
fn bounding_spheres_meet(a: &ColliderState, b: &ColliderState, relative: Vector3<f32>) -> bool {
    let reach = a.shape.bounding_radius() + b.shape.bounding_radius();
    let offset = a.center - b.center;
    let travel_sq = relative.magnitude_squared();
    let t = if travel_sq > 1e-12 {
        (-offset.dot(&relative) / travel_sq).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (offset + relative * t).magnitude_squared() <= reach * reach
}

/// Turn a manifold generated at a predicted pose into speculative contacts at
/// the present one.
///
/// `travel_a` and `travel_b` are each side's displacement from now to the pose
/// the manifold was generated at; `travel_a` is `None` for static geometry,
/// which does not move. Contact normals point from A to B, as everywhere.
///
/// Each point's separation now is its separation at the predicted pose plus
/// the approach along its normal in between. That becomes the point's gap, and
/// its solver depth is zero: nothing overlaps yet, so there is nothing for
/// position correction to push out. The point moves back with the bodies so
/// its lever arms are measured from where they are — with both sides moving,
/// it splits the difference, which errs only along the direction of approach.
pub(super) fn rewind_to_now(
    manifold: &mut ContactManifold,
    travel_a: Option<Vector3<f32>>,
    travel_b: Vector3<f32>,
) {
    let relative = travel_a.unwrap_or_else(Vector3::zeros) - travel_b;
    let point_shift = match travel_a {
        Some(travel_a) => (travel_a + travel_b) * 0.5,
        None => travel_b,
    };
    for cp in &mut manifold.points {
        let separation = -cp.raw_depth + relative.dot(&cp.normal);
        cp.gap = separation.max(0.0);
        cp.raw_depth = -separation;
        cp.depth = 0.0;
        cp.point -= point_shift;
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;
    use crate::collision::contact::{ContactPoint, FeatureId};

    /// A sphere falling onto a floor, predicted 0.3 m ahead where it sinks
    /// 0.1 m in: it is 0.2 m clear now.
    #[test]
    fn gap_is_the_separation_before_the_predicted_travel() {
        let mut manifold = ContactManifold::single(ContactPoint::new(
            Point3::new(0.0, -0.1, 0.0),
            Vector3::y(),
            0.1,
            FeatureId::SINGLE,
        ));
        rewind_to_now(&mut manifold, None, Vector3::new(0.0, -0.3, 0.0));

        let cp = &manifold.points[0];
        assert!((cp.gap - 0.2).abs() < 1e-6, "gap {}", cp.gap);
        assert!(
            (cp.raw_depth + 0.2).abs() < 1e-6,
            "raw depth {}",
            cp.raw_depth
        );
        assert_eq!(cp.depth, 0.0);
        assert!((cp.point.y - 0.2).abs() < 1e-6, "point {:?}", cp.point);
    }

    /// Two bodies closing head on: each covers half the gap, and the point
    /// stays between them.
    #[test]
    fn a_pair_closing_from_both_sides_shares_the_gap() {
        let mut manifold = ContactManifold::single(ContactPoint::new(
            Point3::origin(),
            Vector3::x(),
            0.0,
            FeatureId::SINGLE,
        ));
        rewind_to_now(
            &mut manifold,
            Some(Vector3::new(0.25, 0.0, 0.0)),
            Vector3::new(-0.25, 0.0, 0.0),
        );

        let cp = &manifold.points[0];
        assert!((cp.gap - 0.5).abs() < 1e-6, "gap {}", cp.gap);
        assert!(cp.point.coords.magnitude() < 1e-6, "point {:?}", cp.point);
    }

    fn sphere_state(center: Point3<f32>, velocity: Vector3<f32>, radius: f32) -> ColliderState {
        let handle = generational_arena::Index::from_raw_parts(0, 0);
        ColliderState {
            body_handle: crate::physics::handle::RigidBodyHandle(handle),
            collider_handle: crate::physics::handle::ColliderHandle(handle),
            center,
            rotation: nalgebra::UnitQuaternion::identity(),
            shape: crate::physics::collider::ColliderShape::Sphere { radius },
            velocity,
            material: crate::physics::collider::ColliderMaterial::default(),
            is_sleeping: false,
            is_static: false,
        }
    }

    /// Two balls flying apart never meet, and are answered without a search.
    #[test]
    fn a_receding_pair_has_no_time_of_impact() {
        let a = sphere_state(
            Point3::new(-0.5, 0.0, 0.0),
            Vector3::new(-12.0, 0.0, 0.0),
            0.2,
        );
        let b = sphere_state(
            Point3::new(0.5, 0.0, 0.0),
            Vector3::new(12.0, 0.0, 0.0),
            0.2,
        );
        assert!(!bounding_spheres_meet(
            &a,
            &b,
            (a.velocity - b.velocity) / 60.0
        ));
        assert_eq!(time_of_impact(&a, &b, 1.0 / 60.0), None);
    }

    /// A pair that passes within reach of each other mid-frame is kept, even
    /// though it is further apart at both ends of the frame than at the start.
    #[test]
    fn a_pair_passing_close_mid_frame_is_kept() {
        let a = sphere_state(
            Point3::new(-0.3, 0.35, 0.0),
            Vector3::new(36.0, 0.0, 0.0),
            0.2,
        );
        let b = sphere_state(Point3::origin(), Vector3::zeros(), 0.2);
        let relative = a.velocity / 60.0;
        assert!((a.center + relative - b.center).magnitude() > 0.4);
        assert!(bounding_spheres_meet(&a, &b, relative));
        assert!(time_of_impact(&a, &b, 1.0 / 60.0).is_some());
    }

    /// Two balls closing head on meet where their surfaces touch.
    #[test]
    fn a_closing_pair_meets_where_the_surfaces_touch() {
        let a = sphere_state(
            Point3::new(-0.5, 0.0, 0.0),
            Vector3::new(12.0, 0.0, 0.0),
            0.2,
        );
        let b = sphere_state(
            Point3::new(0.5, 0.0, 0.0),
            Vector3::new(-12.0, 0.0, 0.0),
            0.2,
        );
        // A 0.6 m gap, closed at 24 m/s over a 1/30 s frame of 0.8 m.
        let t = time_of_impact(&a, &b, 1.0 / 30.0).expect("the pair meets");
        assert!((t - 0.75).abs() < 1e-3, "t = {t}");
    }

    /// A point the pair is moving away from is not a gap to allow.
    #[test]
    fn a_receding_point_has_no_gap() {
        let mut manifold = ContactManifold::single(ContactPoint::new(
            Point3::origin(),
            Vector3::y(),
            0.05,
            FeatureId::SINGLE,
        ));
        rewind_to_now(&mut manifold, None, Vector3::new(0.0, 0.02, 0.0));
        assert_eq!(manifold.points[0].gap, 0.0);
    }
}
