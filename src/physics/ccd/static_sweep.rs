//! Sweeping a moving collider against the static geometry of the world.
//!
//! Static geometry is a triangle soup queried by region, not a body, so there
//! is no partner to sweep against — the candidate's own displacement is the
//! relative motion. The result is still a [`SweptImpact`], so the caller cannot
//! tell a terrain hit from a body hit.

use nalgebra::{Point3, Vector3};

use crate::collision::continuous::{gjk_raycast, swept_sphere_triangle, SweptContact};
use crate::collision::shape_view::ShapeView;
use crate::collision::AABB;
use crate::physics::collider::ColliderShape;
use crate::physics::pipeline::pair::PairHeader;
use crate::physics::static_geometry::StaticGeometry;

use super::candidate::CcdCandidate;
use super::patch_cache::SweptPatchCache;
use super::swept_impact::SweptImpact;

/// Sweep `candidate` against static geometry and report its earliest impact.
///
/// Convex shapes are swept by GJK raycast, which follows the actual silhouette
/// and gives a tighter time of impact. Spheres skip straight to the analytic
/// sweep, and any other shape falls back to it when the raycast finds nothing —
/// the bounding sphere is a conservative envelope, so a miss there is a real
/// miss while a hit is at worst early.
pub(super) fn sweep_against_static(
    candidate: &CcdCandidate,
    static_geometry: &dyn StaticGeometry,
    patch_cache: &mut SweptPatchCache,
) -> Option<SweptImpact> {
    let hit = match &candidate.shape {
        ColliderShape::Sphere { .. } => {
            sweep_sphere_against_static(candidate, static_geometry, patch_cache)
        }
        _ => sweep_shape_against_static(candidate, static_geometry, patch_cache)
            .or_else(|| sweep_sphere_against_static(candidate, static_geometry, patch_cache)),
    }?;

    let rotation = candidate.rotation_at(hit.t);
    Some(SweptImpact {
        header: PairHeader {
            body_a: None,
            body_b: candidate.body_handle,
            collider_a: None,
            // Left empty deliberately: it is what keeps this transient contact
            // out of the manifold cache. See `cold_solver_contact`.
            collider_b: None,
            restitution: candidate.material.restitution,
            friction: candidate.material.friction(),
        },
        toi: hit.t,
        point: hit.point,
        normal: hit.normal,
        clamped: candidate.body_handle,
        clamped_rotation: rotation,
    })
}

/// Sweep a convex shape against static geometry using GJK raycast.
fn sweep_shape_against_static(
    candidate: &CcdCandidate,
    static_geometry: &dyn StaticGeometry,
    patch_cache: &mut SweptPatchCache,
) -> Option<SweptContact> {
    let displacement = candidate.post_center - candidate.pre_center;
    let triangles = swept_triangles(candidate, &displacement, static_geometry, patch_cache);

    let shape_view = ShapeView {
        center: candidate.pre_center,
        rotation: candidate.pre_rot,
        shape: &candidate.shape,
    };

    let mut earliest: Option<SweptContact> = None;
    for triangle in triangles {
        let Some(hit) = gjk_raycast(&shape_view, triangle, displacement, Vector3::zeros()) else {
            continue;
        };
        if !is_tunnelling_hit(&displacement, &hit.normal, candidate.min_approach) {
            continue;
        }
        if earliest.as_ref().is_none_or(|e| hit.t < e.t) {
            earliest = Some(SweptContact::new(hit.t, hit.point, hit.normal));
        }
    }
    earliest
}

/// Sweep the candidate's bounding sphere against static geometry.
///
/// Also serves as the fallback when a shape sweep finds nothing to build a
/// manifold from.
pub(super) fn sweep_sphere_against_static(
    candidate: &CcdCandidate,
    static_geometry: &dyn StaticGeometry,
    patch_cache: &mut SweptPatchCache,
) -> Option<SweptContact> {
    let start = candidate.pre_center;
    let end = candidate.post_center;
    let displacement = end - start;
    let triangles = swept_triangles(candidate, &displacement, static_geometry, patch_cache);

    let mut earliest: Option<SweptContact> = None;
    for triangle in triangles {
        let Some(contact) = swept_sphere_triangle(start, end, candidate.radius, triangle) else {
            continue;
        };
        if !is_tunnelling_hit(&displacement, &contact.normal, candidate.min_approach) {
            continue;
        }
        if earliest.as_ref().is_none_or(|e| contact.t < e.t) {
            earliest = Some(contact);
        }
    }
    earliest
}

/// Triangles within reach of the candidate's swept bounding sphere.
fn swept_triangles<'a>(
    candidate: &CcdCandidate,
    displacement: &Vector3<f32>,
    static_geometry: &dyn StaticGeometry,
    patch_cache: &'a mut SweptPatchCache,
) -> &'a [crate::collision::Triangle] {
    let start = candidate.pre_center;
    let end = candidate.post_center;
    let radius = candidate.radius;
    let query = AABB::new(
        Point3::new(
            start.x.min(end.x) - radius,
            start.y.min(end.y) - radius,
            start.z.min(end.z) - radius,
        ),
        Point3::new(
            start.x.max(end.x) + radius,
            start.y.max(end.y) + radius,
            start.z.max(end.z) + radius,
        ),
    );
    patch_cache.triangles(
        candidate.collider_handle,
        &query,
        displacement,
        static_geometry,
    )
}

/// Whether a swept hit is a genuine tunnelling threat rather than a graze.
///
/// A body travelling along a surface it is already touching reports a hit at
/// `t≈0` with a normal it is barely moving into. Clamping to such a hit would
/// teleport the body back to its substep-start position and re-solve friction
/// the solver already owns. Only hits the body drives into far enough to pass
/// through during this substep count — the same travel window that activates
/// CCD in the first place, measured along the hit normal.
///
/// `displacement` is *relative* motion: for terrain that is the body's own,
/// but a body pair closing on each other tunnels at their closing speed, not
/// at either one's.
pub(super) fn is_tunnelling_hit(
    displacement: &Vector3<f32>,
    normal: &Vector3<f32>,
    min_approach: f32,
) -> bool {
    -displacement.dot(normal) > min_approach
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn motion_into_a_surface_is_tunnelling_past_the_margin() {
        let normal = Vector3::new(0.0, 1.0, 0.0);
        assert!(is_tunnelling_hit(
            &Vector3::new(0.0, -0.5, 0.0),
            &normal,
            0.02
        ));
        assert!(!is_tunnelling_hit(
            &Vector3::new(0.0, -0.01, 0.0),
            &normal,
            0.02
        ));
    }

    /// The grazing case: fast travel *along* a surface is not an impact on it.
    #[test]
    fn motion_along_a_surface_is_not_tunnelling() {
        assert!(!is_tunnelling_hit(
            &Vector3::new(20.0, 0.0, 0.0),
            &Vector3::new(0.0, 1.0, 0.0),
            0.02
        ));
    }

    #[test]
    fn motion_away_from_a_surface_is_not_tunnelling() {
        assert!(!is_tunnelling_hit(
            &Vector3::new(0.0, 5.0, 0.0),
            &Vector3::new(0.0, 1.0, 0.0),
            0.02
        ));
    }
}
