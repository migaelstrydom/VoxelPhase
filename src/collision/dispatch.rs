//! Centralized shape-pair dispatch for collision detection.
//!
//! Routes shape pairs to specialized collision functions (analytic/SAT fast-paths)
//! based on `ColliderShape` variants. Unknown pairs fall through to GJK/EPA.

use super::capsule::Capsule;
use super::contact::ContactManifold;
use super::discrete::capsule_capsule::capsule_capsule_manifold;
use super::discrete::gjk_epa_manifold::gjk_epa_manifold;
use super::discrete::obb_capsule::obb_capsule_manifold;
use super::discrete::obb_obb::obb_obb_manifold_cached;
use super::discrete::sphere_capsule::sphere_capsule_manifold;
use super::discrete::sphere_obb::sphere_obb_manifold;
use super::discrete::sphere_sphere::sphere_sphere_manifold;
use super::mesh::capsule_patch::capsule_patch_manifold;
use super::mesh::obb_patch::obb_patch_manifold;
use super::mesh::seam_filter::FilteredPatch;
use super::mesh::sphere_patch::sphere_patch_manifold;
use super::obb::Obb;
use super::sat::SatCache;
use super::shape_view::ShapeView;
use crate::physics::ColliderShape;

/// Generate a contact manifold between two convex shapes.
///
/// Routes to analytic/SAT fast-paths for known shape pairs. The `sat_cache`
/// parameter is only used for OBB-OBB pairs; pass `None` for all others.
pub fn generate_manifold(
    a: &ShapeView,
    b: &ShapeView,
    margin: f32,
    sat_cache: Option<&mut SatCache>,
) -> ContactManifold {
    match (a.shape, b.shape) {
        (ColliderShape::Sphere { radius: ra }, ColliderShape::Sphere { radius: rb }) => {
            sphere_sphere_manifold(a.center, *ra, b.center, *rb, margin)
        }

        (ColliderShape::Sphere { radius }, ColliderShape::Box { half_extents }) => {
            let obb = Obb::new(b.center, b.rotation, *half_extents);
            sphere_obb_manifold(&obb, a.center, *radius, margin)
        }

        (ColliderShape::Box { half_extents }, ColliderShape::Sphere { radius }) => {
            let obb = Obb::new(a.center, a.rotation, *half_extents);
            sphere_obb_manifold(&obb, b.center, *radius, margin)
        }

        (
            ColliderShape::Box {
                half_extents: he_a,
            },
            ColliderShape::Box {
                half_extents: he_b,
            },
        ) => {
            let obb_a = Obb::new(a.center, a.rotation, *he_a);
            let obb_b = Obb::new(b.center, b.rotation, *he_b);
            match sat_cache {
                Some(cache) => obb_obb_manifold_cached(&obb_a, &obb_b, margin, cache),
                None => {
                    obb_obb_manifold_cached(&obb_a, &obb_b, margin, &mut SatCache::default())
                }
            }
        }

        (
            ColliderShape::Sphere { radius },
            ColliderShape::Capsule {
                half_height,
                radius: cap_r,
            },
        ) => {
            let capsule = Capsule::new(b.center, b.rotation, *half_height, *cap_r);
            sphere_capsule_manifold(&capsule, a.center, *radius, margin)
        }

        (
            ColliderShape::Capsule {
                half_height,
                radius: cap_r,
            },
            ColliderShape::Sphere { radius },
        ) => {
            let capsule = Capsule::new(a.center, a.rotation, *half_height, *cap_r);
            sphere_capsule_manifold(&capsule, b.center, *radius, margin)
        }

        (
            ColliderShape::Box { half_extents },
            ColliderShape::Capsule {
                half_height,
                radius: cap_r,
            },
        ) => {
            let obb = Obb::new(a.center, a.rotation, *half_extents);
            let capsule = Capsule::new(b.center, b.rotation, *half_height, *cap_r);
            obb_capsule_manifold(&obb, &capsule, margin)
        }

        (
            ColliderShape::Capsule {
                half_height,
                radius: cap_r,
            },
            ColliderShape::Box { half_extents },
        ) => {
            let obb = Obb::new(b.center, b.rotation, *half_extents);
            let capsule = Capsule::new(a.center, a.rotation, *half_height, *cap_r);
            obb_capsule_manifold(&obb, &capsule, margin)
        }

        (
            ColliderShape::Capsule {
                half_height: hh_a,
                radius: r_a,
            },
            ColliderShape::Capsule {
                half_height: hh_b,
                radius: r_b,
            },
        ) => {
            let cap_a = Capsule::new(a.center, a.rotation, *hh_a, *r_a);
            let cap_b = Capsule::new(b.center, b.rotation, *hh_b, *r_b);
            capsule_capsule_manifold(&cap_a, &cap_b, margin)
        }

        // GJK/EPA fallback for any shape pair without a specialized fast-path.
        // Unreachable with current variants; activates when ConvexHull is added.
        #[allow(unreachable_patterns)]
        _ => gjk_epa_manifold(a, b, margin),
    }
}

/// Generate a contact manifold between a convex shape and a filtered mesh patch.
///
/// Routes to shape-specific mesh functions for known shapes. Unknown shapes
/// fall through to the GJK/EPA mesh path (added in Step 6.5).
pub fn generate_mesh_manifold(
    shape: &ShapeView,
    patch: &FilteredPatch,
    margin: f32,
) -> ContactManifold {
    match shape.shape {
        ColliderShape::Sphere { radius } => {
            sphere_patch_manifold(shape.center, *radius, patch, margin)
        }
        ColliderShape::Box { half_extents } => {
            let obb = Obb::new(shape.center, shape.rotation, *half_extents);
            obb_patch_manifold(&obb, patch, margin)
        }
        ColliderShape::Capsule {
            half_height,
            radius,
        } => {
            let capsule = Capsule::new(shape.center, shape.rotation, *half_height, *radius);
            let (seg_a, seg_b) = capsule.segment_endpoints();
            capsule_patch_manifold(seg_a, seg_b, *radius, patch, margin)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::contact::FeatureId;
    use nalgebra::{Point3, UnitQuaternion, Vector3};

    fn view_from(
        center: Point3<f32>,
        rotation: UnitQuaternion<f32>,
        shape: &ColliderShape,
    ) -> ShapeView<'_> {
        ShapeView {
            center,
            rotation,
            shape,
        }
    }

    #[test]
    fn dispatch_sphere_sphere() {
        let sa = ColliderShape::Sphere { radius: 1.0 };
        let sb = ColliderShape::Sphere { radius: 1.0 };
        let va = view_from(Point3::new(0.0, 0.0, 0.0), UnitQuaternion::identity(), &sa);
        let vb = view_from(Point3::new(1.5, 0.0, 0.0), UnitQuaternion::identity(), &sb);
        let margin = 0.02;

        let dispatched = generate_manifold(&va, &vb, margin, None);
        let direct = sphere_sphere_manifold(va.center, 1.0, vb.center, 1.0, margin);

        assert_eq!(dispatched.len(), direct.len());
        for (dp, dd) in dispatched.points.iter().zip(direct.points.iter()) {
            assert!((dp.point - dd.point).magnitude() < 1e-6);
            assert!((dp.normal - dd.normal).magnitude() < 1e-6);
            assert!((dp.depth - dd.depth).abs() < 1e-6);
        }
    }

    #[test]
    fn dispatch_sphere_obb() {
        let sphere = ColliderShape::Sphere { radius: 0.5 };
        let box_shape = ColliderShape::Box {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let vs = view_from(
            Point3::new(1.3, 0.0, 0.0),
            UnitQuaternion::identity(),
            &sphere,
        );
        let vb = view_from(Point3::origin(), UnitQuaternion::identity(), &box_shape);
        let margin = 0.02;

        let dispatched = generate_manifold(&vs, &vb, margin, None);
        let obb = Obb::new(Point3::origin(), UnitQuaternion::identity(), Vector3::new(1.0, 1.0, 1.0));
        let direct = sphere_obb_manifold(&obb, Point3::new(1.3, 0.0, 0.0), 0.5, margin);

        assert_eq!(dispatched.len(), direct.len());
        for (dp, dd) in dispatched.points.iter().zip(direct.points.iter()) {
            assert!((dp.point - dd.point).magnitude() < 1e-6);
            assert!((dp.depth - dd.depth).abs() < 1e-6);
        }
    }

    #[test]
    fn dispatch_obb_obb_with_sat_cache() {
        let ba = ColliderShape::Box {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let bb = ColliderShape::Box {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let rot = UnitQuaternion::identity();
        let va = view_from(Point3::origin(), rot, &ba);
        let vb = view_from(Point3::new(1.5, 0.0, 0.0), rot, &bb);
        let margin = 0.02;

        let mut cache = SatCache::default();
        let dispatched = generate_manifold(&va, &vb, margin, Some(&mut cache));

        let obb_a = Obb::new(Point3::origin(), rot, Vector3::new(1.0, 1.0, 1.0));
        let obb_b = Obb::new(Point3::new(1.5, 0.0, 0.0), rot, Vector3::new(1.0, 1.0, 1.0));
        let mut cache2 = SatCache::default();
        let direct = obb_obb_manifold_cached(&obb_a, &obb_b, margin, &mut cache2);

        assert_eq!(dispatched.len(), direct.len());
        for (dp, dd) in dispatched.points.iter().zip(direct.points.iter()) {
            assert!((dp.depth - dd.depth).abs() < 1e-6);
            assert_eq!(dp.feature_id, dd.feature_id);
        }
    }

    #[test]
    fn dispatch_symmetric_sphere_box() {
        let sphere = ColliderShape::Sphere { radius: 0.5 };
        let box_shape = ColliderShape::Box {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let rot = UnitQuaternion::identity();
        let margin = 0.02;

        let vs = view_from(Point3::new(1.3, 0.0, 0.0), rot, &sphere);
        let vb = view_from(Point3::origin(), rot, &box_shape);

        let ab = generate_manifold(&vs, &vb, margin, None);
        let ba = generate_manifold(&vb, &vs, margin, None);

        assert_eq!(ab.len(), ba.len());
        if !ab.is_empty() {
            assert!((ab.points[0].depth - ba.points[0].depth).abs() < 1e-6);
            // Normals should be consistent (pointing from A's reference to B's).
            // For sphere-OBB the normal direction depends on which is the OBB.
            // Both routes delegate to sphere_obb_manifold with the same arguments.
            assert!(
                (ab.points[0].normal - ba.points[0].normal).magnitude() < 1e-6,
                "Normals should match: {:?} vs {:?}",
                ab.points[0].normal,
                ba.points[0].normal,
            );
        }
    }

    #[test]
    fn dispatch_mesh_sphere() {
        use crate::collision::mesh::seam_filter::{ContactFace, FilteredPatch};
        use smallvec::smallvec;

        let shape = ColliderShape::Sphere { radius: 0.5 };
        let view = view_from(
            Point3::new(0.0, 0.5, 0.0),
            UnitQuaternion::identity(),
            &shape,
        );
        let margin = 0.02;

        let patch = FilteredPatch {
            faces: smallvec![ContactFace {
                vertices: smallvec![
                    Point3::new(-5.0, 0.0, -5.0),
                    Point3::new(5.0, 0.0, -5.0),
                    Point3::new(5.0, 0.0, 5.0),
                    Point3::new(-5.0, 0.0, 5.0),
                ],
                normal: Vector3::y(),
                feature_id: FeatureId::SINGLE,
            }],
            boundary_edges: smallvec![],
        };

        let dispatched = generate_mesh_manifold(&view, &patch, margin);
        let direct = sphere_patch_manifold(view.center, 0.5, &patch, margin);

        assert_eq!(dispatched.len(), direct.len());
        for (dp, dd) in dispatched.points.iter().zip(direct.points.iter()) {
            assert!((dp.point - dd.point).magnitude() < 1e-6);
            assert!((dp.depth - dd.depth).abs() < 1e-6);
        }
    }

    #[test]
    fn dispatch_mesh_obb() {
        use crate::collision::mesh::seam_filter::{ContactFace, FilteredPatch};
        use smallvec::smallvec;

        let shape = ColliderShape::Box {
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let rot = UnitQuaternion::identity();
        let center = Point3::new(0.0, 0.4, 0.0);
        let view = view_from(center, rot, &shape);
        let margin = 0.02;

        let patch = FilteredPatch {
            faces: smallvec![ContactFace {
                vertices: smallvec![
                    Point3::new(-5.0, 0.0, -5.0),
                    Point3::new(5.0, 0.0, -5.0),
                    Point3::new(5.0, 0.0, 5.0),
                    Point3::new(-5.0, 0.0, 5.0),
                ],
                normal: Vector3::y(),
                feature_id: FeatureId::SINGLE,
            }],
            boundary_edges: smallvec![],
        };

        let dispatched = generate_mesh_manifold(&view, &patch, margin);
        let obb = Obb::new(center, rot, Vector3::new(0.5, 0.5, 0.5));
        let direct = obb_patch_manifold(&obb, &patch, margin);

        assert_eq!(dispatched.len(), direct.len());
        for (dp, dd) in dispatched.points.iter().zip(direct.points.iter()) {
            assert!((dp.depth - dd.depth).abs() < 1e-6);
        }
    }

    #[test]
    fn dispatch_obb_obb_uses_sat_not_gjk() {
        let ba = ColliderShape::Box {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let bb = ColliderShape::Box {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let rot = UnitQuaternion::identity();
        let va = view_from(Point3::origin(), rot, &ba);
        let vb = view_from(Point3::new(1.5, 0.0, 0.0), rot, &bb);
        let margin = 0.02;

        let mut cache = SatCache::default();
        let dispatched = generate_manifold(&va, &vb, margin, Some(&mut cache));

        let obb_a = Obb::new(Point3::origin(), rot, Vector3::new(1.0, 1.0, 1.0));
        let obb_b = Obb::new(Point3::new(1.5, 0.0, 0.0), rot, Vector3::new(1.0, 1.0, 1.0));
        let mut cache2 = SatCache::default();
        let direct = obb_obb_manifold_cached(&obb_a, &obb_b, margin, &mut cache2);

        // Dispatch must route to the SAT fast-path, producing identical FeatureIds.
        assert_eq!(dispatched.len(), direct.len());
        for (dp, dd) in dispatched.points.iter().zip(direct.points.iter()) {
            assert_eq!(
                dp.feature_id, dd.feature_id,
                "FeatureId mismatch — dispatch may have used GJK/EPA instead of SAT"
            );
        }
    }
}
