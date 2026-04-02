//! Centralized shape-pair dispatch for collision detection.
//!
//! Routes shape pairs to specialized collision functions (analytic/SAT fast-paths)
//! based on `ColliderShape` variants. Unknown pairs fall through to GJK/EPA.

use super::capsule::Capsule;
use super::contact::ContactManifold;
use super::discrete::capsule_capsule::capsule_capsule_manifold;
use super::discrete::gjk::GjkCache;
use super::discrete::gjk_epa_manifold::gjk_epa_manifold_cached;
use super::discrete::obb_capsule::obb_capsule_manifold;
use super::discrete::obb_obb::obb_obb_manifold_cached;
use super::discrete::sphere_capsule::sphere_capsule_manifold;
use super::discrete::sphere_obb::sphere_obb_manifold;
use super::discrete::sphere_sphere::sphere_sphere_manifold;
use super::mesh::capsule_patch::capsule_patch_manifold;
use super::mesh::gjk_patch::gjk_patch_manifold;
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
/// parameter is only used for OBB-OBB pairs; `gjk_cache` is only used by
/// the GJK/EPA fallback arm.
pub fn generate_manifold(
    a: &ShapeView,
    b: &ShapeView,
    margin: f32,
    sat_cache: Option<&mut SatCache>,
    gjk_cache: Option<&mut GjkCache>,
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

        (ColliderShape::Box { half_extents: he_a }, ColliderShape::Box { half_extents: he_b }) => {
            let obb_a = Obb::new(a.center, a.rotation, *he_a);
            let obb_b = Obb::new(b.center, b.rotation, *he_b);
            match sat_cache {
                Some(cache) => obb_obb_manifold_cached(&obb_a, &obb_b, margin, cache),
                None => obb_obb_manifold_cached(&obb_a, &obb_b, margin, &mut SatCache::default()),
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
        // The normal convention is a→b, so we must pass the higher-ranked shape
        // as `a` to match the solver's body_a→body_b expectation (same as the
        // analytic arms above which always put the "larger" shape first).
        _ => {
            if shape_rank(a.shape) >= shape_rank(b.shape) {
                gjk_epa_manifold_cached(a, b, margin, gjk_cache)
            } else {
                gjk_epa_manifold_cached(b, a, margin, gjk_cache)
            }
        }
    }
}

/// Shape type rank for canonical argument ordering in the GJK/EPA fallback.
///
/// Higher-ranked shapes are passed as `a` so the normal points from the "larger"
/// shape toward the "smaller", matching the analytic paths and the solver's
/// body_a → body_b convention.
fn shape_rank(shape: &ColliderShape) -> u8 {
    match shape {
        ColliderShape::ConvexHull { .. } => 3,
        ColliderShape::Box { .. } => 2,
        ColliderShape::Capsule { .. } => 1,
        ColliderShape::Sphere { .. } => 0,
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

        // GJK/EPA fallback for any shape without a specialized mesh path.
        // Unreachable with current variants; activates when ConvexHull is added.
        #[allow(unreachable_patterns)]
        _ => gjk_patch_manifold(shape, patch, margin),
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

        let dispatched = generate_manifold(&va, &vb, margin, None, None);
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

        let dispatched = generate_manifold(&vs, &vb, margin, None, None);
        let obb = Obb::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        );
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
        let dispatched = generate_manifold(&va, &vb, margin, Some(&mut cache), None);

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

        let ab = generate_manifold(&vs, &vb, margin, None, None);
        let ba = generate_manifold(&vb, &vs, margin, None, None);

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

    // --- Sweep comparison: ConvexHull cube vs equivalent OBB ---

    /// Configuration for a sweep comparison test.
    struct SweepConfig {
        probe_shape: ColliderShape,
        half_extents: Vector3<f32>,
        /// Rotation applied to the hull/OBB shape A.
        shape_a_rotation: UnitQuaternion<f32>,
        /// Rotation applied to the probe shape B.
        probe_rotation: UnitQuaternion<f32>,
        /// Unit direction to sweep the probe along.
        sweep_dir: Vector3<f32>,
        /// Start distance along sweep_dir (from origin).
        start_t: f32,
        /// End distance along sweep_dir.
        end_t: f32,
        steps: usize,
    }

    /// Sweep a probe shape along a direction, comparing hull-based GJK/EPA results
    /// against OBB-based analytic results at each step.
    fn sweep_compare(cfg: &SweepConfig) {
        use crate::collision::convex_hull::cube_hull;
        use std::sync::Arc;

        let hull_data = Arc::new(cube_hull(cfg.half_extents));
        let hull_shape = ColliderShape::ConvexHull {
            hull: hull_data.clone(),
        };
        let obb_shape = ColliderShape::Box {
            half_extents: cfg.half_extents,
        };
        let origin = Point3::origin();
        let margin = 0.02;
        let dt = (cfg.end_t - cfg.start_t) / cfg.steps as f32;

        for i in 0..=cfg.steps {
            let t = cfg.start_t + dt * i as f32;
            let probe_center = Point3::from(cfg.sweep_dir * t);

            let hull_view = view_from(origin, cfg.shape_a_rotation, &hull_shape);
            let obb_view = view_from(origin, cfg.shape_a_rotation, &obb_shape);
            let probe_view = view_from(probe_center, cfg.probe_rotation, &cfg.probe_shape);

            let hull_manifold = generate_manifold(&hull_view, &probe_view, margin, None, None);
            let obb_manifold = generate_manifold(&obb_view, &probe_view, margin, None, None);

            let hull_n = hull_manifold.len();
            let obb_n = obb_manifold.len();

            assert!(
                (hull_n as i32 - obb_n as i32).unsigned_abs() <= 1,
                "step {i}: t={t:.3} — count mismatch: hull={hull_n}, obb={obb_n}",
            );

            // Contact order can legitimately differ between implementations.
            // Match points from the smaller manifold into the larger one.
            let (src_points, dst_points, src_label, dst_label) = if hull_n <= obb_n {
                (&hull_manifold.points, &obb_manifold.points, "hull", "obb")
            } else {
                (&obb_manifold.points, &hull_manifold.points, "obb", "hull")
            };

            for (j, sp) in src_points.iter().enumerate() {
                let mut best_dot = f32::NEG_INFINITY;
                let mut best_dp = &dst_points[0];
                let mut best_depth_diff = f32::INFINITY;
                let mut best_dist_sq = f32::INFINITY;
                for dp in dst_points.iter() {
                    let d = sp.normal.dot(&dp.normal);
                    let depth_diff = (sp.depth - dp.depth).abs();
                    let dist_sq = (sp.point - dp.point).magnitude_squared();
                    let better_dot = d > best_dot + 1e-6;
                    let tied_dot = (d - best_dot).abs() <= 1e-6;
                    let better_depth = depth_diff < best_depth_diff - 1e-6;
                    let tied_depth = (depth_diff - best_depth_diff).abs() <= 1e-6;
                    let better_dist = dist_sq < best_dist_sq;
                    if better_dot || (tied_dot && (better_depth || (tied_depth && better_dist))) {
                        best_dot = d;
                        best_dp = dp;
                        best_depth_diff = depth_diff;
                        best_dist_sq = dist_sq;
                    }
                }

                assert!(
                    best_dot > 0.9,
                    "step {i} contact {j}: t={t:.3} — \
                     normal mismatch: {src_label}={:?}, best {dst_label}={:?} (dot={best_dot:.4})",
                    sp.normal,
                    best_dp.normal,
                );

                let depth_diff = (sp.depth - best_dp.depth).abs();
                assert!(
                    depth_diff < 0.1,
                    "step {i} contact {j}: t={t:.3} — \
                     depth mismatch: {src_label}={:.4}, best {dst_label}={:.4} (diff={depth_diff:.4})",
                    sp.depth,
                    best_dp.depth,
                );
            }
        }
    }

    // --- Axis-aligned sweeps (identity rotations) ---

    #[test]
    fn sweep_hull_vs_obb_sphere() {
        sweep_compare(&SweepConfig {
            probe_shape: ColliderShape::Sphere { radius: 0.5 },
            half_extents: Vector3::new(1.0, 1.0, 1.0),
            shape_a_rotation: UnitQuaternion::identity(),
            probe_rotation: UnitQuaternion::identity(),
            sweep_dir: Vector3::x(),
            start_t: 1.0,
            end_t: 2.0,
            steps: 20,
        });
    }

    #[test]
    fn sweep_hull_vs_obb_box() {
        sweep_compare(&SweepConfig {
            probe_shape: ColliderShape::Box {
                half_extents: Vector3::new(0.5, 0.5, 0.5),
            },
            half_extents: Vector3::new(1.0, 1.0, 1.0),
            shape_a_rotation: UnitQuaternion::identity(),
            probe_rotation: UnitQuaternion::identity(),
            sweep_dir: Vector3::x(),
            start_t: 1.0,
            end_t: 2.0,
            steps: 20,
        });
    }

    #[test]
    fn sweep_hull_vs_obb_capsule() {
        sweep_compare(&SweepConfig {
            probe_shape: ColliderShape::Capsule {
                half_height: 0.8,
                radius: 0.3,
            },
            half_extents: Vector3::new(1.0, 1.0, 1.0),
            shape_a_rotation: UnitQuaternion::identity(),
            probe_rotation: UnitQuaternion::identity(),
            sweep_dir: Vector3::x(),
            start_t: 0.8,
            end_t: 2.0,
            steps: 20,
        });
    }

    // --- Rotated hull sweeps (probe approaches a rotated hull) ---

    #[test]
    fn sweep_hull_vs_obb_sphere_rotated() {
        let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.7);
        sweep_compare(&SweepConfig {
            probe_shape: ColliderShape::Sphere { radius: 0.5 },
            half_extents: Vector3::new(1.0, 1.0, 1.0),
            shape_a_rotation: rot,
            probe_rotation: UnitQuaternion::identity(),
            sweep_dir: Vector3::x(),
            start_t: 1.0,
            end_t: 2.5,
            steps: 20,
        });
    }

    #[test]
    fn sweep_hull_vs_obb_box_rotated() {
        let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.7);
        sweep_compare(&SweepConfig {
            probe_shape: ColliderShape::Box {
                half_extents: Vector3::new(0.5, 0.5, 0.5),
            },
            half_extents: Vector3::new(1.0, 1.0, 1.0),
            shape_a_rotation: rot,
            probe_rotation: UnitQuaternion::identity(),
            sweep_dir: Vector3::x(),
            start_t: 1.0,
            end_t: 2.5,
            steps: 20,
        });
    }

    #[test]
    fn sweep_hull_vs_obb_capsule_rotated() {
        let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.7);
        sweep_compare(&SweepConfig {
            probe_shape: ColliderShape::Capsule {
                half_height: 0.8,
                radius: 0.3,
            },
            half_extents: Vector3::new(1.0, 1.0, 1.0),
            shape_a_rotation: rot,
            probe_rotation: UnitQuaternion::identity(),
            sweep_dir: Vector3::x(),
            start_t: 0.8,
            end_t: 2.5,
            steps: 20,
        });
    }

    // --- Diagonal approach (probe sweeps toward a corner/edge, not a face) ---

    #[test]
    fn sweep_hull_vs_obb_sphere_diagonal() {
        let dir = Vector3::new(1.0, 1.0, 0.0).normalize();
        sweep_compare(&SweepConfig {
            probe_shape: ColliderShape::Sphere { radius: 0.5 },
            half_extents: Vector3::new(1.0, 1.0, 1.0),
            shape_a_rotation: UnitQuaternion::identity(),
            probe_rotation: UnitQuaternion::identity(),
            sweep_dir: dir,
            start_t: 1.2,
            end_t: 2.5,
            steps: 20,
        });
    }

    #[test]
    fn sweep_hull_vs_obb_box_diagonal() {
        let dir = Vector3::new(1.0, 1.0, 0.0).normalize();
        sweep_compare(&SweepConfig {
            probe_shape: ColliderShape::Box {
                half_extents: Vector3::new(0.5, 0.5, 0.5),
            },
            half_extents: Vector3::new(1.0, 1.0, 1.0),
            shape_a_rotation: UnitQuaternion::identity(),
            probe_rotation: UnitQuaternion::identity(),
            sweep_dir: dir,
            start_t: 1.2,
            end_t: 2.5,
            steps: 20,
        });
    }

    #[test]
    fn sweep_hull_vs_obb_capsule_diagonal() {
        let dir = Vector3::new(1.0, 1.0, 0.0).normalize();
        sweep_compare(&SweepConfig {
            probe_shape: ColliderShape::Capsule {
                half_height: 0.8,
                radius: 0.3,
            },
            half_extents: Vector3::new(1.0, 1.0, 1.0),
            shape_a_rotation: UnitQuaternion::identity(),
            probe_rotation: UnitQuaternion::identity(),
            sweep_dir: dir,
            start_t: 1.0,
            end_t: 2.5,
            steps: 20,
        });
    }

    // --- Sphere-hull stress cases: off-center, corners, edges, rotated ---

    #[test]
    fn sweep_hull_vs_obb_sphere_corner_approach() {
        // Sphere approaches the (+X, +Y, +Z) corner. At certain steps the sphere
        // center is equidistant from multiple faces, so both the hull and OBB paths
        // pick a valid but different face normal. Kept as a regression target.
        let dir = Vector3::new(1.0, 1.0, 1.0).normalize();
        sweep_compare(&SweepConfig {
            probe_shape: ColliderShape::Sphere { radius: 0.5 },
            half_extents: Vector3::new(1.0, 1.0, 1.0),
            shape_a_rotation: UnitQuaternion::identity(),
            probe_rotation: UnitQuaternion::identity(),
            sweep_dir: dir,
            start_t: 1.0,
            end_t: 2.5,
            steps: 20,
        });
    }

    #[test]
    fn sweep_hull_vs_obb_sphere_edge_slide() {
        // Sphere sweeps along +X but offset in Y, grazing the top edge of the +X face.
        // This tests the edge-contact transition zone.
        let he = Vector3::new(1.0, 1.0, 1.0);
        let hull_data = std::sync::Arc::new(crate::collision::convex_hull::cube_hull(he));
        let hull_shape = ColliderShape::ConvexHull { hull: hull_data };
        let obb_shape = ColliderShape::Box { half_extents: he };
        let sphere = ColliderShape::Sphere { radius: 0.4 };
        let rot = UnitQuaternion::identity();
        let margin = 0.02;

        for i in 0..=20 {
            let x = 0.8 + (i as f32) * 0.1;
            // Offset Y so sphere center is above the face but close to the edge.
            let probe_center = Point3::new(x, 1.15, 0.0);

            let hull_view = view_from(Point3::origin(), rot, &hull_shape);
            let obb_view = view_from(Point3::origin(), rot, &obb_shape);
            let probe_view = view_from(probe_center, rot, &sphere);

            let hull_m = generate_manifold(&hull_view, &probe_view, margin, None, None);
            let obb_m = generate_manifold(&obb_view, &probe_view, margin, None, None);

            assert!(
                (hull_m.len() as i32 - obb_m.len() as i32).unsigned_abs() <= 1,
                "edge_slide x={x:.2}: count hull={}, obb={}",
                hull_m.len(),
                obb_m.len(),
            );
            let n = hull_m.len().min(obb_m.len());
            for j in 0..n {
                let dot = hull_m.points[j].normal.dot(&obb_m.points[j].normal);
                assert!(
                    dot > 0.9,
                    "edge_slide x={x:.2} contact {j}: normal dot={dot:.4}, \
                     hull={:?}, obb={:?}",
                    hull_m.points[j].normal,
                    obb_m.points[j].normal,
                );
                let dd = (hull_m.points[j].depth - obb_m.points[j].depth).abs();
                assert!(
                    dd < 0.1,
                    "edge_slide x={x:.2} contact {j}: depth hull={:.4}, obb={:.4} (diff={dd:.4})",
                    hull_m.points[j].depth,
                    obb_m.points[j].depth,
                );
            }
        }
    }

    #[test]
    fn sweep_hull_vs_obb_sphere_rotated_corner() {
        // Hull rotated 45° around Y, sphere approaches the now-prominent edge.
        // The approach is exactly along the edge symmetry axis, so both adjacent
        // face normals are equally valid. Kept as a regression target.
        let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_4);
        sweep_compare(&SweepConfig {
            probe_shape: ColliderShape::Sphere { radius: 0.5 },
            half_extents: Vector3::new(1.0, 1.0, 1.0),
            shape_a_rotation: rot,
            probe_rotation: UnitQuaternion::identity(),
            sweep_dir: Vector3::x(),
            start_t: 1.0,
            end_t: 2.5,
            steps: 20,
        });
    }

    #[test]
    fn sweep_hull_vs_obb_sphere_rotated_two_axes() {
        // Hull rotated around both Y and X — no face is axis-aligned.
        // This currently fails due to EPA converging to the wrong face for
        // certain steps. Kept as a regression target.
        let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.6)
            * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.4);
        sweep_compare(&SweepConfig {
            probe_shape: ColliderShape::Sphere { radius: 0.5 },
            half_extents: Vector3::new(1.0, 1.0, 1.0),
            shape_a_rotation: rot,
            probe_rotation: UnitQuaternion::identity(),
            sweep_dir: Vector3::x(),
            start_t: 1.0,
            end_t: 2.5,
            steps: 20,
        });
    }

    #[test]
    fn sweep_hull_vs_obb_sphere_asymmetric_hull() {
        // Non-cubic half-extents so the hull isn't a regular cube.
        let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.5);
        sweep_compare(&SweepConfig {
            probe_shape: ColliderShape::Sphere { radius: 0.3 },
            half_extents: Vector3::new(0.5, 1.5, 0.8),
            shape_a_rotation: rot,
            probe_rotation: UnitQuaternion::identity(),
            sweep_dir: Vector3::new(1.0, 0.3, 0.0).normalize(),
            start_t: 0.8,
            end_t: 2.5,
            steps: 20,
        });
    }

    #[test]
    fn sweep_hull_vs_obb_sphere_small_radius() {
        // Tiny sphere near the corner — maximizes edge/vertex proximity effects.
        let dir = Vector3::new(1.0, 0.9, 0.1).normalize();
        sweep_compare(&SweepConfig {
            probe_shape: ColliderShape::Sphere { radius: 0.1 },
            half_extents: Vector3::new(1.0, 1.0, 1.0),
            shape_a_rotation: UnitQuaternion::identity(),
            probe_rotation: UnitQuaternion::identity(),
            sweep_dir: dir,
            start_t: 1.0,
            end_t: 1.8,
            steps: 20,
        });
    }

    /// Signed distance from point to an oriented box surface (negative = inside).
    fn signed_distance_to_box(
        p: Point3<f32>,
        center: Point3<f32>,
        rot: UnitQuaternion<f32>,
        he: Vector3<f32>,
    ) -> f32 {
        let local = rot.inverse() * (p - center);
        let dx = local.x.abs() - he.x;
        let dy = local.y.abs() - he.y;
        let dz = local.z.abs() - he.z;
        if dx <= 0.0 && dy <= 0.0 && dz <= 0.0 {
            dx.max(dy).max(dz)
        } else {
            Vector3::new(dx.max(0.0), dy.max(0.0), dz.max(0.0)).magnitude()
        }
    }

    #[test]
    fn sphere_hull_no_contact_far_weird_angles() {
        use crate::collision::convex_hull::cube_hull;
        use std::sync::Arc;

        let he = Vector3::new(1.0, 1.0, 1.0);
        let sphere_radius = 0.5;
        let sphere = ColliderShape::Sphere {
            radius: sphere_radius,
        };
        let margin = 0.02;

        let weird_cases = [
            (
                UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.83)
                    * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -0.37),
                Vector3::new(0.91, 0.23, 0.34).normalize(),
            ),
            (
                UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -0.61)
                    * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.47),
                Vector3::new(0.68, -0.19, 0.71).normalize(),
            ),
            (
                UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.54)
                    * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.29),
                Vector3::new(0.57, 0.74, -0.36).normalize(),
            ),
        ];

        for (shape_rot, dir) in weird_cases {
            let hull_data = Arc::new(cube_hull(he));
            let hull_shape = ColliderShape::ConvexHull { hull: hull_data };
            let obb_shape = ColliderShape::Box { half_extents: he };

            for i in 0..=40 {
                // Stay in the clearly separated zone.
                let t = 1.8 + i as f32 * 0.05;
                let probe_center = Point3::from(dir * t);
                let sd = signed_distance_to_box(probe_center, Point3::origin(), shape_rot, he);
                if sd < sphere_radius + 0.25 {
                    continue;
                }

                let hull_view = view_from(Point3::origin(), shape_rot, &hull_shape);
                let obb_view = view_from(Point3::origin(), shape_rot, &obb_shape);
                let probe_view = view_from(probe_center, UnitQuaternion::identity(), &sphere);

                let hull_m = generate_manifold(&hull_view, &probe_view, margin, None, None);
                let obb_m = generate_manifold(&obb_view, &probe_view, margin, None, None);

                assert!(
                    obb_m.is_empty(),
                    "OBB baseline should be separated: t={t:.3}, sd={sd:.4}, len={}",
                    obb_m.len()
                );
                assert!(
                    hull_m.is_empty(),
                    "Phantom hull contact: t={t:.3}, sd={sd:.4}, hull_len={}, dir={:?}, rot={:?}",
                    hull_m.len(),
                    dir,
                    shape_rot
                );
            }
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
        let dispatched = generate_manifold(&va, &vb, margin, Some(&mut cache), None);

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

    /// Build a regular tetrahedron ConvexHull with the given edge length,
    /// centered at the origin. Same geometry as the spawnable.
    fn tetrahedron_hull(edge: f32) -> crate::collision::convex_hull::ConvexHull {
        use crate::collision::convex_hull::{ConvexHull, HullFace};
        use smallvec::SmallVec;

        let r = edge * (6.0f32).sqrt() / 4.0;
        let top = Vector3::new(0.0, r, 0.0);
        let y_base = -r / 3.0;
        let base_r = (r * r - y_base * y_base).sqrt();

        let v0 = Vector3::new(0.0, y_base, base_r);
        let v1 = Vector3::new(
            base_r * (2.0 * std::f32::consts::PI / 3.0).sin(),
            y_base,
            base_r * (2.0 * std::f32::consts::PI / 3.0).cos(),
        );
        let v2 = Vector3::new(
            base_r * (4.0 * std::f32::consts::PI / 3.0).sin(),
            y_base,
            base_r * (4.0 * std::f32::consts::PI / 3.0).cos(),
        );

        let vertices = vec![top, v0, v1, v2];

        let face_defs: [(usize, usize, usize, usize); 4] =
            [(1, 2, 3, 0), (0, 2, 1, 3), (0, 3, 2, 1), (0, 1, 3, 2)];

        let faces = face_defs
            .iter()
            .map(|&(a, b, c, opp)| {
                let va = vertices[a];
                let vb = vertices[b];
                let vc = vertices[c];
                let vopp = vertices[opp];

                let raw_normal = (vb - va).cross(&(vc - va));
                let flip = raw_normal.dot(&(va - vopp)) < 0.0;
                let normal = if flip {
                    -raw_normal.normalize()
                } else {
                    raw_normal.normalize()
                };

                let mut indices: SmallVec<[u16; 6]> =
                    SmallVec::from_slice(&[a as u16, b as u16, c as u16]);
                if flip {
                    indices[1..].reverse();
                }

                HullFace {
                    vertex_indices: indices,
                    normal,
                }
            })
            .collect();

        ConvexHull::new(vertices, faces)
    }

    #[test]
    fn box_tetrahedron_no_spurious_contact() {
        use std::sync::Arc;

        let box_shape = ColliderShape::Box {
            half_extents: Vector3::new(10.0, 0.15, 10.0),
        };
        let hull_data = Arc::new(tetrahedron_hull(1.5));
        let hull_shape = ColliderShape::ConvexHull { hull: hull_data };
        let margin = 0.02;

        let box_view = view_from(
            Point3::new(3.0, 0.0, -6.0),
            UnitQuaternion::identity(),
            &box_shape,
        );
        let hull_view = view_from(
            Point3::new(3.0, 1.0, -6.0),
            UnitQuaternion::identity(),
            &hull_shape,
        );

        // Test both orderings — dispatch puts hull (rank 3) as a, box (rank 2) as b.
        let manifold_dispatch = generate_manifold(&hull_view, &box_view, margin, None, None);
        let manifold_reversed = generate_manifold(&box_view, &hull_view, margin, None, None);

        // The tetrahedron's lowest point is at y ≈ 1.0 - 0.306 = 0.694, well above
        // the box top at y = 0.15. There should be no contacts.
        assert!(
            manifold_dispatch.is_empty(),
            "Expected no contacts (dispatch order hull,box) but got {} contacts: {:?}",
            manifold_dispatch.len(),
            manifold_dispatch
                .points
                .iter()
                .map(|p| format!(
                    "({:.3}, {:.3}, {:.3}) depth={:.4}",
                    p.point.x, p.point.y, p.point.z, p.depth
                ))
                .collect::<Vec<_>>(),
        );
        assert!(
            manifold_reversed.is_empty(),
            "Expected no contacts (reversed order box,hull) but got {} contacts: {:?}",
            manifold_reversed.len(),
            manifold_reversed
                .points
                .iter()
                .map(|p| format!(
                    "({:.3}, {:.3}, {:.3}) depth={:.4}",
                    p.point.x, p.point.y, p.point.z, p.depth
                ))
                .collect::<Vec<_>>(),
        );
    }

    /// Same scenario but with a thicker box (half_y=0.5). The gap is only ~0.19,
    /// which is small enough that the tightened GJK stall heuristic still fires.
    /// This test verifies the clipping guard (axis_raw_depth < -margin) catches
    /// the spurious contact even when GJK falsely reports Intersecting.
    #[test]
    fn box_tetrahedron_no_spurious_contact_thick_box() {
        use std::sync::Arc;

        let box_shape = ColliderShape::Box {
            half_extents: Vector3::new(10.0, 0.5, 10.0),
        };
        let hull_data = Arc::new(tetrahedron_hull(1.5));
        let hull_shape = ColliderShape::ConvexHull { hull: hull_data };
        let margin = 0.02;

        let box_view = view_from(
            Point3::new(3.0, 0.0, -6.0),
            UnitQuaternion::identity(),
            &box_shape,
        );
        let hull_view = view_from(
            Point3::new(3.0, 1.0, -6.0),
            UnitQuaternion::identity(),
            &hull_shape,
        );

        let manifold_dispatch = generate_manifold(&hull_view, &box_view, margin, None, None);
        let manifold_reversed = generate_manifold(&box_view, &hull_view, margin, None, None);

        // Box top at y=0.5, tetrahedron bottom at y≈0.694. Gap ≈ 0.19.
        assert!(
            manifold_dispatch.is_empty(),
            "Expected no contacts (dispatch order hull,box) but got {} contacts: {:?}",
            manifold_dispatch.len(),
            manifold_dispatch
                .points
                .iter()
                .map(|p| format!(
                    "({:.3}, {:.3}, {:.3}) depth={:.4}",
                    p.point.x, p.point.y, p.point.z, p.depth
                ))
                .collect::<Vec<_>>(),
        );
        assert!(
            manifold_reversed.is_empty(),
            "Expected no contacts (reversed order box,hull) but got {} contacts: {:?}",
            manifold_reversed.len(),
            manifold_reversed
                .points
                .iter()
                .map(|p| format!(
                    "({:.3}, {:.3}, {:.3}) depth={:.4}",
                    p.point.x, p.point.y, p.point.z, p.depth
                ))
                .collect::<Vec<_>>(),
        );
    }

    #[test]
    fn sphere_tetrahedron_contact_on_sphere_surface() {
        use crate::collision::convex_hull::{ConvexHull, HullFace};
        use nalgebra::Quaternion;
        use smallvec::smallvec;
        use std::sync::Arc;

        let sphere_center = Point3::new(5.72383499, -0.16803811, 0.57539350);
        let sphere_radius: f32 = 0.50000000;
        let hull_center = Point3::new(6.08925867, -0.66028416, -0.04257488);
        let hull_rotation = UnitQuaternion::from_quaternion(Quaternion::new(
            0.99665952,
            -0.04039019,
            0.07095429,
            -0.00201223,
        ));
        let hull_vertices = vec![
            Vector3::new(0.00000000, 0.91855872, 0.00000000),
            Vector3::new(0.00000000, -0.30618623, 0.86602545),
            Vector3::new(0.75000000, -0.30618623, -0.43301278),
            Vector3::new(-0.75000006, -0.30618623, -0.43301263),
        ];
        let hull_faces = vec![
            HullFace {
                vertex_indices: smallvec![1, 3, 2],
                normal: Vector3::new(0.0, -1.0, 0.0),
            },
            HullFace {
                vertex_indices: smallvec![0, 1, 2],
                normal: Vector3::new(0.81649661, 0.33333334, 0.47140452),
            },
            HullFace {
                vertex_indices: smallvec![0, 2, 3],
                normal: Vector3::new(-0.00000009, 0.33333334, -0.94280905),
            },
            HullFace {
                vertex_indices: smallvec![0, 3, 1],
                normal: Vector3::new(-0.81649649, 0.33333334, 0.47140452),
            },
        ];

        let hull_data = Arc::new(ConvexHull::new(hull_vertices, hull_faces));
        let hull_shape = ColliderShape::ConvexHull { hull: hull_data };
        let sphere_shape = ColliderShape::Sphere {
            radius: sphere_radius,
        };
        let margin = 0.02;

        let hull_view = view_from(hull_center, hull_rotation, &hull_shape);
        let sphere_view = view_from(sphere_center, UnitQuaternion::identity(), &sphere_shape);

        let manifold = generate_manifold(&hull_view, &sphere_view, margin, None, None);

        assert!(!manifold.is_empty(), "Expected a contact");
        for (i, cp) in manifold.points.iter().enumerate() {
            let dist = (cp.point - sphere_center).magnitude();
            let error = (dist - sphere_radius).abs();
            assert!(
                error < 0.05,
                "Contact {i}: point not on sphere surface. \
                 dist_from_center={dist:.6}, radius={sphere_radius}, error={error:.4}",
            );
        }
    }
}
