//! Centralized shape-pair dispatch for collision detection.
//!
//! Routes shape pairs to specialized collision functions (analytic/SAT fast-paths)
//! based on `ColliderShape` variants. Unknown pairs fall through to GJK/EPA.

use nalgebra::{Point3, UnitQuaternion};

use super::capsule::Capsule;
use super::contact::ContactManifold;
use super::convex_hull::ConvexHull;

/// Enable to log phantom contact diagnostics for hull-hull pairs to stderr.
const DEBUG_HULL_CONTACTS: bool = false;
use super::discrete::capsule_capsule::capsule_capsule_manifold;
use super::discrete::gjk::GjkCache;
use super::discrete::gjk_epa_manifold::gjk_epa_manifold_cached;
use super::discrete::hull_hull::hull_hull_manifold;
use super::discrete::hull_obb::hull_obb_manifold;
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

        (ColliderShape::Box { half_extents }, ColliderShape::ConvexHull { .. })
        | (ColliderShape::ConvexHull { .. }, ColliderShape::Box { half_extents }) => {
            let (hull_view, obb_half_extents, obb_center, obb_rotation) =
                if matches!(a.shape, ColliderShape::ConvexHull { .. }) {
                    (a, half_extents, b.center, b.rotation)
                } else {
                    (b, half_extents, a.center, a.rotation)
                };
            let obb = Obb::new(obb_center, obb_rotation, *obb_half_extents);
            let cache = match sat_cache {
                Some(c) => c,
                None => &mut SatCache::default(),
            };
            hull_obb_manifold(hull_view, &obb, margin, cache, gjk_cache)
        }

        (ColliderShape::ConvexHull { .. }, ColliderShape::ConvexHull { .. }) => {
            let cache = match sat_cache {
                Some(c) => c,
                None => &mut SatCache::default(),
            };
            let manifold = hull_hull_manifold(a, b, margin, cache, gjk_cache);
            if DEBUG_HULL_CONTACTS {
                validate_hull_hull_contacts(&manifold, a, b, margin);
            }
            manifold
        }

        // GJK/EPA fallback for any shape pair without a specialized fast-path.
        // The normal convention is a→b, so we must pass the higher-ranked shape
        // as `a` to match the solver's body_a→body_b expectation (same as the
        // analytic arms above which always put the "larger" shape first).
        _ => {
            let manifold = if shape_rank(a.shape) >= shape_rank(b.shape) {
                gjk_epa_manifold_cached(a, b, margin, gjk_cache)
            } else {
                gjk_epa_manifold_cached(b, a, margin, gjk_cache)
            };
            manifold
        }
    }
}

/// Maximum face-plane violation (signed distance) of a world-space point against
/// a convex hull. Returns the largest `(point - face_vertex).dot(face_normal)`
/// across all faces. Negative means inside, zero means on surface, positive means
/// outside.
fn max_face_plane_violation(
    point: &Point3<f32>,
    hull: &ConvexHull,
    center: &Point3<f32>,
    rotation: &UnitQuaternion<f32>,
) -> f32 {
    let mut max_violation = f32::NEG_INFINITY;
    for face in &hull.faces {
        let world_normal = rotation * face.normal;
        let ref_vertex_local = hull.vertices[face.vertex_indices[0] as usize];
        let ref_vertex_world = center + rotation * ref_vertex_local;
        let violation = (point - ref_vertex_world).dot(&world_normal);
        if violation > max_violation {
            max_violation = violation;
        }
    }
    max_violation
}

/// Validate hull-hull contact points against face planes and log offenders.
fn validate_hull_hull_contacts(
    manifold: &ContactManifold,
    a: &ShapeView,
    b: &ShapeView,
    margin: f32,
) {
    let hull_a = match a.shape {
        ColliderShape::ConvexHull { hull } => hull,
        _ => return,
    };
    let hull_b = match b.shape {
        ColliderShape::ConvexHull { hull } => hull,
        _ => return,
    };

    let tolerance = margin + 0.05;
    let mut geometry_printed = false;

    for (i, cp) in manifold.points.iter().enumerate() {
        let violation_a = max_face_plane_violation(&cp.point, hull_a, &a.center, &a.rotation);
        let violation_b = max_face_plane_violation(&cp.point, hull_b, &b.center, &b.rotation);

        if violation_a > tolerance || violation_b > tolerance {
            eprintln!("=== PHANTOM CONTACT DETECTED (point {i}) ===");
            eprintln!(
                "  contact: point=({:.4}, {:.4}, {:.4}), normal=({:.4}, {:.4}, {:.4}), raw_depth={:.4}",
                cp.point.x, cp.point.y, cp.point.z,
                cp.raw_normal.x, cp.raw_normal.y, cp.raw_normal.z,
                cp.raw_depth,
            );
            eprintln!(
                "  face-plane violation: hull_a={:.4}, hull_b={:.4} (tolerance={:.4})",
                violation_a, violation_b, tolerance,
            );
            if !geometry_printed {
                eprintln!("  --- Hull A (for unit test) ---");
                print_hull_geometry(hull_a, &a.center, &a.rotation);
                eprintln!("  --- Hull B (for unit test) ---");
                print_hull_geometry(hull_b, &b.center, &b.rotation);
                eprintln!("  margin: {margin:.4}");
                geometry_printed = true;
            }
            eprintln!("=== END PHANTOM CONTACT ===");
        }
    }
}

/// Print hull geometry in a format that can be pasted into a unit test.
fn print_hull_geometry(hull: &ConvexHull, center: &Point3<f32>, rotation: &UnitQuaternion<f32>) {
    eprintln!(
        "  center: Point3::new({:.6}, {:.6}, {:.6})",
        center.x, center.y, center.z,
    );
    let (x, y, z, w) = {
        let q = rotation.as_ref();
        (q.i, q.j, q.k, q.w)
    };
    eprintln!(
        "  rotation: UnitQuaternion::from_quaternion(Quaternion::new({:.6}, {:.6}, {:.6}, {:.6}))",
        w, x, y, z,
    );
    eprintln!("  vertices: vec![");
    for v in &hull.vertices {
        eprintln!("    Vector3::new({:.6}, {:.6}, {:.6}),", v.x, v.y, v.z);
    }
    eprintln!("  ]");
    eprintln!("  faces: vec![");
    for face in &hull.faces {
        let indices: Vec<String> = face.vertex_indices.iter().map(|i| i.to_string()).collect();
        eprintln!(
            "    HullFace {{ vertex_indices: SmallVec::from_slice(&[{}]), normal: Vector3::new({:.6}, {:.6}, {:.6}) }},",
            indices.join(", "),
            face.normal.x, face.normal.y, face.normal.z,
        );
    }
    eprintln!("  ]");
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
        _ => gjk_patch_manifold(shape, patch, margin),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::contact::FeatureId;
    use crate::collision::convex_hull::{ConvexHull, HullFace};
    use crate::collision::mesh::seam_filter::{ContactFace, FilteredPatch};
    use nalgebra::{Point3, UnitQuaternion, Vector3};
    use smallvec::SmallVec;

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

    /// Helper: build a ConvexHull shape from raw geometry data.
    fn hull_view_from(
        vertices: Vec<Vector3<f32>>,
        faces: Vec<HullFace>,
    ) -> (std::sync::Arc<ConvexHull>, ColliderShape) {
        let hull = std::sync::Arc::new(ConvexHull::new(vertices, faces));
        let shape = ColliderShape::ConvexHull { hull: hull.clone() };
        (hull, shape)
    }

    /// Assert no contact point in a manifold violates either hull's face planes
    /// beyond `margin + 0.05`.
    fn assert_no_phantom_contacts(
        manifold: &ContactManifold,
        hull_a: &ConvexHull,
        center_a: &Point3<f32>,
        rot_a: &UnitQuaternion<f32>,
        hull_b: &ConvexHull,
        center_b: &Point3<f32>,
        rot_b: &UnitQuaternion<f32>,
        margin: f32,
    ) {
        let tolerance = margin + 0.05;
        for (i, cp) in manifold.points.iter().enumerate() {
            let violation_a = max_face_plane_violation(&cp.point, hull_a, center_a, rot_a);
            let violation_b = max_face_plane_violation(&cp.point, hull_b, center_b, rot_b);
            assert!(
                violation_a <= tolerance && violation_b <= tolerance,
                "Contact {i}: phantom point ({:.4}, {:.4}, {:.4}), \
                 violation_a={violation_a:.4}, violation_b={violation_b:.4}, tolerance={tolerance:.4}",
                cp.point.x, cp.point.y, cp.point.z,
            );
        }
    }

    /// Regression test: two tapered cylinders (20-gon hulls) whose GJK/EPA
    /// manifold produced a contact point 0.365 outside hull A's face planes.
    #[test]
    fn phantom_contact_tapered_cylinders() {
        use nalgebra::Quaternion;
        use smallvec::SmallVec;

        let vertices_a = vec![
            Vector3::new(0.666668, -4.000000, 0.000004),
            Vector3::new(0.634039, -4.000000, 0.206017),
            Vector3::new(0.539347, -4.000000, 0.391861),
            Vector3::new(0.391859, -4.000000, 0.539349),
            Vector3::new(0.206013, -4.000000, 0.634041),
            Vector3::new(0.000002, -4.000000, 0.666672),
            Vector3::new(-0.206009, -4.000000, 0.634041),
            Vector3::new(-0.391855, -4.000000, 0.539349),
            Vector3::new(-0.539343, -4.000000, 0.391861),
            Vector3::new(-0.634035, -4.000000, 0.206017),
            Vector3::new(-0.666664, -4.000000, 0.000004),
            Vector3::new(-0.634035, -4.000000, -0.206009),
            Vector3::new(-0.539343, -4.000000, -0.391853),
            Vector3::new(-0.391855, -4.000000, -0.539341),
            Vector3::new(-0.206009, -4.000000, -0.634033),
            Vector3::new(0.000002, -4.000000, -0.666664),
            Vector3::new(0.206013, -4.000000, -0.634033),
            Vector3::new(0.391859, -4.000000, -0.539341),
            Vector3::new(0.539347, -4.000000, -0.391853),
            Vector3::new(0.634039, -4.000000, -0.206009),
            Vector3::new(0.546669, 4.000000, 0.000004),
            Vector3::new(0.519913, 4.000000, 0.168934),
            Vector3::new(0.442265, 4.000000, 0.321327),
            Vector3::new(0.321325, 4.000000, 0.442265),
            Vector3::new(0.168932, 4.000000, 0.519917),
            Vector3::new(0.000002, 4.000000, 0.546669),
            Vector3::new(-0.168928, 4.000000, 0.519917),
            Vector3::new(-0.321321, 4.000000, 0.442265),
            Vector3::new(-0.442261, 4.000000, 0.321327),
            Vector3::new(-0.519909, 4.000000, 0.168934),
            Vector3::new(-0.546665, 4.000000, 0.000004),
            Vector3::new(-0.519909, 4.000000, -0.168926),
            Vector3::new(-0.442261, 4.000000, -0.321320),
            Vector3::new(-0.321321, 4.000000, -0.442261),
            Vector3::new(-0.168928, 4.000000, -0.519909),
            Vector3::new(0.000002, 4.000000, -0.546661),
            Vector3::new(0.168932, 4.000000, -0.519909),
            Vector3::new(0.321325, 4.000000, -0.442257),
            Vector3::new(0.442265, 4.000000, -0.321320),
            Vector3::new(0.519913, 4.000000, -0.168926),
        ];
        let faces_a = vec![
            HullFace {
                vertex_indices: SmallVec::from_slice(&[
                    19, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
                ]),
                normal: Vector3::new(0.0, -1.0, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[
                    20, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21,
                ]),
                normal: Vector3::new(0.0, 1.0, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 20, 21, 1]),
                normal: Vector3::new(0.987580, 0.014814, 0.156416),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[1, 21, 22, 2]),
                normal: Vector3::new(0.890909, 0.014814, 0.453940),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[2, 22, 23, 3]),
                normal: Vector3::new(0.707029, 0.014814, 0.707029),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[3, 23, 24, 4]),
                normal: Vector3::new(0.453936, 0.014813, 0.890911),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[4, 24, 25, 5]),
                normal: Vector3::new(0.156427, 0.014814, 0.987578),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[5, 25, 26, 6]),
                normal: Vector3::new(-0.156427, 0.014813, 0.987578),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[6, 26, 27, 7]),
                normal: Vector3::new(-0.453936, 0.014814, 0.890911),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[7, 27, 28, 8]),
                normal: Vector3::new(-0.707029, 0.014814, 0.707029),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[8, 28, 29, 9]),
                normal: Vector3::new(-0.890909, 0.014814, 0.453940),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[9, 29, 30, 10]),
                normal: Vector3::new(-0.987580, 0.014814, 0.156416),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[10, 30, 31, 11]),
                normal: Vector3::new(-0.987580, 0.014814, -0.156416),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[11, 31, 32, 12]),
                normal: Vector3::new(-0.890909, 0.014814, -0.453940),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[12, 32, 33, 13]),
                normal: Vector3::new(-0.707029, 0.014813, -0.707029),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[13, 33, 34, 14]),
                normal: Vector3::new(-0.453936, 0.014813, -0.890911),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[14, 34, 35, 15]),
                normal: Vector3::new(-0.156427, 0.014814, -0.987578),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[15, 35, 36, 16]),
                normal: Vector3::new(0.156427, 0.014813, -0.987578),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[16, 36, 37, 17]),
                normal: Vector3::new(0.453936, 0.014814, -0.890911),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[17, 37, 38, 18]),
                normal: Vector3::new(0.707029, 0.014814, -0.707029),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[18, 38, 39, 19]),
                normal: Vector3::new(0.890909, 0.014814, -0.453940),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[19, 39, 20, 0]),
                normal: Vector3::new(0.987580, 0.014814, -0.156416),
            },
        ];

        let vertices_b = vec![
            Vector3::new(0.666668, -4.000000, -0.000004),
            Vector3::new(0.634039, -4.000000, 0.206009),
            Vector3::new(0.539347, -4.000000, 0.391853),
            Vector3::new(0.391859, -4.000000, 0.539341),
            Vector3::new(0.206013, -4.000000, 0.634033),
            Vector3::new(0.000002, -4.000000, 0.666664),
            Vector3::new(-0.206009, -4.000000, 0.634033),
            Vector3::new(-0.391855, -4.000000, 0.539341),
            Vector3::new(-0.539343, -4.000000, 0.391853),
            Vector3::new(-0.634035, -4.000000, 0.206009),
            Vector3::new(-0.666664, -4.000000, -0.000004),
            Vector3::new(-0.634035, -4.000000, -0.206017),
            Vector3::new(-0.539343, -4.000000, -0.391861),
            Vector3::new(-0.391855, -4.000000, -0.539349),
            Vector3::new(-0.206009, -4.000000, -0.634041),
            Vector3::new(0.000002, -4.000000, -0.666672),
            Vector3::new(0.206013, -4.000000, -0.634041),
            Vector3::new(0.391859, -4.000000, -0.539349),
            Vector3::new(0.539347, -4.000000, -0.391861),
            Vector3::new(0.634039, -4.000000, -0.206017),
            Vector3::new(0.546669, 4.000000, -0.000004),
            Vector3::new(0.519913, 4.000000, 0.168926),
            Vector3::new(0.442265, 4.000000, 0.321320),
            Vector3::new(0.321325, 4.000000, 0.442257),
            Vector3::new(0.168932, 4.000000, 0.519909),
            Vector3::new(0.000002, 4.000000, 0.546661),
            Vector3::new(-0.168928, 4.000000, 0.519909),
            Vector3::new(-0.321321, 4.000000, 0.442257),
            Vector3::new(-0.442261, 4.000000, 0.321320),
            Vector3::new(-0.519909, 4.000000, 0.168926),
            Vector3::new(-0.546665, 4.000000, -0.000004),
            Vector3::new(-0.519909, 4.000000, -0.168934),
            Vector3::new(-0.442261, 4.000000, -0.321327),
            Vector3::new(-0.321321, 4.000000, -0.442268),
            Vector3::new(-0.168928, 4.000000, -0.519917),
            Vector3::new(0.000002, 4.000000, -0.546669),
            Vector3::new(0.168932, 4.000000, -0.519917),
            Vector3::new(0.321325, 4.000000, -0.442265),
            Vector3::new(0.442265, 4.000000, -0.321327),
            Vector3::new(0.519913, 4.000000, -0.168934),
        ];
        let faces_b = vec![
            HullFace {
                vertex_indices: SmallVec::from_slice(&[
                    19, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
                ]),
                normal: Vector3::new(0.0, -1.0, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[
                    20, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21,
                ]),
                normal: Vector3::new(0.0, 1.0, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 20, 21, 1]),
                normal: Vector3::new(0.987580, 0.014814, 0.156416),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[1, 21, 22, 2]),
                normal: Vector3::new(0.890909, 0.014814, 0.453940),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[2, 22, 23, 3]),
                normal: Vector3::new(0.707029, 0.014814, 0.707029),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[3, 23, 24, 4]),
                normal: Vector3::new(0.453936, 0.014813, 0.890911),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[4, 24, 25, 5]),
                normal: Vector3::new(0.156427, 0.014814, 0.987578),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[5, 25, 26, 6]),
                normal: Vector3::new(-0.156427, 0.014813, 0.987578),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[6, 26, 27, 7]),
                normal: Vector3::new(-0.453936, 0.014814, 0.890911),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[7, 27, 28, 8]),
                normal: Vector3::new(-0.707029, 0.014814, 0.707029),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[8, 28, 29, 9]),
                normal: Vector3::new(-0.890909, 0.014814, 0.453940),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[9, 29, 30, 10]),
                normal: Vector3::new(-0.987580, 0.014814, 0.156416),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[10, 30, 31, 11]),
                normal: Vector3::new(-0.987580, 0.014814, -0.156416),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[11, 31, 32, 12]),
                normal: Vector3::new(-0.890909, 0.014814, -0.453940),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[12, 32, 33, 13]),
                normal: Vector3::new(-0.707029, 0.014813, -0.707029),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[13, 33, 34, 14]),
                normal: Vector3::new(-0.453936, 0.014813, -0.890911),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[14, 34, 35, 15]),
                normal: Vector3::new(-0.156427, 0.014814, -0.987578),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[15, 35, 36, 16]),
                normal: Vector3::new(0.156427, 0.014813, -0.987578),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[16, 36, 37, 17]),
                normal: Vector3::new(0.453936, 0.014814, -0.890911),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[17, 37, 38, 18]),
                normal: Vector3::new(0.707029, 0.014814, -0.707029),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[18, 38, 39, 19]),
                normal: Vector3::new(0.890909, 0.014814, -0.453940),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[19, 39, 20, 0]),
                normal: Vector3::new(0.987580, 0.014814, -0.156416),
            },
        ];

        let (hull_a, shape_a) = hull_view_from(vertices_a, faces_a);
        let (hull_b, shape_b) = hull_view_from(vertices_b, faces_b);

        let center_a = Point3::new(19.071051, -0.511147, -34.252190);
        let rot_a = UnitQuaternion::from_quaternion(Quaternion::new(
            0.455293, 0.598422, 0.206850, 0.625949,
        ));
        let center_b = Point3::new(19.258074, -1.270623, -31.333380);
        let rot_b = UnitQuaternion::from_quaternion(Quaternion::new(
            0.578825, 0.666153, 0.118727, 0.455088,
        ));

        let va = view_from(center_a, rot_a, &shape_a);
        let vb = view_from(center_b, rot_b, &shape_b);
        let margin = 0.02;

        let manifold = generate_manifold(&va, &vb, margin, None, None);
        assert_no_phantom_contacts(
            &manifold, &hull_a, &center_a, &rot_a, &hull_b, &center_b, &rot_b, margin,
        );
    }

    /// Regression test: tapered cylinder vs tetrahedron, contact 0.076 outside
    /// both hulls' face planes.
    #[test]
    fn phantom_contact_cylinder_vs_tetrahedron() {
        use nalgebra::Quaternion;
        use smallvec::SmallVec;

        let vertices_a = vec![
            Vector3::new(0.666668, -4.000000, -0.000002),
            Vector3::new(0.634039, -4.000000, 0.206009),
            Vector3::new(0.539347, -4.000000, 0.391855),
            Vector3::new(0.391859, -4.000000, 0.539343),
            Vector3::new(0.206013, -4.000000, 0.634035),
            Vector3::new(0.000002, -4.000000, 0.666664),
            Vector3::new(-0.206009, -4.000000, 0.634035),
            Vector3::new(-0.391855, -4.000000, 0.539343),
            Vector3::new(-0.539343, -4.000000, 0.391855),
            Vector3::new(-0.634035, -4.000000, 0.206009),
            Vector3::new(-0.666664, -4.000000, -0.000002),
            Vector3::new(-0.634035, -4.000000, -0.206013),
            Vector3::new(-0.539343, -4.000000, -0.391859),
            Vector3::new(-0.391855, -4.000000, -0.539347),
            Vector3::new(-0.206009, -4.000000, -0.634039),
            Vector3::new(0.000002, -4.000000, -0.666668),
            Vector3::new(0.206013, -4.000000, -0.634039),
            Vector3::new(0.391859, -4.000000, -0.539347),
            Vector3::new(0.539347, -4.000000, -0.391859),
            Vector3::new(0.634039, -4.000000, -0.206013),
            Vector3::new(0.546669, 4.000000, -0.000002),
            Vector3::new(0.519913, 4.000000, 0.168928),
            Vector3::new(0.442265, 4.000000, 0.321321),
            Vector3::new(0.321325, 4.000000, 0.442261),
            Vector3::new(0.168932, 4.000000, 0.519909),
            Vector3::new(0.000002, 4.000000, 0.546665),
            Vector3::new(-0.168928, 4.000000, 0.519909),
            Vector3::new(-0.321321, 4.000000, 0.442261),
            Vector3::new(-0.442261, 4.000000, 0.321321),
            Vector3::new(-0.519909, 4.000000, 0.168928),
            Vector3::new(-0.546665, 4.000000, -0.000002),
            Vector3::new(-0.519909, 4.000000, -0.168932),
            Vector3::new(-0.442261, 4.000000, -0.321325),
            Vector3::new(-0.321321, 4.000000, -0.442265),
            Vector3::new(-0.168928, 4.000000, -0.519913),
            Vector3::new(0.000002, 4.000000, -0.546669),
            Vector3::new(0.168932, 4.000000, -0.519913),
            Vector3::new(0.321325, 4.000000, -0.442265),
            Vector3::new(0.442265, 4.000000, -0.321325),
            Vector3::new(0.519913, 4.000000, -0.168932),
        ];
        let faces_a = vec![
            HullFace {
                vertex_indices: SmallVec::from_slice(&[
                    19, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
                ]),
                normal: Vector3::new(0.0, -1.0, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[
                    20, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21,
                ]),
                normal: Vector3::new(0.0, 1.0, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 20, 21, 1]),
                normal: Vector3::new(0.987580, 0.014814, 0.156418),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[1, 21, 22, 2]),
                normal: Vector3::new(0.890911, 0.014814, 0.453936),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[2, 22, 23, 3]),
                normal: Vector3::new(0.707029, 0.014814, 0.707029),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[3, 23, 24, 4]),
                normal: Vector3::new(0.453936, 0.014814, 0.890911),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[4, 24, 25, 5]),
                normal: Vector3::new(0.156418, 0.014814, 0.987580),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[5, 25, 26, 6]),
                normal: Vector3::new(-0.156418, 0.014814, 0.987580),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[6, 26, 27, 7]),
                normal: Vector3::new(-0.453936, 0.014814, 0.890911),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[7, 27, 28, 8]),
                normal: Vector3::new(-0.707029, 0.014814, 0.707029),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[8, 28, 29, 9]),
                normal: Vector3::new(-0.890911, 0.014814, 0.453936),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[9, 29, 30, 10]),
                normal: Vector3::new(-0.987580, 0.014814, 0.156418),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[10, 30, 31, 11]),
                normal: Vector3::new(-0.987580, 0.014814, -0.156418),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[11, 31, 32, 12]),
                normal: Vector3::new(-0.890911, 0.014814, -0.453936),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[12, 32, 33, 13]),
                normal: Vector3::new(-0.707029, 0.014814, -0.707029),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[13, 33, 34, 14]),
                normal: Vector3::new(-0.453936, 0.014814, -0.890911),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[14, 34, 35, 15]),
                normal: Vector3::new(-0.156418, 0.014814, -0.987580),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[15, 35, 36, 16]),
                normal: Vector3::new(0.156418, 0.014814, -0.987580),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[16, 36, 37, 17]),
                normal: Vector3::new(0.453936, 0.014814, -0.890911),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[17, 37, 38, 18]),
                normal: Vector3::new(0.707029, 0.014814, -0.707029),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[18, 38, 39, 19]),
                normal: Vector3::new(0.890911, 0.014814, -0.453936),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[19, 39, 20, 0]),
                normal: Vector3::new(0.987580, 0.014814, -0.156418),
            },
        ];

        let vertices_b = vec![
            Vector3::new(0.000000, 0.918559, 0.000000),
            Vector3::new(0.000000, -0.306186, 0.866025),
            Vector3::new(0.750000, -0.306186, -0.433013),
            Vector3::new(-0.750000, -0.306186, -0.433013),
        ];
        let faces_b = vec![
            HullFace {
                vertex_indices: SmallVec::from_slice(&[1, 3, 2]),
                normal: Vector3::new(0.0, -1.0, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 1, 2]),
                normal: Vector3::new(0.816497, 0.333333, 0.471405),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 2, 3]),
                normal: Vector3::new(0.0, 0.333333, -0.942809),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 3, 1]),
                normal: Vector3::new(-0.816496, 0.333333, 0.471405),
            },
        ];

        let center_a = Point3::new(25.326393, 0.421691, -22.565800);
        let rot_a = UnitQuaternion::from_quaternion(Quaternion::new(
            0.558867, 0.741737, 0.321328, -0.185047,
        ));
        let center_b = Point3::new(26.925224, -0.258862, -22.313391);
        let rot_b = UnitQuaternion::from_quaternion(Quaternion::new(
            0.673709, -0.002595, -0.737262, -0.050532,
        ));

        let (hull_a, shape_a) = hull_view_from(vertices_a, faces_a);
        let (hull_b, shape_b) = hull_view_from(vertices_b, faces_b);

        let va = view_from(center_a, rot_a, &shape_a);
        let vb = view_from(center_b, rot_b, &shape_b);
        let margin = 0.02;

        let manifold = generate_manifold(&va, &vb, margin, None, None);
        assert_no_phantom_contacts(
            &manifold, &hull_a, &center_a, &rot_a, &hull_b, &center_b, &rot_b, margin,
        );
    }

    /// Regression test: two tapered cylinders, contact 1.99 outside hull A's
    /// face planes.
    #[test]
    fn phantom_contact_cylinders_large_violation() {
        use nalgebra::Quaternion;
        use smallvec::SmallVec;

        let vertices_a = vec![
            Vector3::new(0.666668, -4.000000, -0.000004),
            Vector3::new(0.634039, -4.000000, 0.206009),
            Vector3::new(0.539347, -4.000000, 0.391853),
            Vector3::new(0.391859, -4.000000, 0.539341),
            Vector3::new(0.206013, -4.000000, 0.634033),
            Vector3::new(0.000002, -4.000000, 0.666664),
            Vector3::new(-0.206009, -4.000000, 0.634033),
            Vector3::new(-0.391855, -4.000000, 0.539341),
            Vector3::new(-0.539343, -4.000000, 0.391853),
            Vector3::new(-0.634035, -4.000000, 0.206009),
            Vector3::new(-0.666664, -4.000000, -0.000004),
            Vector3::new(-0.634035, -4.000000, -0.206017),
            Vector3::new(-0.539343, -4.000000, -0.391861),
            Vector3::new(-0.391855, -4.000000, -0.539349),
            Vector3::new(-0.206009, -4.000000, -0.634041),
            Vector3::new(0.000002, -4.000000, -0.666672),
            Vector3::new(0.206013, -4.000000, -0.634041),
            Vector3::new(0.391859, -4.000000, -0.539349),
            Vector3::new(0.539347, -4.000000, -0.391861),
            Vector3::new(0.634039, -4.000000, -0.206017),
            Vector3::new(0.546669, 4.000000, -0.000004),
            Vector3::new(0.519913, 4.000000, 0.168926),
            Vector3::new(0.442265, 4.000000, 0.321320),
            Vector3::new(0.321325, 4.000000, 0.442257),
            Vector3::new(0.168932, 4.000000, 0.519909),
            Vector3::new(0.000002, 4.000000, 0.546661),
            Vector3::new(-0.168928, 4.000000, 0.519909),
            Vector3::new(-0.321321, 4.000000, 0.442257),
            Vector3::new(-0.442261, 4.000000, 0.321320),
            Vector3::new(-0.519909, 4.000000, 0.168926),
            Vector3::new(-0.546665, 4.000000, -0.000004),
            Vector3::new(-0.519909, 4.000000, -0.168934),
            Vector3::new(-0.442261, 4.000000, -0.321327),
            Vector3::new(-0.321321, 4.000000, -0.442268),
            Vector3::new(-0.168928, 4.000000, -0.519917),
            Vector3::new(0.000002, 4.000000, -0.546669),
            Vector3::new(0.168932, 4.000000, -0.519917),
            Vector3::new(0.321325, 4.000000, -0.442265),
            Vector3::new(0.442265, 4.000000, -0.321327),
            Vector3::new(0.519913, 4.000000, -0.168934),
        ];
        let faces_a = vec![
            HullFace {
                vertex_indices: SmallVec::from_slice(&[
                    19, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
                ]),
                normal: Vector3::new(0.0, -1.0, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[
                    20, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21,
                ]),
                normal: Vector3::new(0.0, 1.0, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 20, 21, 1]),
                normal: Vector3::new(0.987580, 0.014814, 0.156416),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[1, 21, 22, 2]),
                normal: Vector3::new(0.890909, 0.014814, 0.453940),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[2, 22, 23, 3]),
                normal: Vector3::new(0.707029, 0.014814, 0.707029),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[3, 23, 24, 4]),
                normal: Vector3::new(0.453936, 0.014813, 0.890911),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[4, 24, 25, 5]),
                normal: Vector3::new(0.156427, 0.014814, 0.987578),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[5, 25, 26, 6]),
                normal: Vector3::new(-0.156427, 0.014813, 0.987578),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[6, 26, 27, 7]),
                normal: Vector3::new(-0.453936, 0.014814, 0.890911),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[7, 27, 28, 8]),
                normal: Vector3::new(-0.707029, 0.014814, 0.707029),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[8, 28, 29, 9]),
                normal: Vector3::new(-0.890909, 0.014814, 0.453940),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[9, 29, 30, 10]),
                normal: Vector3::new(-0.987580, 0.014814, 0.156416),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[10, 30, 31, 11]),
                normal: Vector3::new(-0.987580, 0.014814, -0.156416),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[11, 31, 32, 12]),
                normal: Vector3::new(-0.890909, 0.014814, -0.453940),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[12, 32, 33, 13]),
                normal: Vector3::new(-0.707029, 0.014813, -0.707029),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[13, 33, 34, 14]),
                normal: Vector3::new(-0.453936, 0.014813, -0.890911),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[14, 34, 35, 15]),
                normal: Vector3::new(-0.156427, 0.014814, -0.987578),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[15, 35, 36, 16]),
                normal: Vector3::new(0.156427, 0.014813, -0.987578),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[16, 36, 37, 17]),
                normal: Vector3::new(0.453936, 0.014814, -0.890911),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[17, 37, 38, 18]),
                normal: Vector3::new(0.707029, 0.014814, -0.707029),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[18, 38, 39, 19]),
                normal: Vector3::new(0.890909, 0.014814, -0.453940),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[19, 39, 20, 0]),
                normal: Vector3::new(0.987580, 0.014814, -0.156416),
            },
        ];

        let vertices_b = vec![
            Vector3::new(0.666668, -4.000000, 0.000004),
            Vector3::new(0.634039, -4.000000, 0.206017),
            Vector3::new(0.539347, -4.000000, 0.391861),
            Vector3::new(0.391859, -4.000000, 0.539349),
            Vector3::new(0.206013, -4.000000, 0.634041),
            Vector3::new(0.000002, -4.000000, 0.666672),
            Vector3::new(-0.206009, -4.000000, 0.634041),
            Vector3::new(-0.391855, -4.000000, 0.539349),
            Vector3::new(-0.539343, -4.000000, 0.391861),
            Vector3::new(-0.634035, -4.000000, 0.206017),
            Vector3::new(-0.666664, -4.000000, 0.000004),
            Vector3::new(-0.634035, -4.000000, -0.206009),
            Vector3::new(-0.539343, -4.000000, -0.391853),
            Vector3::new(-0.391855, -4.000000, -0.539341),
            Vector3::new(-0.206009, -4.000000, -0.634033),
            Vector3::new(0.000002, -4.000000, -0.666664),
            Vector3::new(0.206013, -4.000000, -0.634033),
            Vector3::new(0.391859, -4.000000, -0.539341),
            Vector3::new(0.539347, -4.000000, -0.391853),
            Vector3::new(0.634039, -4.000000, -0.206009),
            Vector3::new(0.546669, 4.000000, 0.000004),
            Vector3::new(0.519913, 4.000000, 0.168934),
            Vector3::new(0.442265, 4.000000, 0.321327),
            Vector3::new(0.321325, 4.000000, 0.442268),
            Vector3::new(0.168932, 4.000000, 0.519917),
            Vector3::new(0.000002, 4.000000, 0.546669),
            Vector3::new(-0.168928, 4.000000, 0.519917),
            Vector3::new(-0.321321, 4.000000, 0.442268),
            Vector3::new(-0.442261, 4.000000, 0.321327),
            Vector3::new(-0.519909, 4.000000, 0.168934),
            Vector3::new(-0.546665, 4.000000, 0.000004),
            Vector3::new(-0.519909, 4.000000, -0.168926),
            Vector3::new(-0.442261, 4.000000, -0.321320),
            Vector3::new(-0.321321, 4.000000, -0.442261),
            Vector3::new(-0.168928, 4.000000, -0.519909),
            Vector3::new(0.000002, 4.000000, -0.546661),
            Vector3::new(0.168932, 4.000000, -0.519909),
            Vector3::new(0.321325, 4.000000, -0.442257),
            Vector3::new(0.442265, 4.000000, -0.321320),
            Vector3::new(0.519913, 4.000000, -0.168926),
        ];
        let faces_b = vec![
            HullFace {
                vertex_indices: SmallVec::from_slice(&[
                    19, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
                ]),
                normal: Vector3::new(0.0, -1.0, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[
                    20, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21,
                ]),
                normal: Vector3::new(0.0, 1.0, 0.0),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[0, 20, 21, 1]),
                normal: Vector3::new(0.987580, 0.014814, 0.156416),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[1, 21, 22, 2]),
                normal: Vector3::new(0.890909, 0.014814, 0.453940),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[2, 22, 23, 3]),
                normal: Vector3::new(0.707029, 0.014813, 0.707029),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[3, 23, 24, 4]),
                normal: Vector3::new(0.453936, 0.014813, 0.890911),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[4, 24, 25, 5]),
                normal: Vector3::new(0.156427, 0.014814, 0.987578),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[5, 25, 26, 6]),
                normal: Vector3::new(-0.156427, 0.014813, 0.987578),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[6, 26, 27, 7]),
                normal: Vector3::new(-0.453936, 0.014813, 0.890911),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[7, 27, 28, 8]),
                normal: Vector3::new(-0.707029, 0.014814, 0.707029),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[8, 28, 29, 9]),
                normal: Vector3::new(-0.890909, 0.014814, 0.453940),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[9, 29, 30, 10]),
                normal: Vector3::new(-0.987580, 0.014814, 0.156416),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[10, 30, 31, 11]),
                normal: Vector3::new(-0.987580, 0.014814, -0.156416),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[11, 31, 32, 12]),
                normal: Vector3::new(-0.890909, 0.014814, -0.453940),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[12, 32, 33, 13]),
                normal: Vector3::new(-0.707029, 0.014813, -0.707029),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[13, 33, 34, 14]),
                normal: Vector3::new(-0.453936, 0.014813, -0.890911),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[14, 34, 35, 15]),
                normal: Vector3::new(-0.156427, 0.014814, -0.987578),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[15, 35, 36, 16]),
                normal: Vector3::new(0.156427, 0.014813, -0.987578),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[16, 36, 37, 17]),
                normal: Vector3::new(0.453936, 0.014814, -0.890911),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[17, 37, 38, 18]),
                normal: Vector3::new(0.707029, 0.014814, -0.707029),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[18, 38, 39, 19]),
                normal: Vector3::new(0.890909, 0.014814, -0.453940),
            },
            HullFace {
                vertex_indices: SmallVec::from_slice(&[19, 39, 20, 0]),
                normal: Vector3::new(0.987580, 0.014814, -0.156416),
            },
        ];

        let center_a = Point3::new(19.981676, 2.806889, -38.121651);
        let rot_a = UnitQuaternion::from_quaternion(Quaternion::new(
            0.839891, -0.247287, 0.244666, 0.416618,
        ));
        let center_b = Point3::new(19.560469, 2.277419, -36.412941);
        let rot_b = UnitQuaternion::from_quaternion(Quaternion::new(
            0.707800, -0.501969, 0.292429, 0.401909,
        ));

        let (hull_a, shape_a) = hull_view_from(vertices_a, faces_a);
        let (hull_b, shape_b) = hull_view_from(vertices_b, faces_b);

        let va = view_from(center_a, rot_a, &shape_a);
        let vb = view_from(center_b, rot_b, &shape_b);
        let margin = 0.02;

        // Reproduces only with a stale GJK cache seeded with the phantom's
        // contact normal, and shapes passed in b→a order.
        let mut cache = GjkCache {
            last_direction: Some(Vector3::new(-0.2442, 0.0364, 0.9690)),
        };
        let manifold = generate_manifold(&vb, &va, margin, None, Some(&mut cache));
        assert_no_phantom_contacts(
            &manifold, &hull_a, &center_a, &rot_a, &hull_b, &center_b, &rot_b, margin,
        );
    }

    // ─── Hull/OBB vs concave mesh: mixed-normal regression tests ─────

    /// Build an egg-shaped convex hull matching the menhir's geometry.
    fn menhir_hull(half_height: f32, bottom_radius: f32, top_radius: f32) -> ConvexHull {
        use crate::app::spawnables::shared::models::{build_convex_hull, SolidFace};
        use std::f32::consts::{FRAC_PI_2, TAU};

        let segments = 8usize;
        let rings = 6usize;

        let egg_point = |phi: f32, theta: f32| -> Vector3<f32> {
            let t = (phi.sin() + 1.0) * 0.5;
            let r = (bottom_radius + (top_radius - bottom_radius) * t) * phi.cos();
            Vector3::new(r * theta.cos(), half_height * phi.sin(), r * theta.sin())
        };

        let mut vertices = Vec::new();
        vertices.push(Vector3::new(0.0, -half_height, 0.0));

        for ri in 0..rings {
            let phi = -FRAC_PI_2 + (ri as f32 + 1.0) / (rings as f32 + 1.0) * std::f32::consts::PI;
            for si in 0..segments {
                let theta = si as f32 * TAU / segments as f32;
                vertices.push(egg_point(phi, theta));
            }
        }
        let north = vertices.len();
        vertices.push(Vector3::new(0.0, half_height, 0.0));

        let rv = |ri: usize, si: usize| -> usize { 1 + ri * segments + si };
        let mut faces = Vec::new();

        for si in 0..segments {
            let next = (si + 1) % segments;
            faces.push(SolidFace {
                vertex_indices: vec![0, rv(0, next), rv(0, si)],
                opposite_vertex: north,
            });
        }
        for ri in 0..(rings - 1) {
            for si in 0..segments {
                let next = (si + 1) % segments;
                faces.push(SolidFace {
                    vertex_indices: vec![
                        rv(ri, si),
                        rv(ri, next),
                        rv(ri + 1, next),
                        rv(ri + 1, si),
                    ],
                    opposite_vertex: rv(ri, (si + segments / 2) % segments),
                });
            }
        }
        let last = rings - 1;
        for si in 0..segments {
            let next = (si + 1) % segments;
            faces.push(SolidFace {
                vertex_indices: vec![rv(last, si), rv(last, next), north],
                opposite_vertex: 0,
            });
        }

        build_convex_hull(&vertices, &faces)
    }

    /// Captured patch geometry from an in-game pop dump: two flat floor faces
    /// at y=-3 and two sloped faces forming a step.
    fn pop_replay_patch_minimal() -> FilteredPatch {
        FilteredPatch {
            faces: SmallVec::from_vec(vec![
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(12.0, -3.0, -4.0),
                        Point3::new(10.0, -3.0, -4.0),
                        Point3::new(10.0, -3.0, -2.0),
                        Point3::new(12.0, -3.0, -2.0),
                    ]),
                    normal: Vector3::y(),
                    feature_id: FeatureId::from_face(58),
                },
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(12.0, -3.0, -2.0),
                        Point3::new(10.0, -3.0, -2.0),
                        Point3::new(10.0, -3.0, 0.0),
                        Point3::new(12.0, -3.0, 0.0),
                    ]),
                    normal: Vector3::y(),
                    feature_id: FeatureId::from_face(59),
                },
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(9.0, -2.0, -4.0),
                        Point3::new(10.0, -2.0, -5.0),
                        Point3::new(10.0, -1.0, -6.0),
                        Point3::new(8.0, -1.0, -4.0),
                    ]),
                    normal: Vector3::new(0.577350, 0.577350, 0.577350),
                    feature_id: FeatureId::from_face(35),
                },
                ContactFace {
                    vertices: SmallVec::from_vec(vec![
                        Point3::new(10.0, -3.0, -4.0),
                        Point3::new(10.0, -2.0, -5.0),
                        Point3::new(9.0, -2.0, -4.0),
                    ]),
                    normal: Vector3::new(0.577350, 0.577350, 0.577350),
                    feature_id: FeatureId::from_face(53),
                },
            ]),
            boundary_edges: SmallVec::new(),
        }
    }

    /// Regression test: hull vs concave step terrain should not produce a
    /// manifold with mixed normals from unrelated faces.
    ///
    /// Currently routes through GJK/EPA (no dedicated hull-patch path),
    /// which happens to avoid the mixed-normal issue. When a face-clipping
    /// hull-patch path is (re)introduced, this test should verify that it
    /// also produces consistent normals.
    #[test]
    fn hull_pop_replay_minimal_manifold_should_not_mix_normals() {
        let hull = menhir_hull(4.0, 1.8, 1.0);
        let shape = ColliderShape::ConvexHull {
            hull: std::sync::Arc::new(hull),
        };
        let patch = pop_replay_patch_minimal();
        let center = Point3::new(10.586787, -1.690402, -4.410592);
        let rot = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            0.335174, 0.163065, -0.581374, 0.723237,
        ));
        let margin = 0.02;

        let view = view_from(center, rot, &shape);
        let manifold = generate_mesh_manifold(&view, &patch, margin);
        assert!(
            !manifold.is_empty(),
            "Replay case should produce contacts for analysis"
        );

        let base = manifold.points[0].raw_normal.normalize();
        for (i, cp) in manifold.points.iter().enumerate() {
            let d = base.dot(&cp.raw_normal.normalize());
            assert!(
                d > 0.95,
                "SAT-consistent manifold should keep one contact direction. \
                 Contact {i} has mixed normal {:?} vs base {:?} (dot={d:.4})",
                cp.raw_normal,
                base
            );
        }
    }
}
