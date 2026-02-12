//! OBB vs triangle collision using the Separating Axis Theorem (SAT).
//!
//! Tests 13 candidate axes (3 OBB face normals, 1 triangle normal, 9 edge-edge
//! cross products). Returns contact points on the minimum-penetration axis.

use nalgebra::{Point3, Vector3};

use super::obb::Obb;
use crate::collision::Triangle;
use crate::physics::ContactFeature;

/// Contact from OBB-triangle intersection.
#[derive(Debug, Clone)]
pub struct ObbTriangleContact {
    /// World-space contact point.
    pub point: Point3<f32>,
    /// Contact normal pointing away from the triangle surface (toward the OBB).
    pub normal: Vector3<f32>,
    /// Penetration depth (positive = overlapping, zero = touching).
    pub depth: f32,
    /// Feature path that produced this contact.
    pub feature: ContactFeature,
}

const AXIS_EPS: f32 = 1e-6;
const SEPARATION_EPS: f32 = 1e-5;
const PLANE_EPS: f32 = 1e-4;

fn project_triangle(axis: &Vector3<f32>, tri: &Triangle) -> (f32, f32) {
    let p0 = tri.v0.coords.dot(axis);
    let p1 = tri.v1.coords.dot(axis);
    let p2 = tri.v2.coords.dot(axis);
    let min = p0.min(p1.min(p2));
    let max = p0.max(p1.max(p2));
    (min, max)
}

fn point_in_triangle(point: Point3<f32>, tri: &Triangle, normal: &Vector3<f32>) -> bool {
    let e0 = tri.v1 - tri.v0;
    let e1 = tri.v2 - tri.v1;
    let e2 = tri.v0 - tri.v2;
    let c0 = e0.cross(&(point - tri.v0));
    let c1 = e1.cross(&(point - tri.v1));
    let c2 = e2.cross(&(point - tri.v2));
    c0.dot(normal) >= -1e-5 && c1.dot(normal) >= -1e-5 && c2.dot(normal) >= -1e-5
}

fn point_inside_obb(point: Point3<f32>, obb: &Obb) -> bool {
    let delta = point - obb.center;
    let axes = obb.axes();
    let local = [
        delta.dot(&axes[0]),
        delta.dot(&axes[1]),
        delta.dot(&axes[2]),
    ];
    local[0].abs() <= obb.half_extents.x + 1e-6
        && local[1].abs() <= obb.half_extents.y + 1e-6
        && local[2].abs() <= obb.half_extents.z + 1e-6
}

fn point_segment_distance_sq(p: Point3<f32>, a: Point3<f32>, b: Point3<f32>) -> f32 {
    let ab = b - a;
    let ab_len_sq = ab.magnitude_squared();
    if ab_len_sq <= AXIS_EPS {
        return (p - a).magnitude_squared();
    }
    let t = ((p - a).dot(&ab) / ab_len_sq).clamp(0.0, 1.0);
    let closest = a + ab * t;
    (p - closest).magnitude_squared()
}

fn barycentric_coordinates(point: Point3<f32>, tri: &Triangle) -> Option<(f32, f32, f32)> {
    let v0 = tri.v1 - tri.v0;
    let v1 = tri.v2 - tri.v0;
    let v2 = point - tri.v0;

    let d00 = v0.dot(&v0);
    let d01 = v0.dot(&v1);
    let d11 = v1.dot(&v1);
    let d20 = v2.dot(&v0);
    let d21 = v2.dot(&v1);
    let denom = d00 * d11 - d01 * d01;
    if denom.abs() <= AXIS_EPS {
        return None;
    }

    let v = (d11 * d20 - d01 * d21) / denom;
    let w = (d00 * d21 - d01 * d20) / denom;
    let u = 1.0 - v - w;
    Some((u, v, w))
}

fn classify_projected_triangle_feature(point: Point3<f32>, tri: &Triangle) -> ContactFeature {
    const BARY_EPS: f32 = 1e-4;
    if let Some((u, v, w)) = barycentric_coordinates(point, tri) {
        let on_u = u.abs() <= BARY_EPS;
        let on_v = v.abs() <= BARY_EPS;
        let on_w = w.abs() <= BARY_EPS;

        if on_v && on_w {
            return ContactFeature::Vertex(0);
        }
        if on_u && on_w {
            return ContactFeature::Vertex(1);
        }
        if on_u && on_v {
            return ContactFeature::Vertex(2);
        }

        // Edge indices match Triangle::edge: 0=v0->v1, 1=v1->v2, 2=v2->v0.
        if on_w {
            return ContactFeature::Edge(0);
        }
        if on_u {
            return ContactFeature::Edge(1);
        }
        if on_v {
            return ContactFeature::Edge(2);
        }

        return ContactFeature::Face;
    }

    // Degenerate triangle fallback: preserve behavior with geometric proximity.
    let d0 = point_segment_distance_sq(point, tri.v0, tri.v1);
    let d1 = point_segment_distance_sq(point, tri.v1, tri.v2);
    let d2 = point_segment_distance_sq(point, tri.v2, tri.v0);
    if d0 <= d1 && d0 <= d2 {
        ContactFeature::Edge(0)
    } else if d1 <= d2 {
        ContactFeature::Edge(1)
    } else {
        ContactFeature::Edge(2)
    }
}

fn edge_edge_contact_point(
    obb: &Obb,
    tri: &Triangle,
    normal: &Vector3<f32>,
    depth: f32,
) -> Option<(Point3<f32>, u8)> {
    if depth <= 0.0 {
        return None;
    }

    let obb_edges = obb_edges(obb);
    let tri_edges = [(tri.v0, tri.v1), (tri.v1, tri.v2), (tri.v2, tri.v0)];

    let mut best_dist_sq = f32::INFINITY;
    let mut best_point = None;
    let mut best_tri_edge: u8 = 0;
    for (a0, a1) in &obb_edges {
        for (edge_idx, (b0, b1)) in tri_edges.iter().enumerate() {
            let (pa, pb) = segment_segment_closest_points(*a0, *a1, *b0, *b1);
            let delta = pb - pa;
            let sep = delta.dot(normal).abs();
            if sep > depth + PLANE_EPS {
                continue;
            }
            let dist_sq = delta.magnitude_squared();
            if dist_sq < best_dist_sq {
                best_dist_sq = dist_sq;
                best_point = Some(Point3::from((pa.coords + pb.coords) * 0.5));
                best_tri_edge = edge_idx as u8;
            }
        }
    }

    best_point.map(|p| (p, best_tri_edge))
}

fn obb_edges(obb: &Obb) -> [(Point3<f32>, Point3<f32>); 12] {
    let c = obb.corners();
    [
        (c[0], c[1]),
        (c[1], c[2]),
        (c[2], c[3]),
        (c[3], c[0]),
        (c[4], c[5]),
        (c[5], c[6]),
        (c[6], c[7]),
        (c[7], c[4]),
        (c[0], c[4]),
        (c[1], c[5]),
        (c[2], c[6]),
        (c[3], c[7]),
    ]
}

fn segment_segment_closest_points(
    p1: Point3<f32>,
    q1: Point3<f32>,
    p2: Point3<f32>,
    q2: Point3<f32>,
) -> (Point3<f32>, Point3<f32>) {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(&d1);
    let e = d2.dot(&d2);
    let f = d2.dot(&r);

    if a <= AXIS_EPS && e <= AXIS_EPS {
        return (p1, p2);
    }

    if a <= AXIS_EPS {
        let t = (f / e).clamp(0.0, 1.0);
        return (p1, p2 + d2 * t);
    }

    let c = d1.dot(&r);
    if e <= AXIS_EPS {
        let s = (-c / a).clamp(0.0, 1.0);
        return (p1 + d1 * s, p2);
    }

    let b = d1.dot(&d2);
    let denom = a * e - b * b;
    let mut s = if denom.abs() > AXIS_EPS {
        ((b * f - c * e) / denom).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let mut t = (b * s + f) / e;

    if t < 0.0 {
        t = 0.0;
        s = (-c / a).clamp(0.0, 1.0);
    } else if t > 1.0 {
        t = 1.0;
        s = ((b - c) / a).clamp(0.0, 1.0);
    }

    (p1 + d1 * s, p2 + d2 * t)
}

/// Test an OBB against a triangle using SAT.
///
/// Returns zero or more contacts. Normal always points away from the triangle
/// (toward the OBB center). Depth is the minimum translational distance to
/// separate the shapes along the normal.
///
/// # Arguments
/// * `obb` - The oriented bounding box in world space
/// * `tri` - The triangle in world space
pub fn obb_triangle_contacts(obb: &Obb, tri: &Triangle) -> Vec<ObbTriangleContact> {
    let axes = obb.axes();
    let tri_normal = tri.normal();
    let tri_center = Point3::from((tri.v0.coords + tri.v1.coords + tri.v2.coords) / 3.0);
    let center_dir = obb.center - tri_center;

    let mut best_axis = Vector3::zeros();
    let mut best_depth = f32::MAX;
    let mut best_is_edge_edge = false;
    let mut plane_normal = None;
    let mut plane_depth = 0.0;

    for axis in &axes {
        let mut axis = *axis;
        let len_sq = axis.magnitude_squared();
        if len_sq < AXIS_EPS {
            continue;
        }
        axis /= len_sq.sqrt();
        let (tri_min, tri_max) = project_triangle(&axis, tri);
        let tri_span = tri_max - tri_min;
        let center_proj = obb.center.coords.dot(&axis);
        let radius = obb.project_half_extent(&axis);
        let obb_min = center_proj - radius;
        let obb_max = center_proj + radius;
        let overlap = tri_max.min(obb_max) - tri_min.max(obb_min);
        if overlap < -SEPARATION_EPS {
            return Vec::new();
        }
        if axis.dot(&center_dir) < 0.0 {
            axis = -axis;
        }
        if tri_span > AXIS_EPS && overlap < best_depth {
            best_depth = overlap;
            best_axis = axis;
            best_is_edge_edge = false;
        }
    }

    if tri_normal.magnitude_squared() > AXIS_EPS {
        let axis = tri_normal.normalize();
        let signed_distance = (obb.center - tri.v0).dot(&axis);
        let proj = obb.project_half_extent(&axis);
        if signed_distance.abs() > proj + SEPARATION_EPS {
            return Vec::new();
        }
        let depth = proj - signed_distance.abs();
        plane_depth = depth.max(0.0);
        let oriented = if signed_distance >= 0.0 { axis } else { -axis };
        plane_normal = Some(oriented);
        if depth < best_depth {
            best_depth = depth;
            best_axis = oriented;
            best_is_edge_edge = false;
        }
    }

    let tri_edges = [tri.v1 - tri.v0, tri.v2 - tri.v1, tri.v0 - tri.v2];
    let tri_axis = if tri_normal.magnitude_squared() > AXIS_EPS {
        Some(tri_normal.normalize())
    } else {
        None
    };
    for axis in &axes {
        for edge in &tri_edges {
            let mut cross = axis.cross(edge);
            let len_sq = cross.magnitude_squared();
            if len_sq < AXIS_EPS {
                continue;
            }
            cross /= len_sq.sqrt();
            if let Some(tri_axis) = tri_axis {
                if cross.dot(&tri_axis).abs() > 1.0 - 1e-4 {
                    continue;
                }
            }
            let (tri_min, tri_max) = project_triangle(&cross, tri);
            let tri_span = tri_max - tri_min;
            let center_proj = obb.center.coords.dot(&cross);
            let radius = obb.project_half_extent(&cross);
            let obb_min = center_proj - radius;
            let obb_max = center_proj + radius;
            let overlap = tri_max.min(obb_max) - tri_min.max(obb_min);
            if overlap < -SEPARATION_EPS {
                return Vec::new();
            }
            if cross.dot(&center_dir) < 0.0 {
                cross = -cross;
            }
            if tri_span > AXIS_EPS && overlap < best_depth {
                best_depth = overlap;
                best_axis = cross;
                best_is_edge_edge = true;
            }
        }
    }

    if best_depth == f32::MAX {
        return Vec::new();
    }

    let normal = plane_normal.unwrap_or_else(|| best_axis.normalize());
    let plane_point = tri.v0;
    let depth = if plane_normal.is_some() {
        plane_depth
    } else {
        best_depth
    };
    let mut contacts = Vec::new();

    for corner in obb.corners() {
        let dist = (corner - plane_point).dot(&normal);
        if dist <= PLANE_EPS {
            let projected = corner - normal * dist;
            if point_in_triangle(projected, tri, &tri_normal) {
                contacts.push(ObbTriangleContact {
                    point: projected,
                    normal,
                    depth,
                    feature: classify_projected_triangle_feature(projected, tri),
                });
            }
        }
    }

    let tri_vertices = [tri.v0, tri.v1, tri.v2];
    for (idx, v) in tri_vertices.iter().enumerate() {
        if point_inside_obb(*v, obb) {
            contacts.push(ObbTriangleContact {
                point: *v,
                normal,
                depth,
                feature: ContactFeature::Vertex(idx as u8),
            });
        }
    }

    if contacts.is_empty() {
        if best_is_edge_edge {
            if let Some((edge_point, tri_edge)) = edge_edge_contact_point(obb, tri, &normal, best_depth) {
                contacts.push(ObbTriangleContact {
                    point: edge_point,
                    normal,
                    depth,
                    feature: ContactFeature::Edge(tri_edge),
                });
                return contacts;
            }
        }
        let mut best_corner = obb.corners()[0];
        let mut best_dist = (best_corner - plane_point).dot(&normal);
        for corner in obb.corners() {
            let dist = (corner - plane_point).dot(&normal);
            if dist < best_dist {
                best_dist = dist;
                best_corner = corner;
            }
        }
        if best_dist <= PLANE_EPS {
            let projected = best_corner - normal * best_dist;
            if point_in_triangle(projected, tri, &tri_normal) {
                contacts.push(ObbTriangleContact {
                    point: projected,
                    normal,
                    depth,
                    feature: classify_projected_triangle_feature(projected, tri),
                });
            }
        }
    }

    contacts
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::UnitQuaternion;

    fn point_near(a: Point3<f32>, b: Point3<f32>, eps: f32) -> bool {
        (a - b).magnitude_squared() <= eps * eps
    }

    fn assert_contacts_match(contacts: &[ObbTriangleContact], expected: &[Point3<f32>], eps: f32) {
        let mut matched = vec![false; expected.len()];
        for contact in contacts {
            let mut found = false;
            for (idx, exp) in expected.iter().enumerate() {
                if !matched[idx] && point_near(contact.point, *exp, eps) {
                    matched[idx] = true;
                    found = true;
                    break;
                }
            }
            assert!(
                found,
                "Unexpected contact at {:?}, expected {:?}",
                contact.point, expected
            );
        }
        assert!(
            matched.iter().all(|m| *m),
            "Missing expected contacts. Expected {:?}, got {:?}",
            expected,
            contacts.iter().map(|c| c.point).collect::<Vec<_>>()
        );
        assert!(
            contacts.len() == expected.len(),
            "Expected {} contacts, got {}",
            expected.len(),
            contacts.len()
        );
    }

    fn floor_triangle() -> Triangle {
        Triangle::new(
            Point3::new(-10.0, 0.0, -10.0),
            Point3::new(10.0, 0.0, -10.0),
            Point3::new(0.0, 0.0, 10.0),
        )
    }

    #[test]
    fn box_resting_on_triangle() {
        let obb = Obb::new(
            Point3::new(0.0, 0.5, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(0.5, 0.5, 0.5),
        );
        let tri = floor_triangle();
        let contacts = obb_triangle_contacts(&obb, &tri);
        assert!(
            !contacts.is_empty(),
            "Should have contacts when box bottom touches floor"
        );
        let expected = [
            Point3::new(-0.5, 0.0, -0.5),
            Point3::new(0.5, 0.0, -0.5),
            Point3::new(0.5, 0.0, 0.5),
            Point3::new(-0.5, 0.0, 0.5),
        ];
        assert_contacts_match(&contacts, &expected, 1e-4);
        for c in &contacts {
            assert!(
                c.normal.y > 0.9,
                "Normal should point up, got {:?}",
                c.normal
            );
            assert!(c.depth >= 0.0);
        }
    }

    #[test]
    fn box_above_triangle_no_contact() {
        let obb = Obb::new(
            Point3::new(0.0, 2.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(0.5, 0.5, 0.5),
        );
        let contacts = obb_triangle_contacts(&obb, &floor_triangle());
        assert!(contacts.is_empty());
    }

    #[test]
    fn box_penetrating_triangle() {
        let obb = Obb::new(
            Point3::new(0.0, 0.3, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(0.5, 0.5, 0.5),
        );
        let tri = floor_triangle();
        let contacts = obb_triangle_contacts(&obb, &tri);
        assert!(!contacts.is_empty());
        let expected = [
            Point3::new(-0.5, 0.0, -0.5),
            Point3::new(0.5, 0.0, -0.5),
            Point3::new(0.5, 0.0, 0.5),
            Point3::new(-0.5, 0.0, 0.5),
        ];
        assert_contacts_match(&contacts, &expected, 1e-4);
        for c in &contacts {
            assert!(
                c.depth > 0.1,
                "Box bottom at y=-0.2 vs floor at y=0 should give ~0.2 depth, got {}",
                c.depth
            );
        }
    }

    #[test]
    fn rotated_box_on_triangle() {
        let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f32::consts::FRAC_PI_4);
        let obb = Obb::new(Point3::new(0.0, 0.7, 0.0), rot, Vector3::new(0.5, 0.5, 0.5));
        let tri = floor_triangle();
        let contacts = obb_triangle_contacts(&obb, &tri);
        assert!(
            !contacts.is_empty(),
            "45-deg rotated box should contact floor"
        );
        let expected = [Point3::new(0.0, 0.0, -0.5), Point3::new(0.0, 0.0, 0.5)];
        assert_contacts_match(&contacts, &expected, 1e-4);
    }

    #[test]
    fn rotated_box_above_triangle_no_contact() {
        let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f32::consts::FRAC_PI_4);
        let obb = Obb::new(Point3::new(0.0, 0.8, 0.0), rot, Vector3::new(0.5, 0.5, 0.5));
        let contacts = obb_triangle_contacts(&obb, &floor_triangle());
        assert!(
            contacts.is_empty(),
            "Rotated box above floor should have no contacts"
        );
        assert_contacts_match(&contacts, &[], 1e-4);
    }

    #[test]
    fn box_off_triangle_edge() {
        let obb = Obb::new(
            Point3::new(100.0, 0.5, 100.0),
            UnitQuaternion::identity(),
            Vector3::new(0.5, 0.5, 0.5),
        );
        let contacts = obb_triangle_contacts(&obb, &floor_triangle());
        assert!(
            contacts.is_empty(),
            "Box far from triangle should have no contacts"
        );
    }

    #[test]
    fn small_triangle_inside_box() {
        let obb = Obb::new(
            Point3::new(0.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(5.0, 5.0, 5.0),
        );
        let tri = Triangle::new(
            Point3::new(-0.5, 0.0, -0.5),
            Point3::new(0.5, 0.0, -0.5),
            Point3::new(0.0, 0.0, 0.5),
        );
        let contacts = obb_triangle_contacts(&obb, &tri);
        assert!(
            !contacts.is_empty(),
            "Triangle inside box should produce contacts"
        );
        let expected = [tri.v0, tri.v1, tri.v2];
        assert_contacts_match(&contacts, &expected, 1e-4);
        for (idx, c) in contacts.iter().enumerate() {
            assert_eq!(c.feature, ContactFeature::Vertex(idx as u8));
        }
    }

    #[test]
    fn box_resting_contacts_are_face_features() {
        let obb = Obb::new(
            Point3::new(0.0, 0.5, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(0.5, 0.5, 0.5),
        );
        let contacts = obb_triangle_contacts(&obb, &floor_triangle());
        assert!(!contacts.is_empty());
        for c in &contacts {
            assert_eq!(c.feature, ContactFeature::Face);
        }
    }

    #[test]
    fn segment_segment_closest_points_skew() {
        let p1 = Point3::new(0.0, 0.0, 0.0);
        let q1 = Point3::new(1.0, 0.0, 0.0);
        let p2 = Point3::new(0.5, -1.0, 0.0);
        let q2 = Point3::new(0.5, 1.0, 0.0);

        let (c1, c2) = segment_segment_closest_points(p1, q1, p2, q2);

        assert!((c1 - Point3::new(0.5, 0.0, 0.0)).magnitude() < 1e-6);
        assert!((c2 - Point3::new(0.5, 0.0, 0.0)).magnitude() < 1e-6);
    }

    #[test]
    fn segment_segment_closest_points_parallel() {
        let p1 = Point3::new(0.0, 0.0, 0.0);
        let q1 = Point3::new(1.0, 0.0, 0.0);
        let p2 = Point3::new(0.0, 1.0, 0.0);
        let q2 = Point3::new(1.0, 1.0, 0.0);

        let (c1, c2) = segment_segment_closest_points(p1, q1, p2, q2);

        assert!((c1 - Point3::new(0.0, 0.0, 0.0)).magnitude() < 1e-6);
        assert!((c2 - Point3::new(0.0, 1.0, 0.0)).magnitude() < 1e-6);
    }
}
