//! OBB vs triangle collision using the Separating Axis Theorem (SAT).
//!
//! Tests 13 candidate axes (3 OBB face normals, 1 triangle normal, 9 edge-edge
//! cross products). Returns contact points on the minimum-penetration axis.

use nalgebra::{Point3, Vector3};

use super::obb::Obb;
use crate::collision::Triangle;

/// Contact from OBB-triangle intersection.
#[derive(Debug, Clone)]
pub struct ObbTriangleContact {
    /// World-space contact point.
    pub point: Point3<f32>,
    /// Contact normal pointing away from the triangle surface (toward the OBB).
    pub normal: Vector3<f32>,
    /// Penetration depth (positive = overlapping, zero = touching).
    pub depth: f32,
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
                });
            }
        }
    }

    let tri_vertices = [tri.v0, tri.v1, tri.v2];
    for v in &tri_vertices {
        if point_inside_obb(*v, obb) {
            contacts.push(ObbTriangleContact {
                point: *v,
                normal,
                depth,
            });
        }
    }

    if contacts.is_empty() {
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
    }
}
