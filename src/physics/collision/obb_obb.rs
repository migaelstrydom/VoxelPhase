//! OBB vs OBB collision using SAT with Sutherland-Hodgman face clipping.
//!
//! Tests 15 candidate axes (3+3 face normals, 9 edge-edge cross products).
//! Generates a contact manifold via face clipping, reduced to at most 4 points.

use nalgebra::{Point3, Vector3};

use super::obb::Obb;

/// Contact from OBB-OBB intersection.
#[derive(Debug, Clone)]
pub struct ObbObbContact {
    /// World-space contact point.
    pub point: Point3<f32>,
    /// Contact normal pointing from OBB A toward OBB B.
    pub normal: Vector3<f32>,
    /// Penetration depth (positive = overlapping).
    pub depth: f32,
}

fn face_vertices(
    obb: &Obb,
    axis_index: usize,
    sign: f32,
) -> (Vec<Point3<f32>>, Point3<f32>, Vector3<f32>, Vector3<f32>, f32, f32) {
    let axes = obb.axes();
    let he = obb.half_extents;
    let (u_idx, v_idx) = match axis_index {
        0 => (1, 2),
        1 => (0, 2),
        _ => (0, 1),
    };
    let normal = axes[axis_index] * sign;
    let center = obb.center + normal * he[axis_index];
    let u = axes[u_idx];
    let v = axes[v_idx];
    let hu = he[u_idx];
    let hv = he[v_idx];
    let vertices = vec![
        center + u * hu + v * hv,
        center - u * hu + v * hv,
        center - u * hu - v * hv,
        center + u * hu - v * hv,
    ];
    (vertices, center, u, v, hu, hv)
}

fn clip_polygon(
    polygon: Vec<Point3<f32>>,
    plane_point: Point3<f32>,
    plane_normal: Vector3<f32>,
) -> Vec<Point3<f32>> {
    if polygon.is_empty() {
        return polygon;
    }
    let mut result = Vec::new();
    for i in 0..polygon.len() {
        let p1 = polygon[i];
        let p2 = polygon[(i + 1) % polygon.len()];
        let d1 = (p1 - plane_point).dot(&plane_normal);
        let d2 = (p2 - plane_point).dot(&plane_normal);
        let inside1 = d1 >= 0.0;
        let inside2 = d2 >= 0.0;
        if inside1 && inside2 {
            result.push(p2);
        } else if inside1 && !inside2 {
            let t = d1 / (d1 - d2);
            result.push(p1 + (p2 - p1) * t);
        } else if !inside1 && inside2 {
            let t = d1 / (d1 - d2);
            result.push(p1 + (p2 - p1) * t);
            result.push(p2);
        }
    }
    result
}

fn reduce_contacts(mut contacts: Vec<ObbObbContact>) -> Vec<ObbObbContact> {
    if contacts.len() <= 4 {
        return contacts;
    }
    contacts.sort_by(|a, b| b.depth.partial_cmp(&a.depth).unwrap_or(std::cmp::Ordering::Equal));
    let mut selected = Vec::new();
    selected.push(contacts.remove(0));
    if contacts.is_empty() {
        return selected;
    }
    let mut farthest_idx = 0;
    let mut farthest_dist = 0.0;
    for (i, c) in contacts.iter().enumerate() {
        let dist = (c.point - selected[0].point).magnitude_squared();
        if dist > farthest_dist {
            farthest_dist = dist;
            farthest_idx = i;
        }
    }
    selected.push(contacts.remove(farthest_idx));
    if contacts.is_empty() {
        return selected;
    }
    let p0 = selected[0].point;
    let p1 = selected[1].point;
    let mut best_area_idx = 0;
    let mut best_area = 0.0;
    for (i, c) in contacts.iter().enumerate() {
        let area = (p1 - p0).cross(&(c.point - p0)).magnitude_squared();
        if area > best_area {
            best_area = area;
            best_area_idx = i;
        }
    }
    selected.push(contacts.remove(best_area_idx));
    if contacts.is_empty() {
        return selected;
    }
    let mut best_spread_idx = 0;
    let mut best_spread = 0.0;
    for (i, c) in contacts.iter().enumerate() {
        let mut min_dist = f32::MAX;
        for s in &selected {
            let dist = (c.point - s.point).magnitude_squared();
            if dist < min_dist {
                min_dist = dist;
            }
        }
        if min_dist > best_spread {
            best_spread = min_dist;
            best_spread_idx = i;
        }
    }
    selected.push(contacts.remove(best_spread_idx));
    selected
}

/// Test two OBBs against each other using SAT.
///
/// Returns up to 4 contact points. Normal points from A toward B.
/// Contact manifold is generated via Sutherland-Hodgman clipping of the
/// reference face against the incident face, then reduced to 4 points
/// for solver stability.
///
/// # Arguments
/// * `a` - First OBB in world space
/// * `b` - Second OBB in world space
pub fn obb_obb_contacts(a: &Obb, b: &Obb) -> Vec<ObbObbContact> {
    let axes_a = a.axes();
    let axes_b = b.axes();
    let center_dir = b.center - a.center;

    let mut best_axis = Vector3::zeros();
    let mut best_depth = f32::MAX;

    for axis in &axes_a {
        let mut axis = *axis;
        let len_sq = axis.magnitude_squared();
        if len_sq < 1e-10 {
            continue;
        }
        axis /= len_sq.sqrt();
        let distance = center_dir.dot(&axis).abs();
        let overlap = a.project_half_extent(&axis) + b.project_half_extent(&axis) - distance;
        if overlap < 0.0 {
            return Vec::new();
        }
        if axis.dot(&center_dir) < 0.0 {
            axis = -axis;
        }
        if overlap < best_depth {
            best_depth = overlap;
            best_axis = axis;
        }
    }

    for axis in &axes_b {
        let mut axis = *axis;
        let len_sq = axis.magnitude_squared();
        if len_sq < 1e-10 {
            continue;
        }
        axis /= len_sq.sqrt();
        let distance = center_dir.dot(&axis).abs();
        let overlap = a.project_half_extent(&axis) + b.project_half_extent(&axis) - distance;
        if overlap < 0.0 {
            return Vec::new();
        }
        if axis.dot(&center_dir) < 0.0 {
            axis = -axis;
        }
        if overlap < best_depth {
            best_depth = overlap;
            best_axis = axis;
        }
    }

    for axis_a in &axes_a {
        for axis_b in &axes_b {
            let mut axis = axis_a.cross(axis_b);
            let len_sq = axis.magnitude_squared();
            if len_sq < 1e-10 {
                continue;
            }
            axis /= len_sq.sqrt();
            let distance = center_dir.dot(&axis).abs();
            let overlap = a.project_half_extent(&axis) + b.project_half_extent(&axis) - distance;
            if overlap < 0.0 {
                return Vec::new();
            }
            if axis.dot(&center_dir) < 0.0 {
                axis = -axis;
            }
            if overlap < best_depth {
                best_depth = overlap;
                best_axis = axis;
            }
        }
    }

    if best_depth == f32::MAX {
        return Vec::new();
    }

    let normal = best_axis.normalize();
    let mut max_a = 0.0;
    let mut axis_a_idx = 0;
    let mut axis_a_sign = 1.0;
    for i in 0..3 {
        let dot = normal.dot(&axes_a[i]);
        if dot.abs() > max_a {
            max_a = dot.abs();
            axis_a_idx = i;
            axis_a_sign = if dot >= 0.0 { 1.0 } else { -1.0 };
        }
    }
    let mut max_b = 0.0;
    for i in 0..3 {
        let dot = normal.dot(&axes_b[i]);
        if dot.abs() > max_b {
            max_b = dot.abs();
        }
    }

    let (ref_obb, inc_obb, ref_axis_idx, ref_axis_sign, ref_normal) = if max_a >= max_b {
        (a, b, axis_a_idx, axis_a_sign, normal)
    } else {
        let ref_normal = -normal;
        let mut ref_idx = 0;
        let mut ref_sign = 1.0;
        let mut best = 0.0;
        for i in 0..3 {
            let dot = ref_normal.dot(&axes_b[i]);
            if dot.abs() > best {
                best = dot.abs();
                ref_idx = i;
                ref_sign = if dot >= 0.0 { 1.0 } else { -1.0 };
            }
        }
        (b, a, ref_idx, ref_sign, ref_normal)
    };

    let (_, ref_center, ref_u, ref_v, ref_hu, ref_hv) =
        face_vertices(ref_obb, ref_axis_idx, ref_axis_sign);
    let mut incident = {
        let inc_axes = inc_obb.axes();
        let mut best_abs = 0.0;
        let mut inc_idx = 0;
        let mut inc_sign = 1.0;
        for i in 0..3 {
            let dot = ref_normal.dot(&inc_axes[i]);
            if dot.abs() > best_abs {
                best_abs = dot.abs();
                inc_idx = i;
                inc_sign = if dot >= 0.0 { -1.0 } else { 1.0 };
            }
        }
        let (verts, _, _, _, _, _) = face_vertices(inc_obb, inc_idx, inc_sign);
        verts
    };

    incident = clip_polygon(incident, ref_center + ref_u * ref_hu, -ref_u);
    incident = clip_polygon(incident, ref_center - ref_u * ref_hu, ref_u);
    incident = clip_polygon(incident, ref_center + ref_v * ref_hv, -ref_v);
    incident = clip_polygon(incident, ref_center - ref_v * ref_hv, ref_v);

    let mut contacts = Vec::new();
    for p in incident {
        let dist = (p - ref_center).dot(&ref_normal);
        if dist <= 0.0 {
            contacts.push(ObbObbContact {
                point: p,
                normal,
                depth: -dist,
            });
        }
    }

    reduce_contacts(contacts)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::UnitQuaternion;

    fn unit_box_at(pos: Point3<f32>) -> Obb {
        Obb::new(pos, UnitQuaternion::identity(), Vector3::new(1.0, 1.0, 1.0))
    }

    #[test]
    fn face_to_face_contact() {
        let a = unit_box_at(Point3::new(0.0, 0.0, 0.0));
        let b = unit_box_at(Point3::new(1.5, 0.0, 0.0));
        let contacts = obb_obb_contacts(&a, &b);
        assert!(!contacts.is_empty());
        for c in &contacts {
            assert!(c.normal.x > 0.9, "Normal should point +X, got {:?}", c.normal);
            assert!((c.depth - 0.5).abs() < 0.1, "Expected ~0.5 depth, got {}", c.depth);
        }
    }

    #[test]
    fn separated_boxes() {
        let a = unit_box_at(Point3::new(0.0, 0.0, 0.0));
        let b = unit_box_at(Point3::new(5.0, 0.0, 0.0));
        let contacts = obb_obb_contacts(&a, &b);
        assert!(contacts.is_empty());
    }

    #[test]
    fn stacked_boxes() {
        let a = unit_box_at(Point3::new(0.0, 0.0, 0.0));
        let b = unit_box_at(Point3::new(0.0, 1.8, 0.0));
        let contacts = obb_obb_contacts(&a, &b);
        assert!(!contacts.is_empty());
        for c in &contacts {
            assert!(c.normal.y > 0.9, "Stacked normal should point +Y");
        }
    }

    #[test]
    fn rotated_box_contact() {
        let a = unit_box_at(Point3::new(0.0, 0.0, 0.0));
        let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_4);
        let b = Obb::new(Point3::new(2.0, 0.0, 0.0), rot, Vector3::new(1.0, 1.0, 1.0));
        let contacts = obb_obb_contacts(&a, &b);
        assert!(!contacts.is_empty(), "Rotated box should overlap");
    }

    #[test]
    fn max_four_contacts() {
        let a = unit_box_at(Point3::new(0.0, 0.0, 0.0));
        let b = unit_box_at(Point3::new(0.0, 1.5, 0.0));
        let contacts = obb_obb_contacts(&a, &b);
        assert!(contacts.len() <= 4, "Should reduce to at most 4 contacts, got {}", contacts.len());
    }
}
