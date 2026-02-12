use nalgebra::{Point3, Vector3};

use crate::physics::collision::obb::Obb;
use crate::physics::pipeline::contact_reducer::ContactReducer;
use crate::physics::pipeline::solver::ContactConstraint;

/// Stabilize coplanar clusters for box contacts and reduce to max points.
pub fn stabilize_coplanar_box_groups(
    obb: &Obb,
    contacts: &[ContactConstraint],
    contact_margin: f32,
    max_points: usize,
    normal_dot_threshold: f32,
    plane_thickness: f32,
) -> Option<Vec<ContactConstraint>> {
    if contacts.len() < 2 {
        return None;
    }

    let groups = group_coplanar_contacts(contacts, normal_dot_threshold, plane_thickness);
    if groups.iter().all(|g| g.contacts.len() == 1) {
        return None;
    }

    let mut stabilized = Vec::new();
    for group in groups {
        if group.contacts.len() == 1 {
            stabilized.push(group.contacts[0].clone());
            continue;
        }
        let stable = stabilize_coplanar_box_contacts(obb, &group.contacts, contact_margin)?;
        stabilized.extend(stable);
    }

    if stabilized.len() > max_points {
        let reducer = ContactReducer::new(max_points);
        return Some(reducer.reduce(stabilized));
    }

    Some(stabilized)
}

/// Stabilize coplanar clusters for sphere contacts and reduce to max points.
pub fn stabilize_coplanar_sphere_groups(
    center: Point3<f32>,
    radius: f32,
    contacts: &[ContactConstraint],
    contact_margin: f32,
    max_points: usize,
    normal_dot_threshold: f32,
    plane_thickness: f32,
) -> Option<Vec<ContactConstraint>> {
    if contacts.len() < 2 {
        return None;
    }

    let groups = group_coplanar_contacts(contacts, normal_dot_threshold, plane_thickness);
    if groups.iter().all(|g| g.contacts.len() == 1) {
        return None;
    }

    let mut stabilized = Vec::new();
    for group in groups {
        if group.contacts.len() == 1 {
            stabilized.push(group.contacts[0].clone());
            continue;
        }
        let stable =
            stabilize_coplanar_sphere_contacts(center, radius, &group.contacts, contact_margin)?;
        stabilized.extend(stable);
    }

    if stabilized.len() > max_points {
        let reducer = ContactReducer::new(max_points);
        return Some(reducer.reduce(stabilized));
    }

    Some(stabilized)
}

struct CoplanarGroup {
    normal_sum: Vector3<f32>,
    point_sum: Vector3<f32>,
    contacts: Vec<ContactConstraint>,
}

fn group_coplanar_contacts(
    contacts: &[ContactConstraint],
    normal_dot_threshold: f32,
    plane_thickness: f32,
) -> Vec<CoplanarGroup> {
    let mut groups: Vec<CoplanarGroup> = Vec::new();

    for contact in contacts {
        let mut best_idx = None;
        for (idx, group) in groups.iter().enumerate() {
            let normal = if group.normal_sum.magnitude_squared() > 1e-6 {
                group.normal_sum.normalize()
            } else {
                contact.normal
            };
            let point = Point3::from(group.point_sum / group.contacts.len() as f32);
            let dot = contact.normal.dot(&normal);
            let dist = (contact.point - point).dot(&normal).abs();
            if dot >= normal_dot_threshold && dist <= plane_thickness {
                best_idx = Some(idx);
                break;
            }
        }

        if let Some(idx) = best_idx {
            let group = &mut groups[idx];
            group.normal_sum += contact.normal;
            group.point_sum += contact.point.coords;
            group.contacts.push(contact.clone());
        } else {
            groups.push(CoplanarGroup {
                normal_sum: contact.normal,
                point_sum: contact.point.coords,
                contacts: vec![contact.clone()],
            });
        }
    }

    groups
}

fn stabilize_coplanar_box_contacts(
    obb: &Obb,
    contacts: &[ContactConstraint],
    contact_margin: f32,
) -> Option<Vec<ContactConstraint>> {
    if contacts.len() < 2 {
        return None;
    }

    let mut normal_sum = Vector3::zeros();
    let mut point_sum = Vector3::zeros();
    let mut avg_depth = 0.0;
    let mut avg_raw = 0.0;
    for c in contacts {
        normal_sum += c.normal;
        point_sum += c.point.coords;
        avg_depth += c.depth;
        avg_raw += c.raw_depth;
    }
    let normal_len = normal_sum.magnitude();
    if normal_len < 1e-6 {
        return None;
    }
    let normal = normal_sum / normal_len;
    let plane_point = Point3::from(point_sum / contacts.len() as f32);
    avg_depth /= contacts.len() as f32;
    avg_raw /= contacts.len() as f32;

    let mut min_dot = 1.0f32;
    let mut min_plane = f32::INFINITY;
    let mut max_plane = f32::NEG_INFINITY;
    for c in contacts {
        min_dot = min_dot.min(c.normal.dot(&normal));
        let dist = (c.point - plane_point).dot(&normal);
        min_plane = min_plane.min(dist);
        max_plane = max_plane.max(dist);
    }
    if min_dot < 0.98 {
        return None;
    }
    if (max_plane - min_plane).abs() > contact_margin * 0.5 {
        return None;
    }

    let corners = obb.corners();
    let mut min_proj = f32::INFINITY;
    for corner in &corners {
        min_proj = min_proj.min(corner.coords.dot(&normal));
    }
    let face_eps = 1e-3;
    let face_corners: Vec<_> = corners
        .iter()
        .cloned()
        .filter(|corner| (corner.coords.dot(&normal) - min_proj).abs() <= face_eps)
        .collect();
    if face_corners.len() != 4 {
        return None;
    }

    let base = &contacts[0];
    let mut stabilized = Vec::with_capacity(face_corners.len());
    for corner in face_corners {
        let dist = (corner - plane_point).dot(&normal);
        let projected = corner - normal * dist;
        stabilized.push(ContactConstraint {
            body_a: base.body_a,
            body_b: base.body_b,
            collider_a: base.collider_a,
            collider_b: base.collider_b,
            point: projected,
            normal,
            raw_normal: normal,
            depth: avg_depth.max(0.0),
            raw_depth: avg_raw,
            restitution: base.restitution,
            friction: base.friction,
            warm_normal_impulse: 0.0,
            warm_tangent_impulse: [0.0, 0.0],
        });
    }

    Some(stabilized)
}

fn stabilize_coplanar_sphere_contacts(
    center: Point3<f32>,
    radius: f32,
    contacts: &[ContactConstraint],
    contact_margin: f32,
) -> Option<Vec<ContactConstraint>> {
    if contacts.len() < 2 {
        return None;
    }

    let mut normal_sum = Vector3::zeros();
    let mut point_sum = Vector3::zeros();
    let mut avg_depth = 0.0;
    let mut avg_raw = 0.0;
    for c in contacts {
        normal_sum += c.normal;
        point_sum += c.point.coords;
        avg_depth += c.depth;
        avg_raw += c.raw_depth;
    }
    let normal_len = normal_sum.magnitude();
    if normal_len < 1e-6 {
        return None;
    }
    let normal = normal_sum / normal_len;
    let plane_point = Point3::from(point_sum / contacts.len() as f32);
    avg_depth /= contacts.len() as f32;
    avg_raw /= contacts.len() as f32;

    let mut min_dot = 1.0f32;
    let mut min_plane = f32::INFINITY;
    let mut max_plane = f32::NEG_INFINITY;
    for c in contacts {
        min_dot = min_dot.min(c.normal.dot(&normal));
        let dist = (c.point - plane_point).dot(&normal);
        min_plane = min_plane.min(dist);
        max_plane = max_plane.max(dist);
    }
    if min_dot < 0.98 {
        return None;
    }
    if (max_plane - min_plane).abs() > contact_margin * 0.5 {
        return None;
    }

    let base = &contacts[0];
    let point = center - normal * radius;
    Some(vec![ContactConstraint {
        body_a: base.body_a,
        body_b: base.body_b,
        collider_a: base.collider_a,
        collider_b: base.collider_b,
        point,
        normal,
        raw_normal: normal,
        depth: avg_depth.max(0.0),
        raw_depth: avg_raw,
        restitution: base.restitution,
        friction: base.friction,
        warm_normal_impulse: 0.0,
        warm_tangent_impulse: [0.0, 0.0],
    }])
}
