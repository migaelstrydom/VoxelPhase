//! Unified narrowphase contact generation for all collider shapes vs static geometry.

use std::collections::HashSet;

use generational_arena::Arena;
use nalgebra::{Point3, Vector3};

use crate::collision::{sphere_triangle_collision, AABB};
use crate::physics::body::RigidBody;
use crate::physics::collider::{Collider, ColliderShape};
use crate::physics::collision::obb::Obb;
use crate::physics::collision::obb_triangle::obb_triangle_contacts;
use crate::physics::handle::RigidBodyHandle;
use crate::physics::narrowphase::NormalClusterer;
use crate::physics::pipeline::solver::ContactConstraint;
use crate::physics::static_geometry::StaticGeometry;

/// Generate contacts between all non-static colliders and static geometry.
///
/// Dispatches per collider shape:
/// - Sphere: builds AABB query, tests each triangle with sphere-triangle collision
/// - Box: builds AABB query, runs SAT per triangle
///
/// Both paths share sleeping checks, speculative contacts, and normal clustering.
pub fn generate_static_contacts(
    bodies: &Arena<RigidBody>,
    colliders: &Arena<Collider>,
    static_geometry: &dyn StaticGeometry,
    contact_margin: f32,
    dt: f32,
    ccd_threshold: f32,
    enable_speculative_contacts: bool,
    speculative_min_speed: f32,
    speculative_margin_multiplier: f32,
    normal_cluster: crate::physics::narrowphase::NormalClusterConfig,
    sleeping: Option<&HashSet<RigidBodyHandle>>,
) -> Vec<ContactConstraint> {
    let mut contacts = Vec::new();

    for (idx, body) in bodies.iter() {
        if body.is_static() {
            continue;
        }
        let body_handle = RigidBodyHandle(idx);
        if let Some(sleeping) = sleeping {
            if sleeping.contains(&body_handle) {
                continue;
            }
        }

        for collider_handle in body.colliders() {
            let Some(collider) = colliders.get(collider_handle.0) else {
                continue;
            };

            let center = collider.world_center(body.position(), body.rotation());
            let linear_velocity = body.linear_velocity();
            let travel = linear_velocity.magnitude() * dt;

            let mut batch = match collider.shape() {
                ColliderShape::Sphere { radius } => sphere_vs_static(
                    body_handle,
                    *collider_handle,
                    collider,
                    center,
                    *radius,
                    static_geometry,
                    contact_margin,
                    normal_cluster.max_points,
                    normal_cluster.normal_cluster_dot,
                    normal_cluster.point_cluster_distance,
                ),
                ColliderShape::Box { half_extents } => box_vs_static(
                    body_handle,
                    *collider_handle,
                    collider,
                    center,
                    body.rotation(),
                    *half_extents,
                    static_geometry,
                    contact_margin,
                    normal_cluster.max_points,
                    normal_cluster.normal_cluster_dot,
                    normal_cluster.point_cluster_distance,
                ),
            };

            if batch.contacts.len() > 1 && !batch.stable {
                let clusterer = NormalClusterer::from_config(normal_cluster, contact_margin);
                batch.contacts = clusterer.cluster_with_pre_reduction(batch.contacts);
            }
            if batch.contacts.is_empty()
                && is_speculative_candidate(
                    travel,
                    collider.shape().bounding_radius(),
                    ccd_threshold,
                    contact_margin,
                    enable_speculative_contacts,
                    speculative_min_speed,
                    speculative_margin_multiplier,
                )
            {
                let predicted_center = center + linear_velocity * dt;
                batch.contacts = speculative_static_contacts(
                    body_handle,
                    *collider_handle,
                    collider,
                    predicted_center,
                    body.rotation(),
                    static_geometry,
                    contact_margin,
                    normal_cluster.max_points,
                    normal_cluster.normal_cluster_dot,
                    normal_cluster.point_cluster_distance,
                );
            }
            contacts.extend(batch.contacts);
        }
    }

    contacts
}

/// Contact batch with a stability flag for clustering.
struct StaticContactBatch {
    contacts: Vec<ContactConstraint>,
    stable: bool,
}

/// Generate sphere-static contacts, optionally stabilizing coplanar patches.
fn sphere_vs_static(
    body_handle: RigidBodyHandle,
    collider_handle: crate::physics::handle::ColliderHandle,
    collider: &Collider,
    center: Point3<f32>,
    radius: f32,
    static_geometry: &dyn StaticGeometry,
    contact_margin: f32,
    max_points: usize,
    normal_dot_threshold: f32,
    plane_thickness: f32,
) -> StaticContactBatch {
    let query_radius = radius + contact_margin;
    let query = AABB::new(
        Point3::new(
            center.x - query_radius,
            center.y - query_radius,
            center.z - query_radius,
        ),
        Point3::new(
            center.x + query_radius,
            center.y + query_radius,
            center.z + query_radius,
        ),
    );
    let patch = static_geometry.query_region(&query);

    let mut sphere_contacts = Vec::new();
    for pt in &patch.triangles {
        if let Some(cp) = sphere_triangle_collision(center, query_radius, &pt.triangle) {
            let point = center - cp.normal * (query_radius - cp.depth);
            let raw_depth = cp.depth - contact_margin;
            let solver_depth = raw_depth.max(0.0);
            sphere_contacts.push(ContactConstraint {
                body_a: None,
                body_b: body_handle,
                collider_a: None,
                collider_b: Some(collider_handle),
                point,
                normal: cp.normal,
                raw_normal: cp.normal,
                depth: solver_depth,
                raw_depth,
                restitution: collider.material().restitution,
                friction: collider.material().friction,
                warm_normal_impulse: 0.0,
                warm_tangent_impulse: [0.0, 0.0],
            });
        }
    }

    let stabilized = stabilize_coplanar_sphere_groups(
        center,
        radius,
        &sphere_contacts,
        contact_margin,
        max_points,
        normal_dot_threshold,
        plane_thickness,
    );
    if let Some(stable) = stabilized {
        return StaticContactBatch {
            contacts: stable,
            stable: true,
        };
    }

    StaticContactBatch {
        contacts: sphere_contacts,
        stable: false,
    }
}

/// Generate box-static contacts, optionally stabilizing coplanar patches.
fn box_vs_static(
    body_handle: RigidBodyHandle,
    collider_handle: crate::physics::handle::ColliderHandle,
    collider: &Collider,
    center: Point3<f32>,
    rotation: nalgebra::UnitQuaternion<f32>,
    half_extents: Vector3<f32>,
    static_geometry: &dyn StaticGeometry,
    contact_margin: f32,
    max_points: usize,
    normal_dot_threshold: f32,
    plane_thickness: f32,
) -> StaticContactBatch {
    let obb = Obb::new(center, rotation, half_extents);
    let (aabb_min, aabb_max) = obb.enclosing_aabb();
    let margin_vec = Vector3::new(contact_margin, contact_margin, contact_margin);
    let query = AABB::new(aabb_min - margin_vec, aabb_max + margin_vec);

    let patch = static_geometry.query_region(&query);
    let mut box_contacts = Vec::new();
    for pt in &patch.triangles {
        for c in obb_triangle_contacts(&obb, &pt.triangle) {
            let raw_depth = c.depth - contact_margin;
            let solver_depth = raw_depth.max(0.0);
            box_contacts.push(ContactConstraint {
                body_a: None,
                body_b: body_handle,
                collider_a: None,
                collider_b: Some(collider_handle),
                point: c.point,
                normal: c.normal,
                raw_normal: c.normal,
                depth: solver_depth,
                raw_depth,
                restitution: collider.material().restitution,
                friction: collider.material().friction,
                warm_normal_impulse: 0.0,
                warm_tangent_impulse: [0.0, 0.0],
            });
        }
    }

    if let Some(stable) = stabilize_coplanar_box_groups(
        &obb,
        &box_contacts,
        contact_margin,
        max_points,
        normal_dot_threshold,
        plane_thickness,
    ) {
        return StaticContactBatch {
            contacts: stable,
            stable: true,
        };
    }

    StaticContactBatch {
        contacts: box_contacts,
        stable: false,
    }
}

/// Build speculative contacts at a predicted pose for any shape.
fn speculative_static_contacts(
    body_handle: RigidBodyHandle,
    collider_handle: crate::physics::handle::ColliderHandle,
    collider: &Collider,
    predicted_center: Point3<f32>,
    rotation: nalgebra::UnitQuaternion<f32>,
    static_geometry: &dyn StaticGeometry,
    contact_margin: f32,
    max_points: usize,
    normal_dot_threshold: f32,
    plane_thickness: f32,
) -> Vec<ContactConstraint> {
    match collider.shape() {
        ColliderShape::Sphere { radius } => {
            let query_radius = radius + contact_margin;
            let query = AABB::new(
                Point3::new(
                    predicted_center.x - query_radius,
                    predicted_center.y - query_radius,
                    predicted_center.z - query_radius,
                ),
                Point3::new(
                    predicted_center.x + query_radius,
                    predicted_center.y + query_radius,
                    predicted_center.z + query_radius,
                ),
            );
            let patch = static_geometry.query_region(&query);

            let mut contacts = Vec::new();
            for pt in &patch.triangles {
                if let Some(cp) =
                    sphere_triangle_collision(predicted_center, query_radius, &pt.triangle)
                {
                    let point = predicted_center - cp.normal * (query_radius - cp.depth);
                    contacts.push(ContactConstraint {
                        body_a: None,
                        body_b: body_handle,
                        collider_a: None,
                        collider_b: Some(collider_handle),
                        point,
                        normal: cp.normal,
                        raw_normal: cp.normal,
                        depth: 0.0,
                        raw_depth: -contact_margin,
                        restitution: collider.material().restitution,
                        friction: collider.material().friction,
                        warm_normal_impulse: 0.0,
                        warm_tangent_impulse: [0.0, 0.0],
                    });
                }
            }
            let stabilized = stabilize_coplanar_sphere_groups(
                predicted_center,
                *radius,
                &contacts,
                contact_margin,
                max_points,
                normal_dot_threshold,
                plane_thickness,
            );
            stabilized.unwrap_or(contacts)
        }
        ColliderShape::Box { half_extents } => {
            let expanded =
                *half_extents + Vector3::new(contact_margin, contact_margin, contact_margin);
            let obb = Obb::new(predicted_center, rotation, expanded);
            let (aabb_min, aabb_max) = obb.enclosing_aabb();
            let query = AABB::new(aabb_min, aabb_max);
            let patch = static_geometry.query_region(&query);
            let mut contacts = Vec::new();
            for pt in &patch.triangles {
                for c in obb_triangle_contacts(&obb, &pt.triangle) {
                    contacts.push(ContactConstraint {
                        body_a: None,
                        body_b: body_handle,
                        collider_a: None,
                        collider_b: Some(collider_handle),
                        point: c.point,
                        normal: c.normal,
                        raw_normal: c.normal,
                        depth: 0.0,
                        raw_depth: -contact_margin,
                        restitution: collider.material().restitution,
                        friction: collider.material().friction,
                        warm_normal_impulse: 0.0,
                        warm_tangent_impulse: [0.0, 0.0],
                    });
                }
            }
            if let Some(stable) = stabilize_coplanar_box_groups(
                &obb,
                &contacts,
                contact_margin,
                max_points,
                normal_dot_threshold,
                plane_thickness,
            ) {
                return stable;
            }
            contacts
        }
    }
}

/// Collapse coplanar box contacts into a stable face patch.
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

/// Collapse coplanar sphere contacts into a single stable contact.
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

/// Cluster contacts into coplanar groups by normal and plane proximity.
struct CoplanarGroup {
    normal_sum: Vector3<f32>,
    point_sum: Vector3<f32>,
    contacts: Vec<ContactConstraint>,
}

/// Group contacts into coplanar clusters for stabilization.
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

/// Stabilize coplanar clusters for box contacts and reduce to max points.
fn stabilize_coplanar_box_groups(
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
        let reducer = crate::physics::pipeline::contact_reducer::ContactReducer::new(max_points);
        return Some(reducer.reduce(stabilized));
    }

    Some(stabilized)
}

/// Stabilize coplanar clusters for sphere contacts and reduce to max points.
fn stabilize_coplanar_sphere_groups(
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
        let reducer = crate::physics::pipeline::contact_reducer::ContactReducer::new(max_points);
        return Some(reducer.reduce(stabilized));
    }

    Some(stabilized)
}

/// Gate speculative contacts by travel distance and CCD threshold.
fn is_speculative_candidate(
    travel: f32,
    radius: f32,
    ccd_threshold: f32,
    contact_margin: f32,
    enable_speculative_contacts: bool,
    speculative_min_speed: f32,
    speculative_margin_multiplier: f32,
) -> bool {
    if !enable_speculative_contacts {
        return false;
    }
    if travel < speculative_min_speed {
        return false;
    }
    let margin_gate = contact_margin * speculative_margin_multiplier;
    travel > margin_gate && travel <= radius * ccd_threshold
}
