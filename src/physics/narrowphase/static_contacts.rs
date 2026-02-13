//! Unified narrowphase contact generation for all collider shapes vs static geometry.

use std::collections::HashSet;

use generational_arena::Arena;
use nalgebra::{Point3, Vector3};

use super::adjacency_filter::{fix_internal_edge_normals, fix_internal_vertex_normals};
use super::contact_source::{ContactSource, SourcedContact};
use super::coplanar_stabilizer::{
    stabilize_coplanar_box_groups as stabilize_box_coplanar_groups,
    stabilize_coplanar_sphere_groups as stabilize_sphere_coplanar_groups,
};
use crate::collision::{sphere_triangle_collision_with_feature, AABB};
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

    let mut sourced = Vec::new();
    for (tri_idx, pt) in patch.triangles.iter().enumerate() {
        if let Some((cp, feature)) =
            sphere_triangle_collision_with_feature(center, query_radius, &pt.triangle)
        {
            let point = center - cp.normal * (query_radius - cp.depth);
            let raw_depth = cp.depth - contact_margin;
            let solver_depth = raw_depth.max(0.0);
            sourced.push(SourcedContact {
                constraint: ContactConstraint {
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
                },
                source: ContactSource {
                    triangle_idx: tri_idx as u32,
                    feature,
                },
            });
        }
    }

    let sourced = fix_internal_vertex_normals(
        fix_internal_edge_normals(sourced, &patch, normal_dot_threshold.max(0.95)),
        &patch,
        normal_dot_threshold.max(0.95),
    );

    let sphere_contacts: Vec<_> = sourced.into_iter().map(|c| c.constraint).collect();
    let stabilized = stabilize_sphere_coplanar_groups(
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
    let mut sourced = Vec::new();
    for (tri_idx, pt) in patch.triangles.iter().enumerate() {
        for c in obb_triangle_contacts(&obb, &pt.triangle) {
            let raw_depth = c.depth - contact_margin;
            let solver_depth = raw_depth.max(0.0);
            sourced.push(SourcedContact {
                constraint: ContactConstraint {
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
                },
                source: ContactSource {
                    triangle_idx: tri_idx as u32,
                    feature: c.feature,
                },
            });
        }
    }

    let sourced = fix_internal_vertex_normals(
        fix_internal_edge_normals(sourced, &patch, normal_dot_threshold.max(0.95)),
        &patch,
        normal_dot_threshold.max(0.95),
    );

    let box_contacts: Vec<_> = sourced.into_iter().map(|c| c.constraint).collect();
    if let Some(stable) = stabilize_box_coplanar_groups(
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

            let mut sourced = Vec::new();
            for (tri_idx, pt) in patch.triangles.iter().enumerate() {
                if let Some((cp, feature)) = sphere_triangle_collision_with_feature(
                    predicted_center,
                    query_radius,
                    &pt.triangle,
                ) {
                    let point = predicted_center - cp.normal * (query_radius - cp.depth);
                    sourced.push(SourcedContact {
                        constraint: ContactConstraint {
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
                        },
                        source: ContactSource {
                            triangle_idx: tri_idx as u32,
                            feature,
                        },
                    });
                }
            }
            let sourced = fix_internal_vertex_normals(
                fix_internal_edge_normals(sourced, &patch, normal_dot_threshold.max(0.95)),
                &patch,
                normal_dot_threshold.max(0.95),
            );
            let contacts: Vec<_> = sourced.into_iter().map(|c| c.constraint).collect();
            let stabilized = stabilize_sphere_coplanar_groups(
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
            let mut sourced = Vec::new();
            for (tri_idx, pt) in patch.triangles.iter().enumerate() {
                for c in obb_triangle_contacts(&obb, &pt.triangle) {
                    sourced.push(SourcedContact {
                        constraint: ContactConstraint {
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
                        },
                        source: ContactSource {
                            triangle_idx: tri_idx as u32,
                            feature: c.feature,
                        },
                    });
                }
            }
            let sourced = fix_internal_vertex_normals(
                fix_internal_edge_normals(sourced, &patch, normal_dot_threshold.max(0.95)),
                &patch,
                normal_dot_threshold.max(0.95),
            );
            let contacts: Vec<_> = sourced.into_iter().map(|c| c.constraint).collect();
            if let Some(stable) = stabilize_box_coplanar_groups(
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
