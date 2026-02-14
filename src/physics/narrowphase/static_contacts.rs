//! Unified narrowphase contact generation for all collider shapes vs static geometry.
//!
//! Uses the manifold-first mesh pipeline: seam filter merges coplanar triangles
//! into polygonal contact faces, then shape-specific patch routines generate
//! manifolds directly from the merged geometry. This replaces the old per-triangle
//! contact generation + adjacency_filter + coplanar_stabilizer approach.

use std::collections::HashSet;

use generational_arena::Arena;
use nalgebra::{Point3, Vector3};

use crate::collision::contact::ContactManifold;
use crate::collision::mesh::obb_patch::obb_patch_manifold;
use crate::collision::mesh::seam_filter::filter_patch;
use crate::collision::mesh::sphere_patch::sphere_patch_manifold;
use crate::collision::obb::Obb;
use crate::collision::AABB;
use crate::physics::body::RigidBody;
use crate::physics::collider::{Collider, ColliderShape};
use crate::physics::handle::{ColliderHandle, RigidBodyHandle};
use crate::physics::narrowphase::NormalClusterer;
use crate::physics::pipeline::solver::ContactConstraint;
use crate::physics::static_geometry::StaticGeometry;

/// Default cosine threshold for seam filter coplanar merging.
const SEAM_FILTER_COPLANAR_DOT: f32 = 0.98;

/// Generate contacts between all non-static colliders and static geometry.
///
/// Dispatches per collider shape:
/// - Sphere: seam filter → sphere_patch manifold
/// - Box: seam filter → obb_patch manifold
///
/// Both paths share sleeping checks and speculative contacts.
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
                ),
            };

            // The manifold-first pipeline produces stable manifolds directly,
            // so clustering is rarely needed. Apply it only as a safety net
            // when the batch has many contacts (e.g. GJK fallback in the future).
            if batch.len() > normal_cluster.max_points {
                let clusterer = NormalClusterer::from_config(normal_cluster, contact_margin);
                batch = clusterer.cluster(batch);
            }

            if batch.is_empty()
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
                batch = speculative_static_contacts(
                    body_handle,
                    *collider_handle,
                    collider,
                    predicted_center,
                    body.rotation(),
                    static_geometry,
                    contact_margin,
                );
            }
            contacts.extend(batch);
        }
    }

    contacts
}

/// Generate sphere-static contacts via the mesh pipeline.
fn sphere_vs_static(
    body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    collider: &Collider,
    center: Point3<f32>,
    radius: f32,
    static_geometry: &dyn StaticGeometry,
    contact_margin: f32,
) -> Vec<ContactConstraint> {
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
    let filtered = filter_patch(&patch, SEAM_FILTER_COPLANAR_DOT);
    let manifold = sphere_patch_manifold(center, radius, &filtered, contact_margin);

    manifold_to_constraints(
        manifold,
        body_handle,
        collider_handle,
        collider,
    )
}

/// Generate box-static contacts via the mesh pipeline.
fn box_vs_static(
    body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    collider: &Collider,
    center: Point3<f32>,
    rotation: nalgebra::UnitQuaternion<f32>,
    half_extents: Vector3<f32>,
    static_geometry: &dyn StaticGeometry,
    contact_margin: f32,
) -> Vec<ContactConstraint> {
    let obb = Obb::new(center, rotation, half_extents);
    let (aabb_min, aabb_max) = obb.enclosing_aabb();
    let margin_vec = Vector3::new(contact_margin, contact_margin, contact_margin);
    let query = AABB::new(aabb_min - margin_vec, aabb_max + margin_vec);

    let patch = static_geometry.query_region(&query);
    let filtered = filter_patch(&patch, SEAM_FILTER_COPLANAR_DOT);
    let manifold = obb_patch_manifold(&obb, &filtered, contact_margin);

    manifold_to_constraints(
        manifold,
        body_handle,
        collider_handle,
        collider,
    )
}

/// Build speculative contacts at a predicted pose for any shape.
fn speculative_static_contacts(
    body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    collider: &Collider,
    predicted_center: Point3<f32>,
    rotation: nalgebra::UnitQuaternion<f32>,
    static_geometry: &dyn StaticGeometry,
    contact_margin: f32,
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
            let filtered = filter_patch(&patch, SEAM_FILTER_COPLANAR_DOT);
            let manifold = sphere_patch_manifold(predicted_center, *radius, &filtered, contact_margin);

            // Speculative contacts: force depth=0, raw_depth=-margin.
            manifold_to_speculative_constraints(
                manifold,
                body_handle,
                collider_handle,
                collider,
                contact_margin,
            )
        }
        ColliderShape::Box { half_extents } => {
            let obb = Obb::new(predicted_center, rotation, *half_extents);
            let (aabb_min, aabb_max) = obb.enclosing_aabb();
            let margin_vec = Vector3::new(contact_margin, contact_margin, contact_margin);
            let query = AABB::new(aabb_min - margin_vec, aabb_max + margin_vec);

            let patch = static_geometry.query_region(&query);
            let filtered = filter_patch(&patch, SEAM_FILTER_COPLANAR_DOT);
            let manifold = obb_patch_manifold(&obb, &filtered, contact_margin);

            manifold_to_speculative_constraints(
                manifold,
                body_handle,
                collider_handle,
                collider,
                contact_margin,
            )
        }
    }
}

/// Convert a ContactManifold to solver ContactConstraints.
fn manifold_to_constraints(
    manifold: ContactManifold,
    body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    collider: &Collider,
) -> Vec<ContactConstraint> {
    manifold
        .points
        .into_iter()
        .map(|cp| ContactConstraint {
            body_a: None,
            body_b: body_handle,
            collider_a: None,
            collider_b: Some(collider_handle),
            point: cp.point,
            normal: cp.normal,
            raw_normal: cp.raw_normal,
            depth: cp.depth,
            raw_depth: cp.raw_depth,
            restitution: collider.material().restitution,
            friction: collider.material().friction,
            warm_normal_impulse: 0.0,
            warm_tangent_impulse: [0.0, 0.0],
        })
        .collect()
}

/// Convert a ContactManifold to speculative ContactConstraints.
///
/// Speculative contacts have depth=0 and raw_depth=-margin so the solver
/// applies velocity-only correction without position push.
fn manifold_to_speculative_constraints(
    manifold: ContactManifold,
    body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    collider: &Collider,
    contact_margin: f32,
) -> Vec<ContactConstraint> {
    manifold
        .points
        .into_iter()
        .map(|cp| ContactConstraint {
            body_a: None,
            body_b: body_handle,
            collider_a: None,
            collider_b: Some(collider_handle),
            point: cp.point,
            normal: cp.normal,
            raw_normal: cp.raw_normal,
            depth: 0.0,
            raw_depth: -contact_margin,
            restitution: collider.material().restitution,
            friction: collider.material().friction,
            warm_normal_impulse: 0.0,
            warm_tangent_impulse: [0.0, 0.0],
        })
        .collect()
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
