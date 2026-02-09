//! Unified narrowphase contact generation for all collider shapes vs static geometry.

use std::collections::HashSet;

use generational_arena::Arena;
use nalgebra::Vector3;

use crate::collision::AABB;
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
/// - Sphere: queries static geometry with expanded radius, subtracts contact_margin from depth
/// - Box: queries triangles in the OBB's AABB, runs SAT per triangle
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

            let mut collider_contacts = match collider.shape() {
                ColliderShape::Sphere { radius } => sphere_vs_static(
                    body_handle,
                    *collider_handle,
                    collider,
                    center,
                    *radius,
                    linear_velocity,
                    travel,
                    dt,
                    static_geometry,
                    contact_margin,
                    ccd_threshold,
                    enable_speculative_contacts,
                    speculative_min_speed,
                    speculative_margin_multiplier,
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

            if collider_contacts.len() > 1 {
                let clusterer = NormalClusterer::from_config(normal_cluster, contact_margin);
                collider_contacts = clusterer.cluster_with_pre_reduction(collider_contacts);
            }
            contacts.extend(collider_contacts);
        }
    }

    contacts
}

fn sphere_vs_static(
    body_handle: RigidBodyHandle,
    collider_handle: crate::physics::handle::ColliderHandle,
    collider: &Collider,
    center: nalgebra::Point3<f32>,
    radius: f32,
    linear_velocity: Vector3<f32>,
    travel: f32,
    dt: f32,
    static_geometry: &dyn StaticGeometry,
    contact_margin: f32,
    ccd_threshold: f32,
    enable_speculative_contacts: bool,
    speculative_min_speed: f32,
    speculative_margin_multiplier: f32,
) -> Vec<ContactConstraint> {
    let query_radius = radius + contact_margin;
    let mut sphere_contacts = Vec::new();

    for sc in static_geometry.query_sphere(center, query_radius) {
        let raw_depth = sc.depth - contact_margin;
        let solver_depth = raw_depth.max(0.0);
        sphere_contacts.push(ContactConstraint {
            body_a: None,
            body_b: body_handle,
            collider_a: None,
            collider_b: Some(collider_handle),
            point: sc.point,
            normal: sc.normal,
            raw_normal: sc.normal,
            depth: solver_depth,
            raw_depth,
            restitution: collider.material().restitution,
            friction: collider.material().friction,
            warm_normal_impulse: 0.0,
            warm_tangent_impulse: [0.0, 0.0],
        });
    }

    if sphere_contacts.is_empty()
        && is_speculative_candidate(
            travel,
            radius,
            ccd_threshold,
            contact_margin,
            enable_speculative_contacts,
            speculative_min_speed,
            speculative_margin_multiplier,
        )
    {
        let end = center + linear_velocity * dt;
        if let Some(hit) = static_geometry.sweep_sphere(center, end, query_radius) {
            sphere_contacts.push(ContactConstraint {
                body_a: None,
                body_b: body_handle,
                collider_a: None,
                collider_b: Some(collider_handle),
                point: hit.point,
                normal: hit.normal,
                raw_normal: hit.normal,
                depth: 0.0,
                raw_depth: -contact_margin,
                restitution: collider.material().restitution,
                friction: collider.material().friction,
                warm_normal_impulse: 0.0,
                warm_tangent_impulse: [0.0, 0.0],
            });
        }
    }

    sphere_contacts
}

fn box_vs_static(
    body_handle: RigidBodyHandle,
    collider_handle: crate::physics::handle::ColliderHandle,
    collider: &Collider,
    center: nalgebra::Point3<f32>,
    rotation: nalgebra::UnitQuaternion<f32>,
    half_extents: Vector3<f32>,
    static_geometry: &dyn StaticGeometry,
    contact_margin: f32,
) -> Vec<ContactConstraint> {
    let obb = Obb::new(center, rotation, half_extents);
    let (aabb_min, aabb_max) = obb.enclosing_aabb();
    let margin_vec = Vector3::new(contact_margin, contact_margin, contact_margin);
    let query = AABB::new(aabb_min - margin_vec, aabb_max + margin_vec);

    let triangles = static_geometry.query_triangles(&query);
    let mut box_contacts = Vec::new();

    for tri in &triangles {
        for c in obb_triangle_contacts(&obb, tri) {
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

    box_contacts
}

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

