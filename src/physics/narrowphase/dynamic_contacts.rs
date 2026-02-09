//! Unified narrowphase contact generation for all dynamic-vs-dynamic collider pairs.

use std::collections::HashSet;

use generational_arena::Arena;
use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::physics::body::RigidBody;
use crate::physics::collider::{Collider, ColliderMaterial, ColliderShape};
use crate::physics::collision::obb::Obb;
use crate::physics::collision::obb_obb::obb_obb_contacts;
use crate::physics::collision::obb_sphere::obb_sphere_contact;
use crate::physics::collision::{sphere_sphere_collision, swept_sphere_sphere};
use crate::physics::handle::{ColliderHandle, RigidBodyHandle};
use crate::physics::pipeline::solver::ContactConstraint;

/// Shape-agnostic snapshot of a collider's world-space state for pair dispatch.
struct ColliderState {
    body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    center: Point3<f32>,
    rotation: UnitQuaternion<f32>,
    shape: ColliderShape,
    bounding_radius: f32,
    velocity: Vector3<f32>,
    material: ColliderMaterial,
    is_sleeping: bool,
}

/// Generate contacts between all pairs of non-static colliders.
///
/// Collects shape-agnostic `ColliderState` snapshots, then dispatches each pair
/// by shape combination:
/// - (Sphere, Sphere) → sphere-sphere overlap + speculative sweep
/// - (Sphere, Box) | (Box, Sphere) → OBB closest-point
/// - (Box, Box) → OBB SAT + face clipping
pub fn generate_dynamic_contacts(
    bodies: &Arena<RigidBody>,
    colliders: &Arena<Collider>,
    contact_margin: f32,
    dt: f32,
    ccd_threshold: f32,
    enable_speculative_contacts: bool,
    speculative_min_speed: f32,
    speculative_margin_multiplier: f32,
    sleeping: Option<&HashSet<RigidBodyHandle>>,
) -> Vec<ContactConstraint> {
    let states = collect_collider_states(bodies, colliders, sleeping);
    let mut contacts = Vec::new();

    for i in 0..states.len() {
        for j in (i + 1)..states.len() {
            let si = &states[i];
            let sj = &states[j];

            if si.is_sleeping && sj.is_sleeping {
                continue;
            }

            let pair_contacts = match (&si.shape, &sj.shape) {
                (ColliderShape::Sphere { radius: ra }, ColliderShape::Sphere { radius: rb }) => {
                    sphere_sphere_pair(
                        si,
                        *ra,
                        sj,
                        *rb,
                        contact_margin,
                        dt,
                        ccd_threshold,
                        enable_speculative_contacts,
                        speculative_min_speed,
                        speculative_margin_multiplier,
                    )
                }
                (ColliderShape::Sphere { radius }, ColliderShape::Box { half_extents }) => {
                    sphere_box_pair(si, *radius, sj, *half_extents, contact_margin)
                }
                (ColliderShape::Box { half_extents }, ColliderShape::Sphere { radius }) => {
                    box_sphere_pair(si, *half_extents, sj, *radius, contact_margin)
                }
                (ColliderShape::Box { half_extents: he_a }, ColliderShape::Box { half_extents: he_b }) => {
                    box_box_pair(si, *he_a, sj, *he_b, contact_margin)
                }
            };

            contacts.extend(pair_contacts);
        }
    }

    contacts
}

fn collect_collider_states(
    bodies: &Arena<RigidBody>,
    colliders: &Arena<Collider>,
    sleeping: Option<&HashSet<RigidBodyHandle>>,
) -> Vec<ColliderState> {
    let mut states = Vec::new();

    for (idx, body) in bodies.iter() {
        if body.is_static() {
            continue;
        }
        let body_handle = RigidBodyHandle(idx);
        let is_sleeping = sleeping
            .map(|s| s.contains(&body_handle))
            .unwrap_or(false);

        for collider_handle in body.colliders() {
            let Some(collider) = colliders.get(collider_handle.0) else {
                continue;
            };
            states.push(ColliderState {
                body_handle,
                collider_handle: *collider_handle,
                center: collider.world_center(body.position(), body.rotation()),
                rotation: body.rotation(),
                shape: collider.shape().clone(),
                bounding_radius: collider.shape().bounding_radius(),
                velocity: body.linear_velocity(),
                material: *collider.material(),
                is_sleeping,
            });
        }
    }

    states
}

fn sphere_sphere_pair(
    a: &ColliderState,
    radius_a: f32,
    b: &ColliderState,
    radius_b: f32,
    contact_margin: f32,
    dt: f32,
    ccd_threshold: f32,
    enable_speculative_contacts: bool,
    speculative_min_speed: f32,
    speculative_margin_multiplier: f32,
) -> Vec<ContactConstraint> {
    let test = sphere_sphere_collision(
        a.center,
        radius_a + contact_margin,
        b.center,
        radius_b + contact_margin,
    );

    if let Some(contact) = test {
        let actual_depth = (radius_a + radius_b) - (b.center - a.center).magnitude();
        let solver_depth = actual_depth.max(0.0);
        let (restitution, friction) = ColliderMaterial::combine(&a.material, &b.material);
        return vec![ContactConstraint {
            body_a: Some(a.body_handle),
            body_b: b.body_handle,
            collider_a: Some(a.collider_handle),
            collider_b: Some(b.collider_handle),
            point: contact.point,
            normal: contact.normal,
            raw_normal: contact.normal,
            depth: solver_depth,
            raw_depth: actual_depth,
            restitution,
            friction,
            warm_normal_impulse: 0.0,
            warm_tangent_impulse: [0.0, 0.0],
        }];
    }

    if should_add_speculative(
        a.velocity.magnitude() * dt,
        radius_a,
        b.velocity.magnitude() * dt,
        radius_b,
        ccd_threshold,
        contact_margin,
        enable_speculative_contacts,
        speculative_min_speed,
        speculative_margin_multiplier,
    ) {
        let end_a = a.center + a.velocity * dt;
        let end_b = b.center + b.velocity * dt;
        if let Some(t) = swept_sphere_sphere(
            a.center,
            end_a,
            radius_a + contact_margin,
            b.center,
            end_b,
            radius_b + contact_margin,
        ) {
            let pos_a = a.center + (end_a - a.center) * t;
            let pos_b = b.center + (end_b - b.center) * t;
            let delta = pos_b - pos_a;
            let dist = delta.magnitude();
            let normal = if dist < 1e-6 {
                Vector3::y()
            } else {
                delta / dist
            };
            let actual_depth = (radius_a + radius_b) - dist;
            let solver_depth = actual_depth.max(0.0);
            let point = pos_a + normal * (radius_a - actual_depth * 0.5);
            let (restitution, friction) = ColliderMaterial::combine(&a.material, &b.material);
            return vec![ContactConstraint {
                body_a: Some(a.body_handle),
                body_b: b.body_handle,
                collider_a: Some(a.collider_handle),
                collider_b: Some(b.collider_handle),
                point,
                normal,
                raw_normal: normal,
                depth: solver_depth,
                raw_depth: actual_depth,
                restitution,
                friction,
                warm_normal_impulse: 0.0,
                warm_tangent_impulse: [0.0, 0.0],
            }];
        }
    }

    Vec::new()
}

fn sphere_box_pair(
    sphere: &ColliderState,
    radius: f32,
    box_state: &ColliderState,
    half_extents: Vector3<f32>,
    contact_margin: f32,
) -> Vec<ContactConstraint> {
    let obb = Obb::new(box_state.center, box_state.rotation, half_extents);
    let Some(c) = obb_sphere_contact(&obb, sphere.center, radius + contact_margin) else {
        return Vec::new();
    };
    let raw_depth = c.depth - contact_margin;
    let solver_depth = raw_depth.max(0.0);
    let (restitution, friction) = ColliderMaterial::combine(&box_state.material, &sphere.material);
    // Normal from obb_sphere points OBB→sphere. For body_a=box, body_b=sphere
    // the solver expects normal from A→B, which is OBB→sphere. Correct as-is.
    vec![ContactConstraint {
        body_a: Some(box_state.body_handle),
        body_b: sphere.body_handle,
        collider_a: Some(box_state.collider_handle),
        collider_b: Some(sphere.collider_handle),
        point: c.point,
        normal: c.normal,
        raw_normal: c.normal,
        depth: solver_depth,
        raw_depth,
        restitution,
        friction,
        warm_normal_impulse: 0.0,
        warm_tangent_impulse: [0.0, 0.0],
    }]
}

fn box_sphere_pair(
    box_state: &ColliderState,
    half_extents: Vector3<f32>,
    sphere: &ColliderState,
    radius: f32,
    contact_margin: f32,
) -> Vec<ContactConstraint> {
    sphere_box_pair(sphere, radius, box_state, half_extents, contact_margin)
}

fn box_box_pair(
    a: &ColliderState,
    he_a: Vector3<f32>,
    b: &ColliderState,
    he_b: Vector3<f32>,
    contact_margin: f32,
) -> Vec<ContactConstraint> {
    let obb_a = Obb::new(a.center, a.rotation, he_a);
    let obb_b = Obb::new(b.center, b.rotation, he_b);
    let raw_contacts = obb_obb_contacts(&obb_a, &obb_b);
    let (restitution, friction) = ColliderMaterial::combine(&a.material, &b.material);

    raw_contacts
        .into_iter()
        .map(|c| {
            let raw_depth = c.depth - contact_margin;
            let solver_depth = raw_depth.max(0.0);
            ContactConstraint {
                body_a: Some(a.body_handle),
                body_b: b.body_handle,
                collider_a: Some(a.collider_handle),
                collider_b: Some(b.collider_handle),
                point: c.point,
                normal: c.normal,
                raw_normal: c.normal,
                depth: solver_depth,
                raw_depth,
                restitution,
                friction,
                warm_normal_impulse: 0.0,
                warm_tangent_impulse: [0.0, 0.0],
            }
        })
        .collect()
}

fn should_add_speculative(
    travel_a: f32,
    radius_a: f32,
    travel_b: f32,
    radius_b: f32,
    ccd_threshold: f32,
    contact_margin: f32,
    enable_speculative_contacts: bool,
    speculative_min_speed: f32,
    speculative_margin_multiplier: f32,
) -> bool {
    if !enable_speculative_contacts {
        return false;
    }
    let margin_gate = contact_margin * speculative_margin_multiplier;
    let fast_a = travel_a >= speculative_min_speed
        && travel_a > margin_gate
        && travel_a <= radius_a * ccd_threshold;
    let fast_b = travel_b >= speculative_min_speed
        && travel_b > margin_gate
        && travel_b <= radius_b * ccd_threshold;
    fast_a || fast_b
}
