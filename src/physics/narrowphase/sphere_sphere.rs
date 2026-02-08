//! Narrowphase contact generation for sphere-sphere collider pairs.

use std::collections::HashSet;

use generational_arena::Arena;
use nalgebra::Vector3;

use crate::physics::body::RigidBody;
use crate::physics::collider::{Collider, ColliderMaterial, ColliderShape};
use crate::physics::collision::{sphere_sphere_collision, swept_sphere_sphere};
use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::solver::ContactConstraint;

/// Generate contacts between all pairs of non-static sphere colliders.
///
/// Uses brute-force all-pairs testing (broadphase acceleration comes in step 6).
/// Spheres are tested with an expanded radius (radius + contact_margin) so that
/// contacts are detected slightly before geometric overlap, enabling the solver
/// to prevent penetration proactively.
pub fn generate_sphere_sphere_contacts(
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
    let mut contacts = Vec::new();

    let spheres: Vec<_> = bodies
        .iter()
        .filter(|(_, body)| !body.is_static())
        .filter_map(|(idx, body)| {
            let collider_handle = *body.colliders().first()?;
            let collider = colliders.get(collider_handle.0)?;
            let radius = match collider.shape() {
                ColliderShape::Sphere { radius } => *radius,
            };
            let center = collider.world_center(body.position(), body.rotation());
            let is_sleeping = sleeping
                .map(|sleeping| sleeping.contains(&RigidBodyHandle(idx)))
                .unwrap_or(false);
            Some((
                RigidBodyHandle(idx),
                collider_handle,
                center,
                radius,
                body.linear_velocity(),
                *collider.material(),
                is_sleeping,
            ))
        })
        .collect();

    for i in 0..spheres.len() {
        for j in (i + 1)..spheres.len() {
            let (handle_a, col_a, center_a, radius_a, vel_a, mat_a, sleep_a) = &spheres[i];
            let (handle_b, col_b, center_b, radius_b, vel_b, mat_b, sleep_b) = &spheres[j];

            if *sleep_a && *sleep_b {
                continue;
            }

            // Test with margin-expanded radii for early detection
            let test = sphere_sphere_collision(
                *center_a,
                radius_a + contact_margin,
                *center_b,
                radius_b + contact_margin,
            );

            if let Some(contact) = test {
                // Use actual (non-inflated) depth for the constraint
                let actual_depth = (radius_a + radius_b) - (*center_b - *center_a).magnitude();
                let solver_depth = actual_depth.max(0.0);

                let (restitution, friction) = ColliderMaterial::combine(mat_a, mat_b);
                contacts.push(ContactConstraint {
                    body_a: Some(*handle_a),
                    body_b: *handle_b,
                    collider_a: Some(*col_a),
                    collider_b: Some(*col_b),
                    point: contact.point,
                    normal: contact.normal,
                    raw_normal: contact.normal,
                    depth: solver_depth,
                    raw_depth: actual_depth,
                    restitution,
                    friction,
                    warm_normal_impulse: 0.0,
                    warm_tangent_impulse: [0.0, 0.0],
                });
            } else if should_add_speculative_contact(
                vel_a.magnitude() * dt,
                *radius_a,
                vel_b.magnitude() * dt,
                *radius_b,
                ccd_threshold,
                contact_margin,
                enable_speculative_contacts,
                speculative_min_speed,
                speculative_margin_multiplier,
            ) {
                let end_a = *center_a + *vel_a * dt;
                let end_b = *center_b + *vel_b * dt;
                if let Some(t) = swept_sphere_sphere(
                    *center_a,
                    end_a,
                    radius_a + contact_margin,
                    *center_b,
                    end_b,
                    radius_b + contact_margin,
                ) {
                    let pos_a = *center_a + (end_a - *center_a) * t;
                    let pos_b = *center_b + (end_b - *center_b) * t;
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
                    let (restitution, friction) = ColliderMaterial::combine(mat_a, mat_b);
                    contacts.push(ContactConstraint {
                        body_a: Some(*handle_a),
                        body_b: *handle_b,
                        collider_a: Some(*col_a),
                        collider_b: Some(*col_b),
                        point,
                        normal,
                        raw_normal: normal,
                        depth: solver_depth,
                        raw_depth: actual_depth,
                        restitution,
                        friction,
                        warm_normal_impulse: 0.0,
                        warm_tangent_impulse: [0.0, 0.0],
                    });
                }
            }
        }
    }

    contacts
}

fn should_add_speculative_contact(
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
