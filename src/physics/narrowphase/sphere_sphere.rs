//! Narrowphase contact generation for sphere-sphere collider pairs.

use generational_arena::Arena;

use crate::physics::body::RigidBody;
use crate::physics::collider::{Collider, ColliderShape};
use crate::physics::collision::sphere_sphere_collision;
use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::solver::{combine_materials, ContactConstraint};

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
            Some((
                RigidBodyHandle(idx),
                collider_handle,
                center,
                radius,
                *collider.material(),
            ))
        })
        .collect();

    for i in 0..spheres.len() {
        for j in (i + 1)..spheres.len() {
            let (handle_a, col_a, center_a, radius_a, mat_a) = &spheres[i];
            let (handle_b, col_b, center_b, radius_b, mat_b) = &spheres[j];

            // Test with margin-expanded radii for early detection
            let test = sphere_sphere_collision(
                *center_a,
                radius_a + contact_margin,
                *center_b,
                radius_b + contact_margin,
            );

            if let Some(contact) = test {
                // Use actual (non-inflated) depth for the constraint
                let actual_depth =
                    (radius_a + radius_b) - (*center_b - *center_a).magnitude();
                let solver_depth = actual_depth.max(0.0);

                let (restitution, friction) = combine_materials(mat_a, mat_b);
                contacts.push(ContactConstraint {
                    body_a: Some(*handle_a),
                    body_b: *handle_b,
                    collider_a: Some(*col_a),
                    collider_b: Some(*col_b),
                    point: contact.point,
                    normal: contact.normal,
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

    contacts
}
