//! Narrowphase contact generation for sphere colliders vs static geometry.

use generational_arena::Arena;

use crate::physics::body::RigidBody;
use crate::physics::collider::{Collider, ColliderShape};
use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::solver::ContactConstraint;
use crate::physics::static_geometry::StaticGeometry;

/// Generate contacts between all dynamic sphere colliders and static geometry.
///
/// Queries static geometry at each sphere's current position using an expanded
/// radius (radius + contact_margin). The margin is subtracted from the returned
/// depth so that contacts within the margin skin receive velocity-only correction
/// (depth <= 0) while actual penetrations receive position correction (depth > 0).
pub fn generate_sphere_static_contacts(
    bodies: &Arena<RigidBody>,
    colliders: &Arena<Collider>,
    static_geometry: &dyn StaticGeometry,
    contact_margin: f32,
) -> Vec<ContactConstraint> {
    let mut contacts = Vec::new();

    for (idx, body) in bodies.iter() {
        if !body.is_dynamic() {
            continue;
        }
        let body_handle = RigidBodyHandle(idx);

        for collider_handle in body.colliders() {
            let Some(collider) = colliders.get(collider_handle.0) else {
                continue;
            };
            let radius = match collider.shape() {
                ColliderShape::Sphere { radius } => *radius,
            };

            let center = collider.world_center(body.position(), body.rotation());
            let query_radius = radius + contact_margin;

            let mut best_contact = None;
            for sc in static_geometry.query_sphere(center, query_radius) {
                let replace = match &best_contact {
                    None => true,
                    Some((best_depth, _)) => sc.depth > *best_depth,
                };
                if replace {
                    best_contact = Some((sc.depth, sc));
                }
            }

            if let Some((_, sc)) = best_contact {
                let raw_depth = sc.depth - contact_margin;
                let solver_depth = raw_depth.max(0.0);
                contacts.push(ContactConstraint {
                    body_a: None,
                    body_b: body_handle,
                    collider_a: None,
                    collider_b: Some(*collider_handle),
                    point: sc.point,
                    normal: sc.normal,
                    depth: solver_depth,
                    raw_depth,
                    restitution: collider.material().restitution,
                    friction: collider.material().friction,
                    warm_normal_impulse: 0.0,
                    warm_tangent_impulse: [0.0, 0.0],
                });
            }
        }
    }

    contacts
}
