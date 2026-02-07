//! Narrowphase contact generation for sphere colliders vs static geometry.

use generational_arena::Arena;
use nalgebra::Vector3;

use crate::physics::body::RigidBody;
use crate::physics::collider::{Collider, ColliderShape};
use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::solver::ContactConstraint;
use crate::physics::static_geometry::StaticGeometry;

/// Generate contacts between all non-static sphere colliders and static geometry.
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
    dt: f32,
    ccd_threshold: f32,
    enable_speculative_contacts: bool,
    speculative_min_speed: f32,
    speculative_margin_multiplier: f32,
) -> Vec<ContactConstraint> {
    let mut contacts = Vec::new();

    for (idx, body) in bodies.iter() {
        if body.is_static() {
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
            let linear_velocity = body.linear_velocity();
            let travel = linear_velocity.magnitude() * dt;

            let mut sphere_contacts = Vec::new();
            for sc in static_geometry.query_sphere(center, query_radius) {
                let raw_depth = sc.depth - contact_margin;
                let solver_depth = raw_depth.max(0.0);
                sphere_contacts.push(ContactConstraint {
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
                        collider_b: Some(*collider_handle),
                        point: hit.point,
                        normal: hit.normal,
                        depth: 0.0,
                        raw_depth: -contact_margin,
                        restitution: collider.material().restitution,
                        friction: collider.material().friction,
                        warm_normal_impulse: 0.0,
                        warm_tangent_impulse: [0.0, 0.0],
                    });
                }
            }

            if sphere_contacts.len() > 4 {
                sphere_contacts = reduce_contacts(sphere_contacts, 4);
            }
            contacts.extend(sphere_contacts);
        }
    }

    contacts
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

fn reduce_contacts(contacts: Vec<ContactConstraint>, max_points: usize) -> Vec<ContactConstraint> {
    if contacts.len() <= max_points {
        return contacts;
    }

    let mut selected: Vec<usize> = Vec::new();

    if let Some((idx, _)) = contacts.iter().enumerate().max_by(|(_, a), (_, b)| {
        a.depth
            .partial_cmp(&b.depth)
            .unwrap_or(std::cmp::Ordering::Equal)
    }) {
        selected.push(idx);
    }

    while selected.len() < max_points && selected.len() < contacts.len() {
        let mut best_idx = None;
        let mut best_score = -1.0f32;

        for (idx, contact) in contacts.iter().enumerate() {
            if selected.contains(&idx) {
                continue;
            }
            let mut min_dist_sq = f32::INFINITY;
            for &s_idx in &selected {
                let delta: Vector3<f32> = contact.point - contacts[s_idx].point;
                let dist_sq = delta.magnitude_squared();
                if dist_sq < min_dist_sq {
                    min_dist_sq = dist_sq;
                }
            }
            if min_dist_sq > best_score {
                best_score = min_dist_sq;
                best_idx = Some(idx);
            }
        }

        if let Some(idx) = best_idx {
            selected.push(idx);
        } else {
            break;
        }
    }

    selected
        .into_iter()
        .map(|idx| contacts[idx].clone())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::reduce_contacts;
    use crate::physics::handle::{ColliderHandle, RigidBodyHandle};
    use crate::physics::pipeline::solver::ContactConstraint;
    use generational_arena::Index;
    use nalgebra::{Point3, Vector3};

    fn contact(point: Point3<f32>, depth: f32) -> ContactConstraint {
        ContactConstraint {
            body_a: None,
            body_b: RigidBodyHandle(Index::from_raw_parts(0, 0)),
            collider_a: None,
            collider_b: Some(ColliderHandle(Index::from_raw_parts(1, 0))),
            point,
            normal: Vector3::y(),
            depth,
            raw_depth: depth,
            restitution: 0.0,
            friction: 0.0,
            warm_normal_impulse: 0.0,
            warm_tangent_impulse: [0.0, 0.0],
        }
    }

    #[test]
    fn reduce_contacts_returns_all_when_under_limit() {
        let contacts = vec![
            contact(Point3::new(0.0, 0.0, 0.0), 1.0),
            contact(Point3::new(1.0, 0.0, 0.0), 0.5),
        ];

        let reduced = reduce_contacts(contacts.clone(), 4);

        assert_eq!(reduced.len(), 2);
        assert_eq!(reduced[0].point, contacts[0].point);
        assert_eq!(reduced[0].depth, contacts[0].depth);
        assert_eq!(reduced[1].point, contacts[1].point);
        assert_eq!(reduced[1].depth, contacts[1].depth);
    }

    #[test]
    fn reduce_contacts_picks_spread_points_when_over_limit() {
        let contacts = vec![
            contact(Point3::new(0.0, 0.0, 0.0), 10.0),
            contact(Point3::new(100.0, 0.0, 0.0), 0.5),
            contact(Point3::new(0.0, 50.0, 0.0), 0.4),
            contact(Point3::new(0.0, 0.0, 25.0), 0.3),
            contact(Point3::new(1.0, 1.0, 1.0), 0.2),
        ];

        let reduced = reduce_contacts(contacts, 4);

        assert_eq!(reduced.len(), 4);
        assert!(reduced
            .iter()
            .any(|c| c.point == Point3::new(0.0, 0.0, 0.0)));
        assert!(reduced
            .iter()
            .any(|c| c.point == Point3::new(100.0, 0.0, 0.0)));
        assert!(reduced
            .iter()
            .any(|c| c.point == Point3::new(0.0, 50.0, 0.0)));
        assert!(reduced
            .iter()
            .any(|c| c.point == Point3::new(0.0, 0.0, 25.0)));
        assert!(!reduced
            .iter()
            .any(|c| c.point == Point3::new(1.0, 1.0, 1.0)));
    }
}
