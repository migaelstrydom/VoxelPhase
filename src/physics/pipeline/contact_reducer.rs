use nalgebra::Vector3;

use crate::physics::pipeline::solver::ContactConstraint;

pub struct ContactReducer {
    /// Maximum number of contacts to keep after reduction.
    max_points: usize,
}

impl ContactReducer {
    pub fn new(max_points: usize) -> Self {
        Self { max_points }
    }

    pub fn reduce(&self, contacts: Vec<ContactConstraint>) -> Vec<ContactConstraint> {
        if contacts.len() <= self.max_points {
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

        while selected.len() < self.max_points && selected.len() < contacts.len() {
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
}

#[cfg(test)]
mod tests {
    use super::ContactReducer;
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
            raw_normal: Vector3::y(),
            depth,
            raw_depth: depth,
            restitution: 0.0,
            friction: 0.0,
            warm_normal_impulse: 0.0,
            warm_tangent_impulse: [0.0, 0.0],
        }
    }

    #[test]
    fn reduce_returns_all_when_under_limit() {
        let contacts = vec![
            contact(Point3::new(0.0, 0.0, 0.0), 1.0),
            contact(Point3::new(1.0, 0.0, 0.0), 0.5),
        ];

        let reducer = ContactReducer::new(4);
        let reduced = reducer.reduce(contacts.clone());

        assert_eq!(reduced.len(), 2);
        assert_eq!(reduced[0].point, contacts[0].point);
        assert_eq!(reduced[0].depth, contacts[0].depth);
        assert_eq!(reduced[1].point, contacts[1].point);
        assert_eq!(reduced[1].depth, contacts[1].depth);
    }

    #[test]
    fn reduce_picks_spread_points_when_over_limit() {
        let contacts = vec![
            contact(Point3::new(0.0, 0.0, 0.0), 10.0),
            contact(Point3::new(100.0, 0.0, 0.0), 0.5),
            contact(Point3::new(0.0, 50.0, 0.0), 0.4),
            contact(Point3::new(0.0, 0.0, 25.0), 0.3),
            contact(Point3::new(1.0, 1.0, 1.0), 0.2),
        ];

        let reducer = ContactReducer::new(4);
        let reduced = reducer.reduce(contacts);

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
