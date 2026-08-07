//! Area-maximizing contact reduction.
//!
//! Reduces a set of contact points to at most N using a greedy spread algorithm:
//! 1. Pick the deepest contact.
//! 2. Iteratively pick the contact farthest from the already-selected set.
//!
//! This produces a well-distributed subset that preserves stability for the solver.

use super::contact::ContactPoint;

/// Reduces contact points to a maximum count while preserving spatial spread.
pub struct ContactReducer {
    /// Maximum number of contacts to keep after reduction.
    max_points: usize,
}

impl ContactReducer {
    pub fn new(max_points: usize) -> Self {
        Self { max_points }
    }

    /// Reduce `contacts` to at most `max_points` using area-maximizing selection.
    ///
    /// Returns the reduced set. If already under the limit, returns all contacts unchanged.
    pub fn reduce(&self, contacts: &[ContactPoint]) -> Vec<ContactPoint> {
        if contacts.len() <= self.max_points {
            return contacts.to_vec();
        }

        // Find the deepest contact (highest raw_depth).
        let first = contacts
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| {
                a.raw_depth
                    .partial_cmp(&b.raw_depth)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)
            .unwrap_or(0);

        let mut selected: Vec<usize> = vec![first];

        while selected.len() < self.max_points && selected.len() < contacts.len() {
            let mut best_idx = None;
            let mut best_score = -1.0f32;

            for (idx, contact) in contacts.iter().enumerate() {
                if selected.contains(&idx) {
                    continue;
                }
                let min_dist_sq = selected
                    .iter()
                    .map(|&s| {
                        let delta = contact.point - contacts[s].point;
                        delta.magnitude_squared()
                    })
                    .fold(f32::INFINITY, f32::min);

                if min_dist_sq > best_score {
                    best_score = min_dist_sq;
                    best_idx = Some(idx);
                }
            }

            match best_idx {
                Some(idx) => selected.push(idx),
                None => break,
            }
        }

        selected.iter().map(|&idx| contacts[idx]).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::contact::FeatureId;
    use nalgebra::{Point3, Vector3};

    fn contact_at(point: Point3<f32>, depth: f32) -> ContactPoint {
        ContactPoint::new(point, Vector3::y(), depth, FeatureId::SINGLE)
    }

    #[test]
    fn under_limit_returns_all() {
        let contacts = vec![
            contact_at(Point3::new(0.0, 0.0, 0.0), 1.0),
            contact_at(Point3::new(1.0, 0.0, 0.0), 0.5),
        ];
        let reducer = ContactReducer::new(4);
        let reduced = reducer.reduce(&contacts);
        assert_eq!(reduced.len(), 2);
    }

    #[test]
    fn reduces_to_max_points() {
        let contacts = vec![
            contact_at(Point3::new(0.0, 0.0, 0.0), 10.0),
            contact_at(Point3::new(100.0, 0.0, 0.0), 0.5),
            contact_at(Point3::new(0.0, 50.0, 0.0), 0.4),
            contact_at(Point3::new(0.0, 0.0, 25.0), 0.3),
            contact_at(Point3::new(1.0, 1.0, 1.0), 0.2),
        ];
        let reducer = ContactReducer::new(4);
        let reduced = reducer.reduce(&contacts);
        assert_eq!(reduced.len(), 4);
    }

    #[test]
    fn picks_deepest_first() {
        let contacts = vec![
            contact_at(Point3::new(0.0, 0.0, 0.0), 0.1),
            contact_at(Point3::new(1.0, 0.0, 0.0), 5.0),
            contact_at(Point3::new(2.0, 0.0, 0.0), 0.2),
        ];
        let reducer = ContactReducer::new(1);
        let reduced = reducer.reduce(&contacts);
        assert_eq!(reduced.len(), 1);
        assert_eq!(reduced[0].point, Point3::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn picks_spread_points() {
        let contacts = vec![
            contact_at(Point3::new(0.0, 0.0, 0.0), 10.0),
            contact_at(Point3::new(100.0, 0.0, 0.0), 0.5),
            contact_at(Point3::new(0.0, 50.0, 0.0), 0.4),
            contact_at(Point3::new(0.0, 0.0, 25.0), 0.3),
            contact_at(Point3::new(1.0, 1.0, 1.0), 0.2),
        ];
        let reducer = ContactReducer::new(4);
        let reduced = reducer.reduce(&contacts);

        // Should pick the 4 most spread-out points (origin, 100x, 50y, 25z).
        let has = |p: Point3<f32>| reduced.iter().any(|c| (c.point - p).magnitude() < 1e-6);
        assert!(has(Point3::new(0.0, 0.0, 0.0)));
        assert!(has(Point3::new(100.0, 0.0, 0.0)));
        assert!(has(Point3::new(0.0, 50.0, 0.0)));
        assert!(has(Point3::new(0.0, 0.0, 25.0)));
        assert!(!has(Point3::new(1.0, 1.0, 1.0)));
    }

    #[test]
    fn empty_input() {
        let reducer = ContactReducer::new(4);
        let reduced = reducer.reduce(&[]);
        assert!(reduced.is_empty());
    }

    #[test]
    fn single_contact() {
        let contacts = vec![contact_at(Point3::origin(), 1.0)];
        let reducer = ContactReducer::new(4);
        let reduced = reducer.reduce(&contacts);
        assert_eq!(reduced.len(), 1);
    }
}
