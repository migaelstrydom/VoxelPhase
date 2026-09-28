//! Area-maximizing contact reduction.
//!
//! Reduces a set of contact points to at most N using a greedy spread algorithm:
//! 1. Pick the deepest contact.
//! 2. Iteratively pick the contact farthest from the already-selected set.
//!
//! This produces a well-distributed subset that preserves stability for the solver.
//!
//! Points that push different ways are different surfaces, and spread alone
//! does not see that: a box wedged between two walls could keep three points
//! on one and one on the other, and the solver would weigh the walls three to
//! one. So points are first grouped by normal, each group is guaranteed its
//! deepest point, and the budget is shared out between groups as evenly as it
//! allows before spread picks within each:
//!
//! ```text
//!   contacts ──group by normal──▶ groups ──share budget──▶ quotas
//!                                   │                        │
//!                                   └──spread within each────┘──▶ reduced
//! ```
//!
//! A set whose points all share a normal is one group, reduced exactly as
//! spread alone would.

use nalgebra::Vector3;
use smallvec::SmallVec;

use super::contact::ContactPoint;

/// Normals closer than this (cosine of 30°) belong to the same surface.
///
/// Wide enough that the triangles of one bumpy terrain surface stay one
/// group, narrow enough that a floor and a wall, or two faces of a ridge, do
/// not.
const SAME_SURFACE_COS: f32 = 0.866;

/// Reduces contact points to a maximum count while preserving spatial spread.
pub struct ContactReducer {
    /// Maximum number of contacts to keep after reduction.
    max_points: usize,
}

/// Indices into the contact set.
type Indices = SmallVec<[usize; 16]>;

impl ContactReducer {
    pub fn new(max_points: usize) -> Self {
        Self { max_points }
    }

    /// Reduce `contacts` to at most `max_points`, keeping every surface they
    /// touch and, within each, the most spread-out points.
    ///
    /// Returns the reduced set. If already under the limit, returns all contacts unchanged.
    pub fn reduce(&self, contacts: &[ContactPoint]) -> Vec<ContactPoint> {
        if contacts.len() <= self.max_points {
            return contacts.to_vec();
        }

        let groups = group_by_normal(contacts);
        let quotas = share_budget(contacts, &groups, self.max_points);
        groups
            .iter()
            .zip(quotas)
            .flat_map(|(group, quota)| spread_select(contacts, group, quota))
            .map(|idx| contacts[idx])
            .collect()
    }
}

/// Partition contacts into surfaces: each joins the first group whose first
/// member's normal is within [`SAME_SURFACE_COS`] of its own.
fn group_by_normal(contacts: &[ContactPoint]) -> SmallVec<[Indices; 4]> {
    let mut groups: SmallVec<[Indices; 4]> = SmallVec::new();
    for (idx, contact) in contacts.iter().enumerate() {
        let normal: Vector3<f32> = contact.normal;
        match groups
            .iter_mut()
            .find(|g| contacts[g[0]].normal.dot(&normal) >= SAME_SURFACE_COS)
        {
            Some(group) => group.push(idx),
            None => groups.push(SmallVec::from_slice(&[idx])),
        }
    }
    groups
}

/// How many points each group keeps.
///
/// Slots go round the groups one at a time, deepest group first, skipping any
/// group already used up — so two surfaces split four points two and two, and
/// three split them two, one and one. With more surfaces than slots, the
/// shallowest go without.
fn share_budget(
    contacts: &[ContactPoint],
    groups: &[Indices],
    budget: usize,
) -> SmallVec<[usize; 4]> {
    let deepest = |group: &Indices| {
        group
            .iter()
            .map(|&i| contacts[i].raw_depth)
            .fold(f32::MIN, f32::max)
    };
    let mut order: SmallVec<[usize; 4]> = (0..groups.len()).collect();
    order.sort_by(|&a, &b| {
        deepest(&groups[b])
            .partial_cmp(&deepest(&groups[a]))
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut quotas: SmallVec<[usize; 4]> = SmallVec::from_elem(0, groups.len());
    let mut remaining = budget;
    while remaining > 0 {
        let mut granted = false;
        for &g in &order {
            if remaining == 0 {
                break;
            }
            if quotas[g] < groups[g].len() {
                quotas[g] += 1;
                remaining -= 1;
                granted = true;
            }
        }
        if !granted {
            break;
        }
    }
    quotas
}

/// Up to `quota` of `candidates`: the deepest, then each time the one farthest
/// from everything already picked.
fn spread_select(contacts: &[ContactPoint], candidates: &[usize], quota: usize) -> Indices {
    let mut selected = Indices::new();
    if quota == 0 {
        return selected;
    }

    let first = candidates
        .iter()
        .copied()
        .max_by(|&a, &b| {
            contacts[a]
                .raw_depth
                .partial_cmp(&contacts[b].raw_depth)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .unwrap_or(candidates[0]);
    selected.push(first);

    while selected.len() < quota && selected.len() < candidates.len() {
        let mut best_idx = None;
        let mut best_score = -1.0f32;

        for &idx in candidates {
            if selected.contains(&idx) {
                continue;
            }
            let min_dist_sq = selected
                .iter()
                .map(|&s| (contacts[idx].point - contacts[s].point).magnitude_squared())
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

    selected
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

    fn contact_facing(point: Point3<f32>, normal: Vector3<f32>, depth: f32) -> ContactPoint {
        ContactPoint::new(point, normal, depth, FeatureId::SINGLE)
    }

    /// Four corners of a unit face on each of two walls facing each other,
    /// laid out so that spread alone favours the shallower wall's corners.
    fn wedged_between_two_walls() -> Vec<ContactPoint> {
        let mut contacts = Vec::new();
        for (x, normal, depth) in [(0.1, -Vector3::x(), 0.6), (-0.1, Vector3::x(), 0.2)] {
            for (y, z) in [(1.0, -0.5), (1.0, 0.5), (2.0, -0.5), (2.0, 0.5)] {
                contacts.push(contact_facing(Point3::new(x, y, z), normal, depth));
            }
        }
        contacts
    }

    #[test]
    fn two_surfaces_split_the_points_evenly() {
        let reduced = ContactReducer::new(4).reduce(&wedged_between_two_walls());
        let on_left = reduced.iter().filter(|c| c.normal.x > 0.0).count();
        assert_eq!(on_left, 2, "left wall kept {on_left} of 4");
    }

    #[test]
    fn every_surface_keeps_a_point_and_the_deepest_keeps_more() {
        let mut contacts = wedged_between_two_walls();
        for (x, z) in [(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)] {
            contacts.push(contact_facing(Point3::new(x, 0.0, z), Vector3::y(), 0.05));
        }
        let reduced = ContactReducer::new(4).reduce(&contacts);
        let count = |n: Vector3<f32>| reduced.iter().filter(|c| c.normal.dot(&n) > 0.9).count();
        assert_eq!(
            count(-Vector3::x()),
            2,
            "the deepest wall gets the spare slot"
        );
        assert_eq!(count(Vector3::x()), 1);
        assert_eq!(count(Vector3::y()), 1);
    }

    #[test]
    fn a_bumpy_surface_is_still_one_surface() {
        let tilt = |deg: f32| {
            let r = deg.to_radians();
            Vector3::new(r.sin(), r.cos(), 0.0)
        };
        let contacts = vec![
            contact_facing(Point3::new(0.0, 0.0, 0.0), tilt(0.0), 10.0),
            contact_facing(Point3::new(100.0, 0.0, 0.0), tilt(12.0), 0.5),
            contact_facing(Point3::new(0.0, 0.0, 50.0), tilt(-12.0), 0.4),
            contact_facing(Point3::new(0.0, 0.0, 25.0), tilt(8.0), 0.3),
            contact_facing(Point3::new(1.0, 0.0, 1.0), tilt(20.0), 0.2),
        ];
        let reduced = ContactReducer::new(4).reduce(&contacts);
        let has = |p: Point3<f32>| reduced.iter().any(|c| (c.point - p).magnitude() < 1e-6);
        assert!(
            !has(Point3::new(1.0, 0.0, 1.0)),
            "spread, not grouping, should decide"
        );
    }
}
