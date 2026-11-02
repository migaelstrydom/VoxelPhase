use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::VecDeque;

use generational_arena::{Arena, Index};

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::pair::SolverManifold;

#[derive(Debug, Clone)]
pub struct Island {
    /// Dynamic bodies connected by active contacts.
    pub bodies: Vec<RigidBodyHandle>,
}

pub struct IslandBuilder;

impl IslandBuilder {
    /// The contact islands every one of whose bodies is a sleep candidate.
    ///
    /// An island is a connected group of dynamic bodies, linked by contacts
    /// between two dynamic bodies. Only the candidates are searched from: a
    /// search that reaches a body which is not one gives its island up, since
    /// the whole island has to be still to sleep. The cost follows the
    /// candidates and the contacts, not every body in the world, most of
    /// which are usually asleep.
    pub fn sleepable(
        &self,
        bodies: &Arena<RigidBody>,
        manifolds: &[SolverManifold],
        candidates: &FxHashSet<RigidBodyHandle>,
    ) -> Vec<Island> {
        if candidates.is_empty() {
            return Vec::new();
        }

        let mut adjacency: FxHashMap<Index, Vec<Index>> = FxHashMap::default();
        for manifold in manifolds {
            let Some(handle_a) = manifold.header.body_a else {
                continue;
            };
            let handle_b = manifold.header.body_b;
            let (Some(body_a), Some(body_b)) = (bodies.get(handle_a.0), bodies.get(handle_b.0))
            else {
                continue;
            };
            if !body_a.is_dynamic() || !body_b.is_dynamic() {
                continue;
            }
            adjacency.entry(handle_a.0).or_default().push(handle_b.0);
            adjacency.entry(handle_b.0).or_default().push(handle_a.0);
        }

        let mut visited: FxHashSet<Index> = FxHashSet::default();
        let mut islands = Vec::new();
        let mut queue = VecDeque::new();

        for &start in candidates {
            if !visited.insert(start.0) {
                continue;
            }
            queue.clear();
            queue.push_back(start.0);
            let mut island_bodies = Vec::new();
            let mut all_candidates = true;

            while let Some(current) = queue.pop_front() {
                island_bodies.push(RigidBodyHandle(current));
                let Some(neighbors) = adjacency.get(&current) else {
                    continue;
                };
                for &neighbor in neighbors {
                    if !candidates.contains(&RigidBodyHandle(neighbor)) {
                        all_candidates = false;
                    } else if visited.insert(neighbor) {
                        queue.push_back(neighbor);
                    }
                }
            }

            if all_candidates {
                islands.push(Island {
                    bodies: island_bodies,
                });
            }
        }

        islands
    }
}

#[cfg(test)]
mod tests {
    use smallvec::SmallVec;

    use super::*;
    use crate::physics::body::RigidBodyDesc;
    use crate::physics::pipeline::pair::PairHeader;

    fn dynamic_bodies(count: usize) -> (Arena<RigidBody>, Vec<RigidBodyHandle>) {
        let mut bodies = Arena::new();
        let handles = (0..count)
            .map(|_| RigidBodyHandle(bodies.insert(RigidBody::new(RigidBodyDesc::dynamic()))))
            .collect();
        (bodies, handles)
    }

    fn contact(a: Option<RigidBodyHandle>, b: RigidBodyHandle) -> SolverManifold {
        SolverManifold {
            header: PairHeader {
                body_a: a,
                body_b: b,
                collider_a: None,
                collider_b: None,
                restitution: 0.0,
                friction: 0.5,
            },
            contacts: SmallVec::new(),
        }
    }

    fn sorted(islands: Vec<Island>) -> Vec<Vec<RigidBodyHandle>> {
        let mut islands: Vec<Vec<RigidBodyHandle>> = islands
            .into_iter()
            .map(|island| {
                let mut bodies = island.bodies;
                bodies.sort_by_key(|h| h.0.into_raw_parts());
                bodies
            })
            .collect();
        islands.sort_by_key(|bodies| bodies[0].0.into_raw_parts());
        islands
    }

    #[test]
    fn an_island_of_candidates_sleeps_whole() {
        let (bodies, h) = dynamic_bodies(3);
        let manifolds = [contact(Some(h[0]), h[1]), contact(Some(h[1]), h[2])];
        let candidates = h.iter().copied().collect();

        let islands = IslandBuilder.sleepable(&bodies, &manifolds, &candidates);

        assert_eq!(sorted(islands), vec![h.clone()]);
    }

    #[test]
    fn a_candidate_touching_a_moving_body_stays_awake_through_its_island() {
        // 0 – 1 – 2, with 2 still moving: neither 0 nor 1 may sleep, though
        // 0 never touches 2 itself.
        let (bodies, h) = dynamic_bodies(3);
        let manifolds = [contact(Some(h[0]), h[1]), contact(Some(h[1]), h[2])];
        let candidates = [h[0], h[1]].into_iter().collect();

        let islands = IslandBuilder.sleepable(&bodies, &manifolds, &candidates);

        assert!(islands.is_empty());
    }

    #[test]
    fn separate_islands_are_judged_apart() {
        // 0 – 1 still; 2 – 3 with 3 moving; 4 alone on static ground.
        let (bodies, h) = dynamic_bodies(5);
        let manifolds = [
            contact(Some(h[0]), h[1]),
            contact(Some(h[2]), h[3]),
            contact(None, h[4]),
        ];
        let candidates = [h[0], h[1], h[2], h[4]].into_iter().collect();

        let islands = IslandBuilder.sleepable(&bodies, &manifolds, &candidates);

        assert_eq!(sorted(islands), vec![vec![h[0], h[1]], vec![h[4]]]);
    }

    #[test]
    fn a_static_body_does_not_join_islands() {
        // Two still bodies resting on one static body are two islands, not
        // one held together through it.
        let mut bodies = Arena::new();
        let ground = RigidBodyHandle(bodies.insert(RigidBody::new(RigidBodyDesc::static_body())));
        let a = RigidBodyHandle(bodies.insert(RigidBody::new(RigidBodyDesc::dynamic())));
        let b = RigidBodyHandle(bodies.insert(RigidBody::new(RigidBodyDesc::dynamic())));
        let manifolds = [contact(Some(ground), a), contact(Some(ground), b)];
        let candidates = [a, b].into_iter().collect();

        let islands = IslandBuilder.sleepable(&bodies, &manifolds, &candidates);

        assert_eq!(sorted(islands), vec![vec![a], vec![b]]);
    }
}
