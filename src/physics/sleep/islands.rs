use std::collections::{HashMap, HashSet, VecDeque};

use generational_arena::{Arena, Index};

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::solver::ContactConstraint;

#[derive(Debug, Clone)]
pub struct Island {
    /// Dynamic bodies connected by active contacts.
    pub bodies: Vec<RigidBodyHandle>,
}

pub struct IslandBuilder;

impl IslandBuilder {
    pub fn build(
        &self,
        bodies: &Arena<RigidBody>,
        contacts: &[ContactConstraint],
    ) -> Vec<Island> {
        let mut adjacency: HashMap<Index, Vec<Index>> = HashMap::new();
        let mut dynamic_indices: Vec<Index> = Vec::new();

        for (idx, body) in bodies.iter() {
            if body.is_dynamic() {
                adjacency.entry(idx).or_default();
                dynamic_indices.push(idx);
            }
        }

        for contact in contacts {
            let Some(handle_a) = contact.body_a else {
                continue;
            };
            let handle_b = contact.body_b;
            let (Some(body_a), Some(body_b)) = (
                bodies.get(handle_a.0),
                bodies.get(handle_b.0),
            ) else {
                continue;
            };
            if !body_a.is_dynamic() || !body_b.is_dynamic() {
                continue;
            }
            adjacency.entry(handle_a.0).or_default().push(handle_b.0);
            adjacency.entry(handle_b.0).or_default().push(handle_a.0);
        }

        let mut visited: HashSet<Index> = HashSet::new();
        let mut islands = Vec::new();

        for idx in dynamic_indices {
            if visited.contains(&idx) {
                continue;
            }
            let mut queue = VecDeque::new();
            let mut island_bodies = Vec::new();
            queue.push_back(idx);
            visited.insert(idx);

            while let Some(current) = queue.pop_front() {
                island_bodies.push(RigidBodyHandle(current));
                if let Some(neighbors) = adjacency.get(&current) {
                    for neighbor in neighbors {
                        if visited.insert(*neighbor) {
                            queue.push_back(*neighbor);
                        }
                    }
                }
            }

            islands.push(Island { bodies: island_bodies });
        }

        islands
    }
}
