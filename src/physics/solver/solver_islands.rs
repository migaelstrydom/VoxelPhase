//! Groups of rows that share no movable body, found once per substep.
//!
//! Two rows couple when an impulse one applies changes a velocity the other
//! reads. That only happens through a body both touch whose velocity can
//! change; static geometry and static bodies never do, so a pile of blocks on
//! the ground is one island per block, not one for the whole pile.
//!
//! ```text
//!   manifold pairs ─┐                          ┌─▶ island 0: manifolds [0, 3], joints []
//!                   ├─ union-find ─ group ─────┼─▶ island 1: manifolds [1],    joints [0]
//!   joint pairs ────┘  over arena slots        └─▶ island 2: manifolds [2, 4], joints []
//! ```
//!
//! Islands are found from the bodies' handles, before anything is gathered,
//! so each island can gather, solve and hold its own velocities. They keep
//! their rows in the order they were given, so solving one island at a time
//! gives each of its bodies exactly what a sweep over every row would have,
//! for the same number of iterations.

use std::ops::Range;

use generational_arena::Arena;

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;

/// The two bodies of a row; the first is `None` for static geometry, and
/// either may be `None` for a joint anchored to the world.
pub(crate) type BodyPair = (Option<RigidBodyHandle>, Option<RigidBodyHandle>);

/// The islands of one substep's rows. Rebuilt every substep; the buffers are
/// kept to avoid reallocating.
#[derive(Debug, Default)]
pub(crate) struct SolverIslands {
    /// Union-find parent per arena slot.
    parent: Vec<u32>,
    /// Island of each arena slot's root, `u32::MAX` until one is assigned.
    island_of_root: Vec<u32>,
    /// Island per manifold, then per joint row, while grouping.
    island_of_row: Vec<u32>,
    /// Manifold indices grouped by island.
    manifolds: Vec<usize>,
    /// Joint row indices grouped by island.
    constraints: Vec<usize>,
    /// Each island's range in `manifolds` and in `constraints`.
    ranges: Vec<(Range<usize>, Range<usize>)>,
}

impl SolverIslands {
    /// Partition rows by the movable bodies they share: first the
    /// manifolds, then the joint rows, each given as its pair of bodies. A row
    /// with no movable body — its bodies are gone — is an island of its own.
    pub fn build(
        &mut self,
        bodies: &Arena<RigidBody>,
        manifold_pairs: impl ExactSizeIterator<Item = BodyPair> + Clone,
        constraint_pairs: impl Iterator<Item = BodyPair> + Clone,
    ) {
        let manifold_count = manifold_pairs.len();
        let arena_slots = bodies.capacity();
        self.parent.clear();
        self.parent.extend(0..arena_slots as u32);

        let movable = |handle: Option<RigidBodyHandle>| {
            handle
                .filter(|h| bodies.get(h.0).is_some_and(|body| !body.is_static()))
                .map(|h| h.0.into_raw_parts().0)
        };

        let row_pairs = manifold_pairs.chain(constraint_pairs);
        for (a, b) in row_pairs.clone() {
            if let (Some(a), Some(b)) = (movable(a), movable(b)) {
                self.union(a, b);
            }
        }

        self.island_of_root.clear();
        self.island_of_root.resize(arena_slots, u32::MAX);
        self.island_of_row.clear();
        let mut island_count = 0u32;
        for (a, b) in row_pairs {
            let island = match movable(b).or(movable(a)) {
                Some(slot) => {
                    let root = self.find(slot);
                    if self.island_of_root[root] == u32::MAX {
                        self.island_of_root[root] = island_count;
                        island_count += 1;
                    }
                    self.island_of_root[root]
                }
                None => {
                    island_count += 1;
                    island_count - 1
                }
            };
            self.island_of_row.push(island);
        }

        let (manifold_islands, constraint_islands) = self.island_of_row.split_at(manifold_count);
        let manifold_ranges = group(manifold_islands, island_count, &mut self.manifolds);
        let constraint_ranges = group(constraint_islands, island_count, &mut self.constraints);
        self.ranges.clear();
        self.ranges
            .extend(manifold_ranges.into_iter().zip(constraint_ranges));
    }

    pub fn len(&self) -> usize {
        self.ranges.len()
    }

    /// Reorder per-manifold `items` island by island, each island's in the
    /// order it was given; [`SolverIslands::split_manifolds`] cuts the result
    /// into islands.
    pub fn arrange_manifolds<T>(&self, items: impl Iterator<Item = T>) -> Vec<T> {
        arrange(items, &self.manifolds)
    }

    /// [`SolverIslands::arrange_manifolds`] for per-joint-row items.
    pub fn arrange_constraints<T>(&self, items: impl Iterator<Item = T>) -> Vec<T> {
        arrange(items, &self.constraints)
    }

    /// Cut arranged manifold items into one slice per island, in island order.
    pub fn split_manifolds<'a, T>(&self, arranged: &'a mut [T]) -> Vec<&'a mut [T]> {
        split(arranged, self.ranges.iter().map(|(m, _)| m.len()))
    }

    /// Cut arranged joint row items into one slice per island, in island order.
    pub fn split_constraints<'a, T>(&self, arranged: &'a mut [T]) -> Vec<&'a mut [T]> {
        split(arranged, self.ranges.iter().map(|(_, c)| c.len()))
    }

    fn find(&mut self, slot: usize) -> usize {
        let mut root = slot;
        while self.parent[root] as usize != root {
            root = self.parent[root] as usize;
        }
        let mut current = slot;
        while current != root {
            let next = self.parent[current] as usize;
            self.parent[current] = root as u32;
            current = next;
        }
        root
    }

    fn union(&mut self, a: usize, b: usize) {
        let (a, b) = (self.find(a), self.find(b));
        if a != b {
            self.parent[a.max(b)] = a.min(b) as u32;
        }
    }
}

/// Stable counting sort: fill `grouped` with the indices of `island_of`,
/// island by island, each in its original order, and return each island's
/// range.
fn group(island_of: &[u32], island_count: u32, grouped: &mut Vec<usize>) -> Vec<Range<usize>> {
    let mut ends = vec![0usize; island_count as usize];
    for &island in island_of {
        ends[island as usize] += 1;
    }
    let mut start = 0;
    let ranges: Vec<Range<usize>> = ends
        .iter_mut()
        .map(|count| {
            let range = start..start + *count;
            start = range.end;
            *count = range.start;
            range
        })
        .collect();

    grouped.clear();
    grouped.resize(island_of.len(), 0);
    for (index, &island) in island_of.iter().enumerate() {
        let next = &mut ends[island as usize];
        grouped[*next] = index;
        *next += 1;
    }
    ranges
}

/// `items` in the order `order` lists their indices.
fn arrange<T>(items: impl Iterator<Item = T>, order: &[usize]) -> Vec<T> {
    let mut by_index: Vec<Option<T>> = items.map(Some).collect();
    order
        .iter()
        .map(|&index| {
            by_index[index]
                .take()
                .expect("each row belongs to one island")
        })
        .collect()
}

/// `items` cut into consecutive slices of the given lengths.
fn split<T>(mut items: &mut [T], lengths: impl Iterator<Item = usize>) -> Vec<&mut [T]> {
    lengths
        .map(|length| {
            let (head, tail) = std::mem::take(&mut items).split_at_mut(length);
            items = tail;
            head
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grouping_keeps_each_islands_rows_in_order() {
        let mut grouped = Vec::new();
        let ranges = group(&[1, 0, 1, 2, 0], 3, &mut grouped);
        assert_eq!(grouped, vec![1, 4, 0, 2, 3]);
        assert_eq!(ranges, vec![0..2, 2..4, 4..5]);
    }
}
