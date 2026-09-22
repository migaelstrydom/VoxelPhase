//! Groups of rows that share no movable body, found once per substep.
//!
//! Two rows couple when an impulse one applies changes a velocity the other
//! reads. That only happens through a body both touch whose velocity can
//! change; static geometry and static bodies never do, so a pile of blocks on
//! the ground is one island per block, not one for the whole pile.
//!
//! ```text
//!   ContactRows ─┐                        ┌─▶ island 0: manifolds [0, 3], joints []
//!                ├─ union-find ─ group ───┼─▶ island 1: manifolds [1],    joints [0]
//!   joint slots ─┘   over slots           └─▶ island 2: manifolds [2, 4], joints []
//! ```
//!
//! Islands keep their rows in the order they were given, so solving one island
//! at a time gives each of its bodies exactly what the global sweep would have,
//! for the same number of iterations. What islands buy is the freedom to give
//! each its own count, and later its own thread.

use std::ops::Range;

use super::constraint_row::RowSlots;
use super::contact_row::ContactRows;
use super::solver_bodies::SolverBodies;

/// One island: its manifolds and joint rows, as indices in their original
/// order.
pub(crate) struct SolverIsland<'a> {
    pub manifolds: &'a [usize],
    pub constraints: &'a [usize],
}

/// The islands of one substep's rows. Rebuilt every substep; the buffers are
/// kept to avoid reallocating.
#[derive(Debug, Default)]
pub(crate) struct SolverIslands {
    /// Union-find parent per solver slot.
    parent: Vec<u32>,
    /// Island of each solver slot's root, `u32::MAX` until one is assigned.
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
    /// Partition `manifold_count` manifolds and the joint rows by the movable
    /// bodies they share. A row with no gathered body — its bodies are gone —
    /// is an island of its own.
    pub fn build(
        &mut self,
        solver_bodies: &SolverBodies,
        contact_rows: &ContactRows,
        manifold_count: usize,
        constraint_slots: &[RowSlots],
    ) {
        self.parent.clear();
        self.parent.extend(0..solver_bodies.len() as u32);

        let manifold_slots = |mi: usize| {
            contact_rows
                .manifold_slots(mi)
                .map_or((None, None), |(a, b)| (a, Some(b)))
        };
        let movable = |slot: Option<usize>| slot.filter(|&s| solver_bodies.is_movable(s));

        let row_slots = (0..manifold_count)
            .map(manifold_slots)
            .chain(constraint_slots.iter().copied());
        for (a, b) in row_slots.clone() {
            if let (Some(a), Some(b)) = (movable(a), movable(b)) {
                self.union(a, b);
            }
        }

        self.island_of_root.clear();
        self.island_of_root.resize(solver_bodies.len(), u32::MAX);
        self.island_of_row.clear();
        let mut island_count = 0u32;
        for (a, b) in row_slots {
            let anchor = movable(b).or(movable(a)).or(b).or(a);
            let island = match anchor {
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

    pub fn iter(&self) -> impl Iterator<Item = SolverIsland<'_>> {
        self.ranges.iter().map(|(m, c)| SolverIsland {
            manifolds: &self.manifolds[m.clone()],
            constraints: &self.constraints[c.clone()],
        })
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
