//! Bounds sorted along one axis, asked which of them meet a region.
//!
//! The sort is sweep-and-prune's, kept between queries: built once over a
//! set of world-space bounds, it answers any number of region queries with a
//! binary search and a scan over the entries whose interval along the axis
//! can reach the region, instead of a test against every entry.
//!
//! ```text
//!   bounds ──rebuild──▶ sorted by min along the widest axis
//!                         │ binary search: min ≤ region.max
//!   region ──query──────▶ │ scan back while min ≥ region.min − longest
//!                         └▶ full AABB test ──▶ indices, ascending
//! ```

use crate::collision::AABB;

use super::sweep_and_prune::pick_sweep_axis;

/// Extent along the sorted axis past which an entry is kept apart and tested
/// against every query, in metres. One long entry (a bridge deck, a level's
/// floor slab) would otherwise widen every query's scan to its own length.
const LONG_EXTENT: f32 = 4.0;

/// Bounds indexed for region queries along the axis they spread most along.
#[derive(Debug, Default)]
pub struct IntervalIndex {
    /// The sorted axis: 0 = x, 1 = y, 2 = z.
    axis: usize,
    /// Indices of the short entries, ordered by their minimum along `axis`.
    sorted: Vec<usize>,
    /// Those minima, in the same order: the keys the search runs on.
    starts: Vec<f32>,
    /// Longest extent along `axis` of any short entry: one that starts this
    /// far before a region along the axis can still reach into it.
    longest: f32,
    /// Indices of the entries longer than [`LONG_EXTENT`].
    long: Vec<usize>,
}

impl IntervalIndex {
    /// Index `bounds`, forgetting whatever was indexed before. Keeps its
    /// storage, so a steady world rebuilds without allocating.
    pub fn rebuild(&mut self, bounds: &[AABB]) {
        self.sorted.clear();
        self.starts.clear();
        self.long.clear();
        self.longest = 0.0;
        if bounds.is_empty() {
            return;
        }
        self.axis = pick_sweep_axis(bounds);
        let axis = self.axis;
        for (index, b) in bounds.iter().enumerate() {
            let extent = b.max[axis] - b.min[axis];
            if extent > LONG_EXTENT {
                self.long.push(index);
            } else {
                self.longest = self.longest.max(extent);
                self.sorted.push(index);
            }
        }
        self.sorted
            .sort_unstable_by(|&a, &b| bounds[a].min[axis].total_cmp(&bounds[b].min[axis]));
        self.starts
            .extend(self.sorted.iter().map(|&i| bounds[i].min[axis]));
    }

    /// Append to `out`, in ascending order, the index of every entry of
    /// `bounds` (the slice the index was built over) that meets `region`.
    pub fn query_into(&self, bounds: &[AABB], region: &AABB, out: &mut Vec<usize>) {
        let first = out.len();
        let axis = self.axis;
        // Entries starting past the region's end cannot meet it, nor can
        // those starting further before its start than any entry is long.
        let end = self
            .starts
            .partition_point(|&start| start <= region.max[axis]);
        let begin = self
            .starts
            .partition_point(|&start| start < region.min[axis] - self.longest);
        out.extend(
            self.sorted[begin..end.max(begin)]
                .iter()
                .chain(&self.long)
                .copied()
                .filter(|&i| bounds[i].intersects(region)),
        );
        out[first..].sort_unstable();
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;

    fn cube(x: f32, y: f32, z: f32, half: f32) -> AABB {
        AABB::new(
            Point3::new(x - half, y - half, z - half),
            Point3::new(x + half, y + half, z + half),
        )
    }

    fn brute_force(bounds: &[AABB], region: &AABB) -> Vec<usize> {
        (0..bounds.len())
            .filter(|&i| bounds[i].intersects(region))
            .collect()
    }

    /// Every query answers exactly as testing every entry would, in the same
    /// order, over a scatter of small boxes, a few long ones and regions of
    /// every size.
    #[test]
    fn a_query_finds_exactly_what_testing_everything_finds() {
        let mut bounds: Vec<AABB> = (0..200)
            .map(|i| {
                let f = i as f32;
                cube(
                    (f * 7.31) % 40.0,
                    (f * 3.17) % 5.0,
                    (f * 11.7) % 30.0,
                    0.1 + (f * 0.37) % 0.9,
                )
            })
            .collect();
        bounds.push(AABB::new(
            Point3::new(-5.0, -1.0, -5.0),
            Point3::new(45.0, 0.0, 35.0),
        ));
        bounds.push(AABB::new(
            Point3::new(10.0, 0.0, 2.0),
            Point3::new(30.0, 1.0, 3.0),
        ));
        let mut index = IntervalIndex::default();
        index.rebuild(&bounds);

        for q in 0..100 {
            let f = q as f32;
            let region = cube(
                (f * 5.13) % 42.0,
                (f * 1.9) % 6.0,
                (f * 9.1) % 32.0,
                0.05 + (f * 0.71) % 3.0,
            );
            let mut found = Vec::new();
            index.query_into(&bounds, &region, &mut found);
            assert_eq!(found, brute_force(&bounds, &region), "query {q}");
        }
    }

    #[test]
    fn an_empty_index_finds_nothing() {
        let mut index = IntervalIndex::default();
        index.rebuild(&[]);
        let mut found = Vec::new();
        index.query_into(&[], &cube(0.0, 0.0, 0.0, 1.0), &mut found);
        assert!(found.is_empty());
    }

    /// A rebuild forgets what was indexed before.
    #[test]
    fn a_rebuild_starts_again() {
        let mut index = IntervalIndex::default();
        index.rebuild(&[cube(0.0, 0.0, 0.0, 1.0), cube(9.0, 0.0, 0.0, 1.0)]);
        let moved = [cube(20.0, 0.0, 0.0, 1.0)];
        index.rebuild(&moved);
        let mut found = Vec::new();
        index.query_into(&moved, &cube(0.0, 0.0, 0.0, 1.0), &mut found);
        assert!(found.is_empty());
        index.query_into(&moved, &cube(20.5, 0.0, 0.0, 1.0), &mut found);
        assert_eq!(found, vec![0]);
    }
}
