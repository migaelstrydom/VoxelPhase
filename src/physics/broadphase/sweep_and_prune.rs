//! Sort-and-sweep pair culling over a set of world-space bounds.
//!
//! Knows nothing about colliders, bodies, or time: it is given a list of AABBs
//! and hands back the indices of those that overlap. The discrete narrowphase
//! fills that list with per-collider bounds at frame start; CCD fills it with
//! bounds swept along each body's motion. Both want the same sort-and-sweep,
//! and neither should own a private copy of it.

use crate::collision::AABB;

/// Sort-and-sweep broadphase with reusable scratch storage.
///
/// The sorted index list persists across calls so a steady-state world does no
/// per-frame allocation. Create one per pipeline stage that needs pairs.
#[derive(Default)]
pub struct SweepAndPrune {
    /// Indices into the caller's bounds, ordered along the sweep axis.
    sorted: Vec<usize>,
}

impl SweepAndPrune {
    pub fn new() -> Self {
        Self::default()
    }

    /// Append every overlapping pair of `bounds` to `pairs` as index pairs.
    ///
    /// Each unordered pair appears once, as (lower index, higher index), and
    /// the pairs come in ascending order. What overlaps decides the output,
    /// never where anything else is: the sweep's order does depend on every
    /// bound (its axis is the widest spread, and a stack of boxes ties on the
    /// other two), and a solver given a stack's contacts in another order
    /// settles it differently. A heavy box on a column of crates fell over
    /// when something across the level changed shape.
    ///
    /// `is_mobile` reports whether an entry can move over the interval the
    /// bounds describe. A pair in which neither side can move cannot produce a
    /// contact that did not already exist, so it is dropped before the AABB
    /// test. That is what keeps a world full of static scenery from costing
    /// anything: the sweep still walks them, but no pair survives.
    pub fn pairs_into(
        &mut self,
        bounds: &[AABB],
        is_mobile: impl Fn(usize) -> bool,
        pairs: &mut Vec<(usize, usize)>,
    ) {
        self.sorted.clear();
        if bounds.is_empty() {
            return;
        }
        let first_new = pairs.len();

        let axis = pick_sweep_axis(bounds);

        self.sorted.extend(0..bounds.len());
        self.sorted
            .sort_unstable_by(|&a, &b| bounds[a].min[axis].total_cmp(&bounds[b].min[axis]));

        for ii in 0..self.sorted.len() {
            let i = self.sorted[ii];
            let i_max = bounds[i].max[axis];

            for jj in (ii + 1)..self.sorted.len() {
                let j = self.sorted[jj];

                // Sorted by min: once one entry starts past i's end, so does
                // every entry after it.
                if bounds[j].min[axis] > i_max {
                    break;
                }

                if !is_mobile(i) && !is_mobile(j) {
                    continue;
                }

                if bounds[i].intersects(&bounds[j]) {
                    pairs.push((i.min(j), i.max(j)));
                }
            }
        }
        pairs[first_new..].sort_unstable();
    }
}

/// Pick the axis (0=x, 1=y, 2=z) along which the bounds are most spread out.
///
/// Sweeping along the widest axis is what makes the early `break` pay: the
/// narrower the overlap in that axis, the sooner each scan terminates. Any axis
/// yields the same set of pairs, so this is a cost heuristic and nothing more.
pub(super) fn pick_sweep_axis(bounds: &[AABB]) -> usize {
    let mut min = [f32::MAX; 3];
    let mut max = [f32::MIN; 3];
    for b in bounds {
        let c = b.center();
        for (axis, value) in [c.x, c.y, c.z].into_iter().enumerate() {
            min[axis] = min[axis].min(value);
            max[axis] = max[axis].max(value);
        }
    }
    let spread = [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
    if spread[0] >= spread[1] && spread[0] >= spread[2] {
        0
    } else if spread[1] >= spread[2] {
        1
    } else {
        2
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;

    fn cube(center: Point3<f32>, half: f32) -> AABB {
        AABB::new(
            Point3::new(center.x - half, center.y - half, center.z - half),
            Point3::new(center.x + half, center.y + half, center.z + half),
        )
    }

    fn all_mobile(_: usize) -> bool {
        true
    }

    fn pairs_of(bounds: &[AABB]) -> Vec<(usize, usize)> {
        let mut out = Vec::new();
        SweepAndPrune::new().pairs_into(bounds, all_mobile, &mut out);
        out
    }

    /// A column of boxes ties on the two axes it isn't stacked along, so the
    /// sweep orders it by whatever else is in the world. Its pairs must come
    /// out the same, sides and order, however much else there is and wherever
    /// it lies. Enough else that the sort is not a small, stable one.
    #[test]
    fn a_stack_pairs_the_same_whatever_else_there_is() {
        let column: Vec<AABB> = (0..8)
            .map(|i| cube(Point3::new(0.0, i as f32 * 0.99, 0.0), 0.5))
            .collect();
        let alone = pairs_of(&column);
        assert_eq!(alone.len(), 7, "the column's neighbours should pair");
        for scatter in [16, 64, 200] {
            let mut world = column.clone();
            world.extend((0..scatter).map(|k| {
                let k = k as f32;
                cube(
                    Point3::new(
                        20.0 + (k * 7.3) % 90.0,
                        (k * 3.1) % 5.0,
                        (k * 11.7) % 70.0 - 35.0,
                    ),
                    0.5,
                )
            }));
            let column_pairs: Vec<(usize, usize)> = pairs_of(&world)
                .into_iter()
                .filter(|&(a, b)| a < column.len() && b < column.len())
                .collect();
            assert_eq!(column_pairs, alone, "with {scatter} boxes elsewhere");
        }
    }

    #[test]
    fn empty_input_yields_no_pairs() {
        assert!(pairs_of(&[]).is_empty());
    }

    #[test]
    fn overlapping_boxes_pair_and_separated_ones_do_not() {
        let bounds = [
            cube(Point3::new(0.0, 0.0, 0.0), 1.0),
            cube(Point3::new(1.0, 0.0, 0.0), 1.0),
            cube(Point3::new(10.0, 0.0, 0.0), 1.0),
        ];
        assert_eq!(pairs_of(&bounds), vec![(0, 1)]);
    }

    /// Overlap along the sweep axis alone is not overlap: the early `break`
    /// must not be mistaken for the answer.
    #[test]
    fn sweep_axis_overlap_alone_does_not_pair() {
        let bounds = [
            cube(Point3::new(0.0, 0.0, 0.0), 1.0),
            cube(Point3::new(1.0, 50.0, 0.0), 1.0),
            cube(Point3::new(30.0, 0.0, 0.0), 1.0),
        ];
        assert!(pairs_of(&bounds).is_empty());
    }

    #[test]
    fn immobile_pairs_are_dropped() {
        let bounds = [
            cube(Point3::new(0.0, 0.0, 0.0), 1.0),
            cube(Point3::new(1.0, 0.0, 0.0), 1.0),
        ];
        let mut out = Vec::new();
        SweepAndPrune::new().pairs_into(&bounds, |_| false, &mut out);
        assert!(out.is_empty());

        out.clear();
        SweepAndPrune::new().pairs_into(&bounds, |i| i == 0, &mut out);
        assert_eq!(out.len(), 1);
    }

    #[test]
    fn each_pair_is_emitted_once() {
        let bounds: Vec<AABB> = (0..6)
            .map(|i| cube(Point3::new(i as f32 * 0.1, 0.0, 0.0), 1.0))
            .collect();
        let pairs = pairs_of(&bounds);
        let mut deduped = pairs.clone();
        deduped.dedup();
        assert_eq!(pairs, deduped);
        assert_eq!(pairs.len(), 6 * 5 / 2);
    }

    #[test]
    fn sweep_axis_follows_the_widest_spread() {
        let along_z = [
            cube(Point3::new(0.0, 0.0, 0.0), 0.1),
            cube(Point3::new(0.0, 0.0, 20.0), 0.1),
        ];
        assert_eq!(pick_sweep_axis(&along_z), 2);
    }

    /// Scratch is reused across calls, so a second call must not inherit the
    /// first's ordering or leftovers.
    #[test]
    fn reuse_across_calls_is_clean() {
        let mut broadphase = SweepAndPrune::new();
        let first = [
            cube(Point3::new(0.0, 0.0, 0.0), 1.0),
            cube(Point3::new(0.5, 0.0, 0.0), 1.0),
        ];
        let mut out = Vec::new();
        broadphase.pairs_into(&first, all_mobile, &mut out);
        assert_eq!(out.len(), 1);

        out.clear();
        let second = [cube(Point3::new(0.0, 0.0, 0.0), 1.0)];
        broadphase.pairs_into(&second, all_mobile, &mut out);
        assert!(out.is_empty());
    }
}
