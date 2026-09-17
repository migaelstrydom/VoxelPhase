//! How many times a cell has already cracked.
//!
//! Crazing is recursive by nature: a cell that is hit is replaced by a web
//! of smaller cells, and each of those is a cell that can be hit. Left
//! alone the count of children on one sheet grows without bound — every
//! child a hull collider, a narrowphase pair and a slice of the rebuilt
//! model — and a pane that has been worked over for a few seconds costs
//! more than the rest of the scene put together.
//!
//! So a cell remembers how deep in that recursion it was born. Past the
//! sheet's limit a hit no longer cracks a shard into smaller ones; it
//! knocks the whole shard out, which is what the crazing rule already does
//! for a cell too small to be a pane. The pieces that fall keep the size
//! the first break gave them.

/// Per-child craze depth: 0 for the pane a sheet is spawned as, one more
/// than its parent for every cell a craze creates.
#[derive(Debug, Default)]
pub struct CrazeDepths {
    depth: Vec<u32>,
}

impl CrazeDepths {
    /// A sheet of `child_count` uncracked children.
    pub fn new(child_count: usize) -> Self {
        Self {
            depth: vec![0; child_count],
        }
    }

    /// How deep `child` was born. A child the tracker has never seen counts
    /// as uncracked, so a sheet that gains a collider some other way is
    /// still allowed to break.
    pub fn of(&self, child: usize) -> u32 {
        self.depth.get(child).copied().unwrap_or(0)
    }

    /// Record the depth of a cell the crazing has just created.
    pub fn set(&mut self, child: usize, depth: u32) {
        if self.depth.len() <= child {
            self.depth.resize(child + 1, 0);
        }
        self.depth[child] = depth;
    }

    /// Follow the children through a change to the body's collider list:
    /// `order[new]` is the old index now at `new`, `None` for a fresh cell.
    pub fn reindex(&mut self, order: &[Option<usize>]) {
        self.depth = order
            .iter()
            .map(|old| old.map_or(0, |old| self.of(old)))
            .collect();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fresh_sheet_is_uncracked() {
        let depths = CrazeDepths::new(3);
        assert_eq!(depths.of(0), 0);
        assert_eq!(depths.of(2), 0);
        assert_eq!(depths.of(9), 0);
    }

    #[test]
    fn depth_follows_its_cell_through_a_reindex() {
        let mut depths = CrazeDepths::new(3);
        depths.set(2, 1);
        depths.reindex(&[Some(2), None, Some(0)]);
        assert_eq!(depths.of(0), 1);
        assert_eq!(depths.of(1), 0);
        assert_eq!(depths.of(2), 0);
    }

    #[test]
    fn a_cell_set_past_the_end_grows_the_tracker() {
        let mut depths = CrazeDepths::new(1);
        depths.set(4, 2);
        assert_eq!(depths.of(4), 2);
        assert_eq!(depths.of(3), 0);
    }
}
