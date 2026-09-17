//! How many times a child has already been broken.
//!
//! Breaking a compound's child into pieces is recursive by nature: each
//! piece is itself a child that can be hit. Left alone the count of children
//! on one body grows without bound — every child a collider, a narrowphase
//! pair and a slice of the rebuilt model — and an object that has been
//! worked over for a few seconds costs more than the rest of the scene put
//! together.
//!
//! So a piece remembers how deep in that recursion it was born. Past the
//! object's limit a hit no longer breaks a piece into smaller ones; it
//! knocks the whole piece out, which is what a break rule already does for a
//! piece too small to be worth dividing. What falls keeps the size the first
//! break gave it.
//!
//! Shared by every way of breaking a child — a pane crazing into shards, a
//! block cleaving into wedges — because the cost it guards against is the
//! same one, and so is the remedy.

/// Per-child break depth: 0 for a child the object was spawned with, one
/// more than its parent for every piece a break creates.
#[derive(Debug, Default)]
pub struct BreakDepths {
    depth: Vec<u32>,
}

impl BreakDepths {
    /// An object of `child_count` unbroken children.
    pub fn new(child_count: usize) -> Self {
        Self {
            depth: vec![0; child_count],
        }
    }

    /// How deep `child` was born. A child the tracker has never seen counts
    /// as unbroken, so an object that gains a collider some other way is
    /// still allowed to break.
    pub fn of(&self, child: usize) -> u32 {
        self.depth.get(child).copied().unwrap_or(0)
    }

    /// Record the depth of a piece a break has just created.
    pub fn set(&mut self, child: usize, depth: u32) {
        if self.depth.len() <= child {
            self.depth.resize(child + 1, 0);
        }
        self.depth[child] = depth;
    }

    /// Follow the children through a change to the body's collider list:
    /// `order[new]` is the old index now at `new`, `None` for a fresh piece.
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
    fn a_fresh_object_is_unbroken() {
        let depths = BreakDepths::new(3);
        assert_eq!(depths.of(0), 0);
        assert_eq!(depths.of(2), 0);
        assert_eq!(depths.of(9), 0);
    }

    #[test]
    fn depth_follows_its_piece_through_a_reindex() {
        let mut depths = BreakDepths::new(3);
        depths.set(2, 1);
        depths.reindex(&[Some(2), None, Some(0)]);
        assert_eq!(depths.of(0), 1);
        assert_eq!(depths.of(1), 0);
        assert_eq!(depths.of(2), 0);
    }

    #[test]
    fn a_piece_set_past_the_end_grows_the_tracker() {
        let mut depths = BreakDepths::new(1);
        depths.set(4, 2);
        assert_eq!(depths.of(4), 2);
        assert_eq!(depths.of(3), 0);
    }
}
