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

use rustc_hash::FxHashMap;

use crate::physics::ColliderHandle;

/// Per-child break depth: 0 for a child the object was spawned with, one
/// more than its parent for every piece a break creates.
///
/// Kept by collider handle, so a depth stays with its piece however the
/// body's collider list is reordered, and goes with it when it is split off
/// onto a body of its own.
#[derive(Debug, Default)]
pub struct BreakDepths {
    /// Depth of every piece a break created; a child absent is unbroken.
    depth: FxHashMap<ColliderHandle, u32>,
}

impl BreakDepths {
    /// How deep `child` was born. A child the tracker has never seen counts
    /// as unbroken, so an object that gains a collider some other way is
    /// still allowed to break.
    pub fn of(&self, child: ColliderHandle) -> u32 {
        self.depth.get(&child).copied().unwrap_or(0)
    }

    /// Record the depth of a piece a break has just created.
    pub fn set(&mut self, child: ColliderHandle, depth: u32) {
        self.depth.insert(child, depth);
    }

    /// The depths of `children` alone, for a piece split off with them.
    pub fn share(&self, children: &[ColliderHandle]) -> Self {
        Self {
            depth: children
                .iter()
                .filter_map(|c| Some((*c, *self.depth.get(c)?)))
                .collect(),
        }
    }

    /// Forget every piece but `children`.
    pub fn retain(&mut self, children: &[ColliderHandle]) {
        self.depth.retain(|child, _| children.contains(child));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn child(slot: usize) -> ColliderHandle {
        ColliderHandle::for_test(slot)
    }

    #[test]
    fn a_fresh_object_is_unbroken() {
        let depths = BreakDepths::default();
        assert_eq!(depths.of(child(0)), 0);
        assert_eq!(depths.of(child(2)), 0);
        assert_eq!(depths.of(child(9)), 0);
    }

    #[test]
    fn depth_stays_with_its_piece() {
        let mut depths = BreakDepths::default();
        depths.set(child(2), 1);
        assert_eq!(depths.of(child(2)), 1);
        assert_eq!(depths.of(child(0)), 0);
        assert_eq!(depths.of(child(1)), 0);
    }
}
