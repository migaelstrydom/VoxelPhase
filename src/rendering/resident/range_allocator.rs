/// Hands out ranges of a linear space and takes them back: the bookkeeping
/// behind a buffer that many meshes share, with no Vulkan in it.
///
/// First fit over a free list kept sorted by offset, so a freed range merges
/// with its neighbours at once and the space does not splinter into gaps too
/// small to reuse.
///
/// ```text
///   capacity ─────────────────────────────────────────────────▶
///   [ used ][ free ][ used ][ used ][        free             ]
///            ▲ first fit lands here if it is big enough
/// ```
#[derive(Debug, Clone)]
pub struct RangeAllocator {
    /// Size of the space, in whatever unit the caller counts in.
    capacity: u64,
    /// Free ranges as (offset, length), sorted by offset, never adjacent.
    free: Vec<(u64, u64)>,
}

impl RangeAllocator {
    /// A space of `capacity` units, all of it free.
    pub fn new(capacity: u64) -> Self {
        let free = if capacity > 0 {
            vec![(0, capacity)]
        } else {
            Vec::new()
        };
        Self { capacity, free }
    }

    /// Size of the space.
    pub fn capacity(&self) -> u64 {
        self.capacity
    }

    /// Units currently handed out.
    pub fn used(&self) -> u64 {
        self.capacity - self.free.iter().map(|&(_, len)| len).sum::<u64>()
    }

    /// Take `len` units, returning where they start, or `None` when no free
    /// range is long enough. A zero-length request is always granted, at 0.
    pub fn allocate(&mut self, len: u64) -> Option<u64> {
        if len == 0 {
            return Some(0);
        }
        let index = self.free.iter().position(|&(_, free)| free >= len)?;
        let (offset, free) = self.free[index];
        if free == len {
            self.free.remove(index);
        } else {
            self.free[index] = (offset + len, free - len);
        }
        Some(offset)
    }

    /// Give back a range `allocate` handed out.
    pub fn free(&mut self, offset: u64, len: u64) {
        if len == 0 {
            return;
        }
        debug_assert!(offset + len <= self.capacity);
        let index = self.free.partition_point(|&(start, _)| start < offset);
        debug_assert!(
            index == 0 || self.free[index - 1].0 + self.free[index - 1].1 <= offset,
            "freed range overlaps the free range before it"
        );
        debug_assert!(
            index == self.free.len() || offset + len <= self.free[index].0,
            "freed range overlaps the free range after it"
        );

        let joins_before = index > 0 && self.free[index - 1].0 + self.free[index - 1].1 == offset;
        let joins_after = index < self.free.len() && offset + len == self.free[index].0;
        match (joins_before, joins_after) {
            (true, true) => {
                let after = self.free.remove(index);
                self.free[index - 1].1 += len + after.1;
            }
            (true, false) => self.free[index - 1].1 += len,
            (false, true) => self.free[index] = (offset, len + self.free[index].1),
            (false, false) => self.free.insert(index, (offset, len)),
        }
    }

    /// Extend the space to `capacity` units. Everything handed out keeps its
    /// offset; the new room joins the free list at the end.
    pub fn grow(&mut self, capacity: u64) {
        debug_assert!(capacity >= self.capacity);
        let added = capacity - self.capacity;
        if added == 0 {
            return;
        }
        match self.free.last_mut() {
            Some((start, len)) if *start + *len == self.capacity => *len += added,
            _ => self.free.push((self.capacity, added)),
        }
        self.capacity = capacity;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn allocations_are_first_fit_and_disjoint() {
        let mut space = RangeAllocator::new(100);
        assert_eq!(space.allocate(30), Some(0));
        assert_eq!(space.allocate(30), Some(30));
        assert_eq!(space.allocate(50), None);
        assert_eq!(space.allocate(40), Some(60));
        assert_eq!(space.used(), 100);
    }

    #[test]
    fn a_freed_range_is_reused() {
        let mut space = RangeAllocator::new(100);
        let a = space.allocate(30).unwrap();
        let _b = space.allocate(30).unwrap();
        space.free(a, 30);
        assert_eq!(space.allocate(20), Some(0));
        assert_eq!(space.allocate(10), Some(20));
    }

    #[test]
    fn neighbours_merge_on_free() {
        let mut space = RangeAllocator::new(90);
        let a = space.allocate(30).unwrap();
        let b = space.allocate(30).unwrap();
        let c = space.allocate(30).unwrap();
        space.free(a, 30);
        space.free(c, 30);
        space.free(b, 30);
        // One free range again, or a 90 would not fit.
        assert_eq!(space.allocate(90), Some(0));
    }

    #[test]
    fn growing_extends_the_trailing_free_range() {
        let mut space = RangeAllocator::new(50);
        assert_eq!(space.allocate(40), Some(0));
        space.grow(100);
        // The 10 left at the end and the 50 added are one range.
        assert_eq!(space.allocate(60), Some(40));
    }

    #[test]
    fn growing_a_full_space_starts_a_free_range_at_the_old_end() {
        let mut space = RangeAllocator::new(50);
        assert_eq!(space.allocate(50), Some(0));
        space.grow(80);
        assert_eq!(space.allocate(30), Some(50));
    }

    #[test]
    fn zero_length_requests_take_nothing() {
        let mut space = RangeAllocator::new(10);
        assert_eq!(space.allocate(0), Some(0));
        space.free(0, 0);
        assert_eq!(space.used(), 0);
    }
}
