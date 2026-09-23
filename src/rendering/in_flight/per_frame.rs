use std::ops::{Index, IndexMut};

use crate::core::error::EngineResult;

/// How many frames the CPU may record ahead of the GPU finishing them.
///
/// Two lets the CPU record one frame while the GPU draws the one before, so a
/// frame costs the slower of the two rather than their sum. A third would add
/// a frame of latency and no further overlap: with two, a side only waits on
/// the other when it is the faster one.
pub const FRAMES_IN_FLIGHT: usize = 2;

/// Which of the frames in flight is being recorded.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FrameSlot(usize);

impl FrameSlot {
    /// The slot the frame after this one records into.
    pub fn next(self) -> Self {
        Self((self.0 + 1) % FRAMES_IN_FLIGHT)
    }

    pub fn index(self) -> usize {
        self.0
    }

    /// Every slot, in order.
    pub fn all() -> impl Iterator<Item = FrameSlot> {
        (0..FRAMES_IN_FLIGHT).map(FrameSlot)
    }
}

/// One `T` per frame in flight.
///
/// For anything the CPU writes while recording a frame and the GPU reads while
/// drawing it: the CPU fills one copy while the GPU may still be reading
/// another, and a slot's copy is only touched again once that slot's fence has
/// signalled.
pub struct PerFrame<T> {
    /// Indexed by [`FrameSlot::index`].
    items: Vec<T>,
}

impl<T> PerFrame<T> {
    pub fn new(make: impl FnMut(FrameSlot) -> T) -> Self {
        Self {
            items: FrameSlot::all().map(make).collect(),
        }
    }

    pub fn try_new(make: impl FnMut(FrameSlot) -> EngineResult<T>) -> EngineResult<Self> {
        Ok(Self {
            items: FrameSlot::all().map(make).collect::<EngineResult<_>>()?,
        })
    }

    pub fn iter(&self) -> impl Iterator<Item = &T> {
        self.items.iter()
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut T> {
        self.items.iter_mut()
    }
}

impl<T> Index<FrameSlot> for PerFrame<T> {
    type Output = T;

    fn index(&self, slot: FrameSlot) -> &T {
        &self.items[slot.0]
    }
}

impl<T> IndexMut<FrameSlot> for PerFrame<T> {
    fn index_mut(&mut self, slot: FrameSlot) -> &mut T {
        &mut self.items[slot.0]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slots_cycle_through_every_copy() {
        let mut slot = FrameSlot::default();
        let mut visited = Vec::new();
        for _ in 0..FRAMES_IN_FLIGHT * 2 {
            visited.push(slot.index());
            slot = slot.next();
        }
        let expected: Vec<usize> = (0..FRAMES_IN_FLIGHT).chain(0..FRAMES_IN_FLIGHT).collect();
        assert_eq!(visited, expected);
    }

    #[test]
    fn each_slot_has_its_own_copy() {
        let mut copies = PerFrame::new(|slot| slot.index() * 10);
        copies[FrameSlot::default().next()] += 1;
        assert_eq!(copies.iter().copied().collect::<Vec<_>>(), vec![0, 11]);
    }
}
