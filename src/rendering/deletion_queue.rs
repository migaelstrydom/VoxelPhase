//! Deferred resource deletion for Vulkan.
//!
//! Resources in Vulkan must not be destroyed while the GPU is still using them.
//! This module provides a deletion queue that holds resources until it's safe
//! to destroy them (typically after N frames have completed).
//!
//! # Usage Pattern
//!
//! ```ignore
//! // When you need to destroy a resource that might be in-flight:
//! deletion_queue.queue(old_buffer);
//!
//! // At frame start, after waiting for the frame fence:
//! deletion_queue.flush(current_frame_number);
//! ```

use std::collections::VecDeque;

/// A queue for deferred destruction of GPU resources.
///
/// Resources are held for a configurable number of frames before being dropped.
/// This ensures the GPU has finished using them before destruction.
pub struct DeletionQueue<T> {
    /// Items pending deletion, with the frame number they were queued on.
    pending: VecDeque<(T, u64)>,
    /// Number of frames to wait before deletion (typically frames_in_flight).
    frames_to_wait: u64,
}

impl<T> DeletionQueue<T> {
    /// Create a new deletion queue.
    ///
    /// # Arguments
    /// * `frames_to_wait` - Number of frames to wait before destroying resources.
    ///   Typically this should be your frames_in_flight count (e.g., 2 for double buffering).
    pub fn new(frames_to_wait: u64) -> Self {
        Self {
            pending: VecDeque::new(),
            frames_to_wait,
        }
    }

    /// Queue a resource for deferred deletion.
    ///
    /// The resource will be held until `frames_to_wait` frames have passed.
    pub fn queue(&mut self, resource: T, current_frame: u64) {
        self.pending.push_back((resource, current_frame));
    }

    /// Flush resources that are safe to delete.
    ///
    /// Call this at the start of each frame, after waiting for the frame fence.
    /// Resources queued `frames_to_wait` or more frames ago will be dropped.
    pub fn flush(&mut self, current_frame: u64) {
        drop(self.take_ready(current_frame));
    }

    /// Remove and return the resources `flush` would drop, for a caller that
    /// has to release them through something other than `Drop`.
    pub fn take_ready(&mut self, current_frame: u64) -> Vec<T> {
        let mut ready = Vec::new();
        // Queued in frame order, so the first one not yet old enough ends it.
        while let Some((_, queued_frame)) = self.pending.front() {
            if current_frame < queued_frame + self.frames_to_wait {
                break;
            }
            if let Some((resource, _)) = self.pending.pop_front() {
                ready.push(resource);
            }
        }
        ready
    }

    /// Force-flush all pending deletions.
    ///
    /// Call this during shutdown after device_wait_idle().
    pub fn flush_all(&mut self) {
        self.pending.clear();
    }
}

impl<T> Default for DeletionQueue<T> {
    fn default() -> Self {
        // Default to 2 frames (standard double-buffering)
        Self::new(2)
    }
}

#[cfg(test)]
impl<T> DeletionQueue<T> {
    pub fn pending_count(&self) -> usize {
        self.pending.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn queued_resources_are_held_for_correct_frames() {
        let mut queue: DeletionQueue<String> = DeletionQueue::new(2);

        // Queue a resource on frame 0
        queue.queue("resource_0".to_string(), 0);
        assert_eq!(queue.pending_count(), 1);

        // Frame 1: too early to delete
        queue.flush(1);
        assert_eq!(queue.pending_count(), 1);

        // Frame 2: now safe to delete (0 + 2 = 2)
        queue.flush(2);
        assert_eq!(queue.pending_count(), 0);
    }

    #[test]
    fn multiple_resources_deleted_in_order() {
        let mut queue: DeletionQueue<i32> = DeletionQueue::new(2);

        queue.queue(1, 0);
        queue.queue(2, 1);
        queue.queue(3, 2);

        // Frame 2: only resource from frame 0 is ready
        queue.flush(2);
        assert_eq!(queue.pending_count(), 2);

        // Frame 3: resources from frames 0 and 1 are ready
        queue.flush(3);
        assert_eq!(queue.pending_count(), 1);

        // Frame 4: all ready
        queue.flush(4);
        assert_eq!(queue.pending_count(), 0);
    }
}
