/// Tracks which bodies the narrowphase currently owns, so CCD can stand down
/// for them without standing down for the whole frame.
///
/// The narrowphase runs once per frame, but a frame is integrated over several
/// substeps. A body that had static contacts at frame start is genuinely the
/// solver's to handle — sweeping it as well would resolve the same contact
/// twice and inject phantom impulses. But that ownership expires: once the body
/// has travelled far enough that the frame-start manifold no longer describes
/// where it is, the solver is holding stale contacts and CCD must take over
/// again, or the body can sweep past geometry unchecked for the rest of the
/// frame.
///
/// Ownership is therefore recorded with the position it was established at, and
/// released once the body has moved a release distance away from it.
use nalgebra::Point3;
use rustc_hash::FxHashMap;

use crate::physics::handle::RigidBodyHandle;

#[derive(Default)]
pub struct NarrowphaseOwnership {
    /// Body position at the moment its owning manifold was generated.
    anchors: FxHashMap<RigidBodyHandle, Point3<f32>>,
}

impl NarrowphaseOwnership {
    pub fn new() -> Self {
        Self::default()
    }

    /// Drop all ownership records. Called before each narrowphase pass.
    pub fn clear(&mut self) {
        self.anchors.clear();
    }

    /// Record that the narrowphase owns `handle`, anchored at `position`.
    pub fn insert(&mut self, handle: RigidBodyHandle, position: Point3<f32>) {
        self.anchors.insert(handle, position);
    }

    /// Whether the narrowphase still owns `handle` at its current `position`.
    ///
    /// `release_distance` is how far the body may drift from its anchor before
    /// the frame-start manifold is considered stale. Sized well above resting
    /// jitter so settled bodies never oscillate back into the CCD path.
    pub fn owns(
        &self,
        handle: RigidBodyHandle,
        position: Point3<f32>,
        release_distance: f32,
    ) -> bool {
        let Some(anchor) = self.anchors.get(&handle) else {
            return false;
        };
        (position - anchor).magnitude_squared() <= release_distance * release_distance
    }
}
