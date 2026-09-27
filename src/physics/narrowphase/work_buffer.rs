//! Scratch storage reused across narrowphase passes.

use rustc_hash::FxHashSet;

use crate::collision::AABB;
use crate::physics::broadphase::SweepAndPrune;
use crate::physics::handle::ColliderHandle;
use crate::physics::pipeline::pair::PairManifold;

use super::collider_state::ColliderState;

/// Reusable work buffer for the narrowphase pipeline.
///
/// All internal `Vec`s are cleared (but not deallocated) each frame,
/// eliminating per-frame heap allocations in the narrowphase hot path. The
/// buffer should be created once and reused across frames.
///
/// Both contact-generation passes append into the same `manifolds` list, so
/// resetting it is the caller's job rather than either pass's: call
/// [`begin_frame`](Self::begin_frame) once, then run the static and dynamic
/// passes in that order.
pub struct NarrowphaseWorkBuffer {
    /// Manifolds produced this frame, static contacts first.
    pub(super) manifolds: Vec<PairManifold>,
    pub(super) states: Vec<ColliderState>,
    pub(super) bounds: Vec<AABB>,
    /// Per collider in `states`: whether it is in the speculative band, and so
    /// bounded, paired and predicted over the whole frame's travel.
    pub(super) speculative: Vec<bool>,
    /// Pair culling over `bounds`, holding its own sort scratch.
    pub(super) broadphase: SweepAndPrune,
    pub(super) pairs: Vec<(usize, usize)>,
    pub(super) active_sat_pairs: FxHashSet<(ColliderHandle, ColliderHandle)>,
    pub(super) active_gjk_pairs: FxHashSet<(ColliderHandle, ColliderHandle)>,
}

impl NarrowphaseWorkBuffer {
    pub fn new() -> Self {
        Self {
            manifolds: Vec::new(),
            states: Vec::new(),
            bounds: Vec::new(),
            speculative: Vec::new(),
            broadphase: SweepAndPrune::new(),
            pairs: Vec::new(),
            active_sat_pairs: FxHashSet::default(),
            active_gjk_pairs: FxHashSet::default(),
        }
    }

    /// Drop the previous frame's contents without releasing capacity.
    ///
    /// Call once per frame, before the static and dynamic passes.
    pub fn begin_frame(&mut self) {
        self.manifolds.clear();
        self.clear_pair_scratch();
    }

    /// Clear only the broadphase scratch, leaving accumulated manifolds intact.
    pub(super) fn clear_pair_scratch(&mut self) {
        self.states.clear();
        self.bounds.clear();
        self.speculative.clear();
        self.pairs.clear();
        self.active_sat_pairs.clear();
        self.active_gjk_pairs.clear();
    }

    /// Every manifold generated this frame.
    pub fn manifolds(&self) -> &[PairManifold] {
        &self.manifolds
    }
}

impl Default for NarrowphaseWorkBuffer {
    fn default() -> Self {
        Self::new()
    }
}
