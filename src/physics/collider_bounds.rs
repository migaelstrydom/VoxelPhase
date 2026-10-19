//! Every body collider's bounds where it last stood, indexed so a scene query
//! asks only those near it.
//!
//! ```text
//!   world mutated (a step, a body moved, made or removed) ──▶ pose epoch + 1
//!   query ──▶ epoch moved on? ──yes──▶ re-take every collider's bounds,
//!                │                       re-sort the IntervalIndex
//!                └──────────────────────▶ IntervalIndex::query_into ──▶ colliders
//! ```
//!
//! Bounds are taken lazily, by the first query after anything moved, so a
//! frame's queries share one sort however many there are, and a frame with
//! none pays nothing. The answer is exactly what testing every collider's
//! bounds would give, in the arena order such a scan visits them, so a query
//! that switches to the index finds the same things in the same order.

use generational_arena::Arena;
use nalgebra::Point3;

use crate::collision::shape_view::ShapeView;
use crate::collision::AABB;

use super::body::RigidBody;
use super::broadphase::IntervalIndex;
use super::collider::Collider;
use super::handle::{ColliderHandle, RigidBodyHandle};

/// Margin added to every bound a query filters with, so a pair that touches
/// exactly is never filtered out by rounding.
pub(crate) const BOUNDS_MARGIN: f32 = 0.01;

/// The colliders' bounds as of one pose epoch, and their index.
#[derive(Debug, Default)]
pub(crate) struct ColliderBounds {
    /// The pose epoch the bounds were taken at; `None` before the first.
    epoch: Option<u64>,
    /// Each entry's body and collider, in arena order.
    entries: Vec<(RigidBodyHandle, ColliderHandle)>,
    /// Each entry's bounds, padded by [`BOUNDS_MARGIN`].
    bounds: Vec<AABB>,
    index: IntervalIndex,
    /// Indices the last query found, kept for its storage.
    found: Vec<usize>,
}

impl ColliderBounds {
    /// Every collider whose bounds meet `region`, in arena order, taking
    /// the bounds afresh first if the world has moved since `epoch`.
    pub(crate) fn meeting(
        &mut self,
        epoch: u64,
        bodies: &Arena<RigidBody>,
        colliders: &Arena<Collider>,
        region: &AABB,
        out: &mut Vec<(RigidBodyHandle, ColliderHandle)>,
    ) {
        if self.epoch != Some(epoch) {
            self.take(bodies, colliders);
            self.epoch = Some(epoch);
        }
        self.found.clear();
        self.index.query_into(&self.bounds, region, &mut self.found);
        out.extend(self.found.iter().map(|&i| self.entries[i]));
    }

    fn take(&mut self, bodies: &Arena<RigidBody>, colliders: &Arena<Collider>) {
        self.entries.clear();
        self.bounds.clear();
        for (index, body) in bodies.iter() {
            for &collider_handle in body.colliders() {
                let Some(collider) = colliders.get(collider_handle.0) else {
                    continue;
                };
                let transform = collider.world_transform(body.position(), body.rotation());
                let view = ShapeView {
                    center: Point3::from(transform.translation.vector),
                    rotation: transform.rotation,
                    shape: collider.shape(),
                };
                self.entries.push((RigidBodyHandle(index), collider_handle));
                self.bounds.push(view.query_aabb(BOUNDS_MARGIN));
            }
        }
        self.index.rebuild(&self.bounds);
    }
}
