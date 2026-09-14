//! Per-child contact load tracking for destructible compounds.
//!
//! A compound body breaks on a *spike* in contact load rather than on the load
//! itself, because the load itself is dominated by whatever the object is
//! already carrying. A heavy compound standing still pushes
//! `mass * gravity * frame_dt` through its lowest children every frame, which
//! for anything massive exceeds a sensible fracture threshold on its own.
//!
//! ```text
//!  ImpactLedger::for_collider  ──►  ContactLoadTracker::advance  ──►  spikes
//!        (this frame's level)            (minus last frame's)        (per child)
//! ```
//!
//! Three transients would otherwise read as impacts and are suppressed here:
//!
//! - **Churn.** A compound of many pieces never reaches a perfectly static
//!   equilibrium: the solver keeps redistributing the object's weight between
//!   its children, and every redistribution is a spike on somebody. What bounds
//!   it is the weight being redistributed, so a deadband of one frame of the
//!   body's own weight is subtracted from every spike. Nothing gravity alone
//!   can account for is treated as an impact.
//!
//! - **Sleeping.** The solver stops running a sleeping body's contacts, so its
//!   levels fall to zero and return in a single frame on waking. The baseline is
//!   frozen for the duration instead of following them down.
//! - **Waking.** Even with a frozen baseline the first awake frame re-solves
//!   from cold warm-start state, so the load redistributes across the children
//!   by a fraction of the object's weight. There is no meaningful difference to
//!   take against a frame that was never solved, so that frame re-baselines.

use nalgebra::Point3;

use crate::physics::{ColliderHandle, PhysicsWorld, RigidBodyHandle};

/// One child's contact spike, and where the impulse behind it landed.
#[derive(Debug, Clone, Copy)]
pub struct ContactSpike {
    /// How much more contact impulse the child took this frame than last, less
    /// one frame of the body's own weight (N·s).
    pub magnitude: f32,
    /// World-space location of the contact that carried the largest single
    /// impulse into this child. Meaningless when `magnitude` is zero.
    pub point: Point3<f32>,
}

impl Default for ContactSpike {
    fn default() -> Self {
        Self {
            magnitude: 0.0,
            point: Point3::origin(),
        }
    }
}

/// The contact impulse each child of a compound carried last frame, and enough
/// context to tell a real impact from the solver resuming work.
#[derive(Debug, Default)]
pub struct ContactLoadTracker {
    /// Last frame's contact impulse per child, indexed by child.
    levels: Vec<f32>,
    /// Whether the body was asleep when `advance` last ran, so that the frame
    /// it wakes can re-baseline rather than report a difference.
    was_asleep: bool,
}

impl ContactLoadTracker {
    /// A tracker for a compound with `child_count` children, carrying no load.
    pub fn new(child_count: usize) -> Self {
        Self {
            levels: vec![0.0; child_count],
            was_asleep: false,
        }
    }

    /// This frame's spike per child, advancing the stored baseline.
    ///
    /// Returns one value per entry of `collider_handles`: how much more contact
    /// impulse that child took this frame than last, less one frame of the
    /// body's own weight. A child whose load fell — it came off the ground, or
    /// the impact is over — reports zero rather than a negative, so callers can
    /// add the result straight onto another load.
    ///
    /// Must be called every frame for every tracked body, including frames
    /// where nothing can break: a baseline that stops advancing lets load build
    /// up unwatched and then reads as one large spike later.
    pub fn advance(
        &mut self,
        world: &PhysicsWorld,
        body: RigidBodyHandle,
        collider_handles: &[ColliderHandle],
        frame_dt: f32,
    ) -> Vec<ContactSpike> {
        self.levels.resize(collider_handles.len(), 0.0);

        if world.is_sleeping(body) {
            self.was_asleep = true;
            return vec![ContactSpike::default(); collider_handles.len()];
        }

        let deadband = world
            .body(body)
            .map(|b| b.mass() * world.config().gravity.magnitude() * frame_dt)
            .unwrap_or(0.0);

        let waking = std::mem::replace(&mut self.was_asleep, false);
        let mut spikes = Vec::with_capacity(collider_handles.len());
        for (child, handle) in collider_handles.iter().enumerate() {
            let impact = world.impacts().for_collider(*handle);
            let level = impact.map(|i| i.total_impulse).unwrap_or(0.0);
            spikes.push(ContactSpike {
                magnitude: if waking {
                    0.0
                } else {
                    (level - self.levels[child] - deadband).max(0.0)
                },
                point: impact.map(|i| i.point).unwrap_or_else(Point3::origin),
            });
            self.levels[child] = level;
        }
        spikes
    }

    /// Reindex the baseline after some children have been removed.
    ///
    /// `kept` lists the old indices that survive, in new-index order. A
    /// survivor that keeps carrying a steady load across a break must not read
    /// that load as new just because its index moved.
    pub fn remap(&mut self, kept: &[usize]) {
        self.levels = kept
            .iter()
            .map(|&old| self.levels.get(old).copied().unwrap_or(0.0))
            .collect();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A survivor that keeps carrying the same load across a break must not
    /// read that load as new the moment its index shifts down.
    #[test]
    fn the_baseline_follows_its_child_through_a_remap() {
        let mut tracker = ContactLoadTracker::new(4);
        tracker.levels = vec![10.0, 20.0, 30.0, 40.0];

        tracker.remap(&[1, 3]);

        assert_eq!(tracker.levels, vec![20.0, 40.0]);
    }

    /// Children that were not being tracked at all start from zero rather than
    /// inheriting a neighbour's load.
    #[test]
    fn a_remap_past_the_end_starts_from_no_load() {
        let mut tracker = ContactLoadTracker::new(2);
        tracker.levels = vec![10.0, 20.0];

        tracker.remap(&[1, 7]);

        assert_eq!(tracker.levels, vec![20.0, 0.0]);
    }
}
