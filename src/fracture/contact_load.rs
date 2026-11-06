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
use rustc_hash::FxHashMap;

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

/// Whose weight sets the churn deadband a spike must clear.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum Deadband {
    /// One frame of the whole body's weight. Right for a compound whose
    /// children all carry the structure: a bridge redistributes the whole
    /// deck between its beams.
    #[default]
    WholeBody,
    /// One frame of each child's own weight. Right for a compound where a
    /// heavy part must not hide what happens to a light one: a stone frame
    /// around a pane would otherwise mask every blow the glass takes.
    OwnWeight,
}

/// The contact impulse each child of a compound carried last frame, and enough
/// context to tell a real impact from the solver resuming work.
#[derive(Debug, Default)]
pub struct ContactLoadTracker {
    /// Last frame's contact impulse per child, for the children seen then.
    levels: FxHashMap<ColliderHandle, f32>,
    /// Whose weight the churn deadband is taken from.
    deadband: Deadband,
    /// Whether the body was asleep when `advance` last ran, so that the frame
    /// it wakes can re-baseline rather than report a difference.
    was_asleep: bool,
}

impl ContactLoadTracker {
    /// The same tracker judging spikes against a different weight.
    pub fn with_deadband(mut self, deadband: Deadband) -> Self {
        self.deadband = deadband;
        self
    }

    /// The tracker of a piece split off with `colliders`: their baselines,
    /// the same deadband, and the body's sleep as it was.
    pub fn share(&self, colliders: &[ColliderHandle]) -> Self {
        Self {
            levels: colliders
                .iter()
                .filter_map(|c| Some((*c, *self.levels.get(c)?)))
                .collect(),
            deadband: self.deadband,
            was_asleep: self.was_asleep,
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
    ///
    /// The baseline is kept by collider, so a child keeps its own across any
    /// change to the list around it; a child not seen last frame starts from
    /// no load, and one no longer listed is forgotten.
    pub fn advance(
        &mut self,
        world: &PhysicsWorld,
        body: RigidBodyHandle,
        collider_handles: &[ColliderHandle],
        frame_dt: f32,
    ) -> Vec<ContactSpike> {
        if world.is_sleeping(body) {
            self.was_asleep = true;
            return vec![ContactSpike::default(); collider_handles.len()];
        }

        // A body that never moves, or one the world holds up, carries its
        // weight through what holds it rather than through its children's
        // contacts, so it gets no deadband. With one, a held pane's own mass
        // hid every footstep on it.
        let weight_per_frame = world.config().gravity.magnitude() * frame_dt;
        let body_deadband = world
            .body(body)
            .filter(|b| !b.is_static() && !world.is_held_by_world(body))
            .map(|b| b.mass() * weight_per_frame)
            .unwrap_or(0.0);
        let deadband_of = |handle: ColliderHandle| match self.deadband {
            Deadband::WholeBody => body_deadband,
            Deadband::OwnWeight if body_deadband == 0.0 => 0.0,
            Deadband::OwnWeight => world
                .collider(handle)
                .map_or(0.0, |c| c.mass() * weight_per_frame),
        };

        let waking = std::mem::replace(&mut self.was_asleep, false);
        let readings: Vec<ContactReading> = collider_handles
            .iter()
            .map(|handle| {
                let impact = world.impacts().for_collider(*handle);
                ContactReading {
                    child: *handle,
                    level: impact.map(|i| i.total_impulse).unwrap_or(0.0),
                    point: impact.map(|i| i.point).unwrap_or_else(Point3::origin),
                    deadband: deadband_of(*handle),
                }
            })
            .collect();
        self.take(&readings, waking)
    }

    /// Turn this frame's readings into spikes against the stored baseline,
    /// and make them the baseline.
    fn take(&mut self, readings: &[ContactReading], waking: bool) -> Vec<ContactSpike> {
        let spikes = readings
            .iter()
            .map(|reading| ContactSpike {
                magnitude: if waking {
                    0.0
                } else {
                    let last = self.levels.get(&reading.child).copied().unwrap_or(0.0);
                    (reading.level - last - reading.deadband).max(0.0)
                },
                point: reading.point,
            })
            .collect();
        self.levels = readings.iter().map(|r| (r.child, r.level)).collect();
        spikes
    }
}

/// One child's contact this frame, as the tracker reads it.
struct ContactReading {
    child: ColliderHandle,
    /// Contact impulse the child took this frame, N·s.
    level: f32,
    /// Where the largest single impulse into it landed, in world space.
    point: Point3<f32>,
    /// Share of the body's weight a spike on it must clear, N·s.
    deadband: f32,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reading(slot: usize, level: f32) -> ContactReading {
        ContactReading {
            child: ColliderHandle::for_test(slot),
            level,
            point: Point3::origin(),
            deadband: 0.0,
        }
    }

    fn magnitudes(spikes: Vec<ContactSpike>) -> Vec<f32> {
        spikes.into_iter().map(|s| s.magnitude).collect()
    }

    /// A survivor that keeps carrying the same load across a break must not
    /// read that load as new the moment its place in the list shifts down.
    #[test]
    fn the_baseline_follows_its_child_through_a_break() {
        let mut tracker = ContactLoadTracker::default();
        tracker.take(
            &[
                reading(0, 10.0),
                reading(1, 20.0),
                reading(2, 30.0),
                reading(3, 40.0),
            ],
            false,
        );

        let spikes = tracker.take(&[reading(3, 40.0), reading(1, 20.0)], false);

        assert_eq!(magnitudes(spikes), vec![0.0, 0.0]);
    }

    /// Children that were not being tracked at all start from zero rather than
    /// inheriting a neighbour's load.
    #[test]
    fn a_child_not_seen_before_starts_from_no_load() {
        let mut tracker = ContactLoadTracker::default();
        tracker.take(&[reading(0, 10.0), reading(1, 20.0)], false);

        let spikes = tracker.take(&[reading(1, 20.0), reading(7, 20.0)], false);

        assert_eq!(magnitudes(spikes), vec![0.0, 20.0]);
    }
}
