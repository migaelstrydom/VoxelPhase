//! What the drive gain actually spent, per body, per frame.
//!
//! `Actuator::drive_gain` is a named cheat (`docs/TRACTION_DRIVE_DESIGN.md`
//! §11): above `1.0` a body pushes harder through a contact than that
//! contact's friction honestly permits. The decision to allow it is settled;
//! the obligation that came with it is that "deliberate" must be a property of
//! the code rather than of the document, so the amount borrowed is measured
//! rather than assumed.
//!
//! ```text
//!   tangential rows ──► SolverContact::traction ──► TractionLedger ──► DebugLog
//!                        (borrowed, saturated)        per driven body
//! ```
//!
//! Both quantities are already in hand where the bound is computed, so this
//! costs the solve nothing: the rows record, the ledger sums, and nothing
//! reads any of it back into the simulation.

use rustc_hash::FxHashMap;

use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::pair::SolverManifold;

use super::support::{ContactSite, SupportSets};

/// One body's use of its drive gain.
#[derive(Clone, Copy, Debug, Default)]
pub struct TractionUsage {
    /// Impulse this body's traction rows carried over the frame beyond what a
    /// gain of `1.0` would have permitted, in N·s.
    pub borrowed_impulse: f32,
    /// The same, divided by the frame's duration: the borrowed *force* the
    /// body leaned on, which is the number worth watching against its weight.
    pub borrowed_force: f32,
    /// Frames this body has driven through at least one contact.
    pub driving_frames: u32,
    /// Of those, the frames in which at least one row finished pinned at its
    /// bound — the body asking for more than the surface could give.
    pub saturated_frames: u32,
}

impl TractionUsage {
    /// Fraction of driving frames spent at the bound, in `0..=1`.
    pub fn saturation(&self) -> f32 {
        if self.driving_frames == 0 {
            return 0.0;
        }
        self.saturated_frames as f32 / self.driving_frames as f32
    }
}

/// Per-body traction accounting, accumulated across a frame's substeps.
#[derive(Clone, Debug, Default)]
pub struct TractionLedger {
    entries: FxHashMap<RigidBodyHandle, TractionUsage>,
    /// Borrowed impulse so far this frame, before it is folded into a rate.
    frame_impulse: FxHashMap<RigidBodyHandle, f32>,
    /// Bodies whose rows saturated at any point this frame.
    frame_saturated: FxHashMap<RigidBodyHandle, bool>,
    /// Simulated seconds the current frame's substeps have covered.
    frame_seconds: f32,
}

impl TractionLedger {
    /// Fold whatever the last frame accumulated into each body's totals, and
    /// start a fresh frame.
    ///
    /// One call rather than a matched pair, because a frame's accounting is
    /// only ever closed by the next one starting: the substeps that feed it
    /// are driven from outside and there is no other moment that knows the
    /// frame has ended.
    pub fn open_frame(&mut self) {
        self.close_frame();
        self.frame_impulse.clear();
        self.frame_saturated.clear();
        self.frame_seconds = 0.0;
    }

    /// Read one substep's solved rows.
    ///
    /// A row's borrowed impulse is attributed to the bodies the Support Set
    /// says are driving through that contact, which is the same test the
    /// planner used to put a target there.
    pub fn record_substep(
        &mut self,
        manifolds: &[SolverManifold],
        supports: &SupportSets,
        dt: f32,
    ) {
        self.frame_seconds += dt;
        for (manifold_index, manifold) in manifolds.iter().enumerate() {
            for (contact_index, contact) in manifold.contacts.iter().enumerate() {
                if contact.traction.gain <= 1.0 {
                    continue;
                }
                let site = ContactSite::new(manifold_index, contact_index);
                let mut credit = |handle: RigidBodyHandle| {
                    if !supports.get(handle).is_some_and(|set| set.holds(site)) {
                        return;
                    }
                    *self.frame_impulse.entry(handle).or_default() +=
                        contact.traction.borrowed_impulse;
                    let saturated = self.frame_saturated.entry(handle).or_default();
                    *saturated |= contact.traction.saturated;
                };
                credit(manifold.header.body_b);
                if let Some(body_a) = manifold.header.body_a {
                    credit(body_a);
                }
            }
        }
    }

    /// Fold the frame's substeps into each body's running totals.
    fn close_frame(&mut self) {
        let frame_seconds = self.frame_seconds;
        for (handle, impulse) in self.frame_impulse.iter() {
            let usage = self.entries.entry(*handle).or_default();
            usage.borrowed_impulse = *impulse;
            usage.borrowed_force = if frame_seconds > 0.0 {
                impulse / frame_seconds
            } else {
                0.0
            };
            usage.driving_frames += 1;
            if self.frame_saturated.get(handle).copied().unwrap_or(false) {
                usage.saturated_frames += 1;
            }
        }
    }

    /// What one body has spent its gain on, if it has ever used one.
    pub fn usage(&self, handle: RigidBodyHandle) -> Option<&TractionUsage> {
        self.entries.get(&handle)
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{UnitVector3, Vector3};

    use super::*;
    use crate::physics::drive::support::{tests::manifold, SupportResolver};

    fn down() -> Option<UnitVector3<f32>> {
        Some(UnitVector3::new_normalize(-Vector3::y()))
    }

    fn handle(raw: usize) -> RigidBodyHandle {
        RigidBodyHandle(generational_arena::Index::from_raw_parts(raw, 0))
    }

    /// A gained row that borrowed impulse over two substeps reports the sum,
    /// and the force it stood for.
    #[test]
    fn borrowed_impulse_sums_over_the_frames_substeps() {
        let body = handle(1);
        let mut manifolds = vec![manifold(None, body, &[Vector3::y()])];
        let supports = SupportResolver::default().resolve(&manifolds, down());
        manifolds[0].contacts[0].traction.gain = 5.0;
        manifolds[0].contacts[0].traction.borrowed_impulse = 2.0;

        let mut ledger = TractionLedger::default();
        ledger.open_frame();
        ledger.record_substep(&manifolds, &supports, 0.25);
        ledger.record_substep(&manifolds, &supports, 0.25);
        ledger.open_frame();

        let usage = ledger.usage(body).expect("body drove this frame");
        assert_eq!(usage.borrowed_impulse, 4.0);
        assert_eq!(usage.borrowed_force, 8.0);
        assert_eq!(usage.driving_frames, 1);
        assert_eq!(usage.saturated_frames, 0);
    }

    #[test]
    fn a_row_at_its_bound_marks_the_frame_saturated() {
        let body = handle(1);
        let mut manifolds = vec![manifold(None, body, &[Vector3::y()])];
        let supports = SupportResolver::default().resolve(&manifolds, down());
        manifolds[0].contacts[0].traction.gain = 5.0;
        manifolds[0].contacts[0].traction.saturated = true;

        let mut ledger = TractionLedger::default();
        ledger.open_frame();
        ledger.record_substep(&manifolds, &supports, 1.0);
        ledger.open_frame();
        manifolds[0].contacts[0].traction.saturated = false;
        ledger.record_substep(&manifolds, &supports, 1.0);
        ledger.open_frame();

        let usage = ledger.usage(body).unwrap();
        assert_eq!(usage.driving_frames, 2);
        assert_eq!(usage.saturated_frames, 1);
        assert_eq!(usage.saturation(), 0.5);
    }

    /// An honest row is not the ledger's business, however hard it works.
    #[test]
    fn a_body_driving_at_gain_one_is_not_recorded() {
        let body = handle(1);
        let mut manifolds = vec![manifold(None, body, &[Vector3::y()])];
        let supports = SupportResolver::default().resolve(&manifolds, down());
        manifolds[0].contacts[0].traction.borrowed_impulse = 3.0;

        let mut ledger = TractionLedger::default();
        ledger.open_frame();
        ledger.record_substep(&manifolds, &supports, 1.0);
        ledger.open_frame();

        assert!(ledger.usage(body).is_none());
    }
}
