//! Shared gait phase clock.
//!
//! A `[0, 1)` phase advanced by hip travel over the cycle distance.
//! The right foot's stance window is centred on phase `0.0`, the left
//! foot's on `0.5`, each spanning `duty_factor` of the cycle. A foot is
//! *released* (asked to step) when the phase exits its stance window.
//!
//! Releases are **latched**, not edge-triggered: if a release cannot
//! fire immediately (e.g. the other foot is still mid-swing under a
//! continuous-support gait), the request stays pending until consumed
//! instead of being lost for a full cycle. This is what keeps the feet
//! phase-locked at coarse frame times.
//!
//! The phase has **timing authority**: it free-runs at the gait cadence
//! and is never re-anchored by step events. Feet are pulled to the
//! schedule, not the schedule to the feet — antiphase is a property of
//! the clock, not something the feet must negotiate. The phase is set
//! only where no rhythm exists: gait start (`seed_to_release`) and the
//! landing replant (`reset`).

use super::placer::FootSide;
use super::timing::GaitTiming;

/// Why a release was latched. Determines the support rules the step
/// fires under, and the upgrade priority when several triggers ask for
/// the same foot (declaration order = `Ord`; a stronger kind replaces a
/// weaker pending one, never the reverse):
///
/// - `Turn` — accumulated facing change since plant. Must additionally
///   wait for the other foot to be planted; a repositioning step with
///   both feet off the ground reads as a flail.
/// - `Scheduled` — the phase clock's stance-window exit. Fires under
///   the gait's normal rules (flight allowed when duty < 0.5).
/// - `Overstretch` — the hip slid to the leg's stretch limit (landing
///   slides, hard accelerations, sharp reversals). Strongest kind: it
///   bypasses the fresh-plant stance gate, and past the hard margin
///   even the takeoff stagger — delaying it would stretch the leg
///   without bound, so it must be able to upgrade a pending
///   `Scheduled`.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum ReleaseKind {
    Turn,
    Scheduled,
    Overstretch,
}

/// Phase clock plus per-foot release latches.
#[derive(Clone, Debug)]
pub struct GaitClock {
    /// Gait phase in `[0, 1)`.
    phase: f32,
    /// Whether each foot was inside its stance window last advance.
    left_in_stance: bool,
    right_in_stance: bool,
    /// Latched release requests, set on stance-window exit (Scheduled)
    /// or by event triggers (Reactive), cleared when the step actually
    /// fires (or when movement stops).
    left_pending: Option<ReleaseKind>,
    right_pending: Option<ReleaseKind>,
    /// How far (in phase units) the feet are running ahead of the
    /// schedule, measured at each reactive step fire. While positive,
    /// `advance` runs at `CATCH_UP_RATE` and pays the debt down, so the
    /// schedule recaptures a violently perturbed gait in a fraction of a
    /// cycle instead of drifting back over seconds. Rate-bounded and
    /// one-directional — the phase is still never *set* by step events.
    phase_debt: f32,
}

/// Clock rate multiplier while `phase_debt` is positive. At 1.35 a
/// worst-case half-cycle debt clears in ~1.4 cycles, and the transient
/// shortening of the inter-takeoff gap stays under ~26% — quick enough
/// to recapture post-landing scrambles, subtle enough not to read as a
/// skip.
const CATCH_UP_RATE: f32 = 1.35;

impl GaitClock {
    /// A clock at phase 0 with stance membership consistent with that
    /// phase, so the first advance cannot latch a spurious release. At
    /// phase 0 the right foot is dead-centre in its window and the left
    /// is half a cycle away, for any valid duty factor.
    pub fn new() -> Self {
        Self {
            phase: 0.0,
            left_in_stance: false,
            right_in_stance: true,
            left_pending: None,
            right_pending: None,
            phase_debt: 0.0,
        }
    }

    /// Current phase, for debug output.
    pub fn phase(&self) -> f32 {
        self.phase
    }

    /// Advance the clock by one substep. While moving, the phase
    /// advances by `speed·dt / cycle_distance` and stance-window exits
    /// latch release requests. While not moving, the phase holds and
    /// any stale requests are dropped (idle settling has its own rule).
    pub fn advance(&mut self, dt: f32, gait_speed: f32, timing: &GaitTiming, moving: bool) {
        if moving {
            let cycle = timing.cycle_distance.max(1e-3);
            let base = gait_speed * dt / cycle;
            let extra = (base * (CATCH_UP_RATE - 1.0)).min(self.phase_debt);
            self.phase_debt -= extra;
            self.phase = (self.phase + base + extra).rem_euclid(1.0);
        } else {
            self.left_pending = None;
            self.right_pending = None;
            self.phase_debt = 0.0;
        }

        let left_in = in_stance_window(self.phase, FootSide::Left, timing.duty_factor);
        let right_in = in_stance_window(self.phase, FootSide::Right, timing.duty_factor);
        if moving {
            if self.left_in_stance && !left_in {
                self.left_pending = Some(ReleaseKind::Scheduled);
            }
            if self.right_in_stance && !right_in {
                self.right_pending = Some(ReleaseKind::Scheduled);
            }
        }
        self.left_in_stance = left_in;
        self.right_in_stance = right_in;
    }

    /// The pending release (if any) for the given side.
    pub fn pending(&self, side: FootSide) -> Option<ReleaseKind> {
        match side {
            FootSide::Left => self.left_pending,
            FootSide::Right => self.right_pending,
        }
    }

    /// Seed the phase so `side` is released right now and the other
    /// foot's release lands exactly half a cycle later. Called once on
    /// the idle→moving edge, when both feet are planted and no rhythm
    /// exists yet to corrupt.
    ///
    /// This is the only place the phase may be *set* while the gait
    /// runs. Step events must never re-anchor it (the old
    /// `resync_to_takeoff` did): the moment the clock follows the feet,
    /// every gate or reactive trigger that delays a step reshapes the
    /// schedule itself, and off-antiphase timings become self-consistent
    /// attractors — seen in game as a stable 0.6π/1.4π takeoff split.
    /// With a free-running phase, a delayed or reactive step costs one
    /// odd stance and the fixed schedule pulls the foot straight back.
    pub fn seed_to_release(&mut self, side: FootSide, timing: &GaitTiming) {
        // Epsilon past the exit edge: `side` is just outside its window
        // (release latched directly below), the other side is deep
        // inside its own and will exit half a cycle from now.
        self.phase = (exit_edge(side, timing) + 1e-4).rem_euclid(1.0);
        self.left_in_stance = in_stance_window(self.phase, FootSide::Left, timing.duty_factor);
        self.right_in_stance = in_stance_window(self.phase, FootSide::Right, timing.duty_factor);
        self.left_pending = None;
        self.right_pending = None;
        self.phase_debt = 0.0;
        self.request_release(side, ReleaseKind::Scheduled);
    }

    /// Record that `side`'s step actually fired under `kind`. A
    /// scheduled fire means feet and schedule are aligned — any debt is
    /// cleared. A reactive fire ahead of the foot's window exit means
    /// the feet are running ahead of the clock; the gap to the edge is
    /// taken as the fresh phase debt, paid down at `CATCH_UP_RATE` in
    /// `advance`. Reactive fires *past* the edge (late, e.g. a release
    /// the support rules held back) leave the debt untouched — the
    /// free-running schedule absorbs lateness on its own.
    pub fn note_fire(&mut self, side: FootSide, kind: ReleaseKind, timing: &GaitTiming) {
        if kind == ReleaseKind::Scheduled {
            self.phase_debt = 0.0;
            return;
        }
        let gap = (exit_edge(side, timing) - self.phase).rem_euclid(1.0);
        if gap <= 0.5 {
            self.phase_debt = gap;
        }
    }

    /// Latch a release outside the normal schedule. A stronger kind
    /// upgrades a weaker pending one (`Turn` → `Overstretch` →
    /// `Scheduled`); never downgrades.
    pub fn request_release(&mut self, side: FootSide, kind: ReleaseKind) {
        let slot = match side {
            FootSide::Left => &mut self.left_pending,
            FootSide::Right => &mut self.right_pending,
        };
        *slot = Some(slot.map_or(kind, |existing| existing.max(kind)));
    }

    /// Clear a side's request once its step has fired.
    pub fn consume(&mut self, side: FootSide) {
        match side {
            FootSide::Left => self.left_pending = None,
            FootSide::Right => self.right_pending = None,
        }
    }
}

/// Whether `phase` lies inside the stance window of `side`. Right is
/// centred on 0.0, left on 0.5; each window spans `duty_factor`.
fn in_stance_window(phase: f32, side: FootSide, duty_factor: f32) -> bool {
    let centre = match side {
        FootSide::Right => 0.0,
        FootSide::Left => 0.5,
    };
    phase_distance(phase, centre) <= duty_factor.clamp(0.05, 0.95) * 0.5
}

/// Phase at which `side` exits its stance window (its swing begins).
fn exit_edge(side: FootSide, timing: &GaitTiming) -> f32 {
    let centre = match side {
        FootSide::Right => 0.0,
        FootSide::Left => 0.5,
    };
    (centre + timing.duty_factor.clamp(0.05, 0.95) * 0.5).rem_euclid(1.0)
}

/// Shortest wrapped distance between two phases in `[0, 1)`.
fn phase_distance(a: f32, b: f32) -> f32 {
    let d = (a - b).abs().rem_euclid(1.0);
    d.min(1.0 - d)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn timing() -> GaitTiming {
        // Walk-band timing: duty > 0.5 with a comfortable cycle.
        GaitTiming {
            trigger_threshold: 0.2,
            cycle_distance: 0.4,
            duty_factor: 0.6,
            swing_duration: 0.2,
            continuous_support: true,
        }
    }

    #[test]
    fn new_clock_has_consistent_membership_and_no_pending() {
        let c = GaitClock::new();
        assert!(c.right_in_stance);
        assert!(!c.left_in_stance);
        assert!(c.pending(FootSide::Left).is_none());
        assert!(c.pending(FootSide::Right).is_none());
    }

    #[test]
    fn advance_moves_phase_while_moving() {
        let mut c = GaitClock::new();
        c.advance(0.1, 1.0, &timing(), true);
        assert!((c.phase() - 0.25).abs() < 1e-5);
    }

    #[test]
    fn advance_holds_phase_while_idle() {
        let mut c = GaitClock::new();
        c.advance(0.1, 1.0, &timing(), false);
        assert_eq!(c.phase(), 0.0);
    }

    #[test]
    fn stance_exit_latches_scheduled_release() {
        let mut c = GaitClock::new();
        // Right window is [-0.3, 0.3]; one big step to 0.35 exits it.
        c.advance(0.14, 1.0, &timing(), true);
        assert_eq!(c.pending(FootSide::Right), Some(ReleaseKind::Scheduled));
        assert!(c.pending(FootSide::Left).is_none());
    }

    #[test]
    fn release_survives_until_consumed() {
        let mut c = GaitClock::new();
        c.advance(0.14, 1.0, &timing(), true);
        c.advance(0.01, 1.0, &timing(), true);
        assert!(c.pending(FootSide::Right).is_some());
        c.consume(FootSide::Right);
        assert!(c.pending(FootSide::Right).is_none());
    }

    #[test]
    fn stopping_drops_stale_requests() {
        let mut c = GaitClock::new();
        c.advance(0.14, 1.0, &timing(), true);
        assert!(c.pending(FootSide::Right).is_some());
        c.advance(0.01, 0.0, &timing(), false);
        assert!(c.pending(FootSide::Right).is_none());
    }

    #[test]
    fn seed_to_release_latches_seeded_side_at_window_exit() {
        let mut c = GaitClock::new();
        c.advance(0.05, 1.0, &timing(), true);
        c.seed_to_release(FootSide::Left, &timing());
        // Left window is centred 0.5 with half-width 0.3.
        assert!((c.phase() - 0.8).abs() < 1e-3);
        assert_eq!(c.pending(FootSide::Left), Some(ReleaseKind::Scheduled));
        assert!(c.pending(FootSide::Right).is_none());
    }

    #[test]
    fn seed_to_release_schedules_other_side_half_a_cycle_later() {
        let mut c = GaitClock::new();
        c.seed_to_release(FootSide::Right, &timing());
        c.consume(FootSide::Right);
        // Right exits at 0.3; left's exit edge is at 0.8 — half a cycle
        // (0.5 phase = 0.2 distance units at cycle 0.4) further on.
        c.advance(0.199, 1.0, &timing(), true);
        assert!(c.pending(FootSide::Left).is_none());
        c.advance(0.002, 1.0, &timing(), true);
        assert_eq!(c.pending(FootSide::Left), Some(ReleaseKind::Scheduled));
    }

    #[test]
    fn seed_to_release_does_not_relatch_seeded_side_on_next_advance() {
        let mut c = GaitClock::new();
        c.seed_to_release(FootSide::Right, &timing());
        c.consume(FootSide::Right);
        // Right is just past its own exit edge; a small further advance
        // must not report a fresh window exit for it.
        c.advance(0.001, 1.0, &timing(), true);
        assert!(c.pending(FootSide::Right).is_none());
    }

    #[test]
    fn reactive_fire_ahead_of_edge_speeds_clock_until_debt_paid() {
        let mut c = GaitClock::new();
        // Right's exit edge is at 0.3; an overstretch fire at phase 0
        // leaves the feet 0.3 ahead of the schedule.
        c.note_fire(FootSide::Right, ReleaseKind::Overstretch, &timing());
        let mut boosted = GaitClock::new();
        boosted.phase_debt = c.phase_debt;
        // One normal-cycle's worth of advance must cover extra ground...
        boosted.advance(0.1, 1.0, &timing(), true);
        let mut plain = GaitClock::new();
        plain.advance(0.1, 1.0, &timing(), true);
        assert!(boosted.phase() > plain.phase());
        // ...and the total extra ground equals the debt once paid off.
        for _ in 0..20 {
            boosted.advance(0.1, 1.0, &timing(), true);
            plain.advance(0.1, 1.0, &timing(), true);
        }
        assert!(boosted.phase_debt <= 1e-6);
        let lead = (boosted.phase() - plain.phase()).rem_euclid(1.0);
        assert!((lead - 0.3).abs() < 1e-3, "lead {lead} != original debt");
    }

    #[test]
    fn scheduled_fire_clears_debt() {
        let mut c = GaitClock::new();
        c.note_fire(FootSide::Right, ReleaseKind::Overstretch, &timing());
        assert!(c.phase_debt > 0.0);
        c.note_fire(FootSide::Left, ReleaseKind::Scheduled, &timing());
        assert_eq!(c.phase_debt, 0.0);
    }

    #[test]
    fn late_reactive_fire_leaves_debt_untouched() {
        let mut c = GaitClock::new();
        // Phase 0.4: right's edge (0.3) is 0.9 ahead going forward,
        // i.e. just behind — a late fire, not an early one.
        c.advance(0.16, 1.0, &timing(), true);
        c.note_fire(FootSide::Right, ReleaseKind::Overstretch, &timing());
        assert_eq!(c.phase_debt, 0.0);
    }

    #[test]
    fn request_release_latches_given_kind() {
        let mut c = GaitClock::new();
        c.request_release(FootSide::Left, ReleaseKind::Turn);
        assert_eq!(c.pending(FootSide::Left), Some(ReleaseKind::Turn));
    }

    #[test]
    fn request_release_upgrades_weaker_kind() {
        let mut c = GaitClock::new();
        c.request_release(FootSide::Left, ReleaseKind::Turn);
        c.request_release(FootSide::Left, ReleaseKind::Overstretch);
        assert_eq!(c.pending(FootSide::Left), Some(ReleaseKind::Overstretch));
    }

    #[test]
    fn request_release_does_not_downgrade_stronger_kind() {
        let mut c = GaitClock::new();
        c.advance(0.14, 1.0, &timing(), true);
        assert_eq!(c.pending(FootSide::Right), Some(ReleaseKind::Scheduled));
        c.request_release(FootSide::Right, ReleaseKind::Turn);
        assert_eq!(c.pending(FootSide::Right), Some(ReleaseKind::Scheduled));
    }
}
