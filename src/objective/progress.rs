//! The level's objective state, and the readout of it.

use specs::{Read, System, Write};

use crate::debug::DebugLines;
use crate::time::Time;

/// How far through the level's objective the player is.
///
/// One resource rather than a component so that every writer — the collector,
/// the goal, the clock — is talking about the same run, and the HUD has one
/// place to read.
#[derive(Debug, Default)]
pub struct LevelProgress {
    /// Solids taken so far.
    pub solids_taken: u32,

    /// Solids the level spawned. Counted as each one is created, so a level
    /// that places none reports none.
    pub solids_total: u32,

    /// Seconds since the level started, frozen once the goal is reached.
    pub elapsed: f32,

    /// The elapsed time at which the goal was reached, if it has been.
    pub completed: Option<f32>,

    /// Whether the level can no longer be won: every goal in it is ruined.
    pub lost: bool,
}

impl LevelProgress {
    /// Whether the goal has already been reached.
    pub fn is_complete(&self) -> bool {
        self.completed.is_some()
    }

    /// Register a solid that has just been spawned into the level.
    pub fn add_solid(&mut self) {
        self.solids_total += 1;
    }

    /// Record a solid taken. Solids taken after the goal still count, so a
    /// level can be finished and then cleaned out.
    pub fn take_solid(&mut self) {
        self.solids_taken += 1;
    }

    /// How many of the level's solids are still to be taken.
    pub fn solids_outstanding(&self) -> u32 {
        self.solids_total.saturating_sub(self.solids_taken)
    }

    /// Finish the level at the current elapsed time, if it is not finished
    /// or lost already. Returns whether this call was the one that finished
    /// it.
    pub fn complete(&mut self) -> bool {
        if self.completed.is_some() || self.lost {
            return false;
        }
        self.completed = Some(self.elapsed);
        true
    }

    /// The level can no longer be won. A level already finished stays
    /// finished.
    pub fn forfeit(&mut self) {
        if self.completed.is_none() {
            self.lost = true;
        }
    }
}

/// Runs the level clock until the goal is reached or the level is lost.
pub struct ProgressSystem;

impl<'a> System<'a> for ProgressSystem {
    type SystemData = (Read<'a, Time>, Write<'a, LevelProgress>);

    fn run(&mut self, (time, mut progress): Self::SystemData) {
        if progress.is_complete() || progress.lost {
            return;
        }
        progress.elapsed += time.delta_seconds();
    }
}

/// Announces on screen that the level is complete, with the run time, or
/// that it is lost and must be restarted.
///
/// The solid count and running clock are tracked in [`LevelProgress`] but
/// not shown: the overlay is kept to the frame rate while the level plays.
pub struct ObjectiveHudSystem;

impl<'a> System<'a> for ObjectiveHudSystem {
    type SystemData = (Read<'a, LevelProgress>, Write<'a, DebugLines>);

    fn run(&mut self, (progress, mut debug): Self::SystemData) {
        if let Some(at) = progress.completed {
            debug.add(HUD_COMPLETE_KEY, format_clock(at));
        } else if progress.lost {
            debug.add(HUD_LOST_KEY, HUD_LOST_HINT.to_string());
        }
    }
}

/// Shown only once the goal has been reached. Leading punctuation sorts it
/// above the alphabetical diagnostics that share the overlay.
pub const HUD_COMPLETE_KEY: &str = "!! LEVEL COMPLETE";

/// Shown once the level can no longer be won, and what to do about it.
pub const HUD_LOST_KEY: &str = "!! LEVEL LOST";
const HUD_LOST_HINT: &str = "R to restart";

/// Seconds as `m:ss.t` — the shape a run time is read in.
pub fn format_clock(seconds: f32) -> String {
    let seconds = seconds.max(0.0);
    let minutes = (seconds / 60.0).floor();
    let rest = seconds - minutes * 60.0;
    format!("{minutes:.0}:{rest:04.1}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spawning_and_taking_solids_move_separate_counts() {
        let mut progress = LevelProgress::default();
        progress.add_solid();
        progress.add_solid();
        progress.take_solid();

        assert_eq!(progress.solids_total, 2);
        assert_eq!(progress.solids_taken, 1);
        assert_eq!(progress.solids_outstanding(), 1);
    }

    #[test]
    fn outstanding_solids_never_go_below_zero() {
        let mut progress = LevelProgress::default();
        progress.add_solid();
        progress.take_solid();
        progress.take_solid();

        assert_eq!(progress.solids_outstanding(), 0);
    }

    /// The finish time is the first one. A player standing in the goal must not
    /// see the clock they finished on creep upward.
    #[test]
    fn completion_keeps_the_first_time() {
        let mut progress = LevelProgress::default();
        progress.elapsed = 12.0;

        assert!(progress.complete());
        progress.elapsed = 30.0;
        assert!(!progress.complete());

        assert_eq!(progress.completed, Some(12.0));
    }

    #[test]
    fn a_lost_level_cannot_be_finished_and_a_finished_one_cannot_be_lost() {
        let mut lost = LevelProgress::default();
        lost.forfeit();
        assert!(!lost.complete());
        assert!(lost.lost);

        let mut won = LevelProgress::default();
        assert!(won.complete());
        won.forfeit();
        assert!(!won.lost);
    }

    #[test]
    fn the_clock_reads_as_minutes_and_seconds() {
        assert_eq!(format_clock(0.0), "0:00.0");
        assert_eq!(format_clock(9.21), "0:09.2");
        assert_eq!(format_clock(102.34), "1:42.3");
    }
}
