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
    /// Gems caught so far.
    pub gems_collected: u32,

    /// Gems the level spawned. Counted as each one is created, so a level that
    /// places none reports none and the gem readout disappears.
    pub gems_total: u32,

    /// Seconds since the level started, frozen once the goal is reached.
    pub elapsed: f32,

    /// The elapsed time at which the goal was reached, if it has been.
    pub completed: Option<f32>,
}

impl LevelProgress {
    /// Whether the goal has already been reached.
    pub fn is_complete(&self) -> bool {
        self.completed.is_some()
    }

    /// Register a gem that has just been spawned into the level.
    pub fn add_gem(&mut self) {
        self.gems_total += 1;
    }

    /// Record a caught gem. Gems caught after the goal still count, so a level
    /// can be finished and then cleaned out.
    pub fn collect_gem(&mut self) {
        self.gems_collected += 1;
    }

    /// How many more gems are needed to reach `required`.
    pub fn gems_outstanding(&self, required: u32) -> u32 {
        required.saturating_sub(self.gems_collected)
    }

    /// Finish the level at the current elapsed time, if it is not finished
    /// already. Returns whether this call was the one that finished it.
    pub fn complete(&mut self) -> bool {
        if self.completed.is_some() {
            return false;
        }
        self.completed = Some(self.elapsed);
        true
    }
}

/// Runs the level clock until the goal is reached.
pub struct ProgressSystem;

impl<'a> System<'a> for ProgressSystem {
    type SystemData = (Read<'a, Time>, Write<'a, LevelProgress>);

    fn run(&mut self, (time, mut progress): Self::SystemData) {
        if progress.is_complete() {
            return;
        }
        progress.elapsed += time.delta_seconds();
    }
}

/// Draws the objective readout on screen every frame.
pub struct ObjectiveHudSystem;

impl<'a> System<'a> for ObjectiveHudSystem {
    type SystemData = (Read<'a, LevelProgress>, Write<'a, DebugLines>);

    fn run(&mut self, (progress, mut debug): Self::SystemData) {
        if progress.gems_total > 0 {
            debug.add(
                HUD_GEMS_KEY,
                format!("{} / {}", progress.gems_collected, progress.gems_total),
            );
        }

        debug.add(HUD_TIME_KEY, format_clock(progress.elapsed));

        if let Some(at) = progress.completed {
            debug.add(HUD_COMPLETE_KEY, format_clock(at));
        }
    }
}

/// Gem count. Leading punctuation sorts the objective above the alphabetical
/// diagnostics that share the overlay.
pub const HUD_GEMS_KEY: &str = "! Gems";

/// Level clock.
pub const HUD_TIME_KEY: &str = "! Time";

/// Shown only once the goal has been reached.
pub const HUD_COMPLETE_KEY: &str = "!! LEVEL COMPLETE";

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
    fn spawning_and_catching_gems_move_separate_counts() {
        let mut progress = LevelProgress::default();
        progress.add_gem();
        progress.add_gem();
        progress.collect_gem();

        assert_eq!(progress.gems_total, 2);
        assert_eq!(progress.gems_collected, 1);
    }

    #[test]
    fn outstanding_gems_never_go_below_zero() {
        let mut progress = LevelProgress::default();
        progress.collect_gem();
        progress.collect_gem();

        assert_eq!(progress.gems_outstanding(1), 0);
        assert_eq!(progress.gems_outstanding(5), 3);
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
    fn the_clock_reads_as_minutes_and_seconds() {
        assert_eq!(format_clock(0.0), "0:00.0");
        assert_eq!(format_clock(9.21), "0:09.2");
        assert_eq!(format_clock(102.34), "1:42.3");
    }
}
