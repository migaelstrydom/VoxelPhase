//! The level's objective state.

use specs::{Read, System, Write};

use crate::time::Time;

/// How far through the level's objective the player is.
///
/// One resource rather than a component so that every writer — the collector,
/// the goal, the clock — is talking about the same run. Nothing of it is
/// shown in words: the brazier's flame is the news that the level is won.
#[derive(Debug, Default)]
pub struct LevelProgress {
    /// Solids taken so far.
    pub solids_taken: u32,

    /// Solids the level spawned. Counted as each one is created, so a level
    /// that places none reports none.
    pub solids_total: u32,

    /// Scrolls taken so far, and how many the level spawned. Kept apart
    /// from the solids: no level needs them.
    pub scrolls_taken: u32,
    pub scrolls_total: u32,

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

    /// Register a scroll that has just been spawned into the level.
    pub fn add_scroll(&mut self) {
        self.scrolls_total += 1;
    }

    /// Record a scroll taken.
    pub fn take_scroll(&mut self) {
        self.scrolls_taken += 1;
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

    /// A scroll taken counts as a scroll, and leaves the solids still owed.
    #[test]
    fn scrolls_are_counted_apart_from_solids() {
        let mut progress = LevelProgress::default();
        progress.add_solid();
        progress.add_scroll();
        progress.take_scroll();

        assert_eq!(progress.scrolls_total, 1);
        assert_eq!(progress.scrolls_taken, 1);
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

    /// The finish time is the first one: standing on in the goal does not
    /// move it.
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
}
