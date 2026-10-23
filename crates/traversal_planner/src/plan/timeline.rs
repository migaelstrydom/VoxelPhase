use serde::{Deserialize, Serialize};

use super::phase::Phase;

/// The body's phases from now to the planning horizon.
///
/// Phases `[0, committed)` are settled: the next replan keeps them unless the
/// body strays from them, so the drive and the legs can act on them without
/// the plan flickering under them. The rest is tentative.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Plan {
    /// In time order, each starting where the one before ends.
    pub phases: Vec<Phase>,
    /// How many leading phases are committed.
    pub committed: usize,
}
