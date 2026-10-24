use serde::{Deserialize, Serialize};

use super::phase::{Contact, Phase};

/// The body's phases from now to the planning horizon. The last phase is the
/// last footfall landing, zero long.
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

impl Plan {
    /// Every footfall with the time it lands: a contact whose limb did not
    /// hold the body in the phase before. The first phase's contacts were
    /// planted before the plan began.
    pub fn footfalls(&self) -> impl Iterator<Item = (f32, &Contact)> {
        self.phases.windows(2).flat_map(|pair| {
            let (before, phase) = (&pair[0], &pair[1]);
            phase
                .contacts
                .iter()
                .filter(|contact| before.contacts.iter().all(|held| held.limb != contact.limb))
                .map(move |contact| (phase.start, contact))
        })
    }
}
