use std::time::Duration;

use super::stage::PhysicsStage;

/// Wall-clock time spent in each stage of one physics frame, plus the counts
/// that explain it.
///
/// Opened by `update_contacts` and added to by every `substep` after it, so
/// once a frame's substeps have run this holds that whole frame.
#[derive(Debug, Clone, Default)]
pub struct FrameProfile {
    /// Time per stage, indexed by [`PhysicsStage::slot`].
    stages: [Duration; PhysicsStage::COUNT],
    /// Substeps run against this frame's contacts.
    pub substeps: u32,
    /// Corrections CCD applied, summed over substeps.
    pub ccd_corrections: u32,
}

impl FrameProfile {
    /// Forget the previous frame.
    pub fn open(&mut self) {
        *self = Self::default();
    }

    pub fn record(&mut self, stage: PhysicsStage, elapsed: Duration) {
        self.stages[stage.slot()] += elapsed;
    }

    pub fn stage(&self, stage: PhysicsStage) -> Duration {
        self.stages[stage.slot()]
    }

    /// The frame's whole physics cost.
    pub fn total(&self) -> Duration {
        self.stages.iter().sum()
    }

    /// Every stage with its time, in the order a frame runs them.
    pub fn iter(&self) -> impl Iterator<Item = (PhysicsStage, Duration)> + '_ {
        PhysicsStage::ALL
            .iter()
            .map(|&stage| (stage, self.stage(stage)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn substep_stages_accumulate_and_opening_clears_them() {
        let mut profile = FrameProfile::default();
        profile.record(PhysicsStage::Solve, Duration::from_micros(300));
        profile.record(PhysicsStage::Solve, Duration::from_micros(200));
        profile.record(PhysicsStage::StaticNarrowphase, Duration::from_micros(100));

        assert_eq!(
            profile.stage(PhysicsStage::Solve),
            Duration::from_micros(500)
        );
        assert_eq!(profile.total(), Duration::from_micros(600));

        profile.open();
        assert_eq!(profile.total(), Duration::ZERO);
    }
}
