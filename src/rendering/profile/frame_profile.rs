use std::time::Duration;

use super::counters::RenderCounters;
use super::gpu_timer::GpuTimings;
use super::stage::RenderStage;

/// One rendered frame: CPU time per stage, GPU time per span, and the work
/// counts that explain both.
#[derive(Debug, Clone, Default)]
pub struct RenderProfile {
    /// Which frame this is, counted from 1 by the renderer that drew it. With
    /// frames in flight a profile is published some frames after it was
    /// recorded; this is what matches the two up.
    pub frame: u64,
    /// Time per stage, indexed by [`RenderStage::slot`].
    stages: [Duration; RenderStage::COUNT],
    /// What the frame was asked to draw.
    pub counters: RenderCounters,
    /// The frame's GPU time, once it has been read back. `None` when the
    /// device cannot time, or the frame never reached the GPU.
    pub gpu: Option<GpuTimings>,
}

impl RenderProfile {
    /// An empty profile for the renderer's `frame`th frame.
    pub fn for_frame(frame: u64) -> Self {
        Self {
            frame,
            ..Self::default()
        }
    }

    pub fn record(&mut self, stage: RenderStage, elapsed: Duration) {
        self.stages[stage.slot()] += elapsed;
    }

    pub fn stage(&self, stage: RenderStage) -> Duration {
        self.stages[stage.slot()]
    }

    /// CPU time spent working on the frame, leaving out the waits.
    pub fn cpu_work(&self) -> Duration {
        self.iter()
            .filter(|(stage, _)| !stage.is_wait())
            .map(|(_, time)| time)
            .sum()
    }

    /// Every stage with its time, in the order a frame runs them.
    pub fn iter(&self) -> impl Iterator<Item = (RenderStage, Duration)> + '_ {
        RenderStage::ALL
            .iter()
            .map(|&stage| (stage, self.stage(stage)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stages_accumulate_and_waits_are_not_work() {
        let mut profile = RenderProfile::default();
        profile.record(RenderStage::Models, Duration::from_micros(300));
        profile.record(RenderStage::Models, Duration::from_micros(200));
        profile.record(RenderStage::Terrain, Duration::from_micros(100));
        profile.record(RenderStage::FenceWait, Duration::from_millis(5));

        assert_eq!(
            profile.stage(RenderStage::Models),
            Duration::from_micros(500)
        );
        assert_eq!(profile.cpu_work(), Duration::from_micros(600));
    }
}
