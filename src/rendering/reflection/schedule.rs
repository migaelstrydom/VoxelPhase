use crate::rendering::reflection::face::CubeFace;
use crate::rendering::reflection::pool::{ProbePool, ProbeSlot};

/// One face to capture this frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FaceCapture {
    pub slot: ProbeSlot,
    pub face: CubeFace,
}

/// Which probe faces are captured each frame.
///
/// ```text
///   newly admitted probes ──▶ all six faces, now
///   probes still asked for ──▶ the next `budget` faces after the cursor,
///                             walking slot 0 +X … slot 0 −Z, slot 1 +X …
///   lingering probes       ──▶ nothing: their objects are out of view
/// ```
///
/// A new probe has to be whole before it is sampled — its faces still hold
/// the last owner's surroundings — so it goes first and outside the budget.
/// The pool caps how many are admitted per frame, which is what bounds that.
///
/// The cursor walks the atlas rather than the live probes, so a probe that
/// comes or goes does not shift which face every other probe gets next.
#[derive(Debug, Default)]
pub struct FaceSchedule {
    /// The next (slot, face) position to consider, as `6 * slot + face`.
    cursor: u32,
}

impl FaceSchedule {
    /// The faces to capture this frame.
    pub fn plan(&mut self, pool: &ProbePool, capacity: u32, budget: u32) -> Vec<FaceCapture> {
        let mut plan: Vec<FaceCapture> = pool
            .live()
            .filter(|(_, probe)| !probe.captured)
            .flat_map(|(slot, _)| CubeFace::ALL.map(|face| FaceCapture { slot, face }))
            .collect();

        let positions = capacity * CubeFace::COUNT as u32;
        let mut refreshed = 0;
        for _ in 0..positions {
            if refreshed >= budget {
                break;
            }
            let position = self.cursor;
            self.cursor = (self.cursor + 1) % positions.max(1);

            let slot = ProbeSlot(position / CubeFace::COUNT as u32);
            let face = CubeFace::ALL[(position % CubeFace::COUNT as u32) as usize];
            if pool
                .get(slot)
                .is_some_and(|probe| probe.captured && probe.idle == 0)
            {
                plan.push(FaceCapture { slot, face });
                refreshed += 1;
            }
        }
        plan
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;
    use crate::rendering::reflection::owner::ProbeOwner;

    /// A pool with `live` probes, all already captured.
    fn pool_with(capacity: u32, live: u64) -> ProbePool {
        let mut pool = ProbePool::new(capacity);
        for owner in 0..live {
            pool.request(ProbeOwner(owner), Vector3::zeros(), owner as f32);
        }
        pool.begin_frame(capacity as usize);
        for owner in 0..live {
            let slot = pool
                .request(ProbeOwner(owner), Vector3::zeros(), owner as f32)
                .unwrap();
            pool.mark_captured(slot);
        }
        pool
    }

    #[test]
    fn a_new_probe_is_captured_whole_and_outside_the_budget() {
        let mut pool = ProbePool::new(4);
        pool.request(ProbeOwner(1), Vector3::zeros(), 1.0);
        pool.begin_frame(4);
        let plan = FaceSchedule::default().plan(&pool, 4, 2);
        assert_eq!(plan.len(), 6);
        assert!(plan.iter().all(|capture| capture.slot == ProbeSlot(0)));
    }

    #[test]
    fn faces_are_refreshed_round_robin_across_probes() {
        let pool = pool_with(4, 2);
        let mut schedule = FaceSchedule::default();
        let mut seen = Vec::new();
        for _ in 0..6 {
            let plan = schedule.plan(&pool, 4, 2);
            assert_eq!(plan.len(), 2);
            seen.extend(plan);
        }
        // Twelve faces in six frames of two: every face of both probes once.
        for slot in [ProbeSlot(0), ProbeSlot(1)] {
            for face in CubeFace::ALL {
                let times = seen
                    .iter()
                    .filter(|capture| capture.slot == slot && capture.face == face)
                    .count();
                assert_eq!(times, 1, "{slot:?} {face:?}");
            }
        }
    }

    #[test]
    fn a_budget_beyond_the_live_faces_captures_each_once() {
        let pool = pool_with(4, 1);
        let plan = FaceSchedule::default().plan(&pool, 4, 100);
        assert_eq!(plan.len(), 6);
    }

    #[test]
    fn a_lingering_probe_is_not_refreshed() {
        let mut pool = ProbePool::new(4);
        for owner in 0..2 {
            pool.request(ProbeOwner(owner), Vector3::zeros(), 1.0);
        }
        pool.begin_frame(4);
        pool.mark_captured(ProbeSlot(0));
        pool.mark_captured(ProbeSlot(1));
        // Only owner 1 asks this time; owner 0's probe lingers.
        pool.request(ProbeOwner(1), Vector3::zeros(), 1.0);
        pool.begin_frame(4);
        let plan = FaceSchedule::default().plan(&pool, 4, 100);
        assert_eq!(plan.len(), 6);
        assert!(plan.iter().all(|capture| capture.slot == ProbeSlot(1)));
    }

    #[test]
    fn no_live_probes_plan_nothing() {
        let pool = ProbePool::new(4);
        assert!(FaceSchedule::default().plan(&pool, 4, 6).is_empty());
    }
}
