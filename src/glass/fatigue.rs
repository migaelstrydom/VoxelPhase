//! Failure under a load that is merely held.
//!
//! The fracture system reads contact as a *spike* so that a heavy object's
//! own weight is not a hit. Glass has the opposite problem: a pane can take
//! a knock it cannot take a person standing on, and standing is exactly
//! zero spike. So a sheet may also carry a bearing capacity — a force it
//! holds indefinitely — and accumulate damage for every second it is asked
//! to hold more. The damage never heals: a pane that has been stood on is
//! weaker for it, which is also what makes a glass floor a place to keep
//! moving on.

/// What a pane can bear, and for how long it can bear more.
#[derive(Debug, Clone, Copy)]
pub struct FatigueRule {
    /// Force in newtons a cell holds indefinitely.
    pub bearing: f32,
    /// Seconds a cell survives at twice its bearing. Damage accrues in
    /// proportion to the excess, so four times the bearing fails in a third
    /// of this.
    pub endurance: f32,
}

/// Per-child damage, from 0 (sound) to 1 (failed).
#[derive(Debug, Default)]
pub struct FatigueTracker {
    damage: Vec<f32>,
}

impl FatigueTracker {
    pub fn new(child_count: usize) -> Self {
        Self {
            damage: vec![0.0; child_count],
        }
    }

    /// Add `dt` seconds of `forces` (newtons, indexed by child) and report
    /// the children that have just failed. A failed child stays at full
    /// damage until it is removed.
    pub fn advance(&mut self, rule: &FatigueRule, forces: &[f32], dt: f32) -> Vec<usize> {
        self.damage.resize(forces.len(), 0.0);
        let mut failed = Vec::new();
        for (child, force) in forces.iter().enumerate() {
            if self.damage[child] >= 1.0 {
                continue;
            }
            let excess = (force / rule.bearing - 1.0).max(0.0);
            self.damage[child] += excess * dt / rule.endurance;
            if self.damage[child] >= 1.0 {
                failed.push(child);
            }
        }
        failed
    }

    pub fn damage_of(&self, child: usize) -> f32 {
        self.damage.get(child).copied().unwrap_or(0.0)
    }

    /// Follow the children through a change to the body's collider list:
    /// `order[new]` is the old index now at `new`, `None` for a fresh cell.
    pub fn reindex(&mut self, order: &[Option<usize>]) {
        self.damage = order
            .iter()
            .map(|old| {
                old.and_then(|old| self.damage.get(old).copied())
                    .unwrap_or(0.0)
            })
            .collect();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RULE: FatigueRule = FatigueRule {
        bearing: 400.0,
        endurance: 0.5,
    };

    #[test]
    fn a_load_within_the_bearing_never_fails() {
        let mut tracker = FatigueTracker::new(1);
        for _ in 0..6000 {
            assert!(tracker.advance(&RULE, &[400.0], 1.0 / 60.0).is_empty());
        }
        assert_eq!(tracker.damage_of(0), 0.0);
    }

    #[test]
    fn twice_the_bearing_fails_after_the_endurance() {
        let mut tracker = FatigueTracker::new(2);
        let mut failed_at = None;
        for frame in 0..120 {
            let failed = tracker.advance(&RULE, &[800.0, 100.0], 1.0 / 60.0);
            if !failed.is_empty() {
                failed_at = Some((frame, failed));
                break;
            }
        }
        let (frame, failed) = failed_at.expect("the loaded cell fails");
        assert_eq!(failed, vec![0]);
        assert!((28..=31).contains(&frame), "failed at frame {frame}");
        assert_eq!(tracker.damage_of(1), 0.0);
    }

    #[test]
    fn damage_follows_its_cell_through_a_reindex() {
        let mut tracker = FatigueTracker::new(3);
        tracker.advance(&RULE, &[1200.0, 400.0, 800.0], 0.1);
        let before = tracker.damage_of(2);
        tracker.reindex(&[Some(2), None, Some(0)]);
        assert_eq!(tracker.damage_of(0), before);
        assert_eq!(tracker.damage_of(1), 0.0);
        assert!(tracker.damage_of(2) > before);
    }
}
