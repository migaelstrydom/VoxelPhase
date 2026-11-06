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

use rustc_hash::FxHashMap;

use crate::physics::ColliderHandle;

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

/// Per-child damage, from 0 (sound) to 1 (failed), kept by collider.
#[derive(Debug, Default)]
pub struct FatigueTracker {
    /// Damage of every child seen on the last advance.
    damage: FxHashMap<ColliderHandle, f32>,
}

impl FatigueTracker {
    /// Add `dt` seconds of `forces` (newtons, by child) and report the
    /// children that have just failed. A failed child stays at full damage
    /// until it is removed; a child not listed is forgotten, and one not seen
    /// before starts sound.
    pub fn advance(
        &mut self,
        rule: &FatigueRule,
        forces: &[(ColliderHandle, f32)],
        dt: f32,
    ) -> Vec<ColliderHandle> {
        let mut damage = FxHashMap::default();
        let mut failed = Vec::new();
        for &(child, force) in forces {
            let mut sum = self.damage.get(&child).copied().unwrap_or(0.0);
            if sum < 1.0 {
                let excess = (force / rule.bearing - 1.0).max(0.0);
                sum += excess * dt / rule.endurance;
                if sum >= 1.0 {
                    failed.push(child);
                }
            }
            damage.insert(child, sum);
        }
        self.damage = damage;
        failed
    }

    pub fn damage_of(&self, child: ColliderHandle) -> f32 {
        self.damage.get(&child).copied().unwrap_or(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RULE: FatigueRule = FatigueRule {
        bearing: 400.0,
        endurance: 0.5,
    };

    fn child(slot: usize) -> ColliderHandle {
        ColliderHandle::for_test(slot)
    }

    #[test]
    fn a_load_within_the_bearing_never_fails() {
        let mut tracker = FatigueTracker::default();
        for _ in 0..6000 {
            assert!(tracker
                .advance(&RULE, &[(child(0), 400.0)], 1.0 / 60.0)
                .is_empty());
        }
        assert_eq!(tracker.damage_of(child(0)), 0.0);
    }

    #[test]
    fn twice_the_bearing_fails_after_the_endurance() {
        let mut tracker = FatigueTracker::default();
        let mut failed_at = None;
        for frame in 0..120 {
            let failed =
                tracker.advance(&RULE, &[(child(0), 800.0), (child(1), 100.0)], 1.0 / 60.0);
            if !failed.is_empty() {
                failed_at = Some((frame, failed));
                break;
            }
        }
        let (frame, failed) = failed_at.expect("the loaded cell fails");
        assert_eq!(failed, vec![child(0)]);
        assert!((28..=31).contains(&frame), "failed at frame {frame}");
        assert_eq!(tracker.damage_of(child(1)), 0.0);
    }

    #[test]
    fn damage_follows_its_cell_through_a_reshuffle() {
        let mut tracker = FatigueTracker::default();
        tracker.advance(
            &RULE,
            &[(child(0), 1200.0), (child(1), 400.0), (child(2), 800.0)],
            0.1,
        );
        let before = tracker.damage_of(child(2));
        tracker.advance(
            &RULE,
            &[(child(2), 400.0), (child(3), 400.0), (child(0), 400.0)],
            0.1,
        );
        assert_eq!(tracker.damage_of(child(2)), before);
        assert_eq!(tracker.damage_of(child(3)), 0.0);
        assert!(tracker.damage_of(child(0)) > before);
    }
}
