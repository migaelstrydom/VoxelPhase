//! A lobber's cheeks puffing: quicker and harder as its bag fills, kept
//! from frame to frame so that neither a change of pace nor a throw makes
//! them jump.
//!
//! ```text
//!   fullness ──▶ rate ──▶ phase += rate·dt ──▶ puff ──┐
//!            └─▶ depth ──lag──────────────────────────┴─▶ blown = depth · puff
//! ```

use std::f32::consts::TAU;

/// Puffs a second just after a throw, and as the next is due.
const RATE: (f32, f32) = (0.45, 1.0);

/// How hard a puff blows just after a throw, as a share of how hard it
/// blows as the next is due.
const FLOOR: f32 = 0.35;

/// Seconds the depth of the puffs takes to follow the bag most of the way
/// (to within a third): after a throw they ease off, they do not stop.
const LAG: f32 = 0.3;

/// Where a lobber's cheeks are in their puffing.
#[derive(Debug, Clone, Copy, Default)]
pub struct Puffing {
    /// How far through a puff, in turns: kept, not worked out from a
    /// clock, so that a change of rate does not move it.
    phase: f32,
    /// How hard the puffs blow, 0 to 1, following the bag.
    depth: f32,
}

impl Puffing {
    /// Puff on for `dt` seconds with the bag `fullness` of the way from one
    /// throw to the next.
    pub fn advance(&mut self, fullness: f32, dt: f32) {
        let fullness = fullness.clamp(0.0, 1.0);
        let rate = RATE.0 + (RATE.1 - RATE.0) * fullness;
        self.phase = (self.phase + rate * dt).fract();
        let wanted = FLOOR + (1.0 - FLOOR) * fullness;
        self.depth += (wanted - self.depth) * (1.0 - (-dt / LAG).exp());
    }

    /// How far the cheeks are blown, 0 between puffs to 1 at the top of
    /// the hardest.
    pub fn blown(&self) -> f32 {
        self.depth * (0.5 - 0.5 * (self.phase * TAU).cos())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn its_cheeks_never_jump_and_puff_harder_as_the_bag_fills() {
        let dt = 1.0 / 60.0;
        let mut puffing = Puffing::default();
        let (mut fullness, mut last) = (0.0, 0.0);
        let (mut early, mut late) = (0.0f32, 0.0f32);
        // Three throws' worth, the bag emptying at each.
        for frame in 0..(3.0 * 3.5 / dt) as usize {
            fullness += dt / 3.5;
            if fullness >= 1.0 {
                fullness = 0.0;
            }
            puffing.advance(fullness, dt);
            let blown = puffing.blown();
            assert!((0.0..=1.0).contains(&blown));
            // Never faster than the quickest puff fills them.
            assert!(
                (blown - last).abs() < (0.5 * TAU * RATE.1 + 0.1) * dt,
                "jumped from {last} to {blown} at frame {frame}"
            );
            last = blown;
            // Early, once the puffs have eased off after the throw.
            if (0.2..0.35).contains(&fullness) {
                early = early.max(blown);
            } else if fullness > 0.8 {
                late = late.max(blown);
            }
        }
        assert!(
            late > early + 0.3,
            "as hard early ({early}) as late ({late})"
        );
    }
}
