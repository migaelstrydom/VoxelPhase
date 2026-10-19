//! A fixed rate of spray shared out over whatever throws it, by weight.
//!
//! ```text
//!   sources, each with a power ──▶ share = total power / full power, ≤ 1
//!                              ──▶ rate × share × dt owed this frame,
//!                                  the fraction carried to the next
//!                              ──▶ each one owed goes to a source drawn
//!                                  by power
//! ```
//!
//! The rate is for every source at once, so a level with many falls or a
//! long front throws no more than one with a few; the bigger take more of
//! it, and a scene of small ones throws less than one big one.

use rand::Rng;

/// One kind of particle's rate, and what is owed of it between frames.
#[derive(Debug, Clone, Copy)]
pub struct SprayBudget {
    /// Particles a second, when the sources' power reaches `full_power`.
    per_second: f32,
    /// Fraction of a particle owed from earlier frames.
    owed: f32,
}

impl SprayBudget {
    pub const fn new(per_second: f32) -> Self {
        Self {
            per_second,
            owed: 0.0,
        }
    }

    /// Spend `dt`'s share on `sources`, each `(source, power)`, at `share`
    /// of the full rate: `spawn` is called once per particle with the source
    /// drawn for it. Nothing is carried over a frame with no sources.
    pub fn spend<T, R: Rng>(
        &mut self,
        sources: &[(T, f32)],
        share: f32,
        dt: f32,
        rng: &mut R,
        mut spawn: impl FnMut(&T, &mut R),
    ) {
        let total: f32 = sources.iter().map(|&(_, power)| power).sum();
        if total <= 0.0 {
            self.owed = 0.0;
            return;
        }
        self.owed += self.per_second * share * dt;
        while self.owed >= 1.0 {
            self.owed -= 1.0;
            let source = pick(sources, total, rng);
            spawn(source, rng);
        }
    }
}

/// Share of the full rate that sources of `total` power take: all of it
/// from `full_power` up.
pub fn share(total: f32, full_power: f32) -> f32 {
    (total / full_power).min(1.0)
}

/// A source drawn with chance in proportion to its power.
fn pick<'a, T>(sources: &'a [(T, f32)], total: f32, rng: &mut impl Rng) -> &'a T {
    let mut left = rng.gen_range(0.0..total);
    for (source, power) in sources {
        left -= power;
        if left <= 0.0 {
            return source;
        }
    }
    &sources[sources.len() - 1].0
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::rngs::StdRng;
    use rand::SeedableRng;

    #[test]
    fn the_rate_is_spent_whatever_the_number_of_sources() {
        let mut rng = StdRng::seed_from_u64(1);
        for count in [1, 10, 100] {
            let sources: Vec<(usize, f32)> = (0..count).map(|i| (i, 1.0)).collect();
            let mut budget = SprayBudget::new(100.0);
            let mut spawned = 0;
            for _ in 0..60 {
                budget.spend(&sources, 1.0, 1.0 / 60.0, &mut rng, |_, _| spawned += 1);
            }
            assert!((99..=100).contains(&spawned), "{count} sources: {spawned}");
        }
    }

    #[test]
    fn the_more_powerful_source_takes_more() {
        let mut rng = StdRng::seed_from_u64(2);
        let sources = [(0usize, 1.0), (1usize, 9.0)];
        let mut budget = SprayBudget::new(1000.0);
        let mut counts = [0; 2];
        budget.spend(&sources, 1.0, 1.0, &mut rng, |&s, _| counts[s] += 1);
        assert!(counts[1] > counts[0] * 5, "{counts:?}");
    }

    #[test]
    fn nothing_is_owed_across_a_frame_without_sources() {
        let mut rng = StdRng::seed_from_u64(3);
        let mut budget = SprayBudget::new(30.0);
        budget.spend(&[(0usize, 1.0)], 1.0, 0.02, &mut rng, |_, _| {});
        budget.spend::<usize, _>(&[], 1.0, 0.02, &mut rng, |_, _| {});
        let mut spawned = 0;
        budget.spend(&[(0usize, 1.0)], 1.0, 0.02, &mut rng, |_, _| spawned += 1);
        assert_eq!(spawned, 0);
    }
}
