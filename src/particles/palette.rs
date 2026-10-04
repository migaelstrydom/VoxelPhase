//! Colours an emitter's particles are drawn in, mixed in proportion.

use nalgebra::Vector3;
use rand::Rng;

/// Up to [`Palette::MAX`] colours, each with a weight: every particle takes
/// one, chosen in proportion to the weights. Fixed capacity and `Copy`, like
/// `ColourRamp`, so an emitter carrying one does not allocate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Palette {
    entries: [(Vector3<f32>, f32); Self::MAX],
    count: usize,
}

impl Palette {
    /// Colours one palette holds. Past this, the lightest weights are dropped.
    pub const MAX: usize = 4;

    /// One colour for every particle.
    pub fn single(colour: Vector3<f32>) -> Self {
        Self::mixed(&[(colour, 1.0)])
    }

    /// Colours with their weights. Non-positive weights are dropped; with
    /// none left, particles are white.
    pub fn mixed(entries: &[(Vector3<f32>, f32)]) -> Self {
        let mut kept: Vec<(Vector3<f32>, f32)> =
            entries.iter().copied().filter(|&(_, w)| w > 0.0).collect();
        kept.sort_by(|a, b| b.1.total_cmp(&a.1));
        kept.truncate(Self::MAX);
        if kept.is_empty() {
            kept.push((Vector3::repeat(1.0), 1.0));
        }
        let mut palette = Self {
            entries: [(Vector3::zeros(), 0.0); Self::MAX],
            count: kept.len(),
        };
        palette.entries[..kept.len()].copy_from_slice(&kept);
        palette
    }

    /// One colour, chosen in proportion to the weights.
    pub fn pick(&self, rng: &mut impl Rng) -> Vector3<f32> {
        let entries = &self.entries[..self.count];
        let total: f32 = entries.iter().map(|&(_, w)| w).sum();
        let mut roll = rng.gen_range(0.0..total);
        for &(colour, weight) in entries {
            if roll < weight {
                return colour;
            }
            roll -= weight;
        }
        entries[entries.len() - 1].0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::rngs::StdRng;
    use rand::SeedableRng;

    /// Three parts grey to one part green comes out about three to one.
    #[test]
    fn colours_are_picked_in_proportion() {
        let (grey, green) = (Vector3::repeat(0.5), Vector3::new(0.2, 0.4, 0.1));
        let palette = Palette::mixed(&[(grey, 3.0), (green, 1.0)]);
        let mut rng = StdRng::seed_from_u64(1);
        let greens = (0..4000)
            .filter(|_| palette.pick(&mut rng) == green)
            .count();
        assert!((900..1100).contains(&greens), "{greens} of 4000 green");
    }

    #[test]
    fn the_lightest_colours_are_dropped_past_capacity() {
        let entries: Vec<_> = (1..=6)
            .map(|i| (Vector3::repeat(i as f32), i as f32))
            .collect();
        let palette = Palette::mixed(&entries);
        let mut rng = StdRng::seed_from_u64(2);
        assert!((0..500).all(|_| palette.pick(&mut rng).x >= 3.0));
    }
}
