//! A fragment too small to be anything else, crumbling where it broke, and
//! scree crumbling where it lands.

use nalgebra::Vector3;

use crate::particles::{Palette, ParticleEffectType, ParticleEmitter};
use crate::terrain::Fragment;

/// What a crumble is made of: how much, and in what colours.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CrumbleSize {
    pub samples: usize,
    /// Solid volume, in m³.
    pub volume: f32,
    /// Its materials' colours, weighted by how many samples of each: a pillar
    /// of rock under a skin of grass crumbles mostly grey, partly green.
    pub palette: Palette,
}

impl CrumbleSize {
    pub fn of(fragment: &Fragment) -> Self {
        let colours: Vec<(Vector3<f32>, f32)> = fragment
            .materials()
            .into_iter()
            .map(|(material, samples)| {
                let [r, g, b, _] = material.color();
                (Vector3::new(r, g, b), samples as f32)
            })
            .collect();
        Self {
            samples: fragment.sample_count(),
            volume: fragment.volume(),
            palette: Palette::mixed(&colours),
        }
    }
}

/// How a crumble is drawn: a burst of debris sized to what crumbled.
#[derive(Debug, Clone, Copy)]
pub struct Crumble {
    /// Flecks per sample.
    pub flecks_per_sample: f32,
    /// Bounds on the flecks one crumble throws, whatever its size.
    pub min_flecks: u32,
    pub max_flecks: u32,
    /// The debris effect's scale per metre of the crumble's own size (the
    /// cube root of its volume). The effect is authored to be thrown by a
    /// blast; at a sliver's size it reads as falling apart instead, and at a
    /// slab's it covers where the slab came down.
    pub scale_per_metre: f32,
    /// Bounds on that scale.
    pub min_scale: f32,
    pub max_scale: f32,
}

impl Default for Crumble {
    fn default() -> Self {
        Self {
            flecks_per_sample: 2.0,
            min_flecks: 6,
            max_flecks: 120,
            scale_per_metre: 0.6,
            min_scale: 0.15,
            max_scale: 2.0,
        }
    }
}

/// Seconds the emitter lives: long enough to fire its burst on its first
/// update, which is all it is for.
const EMITTER_LIFETIME: f32 = 0.1;

impl Crumble {
    /// The emitter for a crumble of `size`, to be placed where it happens.
    pub fn emitter(&self, size: CrumbleSize) -> ParticleEmitter {
        let flecks = (size.samples as f32 * self.flecks_per_sample).round() as u32;
        let scale =
            (self.scale_per_metre * size.volume.cbrt()).clamp(self.min_scale, self.max_scale);
        ParticleEmitter::new(ParticleEffectType::Debris)
            .with_burst(flecks.clamp(self.min_flecks, self.max_flecks))
            .with_scale(scale)
            .with_palette(size.palette)
            .with_lifetime(EMITTER_LIFETIME)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn size(samples: usize, volume: f32) -> CrumbleSize {
        CrumbleSize {
            samples,
            volume,
            palette: Palette::single(Vector3::repeat(0.5)),
        }
    }

    /// A one-sample chip at 0.5 m voxels crumbles as it always did; a slab
    /// throws more and bigger flecks, up to the bounds.
    #[test]
    fn a_crumble_grows_with_what_crumbled() {
        let crumble = Crumble::default();
        let chip = crumble.emitter(size(1, 0.125));
        assert_eq!(chip.initial_burst, 6);
        assert!((chip.scale - 0.3).abs() < 1e-5);
        let slab = crumble.emitter(size(500, 62.5));
        assert_eq!(slab.initial_burst, 120);
        assert_eq!(slab.scale, 2.0);
    }
}
