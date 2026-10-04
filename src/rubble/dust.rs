//! A fragment too small to be anything else, crumbling where it broke, and
//! scree crumbling where it lands.

use crate::particles::{ParticleEffectType, ParticleEmitter};
use crate::terrain::Fragment;

/// How big a crumble is: what it is made of, counted in samples.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CrumbleSize {
    pub samples: usize,
    /// Edge of one sample, in metres.
    pub voxel_size: f32,
}

impl CrumbleSize {
    pub fn of(fragment: &Fragment) -> Self {
        Self {
            samples: fragment.sample_count(),
            voxel_size: fragment.voxel_size(),
        }
    }
}

/// How a crumble is drawn: a short burst of debris sized to it.
#[derive(Debug, Clone, Copy)]
pub struct Crumble {
    /// Flecks per sample.
    pub flecks_per_sample: f32,
    /// Bounds on the flecks one crumble throws, whatever its size.
    pub min_flecks: u32,
    pub max_flecks: u32,
    /// The debris effect's scale per metre of voxel. The effect is authored to
    /// be thrown by a blast; scaled down, it reads as falling apart instead.
    pub scale_per_voxel_metre: f32,
}

impl Default for Crumble {
    fn default() -> Self {
        Self {
            flecks_per_sample: 2.0,
            min_flecks: 4,
            max_flecks: 48,
            scale_per_voxel_metre: 0.6,
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
        ParticleEmitter::new(ParticleEffectType::Debris)
            .with_burst(flecks.clamp(self.min_flecks, self.max_flecks))
            .with_scale(self.scale_per_voxel_metre * size.voxel_size.min(1.0))
            .with_lifetime(EMITTER_LIFETIME)
    }
}
