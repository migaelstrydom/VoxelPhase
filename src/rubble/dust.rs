//! A fragment too small to be anything else, crumbling where it broke.

use crate::particles::{ParticleEffectType, ParticleEmitter};
use crate::terrain::Fragment;

/// How a fragment's crumbling is drawn: a short burst of debris sized to it.
#[derive(Debug, Clone, Copy)]
pub struct Crumble {
    /// Flecks per sample of the fragment.
    pub flecks_per_sample: f32,
    /// Bounds on the flecks one fragment throws, whatever its size.
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
    /// The emitter that crumbles `fragment`, to be placed at its centroid.
    pub fn emitter(&self, fragment: &Fragment) -> ParticleEmitter {
        let flecks = (fragment.sample_count() as f32 * self.flecks_per_sample).round() as u32;
        ParticleEmitter::new(ParticleEffectType::Debris)
            .with_burst(flecks.clamp(self.min_flecks, self.max_flecks))
            .with_scale(self.scale_per_voxel_metre * fragment.voxel_size().min(1.0))
            .with_lifetime(EMITTER_LIFETIME)
    }
}
