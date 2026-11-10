//! What a level asks of its meadow.

use serde::Deserialize;

/// How thickly a level's lawns are sown.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeadowConfig {
    /// Tufts of long grass to the square metre of level turf, about, over
    /// the whole lawn: they gather where it is lush and thin out between.
    #[serde(default = "MeadowConfig::default_tufts")]
    pub tufts: f32,
    /// Flowers and berries to the square metre, about: they grow in drifts.
    #[serde(default = "MeadowConfig::default_blooms")]
    pub blooms: f32,
    /// Which meadow of all those the ground could bear.
    #[serde(default)]
    pub seed: u32,
}

impl MeadowConfig {
    pub fn default_tufts() -> f32 {
        4.0
    }

    pub fn default_blooms() -> f32 {
        2.0
    }
}

impl Default for MeadowConfig {
    fn default() -> Self {
        Self {
            tufts: Self::default_tufts(),
            blooms: Self::default_blooms(),
            seed: 0,
        }
    }
}
