//! Configuration for particle effects.

use nalgebra::Vector4;

/// Base configuration for all particle effects.
pub trait ParticleEffectConfig {
    /// Minimum particle lifetime in seconds.
    fn min_lifetime(&self) -> f32;
    /// Maximum particle lifetime in seconds.
    fn max_lifetime(&self) -> f32;
    /// Minimum particle size.
    fn min_size(&self) -> f32;
    /// Maximum particle size.
    fn max_size(&self) -> f32;
    /// Minimum initial speed.
    fn min_speed(&self) -> f32;
    /// Maximum initial speed.
    fn max_speed(&self) -> f32;
    /// Starting color.
    fn start_color(&self) -> Vector4<f32>;
    /// Ending color (for fading).
    fn end_color(&self) -> Vector4<f32>;
    /// Gravity scale (0.0 = no gravity, 1.0 = normal).
    fn gravity_scale(&self) -> f32;
    /// Air drag coefficient.
    fn drag(&self) -> f32;
}

/// Configuration for explosion flash particles.
#[derive(Debug, Clone)]
pub struct ExplosionFlashConfig {
    pub min_lifetime: f32,
    pub max_lifetime: f32,
    pub min_size: f32,
    pub max_size: f32,
    pub min_speed: f32,
    pub max_speed: f32,
    pub start_color: Vector4<f32>,
    pub end_color: Vector4<f32>,
}

impl Default for ExplosionFlashConfig {
    fn default() -> Self {
        Self {
            min_lifetime: 0.1,
            max_lifetime: 1.3,
            min_size: 0.3,
            max_size: 0.8,
            min_speed: 5.0,
            max_speed: 15.0,
            start_color: Vector4::new(1.0, 0.9, 0.3, 1.0), // Bright yellow
            end_color: Vector4::new(1.0, 0.3, 0.0, 0.0),   // Fade to transparent orange
        }
    }
}

impl ParticleEffectConfig for ExplosionFlashConfig {
    fn min_lifetime(&self) -> f32 {
        self.min_lifetime
    }
    fn max_lifetime(&self) -> f32 {
        self.max_lifetime
    }
    fn min_size(&self) -> f32 {
        self.min_size
    }
    fn max_size(&self) -> f32 {
        self.max_size
    }
    fn min_speed(&self) -> f32 {
        self.min_speed
    }
    fn max_speed(&self) -> f32 {
        self.max_speed
    }
    fn start_color(&self) -> Vector4<f32> {
        self.start_color
    }
    fn end_color(&self) -> Vector4<f32> {
        self.end_color
    }
    fn gravity_scale(&self) -> f32 {
        0.0
    } // No gravity for flash
    fn drag(&self) -> f32 {
        0.5
    }
}

/// Configuration for smoke particles.
#[derive(Debug, Clone)]
pub struct SmokeConfig {
    pub min_lifetime: f32,
    pub max_lifetime: f32,
    pub min_size: f32,
    pub max_size: f32,
    pub min_speed: f32,
    pub max_speed: f32,
    pub start_color: Vector4<f32>,
    pub end_color: Vector4<f32>,
    pub rise_speed: f32,
}

impl Default for SmokeConfig {
    fn default() -> Self {
        Self {
            min_lifetime: 1.5,
            max_lifetime: 3.0,
            min_size: 0.5,
            max_size: 1.5,
            min_speed: 1.0,
            max_speed: 3.0,
            start_color: Vector4::new(0.3, 0.3, 0.3, 0.8), // Dark gray
            end_color: Vector4::new(0.5, 0.5, 0.5, 0.0),   // Fade to transparent
            rise_speed: 2.0,
        }
    }
}

impl ParticleEffectConfig for SmokeConfig {
    fn min_lifetime(&self) -> f32 {
        self.min_lifetime
    }
    fn max_lifetime(&self) -> f32 {
        self.max_lifetime
    }
    fn min_size(&self) -> f32 {
        self.min_size
    }
    fn max_size(&self) -> f32 {
        self.max_size
    }
    fn min_speed(&self) -> f32 {
        self.min_speed
    }
    fn max_speed(&self) -> f32 {
        self.max_speed
    }
    fn start_color(&self) -> Vector4<f32> {
        self.start_color
    }
    fn end_color(&self) -> Vector4<f32> {
        self.end_color
    }
    fn gravity_scale(&self) -> f32 {
        -0.1
    } // Slight upward drift
    fn drag(&self) -> f32 {
        0.8
    }
}

/// Configuration for debris particles.
#[derive(Debug, Clone)]
pub struct DebrisConfig {
    pub min_lifetime: f32,
    pub max_lifetime: f32,
    pub min_size: f32,
    pub max_size: f32,
    pub min_speed: f32,
    pub max_speed: f32,
    pub color: Vector4<f32>,
}

impl Default for DebrisConfig {
    fn default() -> Self {
        Self {
            min_lifetime: 1.0,
            max_lifetime: 3.0,
            min_size: 0.1,
            max_size: 0.3,
            min_speed: 8.0,
            max_speed: 20.0,
            color: Vector4::new(0.482, 0.247, 0.0, 1.0), // Brown/gray dirt
        }
    }
}

impl ParticleEffectConfig for DebrisConfig {
    fn min_lifetime(&self) -> f32 {
        self.min_lifetime
    }
    fn max_lifetime(&self) -> f32 {
        self.max_lifetime
    }
    fn min_size(&self) -> f32 {
        self.min_size
    }
    fn max_size(&self) -> f32 {
        self.max_size
    }
    fn min_speed(&self) -> f32 {
        self.min_speed
    }
    fn max_speed(&self) -> f32 {
        self.max_speed
    }
    fn start_color(&self) -> Vector4<f32> {
        self.color
    }
    fn end_color(&self) -> Vector4<f32> {
        self.color
    }
    fn gravity_scale(&self) -> f32 {
        1.0
    } // Full gravity
    fn drag(&self) -> f32 {
        0.1
    }
}

/// Configuration for spark particles.
#[derive(Debug, Clone)]
pub struct SparksConfig {
    pub min_lifetime: f32,
    pub max_lifetime: f32,
    pub min_size: f32,
    pub max_size: f32,
    pub min_speed: f32,
    pub max_speed: f32,
    pub start_color: Vector4<f32>,
    pub end_color: Vector4<f32>,
}

impl Default for SparksConfig {
    fn default() -> Self {
        Self {
            min_lifetime: 0.2,
            max_lifetime: 0.6,
            min_size: 0.05,
            max_size: 0.15,
            min_speed: 10.0,
            max_speed: 25.0,
            start_color: Vector4::new(1.0, 0.8, 0.2, 1.0), // Bright orange-yellow
            end_color: Vector4::new(1.0, 0.2, 0.0, 0.0),   // Fade to red/transparent
        }
    }
}

impl ParticleEffectConfig for SparksConfig {
    fn min_lifetime(&self) -> f32 {
        self.min_lifetime
    }
    fn max_lifetime(&self) -> f32 {
        self.max_lifetime
    }
    fn min_size(&self) -> f32 {
        self.min_size
    }
    fn max_size(&self) -> f32 {
        self.max_size
    }
    fn min_speed(&self) -> f32 {
        self.min_speed
    }
    fn max_speed(&self) -> f32 {
        self.max_speed
    }
    fn start_color(&self) -> Vector4<f32> {
        self.start_color
    }
    fn end_color(&self) -> Vector4<f32> {
        self.end_color
    }
    fn gravity_scale(&self) -> f32 {
        0.5
    } // Half gravity for floaty sparks
    fn drag(&self) -> f32 {
        0.3
    }
}

/// Combined configuration resource for all particle effects.
#[derive(Debug, Clone, Default)]
pub struct ParticleConfig {
    pub flash: ExplosionFlashConfig,
    pub smoke: SmokeConfig,
    pub debris: DebrisConfig,
    pub sparks: SparksConfig,
    pub gravity: f32,
}

impl ParticleConfig {
    pub fn new() -> Self {
        Self {
            flash: ExplosionFlashConfig::default(),
            smoke: SmokeConfig::default(),
            debris: DebrisConfig::default(),
            sparks: SparksConfig::default(),
            gravity: 10.0,
        }
    }
}
