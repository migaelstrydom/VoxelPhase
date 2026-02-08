use nalgebra::Vector3;

#[derive(Clone, Copy, Debug)]
pub struct NormalSmoothingConfig {
    /// Cosine threshold required to treat two normals as aligned.
    pub alignment_threshold: f32,
    /// Blend factor toward the current normal when smoothing.
    pub smoothing: f32,
}

impl Default for NormalSmoothingConfig {
    fn default() -> Self {
        Self {
            alignment_threshold: 0.95,
            smoothing: 0.2,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct NormalSmoother {
    /// Cosine threshold required to treat two normals as aligned.
    alignment_threshold: f32,
    /// Blend factor toward the current normal when smoothing.
    smoothing: f32,
}

impl NormalSmoother {
    pub fn new(alignment_threshold: f32, smoothing: f32) -> Self {
        Self {
            alignment_threshold,
            smoothing,
        }
    }

    pub fn from_config(config: NormalSmoothingConfig) -> Self {
        Self::new(config.alignment_threshold, config.smoothing)
    }

    pub fn aligned(&self, cached: Vector3<f32>, current: Vector3<f32>) -> bool {
        cached.dot(&current) >= self.alignment_threshold
    }

    pub fn smooth(&self, cached: Vector3<f32>, current: Vector3<f32>) -> Vector3<f32> {
        if self.smoothing <= 0.0 {
            return cached;
        }
        (cached * (1.0 - self.smoothing) + current * self.smoothing).normalize()
    }
}
