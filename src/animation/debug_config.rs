//! Runtime toggles for animation debug visualisations.

/// Flags controlling which animation debug overlays are drawn. Lives as a
/// world resource so ECS systems can read it without being re-plumbed.
#[derive(Clone, Copy, Debug)]
pub struct AnimationDebugConfig {
    /// Draw the foot placer overlay (ideal targets, planted positions,
    /// error vectors, in-flight swing arcs).
    pub foot_placer_overlay: bool,
}

impl Default for AnimationDebugConfig {
    fn default() -> Self {
        Self {
            foot_placer_overlay: false,
        }
    }
}
