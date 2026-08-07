//! Sun shadow mapping: a single orthographic depth map rendered from the
//! directional light and sampled by every lit surface.

pub mod map;
pub mod pipeline;
pub mod renderer;
pub mod volume;

pub use map::ShadowMap;
pub use renderer::ShadowRenderer;
pub use volume::ShadowVolume;
