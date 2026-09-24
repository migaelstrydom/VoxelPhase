//! Sun shadow mapping: a single orthographic depth map rendered from the
//! directional light and sampled by every lit surface.

pub mod frustum;
pub mod map;
pub mod pipeline;
pub mod renderer;
pub mod volume;

pub use frustum::{FrustumSlice, ViewFrustum};
pub use map::ShadowMap;
pub use renderer::{CasterBindings, ShadowRenderer};
pub use volume::{ShadowFraming, ShadowVolume};
