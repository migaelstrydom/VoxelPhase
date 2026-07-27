//! Point lighting: which local lights illuminate the frame.
//!
//! Lighting uses a single global per-frame light set rather than per-object
//! light lists — a large terrain chunk spans many lights, so per-object
//! selection would produce lighting discontinuities at chunk seams.
//!
//! ```text
//!   PointLight + Position ──▶ LightCollectionSystem ──▶ LightCollector
//!                                                            │
//!                                          ActiveLights ◀────┘
//! ```

mod collector;
mod point_light;
mod system;

pub use collector::{
    ActiveLight, ActiveLights, LightCandidate, LightCollector, LightId, MAX_ACTIVE_LIGHTS,
};
pub use point_light::PointLight;
pub use system::LightCollectionSystem;
