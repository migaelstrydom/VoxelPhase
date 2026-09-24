//! Water as a network: stores that hold volume, joined by links that move it.
//!
//! ```text
//!   presentation   rendering/water (BasinMesher)      static meshes, levels as uniforms
//!   interaction    query · buoyancy · coupling        WaterQuery::sample(point)
//!   network        network · solver · topology        stores, links, ledger
//!   geometry       geometry                           spans, drainage field
//! ```
//!
//! See `docs/WATER_HYDROLOGY_DESIGN.md`.

pub mod buoyancy;
pub mod coupling;
pub mod debug;
pub mod geometry;
pub mod ids;
pub mod network;
pub mod query;
pub mod sleep_tracker;
pub mod solver;
pub mod surface;
pub mod topology;
pub mod world;

pub use coupling::{
    BodySnapshot, Disturbance, Disturbances, RippleField, SplashEvent, StillSurface, WakeEvent,
    WaveBodyCoupler, WaveCouplingConfig,
};
pub use query::{WaterQuery, WaterSample as WaterPointSample};
pub use sleep_tracker::WaterSleepTracker;
pub use world::{HydrologyConfig, WaterTimings, WaterWorld};
