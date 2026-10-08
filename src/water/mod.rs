//! Water as a network: stores that hold volume, joined by links that move it;
//! and, on trial to replace it, as a shallow-water field over the same spans.
//!
//! ```text
//!   presentation   rendering/water (BasinMesher)      static meshes, levels as uniforms
//!   interaction    query · buoyancy · coupling        WaterQuery::sample(point)
//!   network        network · solver · topology        stores, links, ledger
//!   field          field                              depths in cells, discharges on faces
//!   geometry       geometry                           spans, drainage field
//! ```
//!
//! See `docs/WATER_HYDROLOGY_DESIGN.md`; the field, `docs/WATER_FIELD_SPIKE.md`.

pub mod buoyancy;
pub mod coupling;
pub mod debug;
pub mod field;
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
