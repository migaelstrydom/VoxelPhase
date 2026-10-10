//! Water as a shallow-water field over the terrain's spans; and the
//! hydrology network it replaces, which only the water tools still run.
//!
//! ```text
//!   presentation   rendering/water (FieldScene)       the field's surface, rebuilt as it moves
//!   interaction    world · query · buoyancy · coupling WaterQuery::sample(point)
//!   field          field                              depths in cells, discharges on faces
//!   network        network_world · network · solver · topology   stores, links, ledger (tools)
//!   geometry       geometry                           spans, drainage field
//! ```
//!
//! See `docs/WATER_FIELD_SPIKE.md`; the network, `docs/WATER_HYDROLOGY_DESIGN.md`.

pub mod buoyancy;
pub mod coupling;
pub mod debug;
pub mod field;
pub mod geometry;
pub mod ids;
pub mod network;
pub mod network_query;
pub mod network_world;
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
pub use network_query::NetworkQuery;
pub use network_world::{HydrologyConfig, NetworkWater, WaterTimings};
pub use query::{WaterQuery, WaterSample as WaterPointSample, FIELD_BODY};
pub use sleep_tracker::WaterSleepTracker;
pub use world::WaterWorld;
