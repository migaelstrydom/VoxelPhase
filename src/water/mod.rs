//! Water as a shallow-water field over the terrain's spans.
//!
//! ```text
//!   presentation   rendering/water (FieldScene)        the field's surface, read again as it moves
//!   interaction    world · query · buoyancy · coupling  WaterQuery::sample(point)
//!                  falling                             streams in the air, what they strike
//!   waves          waves · surface                     ripples and swell over the field
//!   field          field                               depths in cells, discharges on faces
//!   geometry       geometry                            spans, and the graph joining them
//! ```
//!
//! See `docs/WATER_FIELD_SPIKE.md`.

pub mod buoyancy;
pub mod coupling;
pub mod falling;
pub mod field;
pub mod geometry;
pub mod ids;
pub mod query;
pub mod sleep_tracker;
pub mod surface;
pub mod waves;
pub mod world;

pub use coupling::{
    BodySnapshot, Disturbance, Disturbances, RippleField, SplashEvent, StillSurface, WakeEvent,
    WaveBodyCoupler, WaveCouplingConfig,
};
pub use query::{WaterQuery, WaterSample as WaterPointSample, FIELD_BODY};
pub use sleep_tracker::WaterSleepTracker;
pub use world::WaterWorld;
