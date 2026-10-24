//! The section: the ground ahead as a strip of lanes in path coordinates, and
//! the queries the planner reads it through.

mod query;
mod sampling;
mod strip;
mod strip_query;
mod surface;

pub use query::{FloorHit, Foothold, FootholdWindow, Ride, SectionQuery};
pub use sampling::{sample_section, BodyKey, CastHit, SphereCaster};
pub use strip::{FootholdLane, Profile, Section, StripLayout};
pub use strip_query::StripQuery;
pub use surface::{Footing, Surface, SurfacePoint};
