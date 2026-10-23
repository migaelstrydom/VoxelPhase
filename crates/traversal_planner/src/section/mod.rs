//! The section: the ground ahead as a strip of lanes in path coordinates, and
//! the queries the planner reads it through.

mod query;
mod strip;
mod surface;

pub use query::{FloorHit, Foothold, FootholdWindow, SectionQuery};
pub use strip::{FootholdLane, Profile, Section, StripLayout};
pub use surface::{BodyId, Support, Surface, SurfacePoint};
