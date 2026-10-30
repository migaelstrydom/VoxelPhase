//! The plan: contact and flight phases from now to the horizon, the one thing
//! the body's drive and the legs both read.

mod body;
mod phase;
mod timeline;

pub use body::BodyState;
pub use phase::{climb_reaching, flight_at, Contact, FootfallId, Limb, Motion, Phase};
pub use timeline::Plan;
