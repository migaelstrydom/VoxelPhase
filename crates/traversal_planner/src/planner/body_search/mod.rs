//! The body's path alone, searched over stations along the path: where it
//! rides, where it flies and how fast. Footfalls are placed along it after.

mod feet;
mod flights;
mod previous;
mod rides;
mod search;

pub use previous::{FlightSpan, PreviousPath, RideSpan};
pub use search::BodySearch;
