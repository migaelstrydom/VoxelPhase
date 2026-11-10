//! The fountain tower: a carved stone spire that stands in a lake and
//! throws the lake's water from its spouts.
//!
//! ```text
//!   tower    FountainDef: the design, and the compound it is spawned as
//!   blocks   Block: one collider of the tower, and the hull it breaks as
//! ```

mod blocks;
mod tower;

pub use tower::FountainDef;
