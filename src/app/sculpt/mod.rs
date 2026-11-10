//! Sculpted forms: things modelled as a handful of rounded pieces, warped
//! off the ellipsoids they start as and painted, so that they read as made
//! by hand rather than moulded. Creatures' bodies and the garden's fruit
//! and ornaments are built from them.
//!
//! ```text
//!   SculptedPart ──mesh──▶ mesh ──Mottle::paint──▶ ──ContactShade::shade──▶ model
//!   (Ellipsoid,    (warped,      (patches of        (shade where one part
//!    Warps)         warp.rs)      colour)            meets another)
//!
//!   SkinPattern ──texture──▶ the material's texture: freckles, veins, pores
//! ```

pub mod paint;
pub mod parts;
pub mod pattern;
pub mod warp;

pub use paint::{painted_meshes, painted_model, scatter, ContactShade, Mottle};
pub use parts::{Ellipsoid, SculptedPart};
pub use pattern::{Clouds, SkinPattern, Spots, Veins};
pub use warp::Warp;
