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
//!
//!   Clay ──skin──▶ mesh: lumps and limbs smoothed into one another, hollowed
//!   (clay.rs)      and carved; for flesh, cloth, shell and cast metal, where
//!                  a crease between parts would give the modelling away
//!
//!   Strand ──lay──▶ mesh: a tube swept along a curve, for what is too thin
//!   (strand.rs)     or too sharp for clay: thorns, threads, stems
//! ```

pub mod clay;
pub mod paint;
pub mod parts;
pub mod pattern;
pub mod strand;
pub mod warp;

pub use clay::{Clay, Lump, Relief};
pub use paint::{
    painted_meshes, painted_model, scatter, Blush, ContactShade, Grime, Mottle, VertexPaint,
};
pub use parts::{Ellipsoid, SculptedPart};
pub use pattern::{Clouds, SkinPattern, Spots, Veins};
pub use strand::Strand;
pub use warp::Warp;
