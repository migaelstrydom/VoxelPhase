//! Meadow: the small things that grow on a level's lawns (tufts of long grass, flowers, berries), sown from the terrain's own turf and drawn a tile at a time.
//!
//! ```text
//!   level file ── MeadowConfig ──▶ Meadow (resource)
//!                                    │  MeadowSystem, whenever the terrain's
//!                                    ▼  drawn mesh has changed
//!   TerrainWorld render mesh ──sow──▶ Sprigs by tile ──mesh──▶ a model per tile
//!   (triangles of level turf)        (what grows where)       (an entity each)
//! ```
//!
//! What grows where is decided by the ground alone: a grid of cells over the
//! world, each sown or not by a hash of where it is, wherever a cell's middle
//! lies on level turf. Ground a blast leaves alone keeps every sprig it had,
//! ground it carves loses its own, and only the tiles that changed are drawn
//! afresh. Nothing here collides or is simulated.

mod config;
mod meadow;
mod sow;
mod sprig;
mod system;

pub use config::MeadowConfig;
pub use meadow::{meadow_material, Meadow};
pub use system::MeadowSystem;
