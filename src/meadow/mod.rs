//! Meadow: the small things that grow on a level's lawns (tufts of long grass, flowers, berries), sown from the terrain's own turf and drawn a tile at a time.
//!
//! ```text
//!   level file ── MeadowConfig ──▶ Meadow (resource)
//!                                    │  MeadowSystem, whenever the terrain's
//!                                    ▼  drawn mesh has changed
//!   TerrainWorld render mesh ──sow──▶ Sprigs by tile ──mesh──▶ a model per tile
//!   (triangles of level turf)        (what grows where)       (an entity each)
//!
//!   after a blast: changed_regions ──▶ plots ──resow──▶ the chunks over them
//!                                                       alone are read again
//! ```
//!
//! What grows where is decided by the ground alone: a grid of cells over the
//! world, each sown or not by a hash of where it is, wherever a cell's middle
//! lies on level turf. So the whole level is sown once, when it opens, and a
//! blast has only the ground it changed sown again: what stood over the
//! crater goes, the rest keeps every sprig it had, and only the tiles that
//! changed are drawn afresh. Nothing here collides or is simulated.

mod config;
mod meadow;
mod sow;
mod sprig;
mod system;

pub use config::MeadowConfig;
pub use meadow::{meadow_material, Meadow};
pub use system::MeadowSystem;
