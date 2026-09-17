mod block;
mod ice_box;
mod igloo;
mod wall;

#[cfg(test)]
pub use block::ice_cleaving;
pub use block::{ice_block_mesh, texture_spread as ice_texture_spread};
pub use ice_box::IceBoxDef;
pub use igloo::IglooDef;
pub use wall::IceWallDef;
