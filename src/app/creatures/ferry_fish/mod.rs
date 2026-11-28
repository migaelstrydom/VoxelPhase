//! The ferry fish: a flatfish the size of a cart that swims through the air
//! with the player on its back. A moving platform in the garden's look.
//!
//! ```text
//!   form    the clay of its body, and the deck it collides as, sampled
//!           from the same clay: what is stood on is what is seen
//!   sculpt  its look: a back of sand spotted vermilion, a pearl belly,
//!           goggling gold eyes, a crooked mouth with a gilt ring through
//!           its lower lip, whose jaw the rig drops to gulp, pale wings,
//!           and fringe fins the rig ripples
//!   def     the level object: the body every platform has, its look, and
//!           its rig
//! ```
//!
//! It is a platform first: a `PoweredBody` patrolling a route like any
//! other, which turns to swim nose first. `visual_bench ferry_fish` shows it.

mod def;
mod form;
mod sculpt;

pub use def::FerryFishDef;
