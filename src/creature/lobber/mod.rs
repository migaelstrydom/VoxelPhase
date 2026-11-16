//! Lobbing: a creature that throws bombs at what it sees.
//!
//! ```text
//!   Perception ──target in range──▶ a grenade, launched on the arc that
//!                                   lands on the target after `flight_time`
//!
//!   seconds to the next throw ──▶ LobberBody ──▶ PosedModel: a bag that sags
//!   seconds since the last ─────▶                after a throw and swells
//!   what it watches ────────────▶                tight as the next comes,
//!   Puffing ────────────────────▶                what is fastened into it
//!                                                riding it, eyes that follow
//!                                                the target, and the whole
//!                                                squatting and trembling as
//!                                                it winds up, heaving up as
//!                                                the bomb leaves
//!                                ──▶ ProceduralMesh: cheeks blown out of
//!                                                the face that fill the bag
//! ```
//!
//! The bombs are the game's own grenades, so they fall, bounce, go off and
//! carve the ground exactly as the player's do: a lobber pins a route down,
//! and breaks it up while it does.
//!
//! ```text
//!   body     the parts that show a throw coming, and how each is posed
//!   puffing  the cheeks' puffing, kept from frame to frame
//!   system   the Lobber component, the throw's arc and LobberSystem
//! ```

mod body;
mod puffing;
mod system;

pub use body::{Cheeks, Gaze, LobberBody, Rider, Showing, Strut, Swell, Throw};
pub use puffing::Puffing;
pub use system::{Lobber, LobberSystem};
