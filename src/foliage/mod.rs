//! Foliage: soft greenery, a firm core physics sees inside a skin of leaves that only drag and drawing see.
//!
//! ```text
//!   ShrubSpec ──grow(seed)──▶ FoliageShape (lobes) ──┬──▶ SkinSpec ──▶ Skin (nubbly mesh, AO baked)
//!                                                    │
//!                     Foliage component ◀────────────┘
//!                     │  Rooted { pose }  ── no body: nothing collides with it
//!                     │  Loose { body }   ── LeafCore: a sphere inside each lobe
//!                     │
//!   PhysicsSyncSystem ─┼─▶ FoliageDrag (SubstepForceProvider): leaves drag
//!                     │    what overlaps them; the air drags loose clumps
//!   CharacterControl ──┼─▶ leaf_depth: a wading character's stride shortens
//!   FoliageSkinSystem ─┼─▶ Press + Squash ──deform──▶ ProceduralMesh (moving)
//!                     │                               or ModelInstance (still)
//!   FoliageUprootSystem┘   a blast near a rooted clump gives it a core
//! ```
//!
//! What is real and what is drawn: the core's spheres are real bodies, the
//! leaves' drag on whatever passes through them is a real force, and so is
//! the air's on a loose clump. The leaves themselves never collide; where
//! they would pass into the ground they are pressed onto it, and a landing
//! squashes them for a moment, in drawing only. `docs/FOLIAGE.md` has the
//! design, its numbers and what is open.

mod components;
mod deform;
mod drag;
mod leaf_core;
mod shape;
mod skin;
mod systems;
mod thicket;

#[cfg(all(test, feature = "foliage_scenarios"))]
mod scenarios;

pub use components::{Foliage, Rooting};
pub use deform::{deform, Bending, ContactPlane, Give, Parting, Press, Pusher, Squash};
pub use drag::{DragVolume, FoliageDrag, LeafDrag, VolumeFrame};
pub use leaf_core::LeafCore;
pub use shape::{FoliageShape, Lobe, ShrubSpec};
pub use skin::{LeafPalette, Skin, SkinSpec};
pub use systems::{leaf_emitter, still_model, FoliageSkinSystem, FoliageUprootSystem};
pub use thicket::leaf_depth;
