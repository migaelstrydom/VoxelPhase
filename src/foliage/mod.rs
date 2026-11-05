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
//!                     │    what overlaps them (the world's air drags loose
//!                     │    clumps through their whole lobes)
//!   CharacterControl ──┼─▶ leaf_depth: a wading character's stride shortens
//!   FoliageLanding ────┼─▶ leaves shed where a loose clump lands hard
//!   FoliageRustle ─────┼─▶ leaves shed by whatever moves through a clump
//!   FoliageTrail ──────┼─▶ a trail of leaves behind a clump flying fast
//!   FoliageUprootSystem┘   a blast near a rooted clump gives it a core
//! ```
//!
//! What is real and what is drawn: the core's spheres are real bodies, the
//! leaves' drag on whatever passes through them is a real force, and so is
//! the air's on a loose clump. The leaves themselves never collide, and
//! their skin is drawn as it was grown: it does not bend round what passes
//! through it. `docs/FOLIAGE.md` has the design, its numbers and what is
//! open.

mod components;
mod drag;
mod leaf_core;
mod rustle;
mod shape;
mod skin;
mod systems;
mod thicket;
mod trail;

#[cfg(all(test, feature = "foliage_scenarios"))]
mod scenarios;

pub use components::{Foliage, Rooting};
pub use drag::{DragVolume, FoliageDrag, LeafDrag, VolumeFrame};
pub use leaf_core::LeafCore;
pub use rustle::{FoliageRustleSystem, Rustle};
pub use shape::{FoliageShape, Lobe, ShrubSpec};
pub use skin::{LeafPalette, Skin, SkinSpec};
pub use systems::{leaf_emitter, FoliageLandingSystem, FoliageUprootSystem};
pub use thicket::leaf_depth;
pub use trail::{FoliageTrailSystem, Trail};
