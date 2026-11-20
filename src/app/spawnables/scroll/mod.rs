//! Scroll spawnable: a page of Euclid's *Elements*, a collectable a level
//! may scatter and never asks for.
//!
//! ```text
//!   Proposition ──figure──▶ Figure (strokes, named points), in a unit square
//!                                │
//!   ScrollDef ──model──▶ recto: the open sheet, papyrus with two columns of
//!                       │        script (texture.rs)
//!                       ├──▶ verso: the sheet's back, its edges, the two
//!                       │        rolls it is wound on, the spiral in
//!                       │        their ends, and the vermilion tag
//!                       │        (sillybos) hung from one
//!                       ├──▶ knobs: gilt, turned, on the rods' ends
//!                       └──▶ figure: the proposition's figure laid on the
//!                                recto in gold that shines, and its light
//!                                through the verso (mesh.rs)
//!   ScrollDef ──spawn──▶ Hover (no rigid body), Collectable, PointLight,
//!                        motes
//! ```
//!
//! A papyrus roll held open, as a reader holds one, rolls upright, on a
//! figure from the *Elements* drawn as Euclid drew it. Like the solids, it
//! belongs to reason, not to the garden (`docs/VISUAL_DIRECTION.md`), and
//! shares their gold. It hangs where it was authored and turns, and is
//! taken by walking into it; taking it moves nothing in the room, and no
//! level needs it (`docs/WORLD_AND_STORY.md`).
//!
//! `visual_bench scrolls` shows them all.

mod def;
mod figure;
mod mesh;
mod texture;

pub use def::ScrollDef;
pub use figure::Proposition;
