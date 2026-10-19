//! Trampolines: a bed sprung from a hoop on legs, drawn dipping and stretching its springs as it rides.
//!
//! ```text
//!   spawnable (app/spawnables/trampoline.rs)
//!        │ frame body ──SpringSlider── bed body      (physics)
//!        ▼
//!   Trampoline component ──► TrampolineBedSystem ──► ProceduralMesh ──► RenderSystem
//!   (bodies, shape)          reads the bed's sag      bed dish + springs
//! ```
//!
//! The spring does the bouncing; nothing here adds energy. The frame is a
//! static model, the bed and springs are rebuilt every frame from where the
//! bed's body is.

mod bed_mesh;
mod shape;
mod system;

pub use bed_mesh::{bed_mesh, springs_mesh, STEEL};
pub use shape::TrampolineShape;
pub use system::{Trampoline, TrampolineBedSystem};
