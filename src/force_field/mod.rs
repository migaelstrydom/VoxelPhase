//! Force fields: the visible field under a suspension, read from its readout every frame, so what is seen is what the physics is doing.
//!
//! ```text
//!   PhysicsWorld ──readout──▶ ForceFieldSystem ──▶ ForceFieldVisual (per body)
//!                                  │                    │ PodField per suspension
//!                                  │                    ▼
//!                                  │                 PodLook ──┬─▶ ColumnMesh ──▶ force_field shader
//!                                  │                           │    (blended, light)
//!                                  │                           └─▶ RingDraw ───▶ emissive rings
//!                                  ├─▶ KickUp ──▶ ParticlePool      (opaque, bloom)
//!                                  └─▶ Underglow ──▶ PointLight
//! ```

pub mod column;
pub mod kick_up;
pub mod pod;
pub mod ring;
pub mod style;
pub mod system;
pub mod visual;

pub use column::ColumnMesh;
pub use pod::{PodField, PodLook, PodSample};
pub use ring::{ring_draws, ring_model, RingDraw};
pub use style::{FieldStyle, Span};
pub use system::ForceFieldSystem;
pub use visual::{ForceFieldVisual, Underglow};
