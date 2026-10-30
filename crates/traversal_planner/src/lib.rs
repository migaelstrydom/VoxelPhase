//! The traversal planner: footfalls and flights planned over a strip of ground
//! ahead, for a body that tracks the plan through its contacts.
//!
//! Pure: no physics, ECS or GPU. The game samples its world into a [`Section`]
//! and acts on the [`Plan`]; the tools build sections from authored
//! [`TraversalScene`]s. Design: `docs/traversal/PLANNER_DESIGN.md`.
//!
//! ```text
//!   TraversalScene ──► ShapeCaster ───┐
//!                                     ├─ sample_section ─► Section ── StripQuery ──► Planner ──► Plan
//!   PhysicsWorld ──► PhysicsCaster ───┘    (SphereCaster)              (SectionQuery)   ▲
//!                    (game side)                                         GaitParams ────┘
//! ```

pub mod fuzz;
pub mod params;
pub mod path;
pub mod plan;
pub mod planner;
pub mod properties;
pub mod scene;
pub mod section;
pub mod svg;

pub use params::GaitParams;
pub use path::PathVec;
pub use plan::{BodyState, Contact, FootfallId, Limb, Motion, Phase, Plan};
pub use planner::{PlanRequest, Planned, Planner, PlannerSettings, Refusal};
pub use scene::TraversalScene;
pub use section::{Section, SectionQuery};
