//! Shock propagation: lower bodies made heavier while the solver works down
//! a stack, so its load reaches the ground in one pass.
//!
//! ```text
//!   manifolds ──► ContactGraph ──── depths from the ground, top-down order,
//!                      │            lower body scaled by α^Δdepth
//!                      ▼
//!                 StaticsGate ───── unscales pairs whose lower body is not in
//!                      │            static equilibrium on what holds it up
//!                      ▼
//!               WarmStartRemap ──── warm starts solved under last frame's
//!                                   scales, carried over to this frame's
//! ```
//!
//! Scaled impulses move bodies exactly as physical impulses would that carry
//! each withheld share of a push down through the supports to the ground, so
//! scaling is energy-free wherever those impulses are a contact solution.
//! The gate keeps it there and the remap keeps a change of scale from popping
//! the stack. The derivation and the measurements behind both are in
//! `docs/SHOCK_PROPAGATION_STATICS.md`.

mod conditioner;
mod contact_graph;
mod statics_gate;
mod warm_start_remap;
mod wrench_cone;

pub use conditioner::{ShockPropagationConditioner, ShockPropagationConfig};
