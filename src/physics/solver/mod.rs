pub(crate) mod body_pair;
pub(crate) mod ccd;
pub mod conditioning;
mod contact_solver;
pub(crate) mod diagnostics;
pub(crate) mod friction;
pub(crate) mod impulse;
pub(crate) mod normal;
pub(crate) mod pgs_ngs;
pub(crate) mod position_correction;
pub mod shock_propagation;
pub(crate) mod warm_start;

pub use conditioning::{IdentityConditioner, ManifoldConditioner, ManifoldConditions};
pub use contact_solver::ContactSolver;
pub use pgs_ngs::{PgsNgsConfig, PgsNgsSolver};
pub use shock_propagation::{ShockPropagationConditioner, ShockPropagationConfig};
