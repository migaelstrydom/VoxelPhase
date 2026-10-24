//! The properties every plan keeps, whatever its scene, checked from the
//! outside: by the fixture tests, the fuzzer, and later the web tool.

mod plan_rules;
mod replanning;
mod violation;

pub use plan_rules::check_plan;
pub use replanning::check_replan;
pub use violation::{Rule, Violation};
