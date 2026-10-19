pub mod framework;
pub mod geometry;
pub mod scenarios;

#[cfg(all(test, feature = "physics_scenarios"))]
mod tests;
