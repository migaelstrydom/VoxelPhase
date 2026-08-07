pub mod framework;
pub mod geometry;
pub mod scenarios;

#[cfg(all(test, feature = "bench_harness"))]
mod tests;
