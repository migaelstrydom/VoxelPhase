//! A short fuzz sweep on every test run: random scenes planned and checked
//! against every property. `plan_fuzz` sweeps further.

use traversal_planner::fuzz::{generate, try_scene};

/// Seeds swept on every test run.
const SEEDS: u64 = 300;

#[test]
fn random_scenes_keep_every_property() {
    let broken: Vec<String> = (0..SEEDS)
        .flat_map(|seed| {
            try_scene(&generate(seed))
                .violations
                .into_iter()
                .map(move |violation| format!("seed {seed}: {violation}"))
        })
        .collect();
    assert!(broken.is_empty(), "{}", broken.join("\n"));
}
