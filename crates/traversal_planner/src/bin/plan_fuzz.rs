//! Fuzz the traversal planner: random authored scenes (`fuzz::generate`)
//! planned and checked against every property (`properties`). A scene that
//! breaks one is shrunk to the fewest shapes that still break it, and with
//! `--write` saved as a fixture to fix and keep.
//!
//! ```text
//! cargo run --release -p traversal_planner --bin plan_fuzz -- --seeds 2000
//! cargo run --release -p traversal_planner --bin plan_fuzz -- --from 500 --seeds 100 --write fixtures
//! ```
//!
//! Prints how many scenes crossed and refused, plan times, and each failing
//! seed with what it broke; exits non-zero if any did.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::process::ExitCode;
use std::{env, fs};

use traversal_planner::fuzz::{generate, shrink, try_scene};
use traversal_planner::properties::Rule;
use traversal_planner::scene::Outcome;

const USAGE: &str = "usage: plan_fuzz [--seeds n] [--from seed] [--write <dir>]";

fn main() -> ExitCode {
    let (mut seeds, mut from, mut write) = (500u64, 0u64, None::<PathBuf>);
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        let value = args.next();
        let parsed = match (arg.as_str(), value) {
            ("--seeds", Some(value)) => value.parse().map(|n| seeds = n).is_ok(),
            ("--from", Some(value)) => value.parse().map(|n| from = n).is_ok(),
            ("--write", Some(value)) => {
                write = Some(PathBuf::from(value));
                true
            }
            _ => false,
        };
        if !parsed {
            eprintln!("{USAGE}");
            return ExitCode::from(2);
        }
    }

    let (mut crossed, mut refused) = (0, 0);
    let mut times = Vec::new();
    let mut failures: BTreeMap<Rule, Vec<u64>> = BTreeMap::new();
    for seed in from..from + seeds {
        let scene = generate(seed);
        let trial = try_scene(&scene);
        times.push(trial.seconds * 1.0e3);
        match trial.outcome {
            Outcome::Crosses => crossed += 1,
            Outcome::Refuses => refused += 1,
        }
        let Some(first) = trial.violations.first() else {
            continue;
        };
        println!("seed {seed}: {first}");
        failures.entry(first.rule).or_default().push(seed);
        if let Some(dir) = &write {
            let small = shrink(&scene, first.rule);
            let outcome = try_scene(&small).outcome;
            let text = small
                .to_ron()
                .map(|ron| {
                    format!(
                        "// Found by plan_fuzz, seed {seed}, shrunk: {first}\n{}",
                        ron.replace("expect: None", &format!("expect: Some({outcome:?})"))
                    )
                })
                .unwrap_or_else(|error| format!("// {error}"));
            let path = dir.join(format!("fuzz_{seed}.ron"));
            if let Err(error) = fs::write(&path, text) {
                eprintln!("{}: {error}", path.display());
            }
        }
    }

    times.sort_by(f64::total_cmp);
    let at = |fraction: f64| times[((times.len() - 1) as f64 * fraction) as usize];
    println!(
        "{seeds} scenes: {crossed} crossed, {refused} refused; plan ms median {:.2}, 99th {:.2}, max {:.2}",
        at(0.5),
        at(0.99),
        at(1.0)
    );
    for (rule, seeds) in &failures {
        println!("{rule:?}: {} scenes, first seed {}", seeds.len(), seeds[0]);
    }
    if failures.is_empty() {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}
