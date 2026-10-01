//! Knock structures down in ways nobody wrote a test for, and check the
//! engine never breaks a rule physics keeps.
//!
//! Each seed builds one of the bench harness's structures with proportions of
//! its own, and disturbs it: the player's body running at it and perhaps
//! jumping in, a ball thrown at it, blocks knocked. Then it checks that the
//! structure's bodies never gain energy nothing gave them, never pass under
//! the floor, and never go non-finite. A seed replays exactly.
//!
//! ```text
//! cargo run --release --bin physics_fuzz -- --seeds 200
//! cargo run --release --bin physics_fuzz -- --seeds 100 --kind jenga
//! cargo run --release --bin physics_fuzz -- --seed 17          # one case, described
//! cargo run --release --bin physics_fuzz -- --seed 17 --trace  # and every frame of it
//! cargo run --release --bin physics_fuzz -- --seeds 200 --no-sleep  # what waking causes
//! ```

use std::panic::{self, AssertUnwindSafe};
use std::process::ExitCode;

use rayon::prelude::*;

use voxel_phase::physics_fuzz::{run, Case, RunOptions, StructureKind, Verdict, ENERGY_GAIN_LIMIT};

const USAGE: &str =
    "usage: physics_fuzz [--seeds N] [--start S] [--kind jenga|arch|temple|wall|pile]
                    [--seed S [--trace]] [--no-sleep]";

struct Options {
    /// First seed of the sweep.
    start: u64,
    /// Seeds in the sweep.
    seeds: u64,
    /// Build every case around this kind of structure.
    kind: Option<StructureKind>,
    /// Run only this seed, and describe it whatever its verdict.
    single: Option<u64>,
    run: RunOptions,
}

/// How one seed went: its verdict, or the panic it raised.
struct Outcome {
    seed: u64,
    description: String,
    result: Result<Verdict, String>,
}

fn main() -> ExitCode {
    let options = match parse_args() {
        Ok(options) => options,
        Err(message) => {
            eprintln!("{message}\n{USAGE}");
            return ExitCode::from(2);
        }
    };
    // Panics are caught and reported per seed; the default hook would print
    // each one again.
    panic::set_hook(Box::new(|_| {}));

    let seeds: Vec<u64> = match options.single {
        Some(seed) => vec![seed],
        None => (options.start..options.start + options.seeds).collect(),
    };
    let mut outcomes: Vec<Outcome> = seeds
        .par_iter()
        .map(|&seed| {
            let case = Case::generate(seed, options.kind);
            let description = case.to_string();
            let result = panic::catch_unwind(AssertUnwindSafe(|| run(&case, options.run)))
                .map_err(|payload| panic_message(&payload));
            Outcome {
                seed,
                description,
                result,
            }
        })
        .collect();
    outcomes.sort_by_key(|o| o.seed);

    let mut failures = 0;
    let mut gains = Vec::new();
    for outcome in &outcomes {
        let findings: Vec<String> = match &outcome.result {
            Ok(verdict) => {
                gains.push(verdict.worst_gain);
                verdict.violations.iter().map(|v| v.to_string()).collect()
            }
            Err(message) => vec![format!("panicked: {message}")],
        };
        if findings.is_empty() && options.single.is_none() {
            continue;
        }
        failures += usize::from(!findings.is_empty());
        println!("seed {}: {}", outcome.seed, outcome.description);
        for finding in &findings {
            println!("  {finding}");
        }
        if let (Some(_), Ok(verdict)) = (options.single, &outcome.result) {
            println!("  worst unexplained gain {:.3} J/kg", verdict.worst_gain);
            for frame in &verdict.trace {
                println!("  {frame}");
            }
        }
    }

    gains.sort_by(f32::total_cmp);
    let at = |q: f32| gains.get(((gains.len() as f32 - 1.0) * q).round() as usize);
    if let (Some(median), Some(p99), Some(max)) = (at(0.5), at(0.99), gains.last()) {
        println!(
            "unexplained gain, J/kg: median {median:.3}, 99th percentile {p99:.3}, \
             worst {max:.3} (limit {ENERGY_GAIN_LIMIT})"
        );
    }
    println!("{failures} of {} seeds found something", outcomes.len());
    if failures > 0 {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}

fn panic_message(payload: &Box<dyn std::any::Any + Send>) -> String {
    payload
        .downcast_ref::<String>()
        .cloned()
        .or_else(|| payload.downcast_ref::<&str>().map(|s| s.to_string()))
        .unwrap_or_else(|| "(no message)".into())
}

fn parse_args() -> Result<Options, String> {
    let mut options = Options {
        start: 0,
        seeds: 100,
        kind: None,
        single: None,
        run: RunOptions::default(),
    };
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = || args.next().ok_or(format!("{arg} needs a value"));
        let number = |text: String| text.parse::<u64>().map_err(|e| format!("{text}: {e}"));
        match arg.as_str() {
            "--seeds" => options.seeds = number(value()?)?,
            "--start" => options.start = number(value()?)?,
            "--seed" => options.single = Some(number(value()?)?),
            "--kind" => {
                let name = value()?;
                options.kind =
                    Some(StructureKind::parse(&name).ok_or(format!("no structure called {name}"))?);
            }
            "--trace" => options.run.trace = true,
            "--no-sleep" => options.run.sleep = false,
            other => return Err(format!("unknown argument {other}")),
        }
    }
    Ok(options)
}
