//! Blow up terrain and see what comes loose.
//!
//! Scripted scenarios on small synthetic terrain: a shelf undercut, an arch
//! cut at its feet, a floating island chipped, thirty grenades on one patch of
//! ground. Each reports what every blast cut loose and audits the whole
//! terrain after it: nothing may be left standing free that was not before.
//! It also times each blast's carve, remesh and plan, and a physics frame of
//! its boulders once they are all at rest: wall clock, so a release build.
//!
//! # Usage
//!
//! ```bash
//! cargo run --release --bin rubble_viewer -- --list
//! cargo run --release --bin rubble_viewer -- rim_cusps
//! cargo run --release --bin rubble_viewer -- all
//! ```
//!
//! No Vulkan, no window, no ECS. Exits non-zero if any scenario fails.

use std::process::ExitCode;

use voxel_phase::rubble_viewer::{catalogue, find, report, run, Scenario};

const USAGE: &str = "usage: rubble_viewer (--list | all | <scenario>)";

fn main() -> ExitCode {
    let Some(arg) = std::env::args().nth(1) else {
        eprintln!("{USAGE}");
        return ExitCode::FAILURE;
    };
    let scenarios: Vec<Scenario> = match arg.as_str() {
        "--list" => {
            for s in catalogue() {
                println!("{:<16} {}", s.name, s.description);
            }
            return ExitCode::SUCCESS;
        }
        "all" => catalogue(),
        name => match find(name) {
            Some(s) => vec![s],
            None => {
                eprintln!("no scenario '{name}'; --list shows them\n{USAGE}");
                return ExitCode::FAILURE;
            }
        },
    };

    let mut passed = true;
    for scenario in &scenarios {
        let run = match run(scenario) {
            Ok(run) => run,
            Err(e) => {
                eprintln!("{e}");
                passed = false;
                continue;
            }
        };
        let verdict = scenario.verdict(&run);
        passed &= verdict.is_ok();
        println!("{}", report(&run, &verdict));
    }
    if passed {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}
