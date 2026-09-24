//! Hydrology spikes (design §21, stage 0.5).
//!
//! ```text
//! cargo run --release --bin water_spike -- spans                 # all five water levels
//! cargo run --release --bin water_spike -- spans levels/skyway.level.ron
//! cargo run --release --bin water_spike -- routing               # spike 0.5b
//! ```

use std::path::PathBuf;
use std::process::ExitCode;

use nalgebra::Point3;
use voxel_phase::water_spike::{
    check_level, hierarchy, hierarchy_report, report, route_level, route_report, route_staircase,
};

const USAGE: &str = "usage: water_spike spans [<level.ron>...]\n       water_spike routing";

const WATER_LEVELS: [&str; 5] = [
    "levels/test_arena.level.ron",
    "levels/skyway.level.ron",
    "levels/subsidence.level.ron",
    "levels/thin_ice.level.ron",
    "levels/wrecking_yard.level.ron",
];

fn main() -> ExitCode {
    env_logger::init();
    let mut args = std::env::args().skip(1);
    let Some(command) = args.next() else {
        eprintln!("{USAGE}");
        return ExitCode::FAILURE;
    };
    let mut levels: Vec<PathBuf> = args.map(PathBuf::from).collect();
    if levels.is_empty() {
        levels = WATER_LEVELS.iter().map(PathBuf::from).collect();
    }
    match command.as_str() {
        "spans" => {
            for level in &levels {
                match check_level(level) {
                    Ok(findings) => println!("{}", report(&findings)),
                    Err(message) => {
                        eprintln!("error: {message}");
                        return ExitCode::FAILURE;
                    }
                }
            }
            ExitCode::SUCCESS
        }
        "routing" => {
            // Skyway's bed is level along its length, so which end a spring
            // runs to depends on where it starts. Route from every 10 m and
            // report the longest.
            let skyway = std::path::Path::new("levels/skyway.level.ron");
            let longest = (1..10)
                .map(|i| {
                    let x = i as f32 * 10.0;
                    route_level(
                        skyway,
                        Point3::new(x, 3.0, 32.0),
                        &format!("skyway river bed, spring at ({x}, 32)"),
                    )
                })
                .collect::<Result<Vec<_>, _>>()
                .map(|routes| {
                    routes
                        .into_iter()
                        .max_by(|a, b| a.length.total_cmp(&b.length))
                        .expect("nine routes")
                });
            let routes = [longest, route_staircase()];
            for route in routes {
                match route {
                    Ok(findings) => println!("{}", route_report(&findings)),
                    Err(message) => {
                        eprintln!("error: {message}");
                        return ExitCode::FAILURE;
                    }
                }
            }
            for level in &levels {
                match hierarchy(level) {
                    Ok(pools) => {
                        for pool in pools {
                            println!("{}", hierarchy_report(&pool));
                        }
                    }
                    Err(message) => {
                        eprintln!("error: {message}");
                        return ExitCode::FAILURE;
                    }
                }
            }
            ExitCode::SUCCESS
        }
        _ => {
            eprintln!("{USAGE}");
            ExitCode::FAILURE
        }
    }
}
