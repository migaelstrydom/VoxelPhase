//! Blow a level's water about at random and check it holds together.
//!
//! Each seed loads the level, then sets off grenades at random points in and
//! under its water, with the frame after each blast a long one. After every
//! frame it checks:
//!
//! - nothing panicked, reading the water at random points as floating bodies
//!   do;
//! - no cell's depth is negative and every depth and discharge is finite;
//! - the ledger balances;
//! - no water runs faster than anything a level could make;
//! - no cell under a roof holds more than it has room for, and no fall's
//!   water comes down above where it left;
//! - nothing is drawn where there is no water: the surface and the falls'
//!   curtains, meshed as the game meshes them, are finite and stand no
//!   higher than the water or the lip they are drawn for.
//!
//! Then it leaves the water alone for a while and, on a level nothing
//! feeds, checks it has come to rest, or is still running out of the world
//! or down to somewhere lower.
//!
//! ```text
//! cargo run --release --bin water_fuzz -- levels/test_arena.level.ron --seeds 20 --blasts 12
//! ```

use std::panic::{self, AssertUnwindSafe};
use std::path::PathBuf;
use std::sync::Mutex;

use nalgebra::Point3;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use voxel_phase::level::load_level;
use voxel_phase::level_check::build_terrain;
use voxel_phase::rendering::water::{indexed_within, worst_stray, RiverParts};
use voxel_phase::terrain::{BlastConfig, TerrainWorld};
use voxel_phase::water::field::WET_DEPTH;
use voxel_phase::water::WaterWorld;

/// Where the last panic came from, for the seed's report.
static PANICKED_AT: Mutex<Option<String>> = Mutex::new(None);

/// Frames run after each blast, at 60 Hz.
const FRAMES_BETWEEN: usize = 45;

/// How long a blast's frame lasts, s: the frame clock's cap.
const BLAST_DT: f32 = 0.1;

/// Deepest below zero a cell may stand, m: rounding, as the field's own
/// tests allow.
const NEGATIVE_DEPTH: f64 = -1.0e-9;

/// Most a ledger may be out by, as a share of the water held.
const LEDGER_TOLERANCE: f64 = 1.0e-6;

/// Fastest water any level could make, m/s: a fall of some twenty metres.
const MAX_SPEED: f32 = 20.0;

/// Furthest a cell's water may stand over its roof, m, and a fall's water
/// come down above its lip, m. Either reads as water where none can be: a
/// level far over a slab, a landing on the ground above a hole.
const OVER_ROOF: f64 = 0.05;
const LANDED_ABOVE: f32 = 0.05;

/// How long the water is left alone after the last blast, s, and the
/// fastest it may still run then on a level nothing feeds, m/s.
const REST: f32 = 120.0;
const REST_SPEED: f32 = 0.05;

/// Water still running then may be a lake still emptying, which is no
/// fault: over the last `WATCHED` seconds, s, it has let at least
/// `DRAINING` m³ out of the world, through a hole a blast opened to the
/// void or the sea, or has come to stand at least `SINKING` m⁴ lower in
/// all, as water running from a higher pond to a lower does. Water going
/// round and round does neither.
const WATCHED: f32 = 10.0;
const DRAINING: f64 = 1.0e-2;
const SINKING: f64 = 1.0;

/// Random points read each frame, as floating bodies would.
const READS: usize = 16;

struct Options {
    level: PathBuf,
    seeds: u64,
    blasts: usize,
}

fn main() {
    let options = match parse_args() {
        Ok(o) => o,
        Err(message) => {
            eprintln!("{message}");
            eprintln!("usage: water_fuzz <level.ron> [--seeds N] [--blasts N]");
            std::process::exit(2);
        }
    };
    // Panics are caught and reported per seed, with where they came from;
    // the default hook would print each one again.
    env_logger::init();
    panic::set_hook(Box::new(|info| {
        let at = info
            .location()
            .map(|l| format!("{}:{}", l.file(), l.line()));
        *PANICKED_AT.lock().unwrap_or_else(|e| e.into_inner()) = at;
    }));
    let mut failures = 0;
    for seed in 0..options.seeds {
        let findings = match panic::catch_unwind(AssertUnwindSafe(|| run_seed(&options, seed))) {
            Ok(Ok(findings)) => findings,
            Ok(Err(e)) => {
                eprintln!("{e}");
                std::process::exit(1);
            }
            Err(payload) => {
                let at = PANICKED_AT.lock().unwrap_or_else(|e| e.into_inner()).take();
                vec![format!(
                    "panicked at {}: {}",
                    at.as_deref().unwrap_or("an unknown place"),
                    panic_message(&payload)
                )]
            }
        };
        if findings.is_empty() {
            println!("seed {seed}: ok");
        } else {
            failures += 1;
            println!("seed {seed}:");
            for f in findings {
                println!("  {f}");
            }
        }
    }
    println!("{failures} of {} seeds found something", options.seeds);
    if failures > 0 {
        std::process::exit(1);
    }
}

fn run_seed(options: &Options, seed: u64) -> Result<Vec<String>, String> {
    let level = load_level(&options.level).map_err(|e| e.to_string())?;
    let config = level
        .water
        .clone()
        .ok_or_else(|| format!("{} has no water", options.level.display()))?;
    let mut terrain = build_terrain(&level);
    let mut water = WaterWorld::from_config(&config, &terrain);
    let mut rng = StdRng::seed_from_u64(seed);
    let mut findings = Vec::new();
    let mut time = 0.0;
    let mut rivers = RiverParts::default();

    let mut frame = |terrain: &mut TerrainWorld,
                     water: &mut WaterWorld,
                     rng: &mut StdRng,
                     dt: f32,
                     findings: &mut Vec<String>| {
        terrain.update();
        water.on_terrain_update(terrain);
        water.step(dt);
        time += dt;
        check(water, rng, time, findings);
        rivers.sync(&*water);
        if !indexed_within(rivers.mesh()) {
            findings.push(format!(
                "t {time:.2}: the drawn mesh has a triangle of vertices it has not got"
            ));
        }
        // A surface waiting its turn to be built again is drawn as the
        // water stood, which is no fault of how it is drawn.
        if rivers.lag().0 == 0 {
            if let Some(stray) = worst_stray(rivers.mesh(), water) {
                findings.push(format!(
                    "t {time:.2}: a {:?} vertex at ({:.1}, {:.1}, {:.1}) is drawn {:.2} m over \
                     its water",
                    stray.drawn, stray.at.x, stray.at.y, stray.at.z, stray.over
                ));
            }
        }
    };

    for blast in 0..options.blasts {
        let Some(at) = blast_site(&water, &mut rng) else {
            break;
        };
        terrain.detonate(at, &BlastConfig::default());
        let before = findings.len();
        frame(&mut terrain, &mut water, &mut rng, BLAST_DT, &mut findings);
        for _ in 0..FRAMES_BETWEEN {
            frame(
                &mut terrain,
                &mut water,
                &mut rng,
                1.0 / 60.0,
                &mut findings,
            );
        }
        if findings.len() > before {
            findings.insert(
                before,
                format!(
                    "after blast {blast} at ({:.1}, {:.1}, {:.1}):",
                    at.x, at.y, at.z
                ),
            );
            return Ok(findings);
        }
    }

    let frames = (REST * 60.0) as usize;
    let mut before = (0.0, 0.0);
    for left in (0..frames).rev() {
        frame(
            &mut terrain,
            &mut water,
            &mut rng,
            1.0 / 60.0,
            &mut findings,
        );
        if left == (WATCHED * 60.0) as usize {
            before = (water.field().ledger().sunk, water.field().potential());
        }
    }
    let field = water.field();
    let draining = (field.ledger().sunk - before.0).abs() > DRAINING;
    let sinking = before.1 - field.potential() > SINKING;
    let speed = field.max_speed();
    if field.sources().is_empty() && !draining && !sinking && speed > REST_SPEED {
        findings.push(format!(
            "still running at {speed:.2} m/s after {REST:.0} s left alone, and getting no lower"
        ));
    }
    Ok(findings)
}

/// One frame's checks.
fn check(water: &WaterWorld, rng: &mut StdRng, time: f32, findings: &mut Vec<String>) {
    let field = water.field();
    let (shallowest, finite) = field.health();
    if !finite {
        findings.push(format!("t {time:.2}: a depth or discharge is not finite"));
    }
    if shallowest < NEGATIVE_DEPTH {
        findings.push(format!("t {time:.2}: a cell holds {shallowest:.3e} m"));
    }
    let volume = water.volume();
    let error = field.ledger().error(volume);
    if error.abs() > LEDGER_TOLERANCE * volume.max(1.0) {
        findings.push(format!(
            "t {time:.2}: the ledger is out by {error:+.3e} m³ of {volume:.1}"
        ));
    }
    let speed = field.max_speed();
    if speed > MAX_SPEED {
        findings.push(format!("t {time:.2}: water runs at {speed:.1} m/s"));
    }
    let over = field.over_roof();
    if over > OVER_ROOF {
        findings.push(format!(
            "t {time:.2}: a cell stands {over:.2} m over its roof"
        ));
    }
    let rise = field.highest_landing();
    if rise > LANDED_ABOVE {
        findings.push(format!(
            "t {time:.2}: a fall lands {rise:.2} m above its lip"
        ));
    }
    let (min, max) = field.graph().column_bounds();
    let query = water.query();
    for _ in 0..READS {
        let at = Point3::new(
            rng.gen_range(min.min_corner().0..max.min_corner().0 + 0.5),
            rng.gen_range(-10.0..20.0),
            rng.gen_range(min.min_corner().1..max.min_corner().1 + 0.5),
        );
        if let Some(sample) = query.sample(at) {
            if !(sample.surface.is_finite() && sample.velocity.iter().all(|v| v.is_finite())) {
                findings.push(format!(
                    "t {time:.2}: the water at ({:.1}, {:.1}, {:.1}) reads {sample:?}",
                    at.x, at.y, at.z
                ));
            }
        }
    }
}

/// A random wet cell's floor, or a little below it: a blast in the bed or
/// the bank under the water.
fn blast_site(water: &WaterWorld, rng: &mut StdRng) -> Option<Point3<f32>> {
    let field = water.field();
    let cells = field.cells();
    let wet: Vec<(usize, usize)> = cells
        .chunks
        .iter()
        .enumerate()
        .flat_map(|(slot, chunk)| {
            (0..chunk.depth.len())
                .filter(move |&flat| chunk.depth[flat] > WET_DEPTH)
                .map(move |flat| (slot, flat))
        })
        .collect();
    if wet.is_empty() {
        return None;
    }
    let (slot, flat) = wet[rng.gen_range(0..wet.len())];
    let chunk = &cells.chunks[slot];
    let (local, _) = chunk.column_of(flat);
    let (x, z) = chunk.coord.column(local).centre();
    let drop = if rng.gen_bool(0.5) {
        rng.gen_range(0.0..3.0)
    } else {
        0.0
    };
    Some(Point3::new(x, chunk.bed[flat] - drop, z))
}

fn panic_message(payload: &Box<dyn std::any::Any + Send>) -> String {
    payload
        .downcast_ref::<String>()
        .cloned()
        .or_else(|| payload.downcast_ref::<&str>().map(|s| s.to_string()))
        .unwrap_or_else(|| "(no message)".into())
}

fn parse_args() -> Result<Options, String> {
    let mut level = None;
    let mut seeds = 10;
    let mut blasts = 10;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut number = |flag: &str| -> Result<u64, String> {
            args.next()
                .and_then(|v| v.parse().ok())
                .ok_or(format!("{flag} expects a number"))
        };
        match arg.as_str() {
            "--seeds" => seeds = number("--seeds")?,
            "--blasts" => blasts = number("--blasts")? as usize,
            other if other.starts_with('-') => return Err(format!("unknown option '{other}'")),
            other => level = Some(PathBuf::from(other)),
        }
    }
    Ok(Options {
        level: level.ok_or("no level file given")?,
        seeds,
        blasts,
    })
}
