//! Blow a level's water about at random and check it holds together.
//!
//! Each seed loads the level, then sets off grenades at random points in and
//! under its basins, with the frame after each blast a long one, and splashes
//! the water between blasts. After every frame it checks:
//!
//! - nothing panicked, reading the current as floating bodies do;
//! - the rasteriser saw no terrain edit raise a floor;
//! - the ledger balances;
//! - every basin holding water has a surface to draw;
//! - no basin lost most of its water in one frame without the ledger saying
//!   where it went.
//!
//! ```text
//! cargo run --release --bin water_fuzz -- levels/test_arena.level.ron --seeds 20 --blasts 12
//! # Every store, link and topology edit, frame by frame, through one blast.
//! cargo run --release --bin water_fuzz -- levels/test_arena.level.ron --trace 17:3
//! ```

use std::panic::{self, AssertUnwindSafe};
use std::path::PathBuf;

use nalgebra::Point3;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use voxel_phase::level::load_level;
use voxel_phase::level_check::build_terrain;
use voxel_phase::rendering::water::WaterScene;
use voxel_phase::terrain::{BlastConfig, TerrainWorld};
use voxel_phase::water::geometry::{Column, Span, COLUMN_SIZE};
use voxel_phase::water::ids::StoreId;
use voxel_phase::water::network::Store;
use voxel_phase::water::{Disturbance, WaterWorld};

/// Frames run after each blast, at 60 Hz.
const FRAMES_BETWEEN: usize = 45;

/// How long a blast's frame lasts, s: the frame clock's cap.
const BLAST_DT: f32 = 0.1;

/// A basin that loses more than this share of its water in one frame, with
/// more than [`SUDDEN_LOSS_FLOOR`] m³ at stake, is reported.
const SUDDEN_LOSS: f64 = 0.5;
const SUDDEN_LOSS_FLOOR: f64 = 1.0;

struct Options {
    level: PathBuf,
    seeds: u64,
    blasts: usize,
    /// Print the network through this (seed, blast).
    trace: Option<(u64, usize)>,
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
    // Panics are caught and reported per seed; the default hook would print
    // each one again.
    env_logger::init();
    panic::set_hook(Box::new(|_| {}));
    let mut failures = 0;
    let seeds = match options.trace {
        Some((seed, _)) => seed..seed + 1,
        None => 0..options.seeds,
    };
    for seed in seeds {
        let findings = match panic::catch_unwind(AssertUnwindSafe(|| run_seed(&options, seed))) {
            Ok(Ok(findings)) => findings,
            Ok(Err(e)) => {
                eprintln!("{e}");
                std::process::exit(1);
            }
            Err(payload) => vec![format!("panicked: {}", panic_message(&payload))],
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
    let mut terrain = build_terrain(&level);
    let config = level.water.as_ref().ok_or("the level has no water")?;
    let (mut water, _) = if options.trace.is_some() {
        WaterWorld::recording(config, &terrain)
    } else {
        WaterWorld::from_config(config, &terrain)
    };
    let mut rng = StdRng::seed_from_u64(seed);
    let mut findings = Vec::new();
    let mut violations = water.geometry().stats().invariant_violations;

    for blast in 0..options.blasts {
        let Some(at) = blast_site(&water, &mut rng) else {
            findings.push(format!("blast {blast}: no basin left to blast"));
            break;
        };
        let tracing = options.trace == Some((seed, blast));
        let profiles: Vec<(Column, String, Vec<Span>)> = if tracing {
            columns_near(at)
                .map(|c| {
                    (
                        c,
                        profile(&terrain, c, at.y),
                        water.geometry().graph().spans(c).to_vec(),
                    )
                })
                .collect()
        } else {
            Vec::new()
        };
        terrain.detonate(at, &BlastConfig::default());
        if tracing {
            println!("blast {blast} at ({:.1}, {:.1}, {:.1})", at.x, at.y, at.z);
        }
        for frame in 0..=FRAMES_BETWEEN {
            let logged = water.topology_log().len();
            let before = basin_volumes(&water);
            let discarded = water.ledger().discarded;
            if frame == 0 {
                terrain.update();
                water.on_terrain_update(&terrain);
            } else if rng.gen_bool(0.3) {
                if let Some(p) = splash_site(&water, &mut rng) {
                    water.disturb(p, 0.3, Disturbance::Velocity(-rng.gen_range(1.0..4.0)));
                }
            }
            water.step(if frame == 0 { BLAST_DT } else { 1.0 / 60.0 });
            // Floating bodies read the current wherever they are.
            for _ in 0..4 {
                if let Some(p) = splash_site(&water, &mut rng) {
                    if let Some(sample) = water.query().sample(p) {
                        water.current_at(sample.body, p);
                    }
                }
            }

            if tracing {
                trace(&water, frame, logged);
            }
            let tag = format!(
                "blast {blast} at ({:.1}, {:.1}, {:.1}), frame {frame}",
                at.x, at.y, at.z
            );
            let now = water.geometry().stats().invariant_violations;
            if now > violations {
                let detail = water
                    .geometry()
                    .stats()
                    .last_violation
                    .map(|(column, old, new)| {
                        let (x, z) = column.centre();
                        // Is the ground really there now, or was the column
                        // misread?
                        let solid: String = (0..8)
                            .map(|i| old + (new - old) * (i as f32 + 0.5) / 8.0)
                            .map(|y| if terrain.is_solid_at(x, y, z) { '#' } else { '.' })
                            .collect();
                        format!(
                            "; last at ({x:.2}, {z:.2}): {old:.2} -> {new:.2}, terrain between [{solid}]"
                        )
                    })
                    .unwrap_or_default();
                findings.push(format!(
                    "{tag}: {} floor(s) raised{detail}",
                    now - violations
                ));
                if let Some((column, _, _)) = water.geometry().stats().last_violation {
                    if let Some((_, old_profile, old_spans)) =
                        profiles.iter().find(|(c, _, _)| *c == column)
                    {
                        println!(
                            "  column {column:?}, voxel {}, solid from y {:.2} up in 0.25 m steps",
                            terrain.voxel_size_at(at),
                            at.y - PROFILE_REACH
                        );
                        println!("  before [{old_profile}]");
                        println!("         {old_spans:?}");
                        println!("  after  [{}]", profile(&terrain, column, at.y));
                        println!("         {:?}", water.geometry().graph().spans(column));
                    }
                }
                violations = now;
            }
            let balance = water.balance();
            if (balance.expected - balance.held).abs() > 1e-3 * balance.expected.max(1.0) {
                findings.push(format!(
                    "{tag}: ledger expects {:.3} m³, holds {:.3}",
                    balance.expected, balance.held
                ));
            }
            findings.extend(undrawn(&water).into_iter().map(|b| format!("{tag}: {b}")));
            let lost_to_ledger = water.ledger().discarded - discarded;
            for (id, volume) in before {
                let after = water.network().store(id).map(|s| s.volume()).unwrap_or(0.0);
                if volume > SUDDEN_LOSS_FLOOR
                    && after < volume * (1.0 - SUDDEN_LOSS)
                    && water.network().store(id).is_some()
                    && lost_to_ledger < (volume - after) * 0.5
                {
                    findings.push(format!(
                        "{tag}: basin {} fell from {volume:.2} to {after:.2} m³",
                        id.0
                    ));
                }
            }
        }
    }
    Ok(findings)
}

/// How far above and below a blast the column profiles reach, m.
const PROFILE_REACH: f32 = 8.0;

/// Columns within reach of a blast.
fn columns_near(at: Point3<f32>) -> impl Iterator<Item = Column> {
    let centre = Column::containing(at.x, at.z);
    let r = (PROFILE_REACH / COLUMN_SIZE) as i32;
    (-r..=r).flat_map(move |dk| (-r..=r).map(move |di| centre.offset(di, dk)))
}

/// A column's solid (`#`) and air (`.`) from `PROFILE_REACH` below `y` to as
/// far above, in 0.25 m steps.
fn profile(terrain: &TerrainWorld, column: Column, y: f32) -> String {
    let (x, z) = column.centre();
    let steps = (2.0 * PROFILE_REACH / 0.25) as usize;
    (0..steps)
        .map(|i| y - PROFILE_REACH + (i as f32 + 0.5) * 0.25)
        .map(|h| {
            if terrain.is_solid_at(x, h, z) {
                '#'
            } else {
                '.'
            }
        })
        .collect()
}

/// A point on the floor of a random basin, dropped up to 3 m to reach
/// whatever lies under it.
fn blast_site(water: &WaterWorld, rng: &mut StdRng) -> Option<Point3<f32>> {
    let basins: Vec<_> = water
        .basins()
        .filter(|(_, b)| !b.region.is_empty())
        .collect();
    if basins.is_empty() {
        return None;
    }
    let (_, basin) = basins[rng.gen_range(0..basins.len())];
    let r = &basin.region[rng.gen_range(0..basin.region.len())];
    let (x, z) = r.span.column.centre();
    let drop = if rng.gen_bool(0.5) {
        rng.gen_range(0.0..3.0)
    } else {
        0.0
    };
    Some(Point3::new(x, r.shape.floor_min - drop, z))
}

/// A point just under the surface of a random basin that holds water.
fn splash_site(water: &WaterWorld, rng: &mut StdRng) -> Option<Point3<f32>> {
    let wet: Vec<_> = water
        .basins()
        .filter(|(_, b)| b.volume > 0.0)
        .flat_map(|(_, b)| {
            let level = b.level();
            b.region
                .iter()
                .filter(move |r| r.shape.floor_min < level - 0.05)
                .map(move |r| (r.span.column, level))
        })
        .collect();
    if wet.is_empty() {
        return None;
    }
    let (column, level) = wet[rng.gen_range(0..wet.len())];
    let (x, z) = column.centre();
    Some(Point3::new(x, level - 0.02, z))
}

/// One frame of the network: its new topology edits, every store and every
/// open link.
fn trace(water: &WaterWorld, frame: usize, logged: usize) {
    println!("frame {frame}");
    for edit in water.topology_log().iter().skip(logged) {
        println!("  edit {edit:?}");
    }
    for (id, store) in water.network().stores() {
        let kind = match store {
            Store::Basin(b) => format!(
                "basin ({} spans, cap {:.2}, outflows {:?})",
                b.region.len(),
                b.cap,
                b.outflows
                    .iter()
                    .map(|o| (o.lip, o.target.map(|t| t.0)))
                    .collect::<Vec<_>>()
            ),
            Store::Reach(_) => "reach".into(),
            Store::Ocean(_) => "ocean".into(),
            Store::Sink => "sink".into(),
            _ => "source".into(),
        };
        println!(
            "  store {}: {kind}, {:.3} m³, surface {:?}",
            id.0,
            store.volume(),
            store.surface()
        );
    }
    for (id, link) in water.network().links() {
        if link.open {
            println!(
                "  link {}: {} -> {}, {:.4} m³/s, {:?}",
                id.0,
                link.up.0,
                link.down.0,
                water.link_discharge(id).unwrap_or(0.0),
                link.law
            );
        }
    }
}

fn basin_volumes(water: &WaterWorld) -> Vec<(StoreId, f64)> {
    water.basins().map(|(id, b)| (id, b.volume)).collect()
}

/// Basins that hold water but would draw nothing.
fn undrawn(water: &WaterWorld) -> Vec<String> {
    let mesh = water.build_mesh();
    water
        .basins()
        .filter(|(_, b)| b.volume > 0.01)
        .filter_map(|(id, b)| {
            let drawn = mesh.draws.iter().any(|d| d.body == id && d.index_count > 0);
            let level = WaterScene::level(water, id);
            (!drawn || level.is_none()).then(|| {
                format!(
                    "basin {} holds {:.2} m³ over {} spans but draws {}",
                    id.0,
                    b.volume,
                    b.region.len(),
                    if drawn { "without a level" } else { "nothing" }
                )
            })
        })
        .collect()
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
    let mut trace = None;
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
            "--trace" => {
                let value = args.next().unwrap_or_default();
                let (seed, blast) = value
                    .split_once(':')
                    .and_then(|(a, b)| Some((a.parse().ok()?, b.parse().ok()?)))
                    .ok_or("--trace expects seed:blast")?;
                trace = Some((seed, blast));
                blasts = blasts.max(blast + 1);
            }
            other if other.starts_with('-') => return Err(format!("unknown option '{other}'")),
            other => level = Some(PathBuf::from(other)),
        }
    }
    Ok(Options {
        level: level.ok_or("no level file given")?,
        seeds,
        blasts,
        trace,
    })
}
