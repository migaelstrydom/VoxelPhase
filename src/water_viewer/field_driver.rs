//! Runs a scenario on the shallow-water field in place of the network.

use std::fmt::Write as _;
use std::time::Duration;

use crate::level::Settle;
use crate::terrain::TerrainWorld;
use crate::water::field::WaterField;

use super::driver::{apply, RunConfig, TICK_RATE};
use super::scenario::Scenario;

/// Longest a steady settle runs before the scenario starts, s.
pub const SETTLE_LIMIT: f32 = 600.0;

/// The field at the end of one recorded frame.
#[derive(Debug, Clone)]
pub struct FieldSample {
    pub time: f32,
    /// Surface at each probe, `None` where dry.
    pub probes: Vec<Option<f32>>,
    pub volume: f64,
    pub sunk: f64,
    pub ledger_error: f64,
    pub awake: usize,
    pub substeps: u32,
    /// Fastest water anywhere, m/s.
    pub max_speed: f32,
    /// Shallowest cell, m: never below zero.
    pub shallowest: f64,
    /// Whether every depth and discharge was finite.
    pub finite: bool,
    /// Stepping time of this frame.
    pub step: Duration,
}

/// Everything recorded from one run of a scenario on the field.
pub struct FieldRun {
    pub scenario: &'static str,
    pub probe_names: Vec<&'static str>,
    /// Simulated seconds the steady settle ran before the scenario started.
    pub settled_for: f32,
    pub samples: Vec<FieldSample>,
    pub events: Vec<(f32, String)>,
    /// Stepping time per tick over the whole run.
    pub step_times: Vec<Duration>,
    /// Carrying time of each terrain edit.
    pub edit_times: Vec<Duration>,
}

/// Run a scenario on the field, handing the terrain and field to `capture` at
/// each of `captures` (simulated seconds, ascending).
pub fn run_field(
    scenario: &Scenario,
    config: RunConfig,
    captures: &[f32],
    mut capture: impl FnMut(f32, &mut TerrainWorld, &mut WaterField),
) -> Result<FieldRun, String> {
    let (level, mut terrain) = scenario.terrain()?;
    let water = level
        .water
        .clone()
        .ok_or_else(|| format!("scenario {} has no water", scenario.name))?;
    let settle = config.settle.unwrap_or(water.settle);
    let mut field = WaterField::build(&water, &terrain);
    let dt = 1.0 / TICK_RATE;
    let settled_for = match settle {
        Settle::Steady => field.settle(dt, SETTLE_LIMIT),
        Settle::AsAuthored => 0.0,
    };

    let ticks = (scenario.duration * TICK_RATE).round() as u64;
    let per_frame = config.fast_forward.max(1) as u64;
    let mut pending = scenario.beats.clone();
    pending.sort_by(|a, b| a.at.total_cmp(&b.at));
    let mut pending = pending.into_iter().peekable();
    let mut captures = captures.iter().copied().peekable();
    let mut run = FieldRun {
        scenario: scenario.name,
        probe_names: scenario.probes.iter().map(|p| p.name).collect(),
        settled_for,
        samples: vec![sample(scenario, &field, 0.0)],
        events: Vec::new(),
        step_times: Vec::new(),
        edit_times: Vec::new(),
    };
    while let Some(at) = captures.next_if(|t| *t <= 0.0) {
        capture(at, &mut terrain, &mut field);
    }
    for tick in 1..=ticks {
        let time = tick as f32 * dt;
        while let Some(beat) = pending.next_if(|b| b.at <= time) {
            run.events.push((time, apply(&mut terrain, beat.action)));
        }
        terrain.update();
        field.on_terrain_update(&terrain);
        field.step(dt);
        let frame = field.frame();
        if let Some(edit) = frame.edit_time {
            run.edit_times.push(edit);
        }
        run.step_times.push(frame.step_time);
        if tick % per_frame == 0 || tick == ticks {
            run.samples.push(sample(scenario, &field, time));
        }
        while let Some(at) = captures.next_if(|t| *t <= time) {
            capture(at, &mut terrain, &mut field);
        }
    }
    Ok(run)
}

fn sample(scenario: &Scenario, field: &WaterField, time: f32) -> FieldSample {
    let ledger = field.ledger();
    let volume = field.volume();
    let frame = field.frame();
    let (shallowest, finite) = field.health();
    FieldSample {
        time,
        probes: scenario
            .probes
            .iter()
            .map(|p| field.level_at(p.at))
            .collect(),
        volume,
        sunk: ledger.sunk,
        ledger_error: ledger.error(volume),
        awake: frame.awake,
        substeps: frame.stats.substeps,
        max_speed: field.max_speed(),
        shallowest,
        finite,
        step: frame.step_time,
    }
}

/// The run as a table, one row every `every` seconds.
pub fn field_report(run: &FieldRun, every: f32) -> String {
    let mut out = String::new();
    let _ = writeln!(
        out,
        "{} (field; settled for {:.1} s)",
        run.scenario, run.settled_for
    );
    for (time, text) in &run.events {
        let _ = writeln!(out, "  {time:>8.2}s  {text}");
    }
    let _ = write!(out, "{:>8}", "t (s)");
    for name in &run.probe_names {
        let _ = write!(out, " {name:>10}");
    }
    let _ = writeln!(
        out,
        " {:>11} {:>10} {:>10} {:>6} {:>4} {:>8} {:>6}",
        "volume m³", "sunk m³", "ledger", "awake", "sub", "step ms", "u m/s"
    );
    let mut next = 0.0;
    for s in &run.samples {
        if s.time + 1e-4 < next {
            continue;
        }
        next = s.time + every;
        let _ = write!(out, "{:>8.2}", s.time);
        for probe in &s.probes {
            match probe {
                Some(level) => {
                    let _ = write!(out, " {level:>10.3}");
                }
                None => {
                    let _ = write!(out, " {:>10}", "dry");
                }
            }
        }
        let _ = writeln!(
            out,
            " {:>11.2} {:>10.3} {:>10.1e} {:>6} {:>4} {:>8.3} {:>6.2}",
            s.volume,
            s.sunk,
            s.ledger_error,
            s.awake,
            s.substeps,
            s.step.as_secs_f64() * 1e3,
            s.max_speed
        );
    }
    let _ = writeln!(out, "{}", timing_line(run));
    out
}

/// Mean, p99 and max stepping time, and the slowest edit.
pub fn timing_line(run: &FieldRun) -> String {
    let mut times: Vec<f64> = run
        .step_times
        .iter()
        .map(|d| d.as_secs_f64() * 1e3)
        .collect();
    times.sort_by(f64::total_cmp);
    let mean = times.iter().sum::<f64>() / times.len().max(1) as f64;
    let at = |q: f64| {
        times
            .get(((times.len() as f64 - 1.0) * q) as usize)
            .copied()
            .unwrap_or(0.0)
    };
    let edit = run
        .edit_times
        .iter()
        .max()
        .map_or(0.0, |d| d.as_secs_f64() * 1e3);
    format!(
        "step ms: mean {mean:.3}, p99 {:.3}, max {:.3}; slowest edit {edit:.3} ms",
        at(0.99),
        at(1.0)
    )
}
