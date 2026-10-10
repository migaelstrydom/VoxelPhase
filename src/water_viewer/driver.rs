//! Runs a scenario: builds its terrain and field, plays its script, records
//! the water.

use std::time::Duration;

use crate::level::Settle;
use crate::terrain::{BlastConfig, TerrainWorld};
use crate::water::field::WaterField;

use super::scenario::{Action, Scenario};

/// Water ticks per simulated second: the game's frame rate.
pub const TICK_RATE: f32 = 60.0;

/// How to run a scenario.
#[derive(Debug, Clone, Copy)]
pub struct RunConfig {
    /// Ticks per recorded frame. Each tick is still the true `dt`, so a larger
    /// value plays a scenario out in fewer frames without changing any law.
    pub fast_forward: u32,
    /// How the water opens, in place of what the scenario's level says.
    pub settle: Option<Settle>,
}

impl Default for RunConfig {
    fn default() -> Self {
        Self {
            fast_forward: 1,
            settle: None,
        }
    }
}

/// Longest a steady settle runs before the scenario starts, s.
pub const SETTLE_LIMIT: f32 = 600.0;

/// The field at the end of one recorded frame.
#[derive(Debug, Clone)]
pub struct Sample {
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

/// Everything recorded from one run of a scenario.
pub struct Run {
    pub scenario: &'static str,
    pub probe_names: Vec<&'static str>,
    /// Simulated seconds the steady settle ran before the scenario started.
    pub settled_for: f32,
    pub samples: Vec<Sample>,
    pub events: Vec<(f32, String)>,
    /// Stepping time per tick over the whole run.
    pub step_times: Vec<Duration>,
    /// Carrying time of each terrain edit.
    pub edit_times: Vec<Duration>,
}

/// Run a scenario, handing the terrain and field to `capture` at
/// each of `captures` (simulated seconds, ascending).
pub fn run(
    scenario: &Scenario,
    config: RunConfig,
    captures: &[f32],
    mut capture: impl FnMut(f32, &mut TerrainWorld, &mut WaterField),
) -> Result<Run, String> {
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
    let mut run = Run {
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

fn sample(scenario: &Scenario, field: &WaterField, time: f32) -> Sample {
    let ledger = field.ledger();
    let volume = field.volume();
    let frame = field.frame();
    let (shallowest, finite) = field.health();
    Sample {
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

fn apply(terrain: &mut TerrainWorld, action: Action) -> String {
    match action {
        Action::Blast { centre, radius } => {
            terrain.detonate(centre, &BlastConfig::fixed_radius(radius));
            format!(
                "blast r={radius:.1} at ({:.1}, {:.1}, {:.1})",
                centre.x, centre.y, centre.z
            )
        }
    }
}
