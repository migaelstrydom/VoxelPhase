//! Runs a scenario: builds its world, plays its script, records the water.

use crate::terrain::{BlastConfig, TerrainWorld};

use super::legacy::LegacyWater;
use super::scenario::{Action, Scenario};

/// Water ticks per simulated second. Hydrology runs at a fixed 1/60 s (§10.1).
pub const TICK_RATE: f32 = 60.0;

/// How to run a scenario.
#[derive(Debug, Clone, Copy)]
pub struct RunConfig {
    /// Ticks per recorded frame. Each tick is still the true `dt`, so a larger
    /// value plays a scenario out in fewer frames without changing any law.
    pub fast_forward: u32,
}

impl Default for RunConfig {
    fn default() -> Self {
        Self { fast_forward: 1 }
    }
}

/// The water at the end of one recorded frame.
#[derive(Debug, Clone)]
pub struct Sample {
    /// Simulated seconds since the start.
    pub time: f32,
    /// Level at each probe, in the scenario's probe order. `None` where dry.
    pub probes: Vec<Option<f32>>,
    /// Total water held, in m³.
    pub volume: f64,
    /// Wet cells.
    pub wet_cells: usize,
}

/// Something that happened during a run, for the report.
#[derive(Debug, Clone)]
pub struct Event {
    pub time: f32,
    pub text: String,
}

/// Everything recorded from one run of a scenario.
pub struct Run {
    pub scenario: &'static str,
    pub probe_names: Vec<&'static str>,
    pub initial_volume: f64,
    pub samples: Vec<Sample>,
    pub events: Vec<Event>,
}

/// Run a scenario to the end of its duration.
pub fn run(scenario: &Scenario, config: RunConfig) -> Result<Run, String> {
    let (level, mut terrain) = scenario.terrain()?;
    let mut water = LegacyWater::from_level(&level, &terrain)
        .ok_or_else(|| format!("scenario {} has no water", scenario.name))?;

    let dt = 1.0 / TICK_RATE;
    let ticks = (scenario.duration * TICK_RATE).round() as u64;
    let per_frame = config.fast_forward.max(1) as u64;
    let mut pending: Vec<_> = scenario.beats.clone();
    pending.sort_by(|a, b| a.at.total_cmp(&b.at));
    let mut pending = pending.into_iter().peekable();

    let mut recorded = Run {
        scenario: scenario.name,
        probe_names: scenario.probes.iter().map(|p| p.name).collect(),
        initial_volume: water.volume(),
        samples: Vec::new(),
        events: Vec::new(),
    };
    recorded.samples.push(sample(scenario, &water, 0.0));

    for tick in 1..=ticks {
        let time = tick as f32 * dt;
        while let Some(beat) = pending.next_if(|b| b.at <= time) {
            let text = apply(&mut terrain, beat.action);
            recorded.events.push(Event { time, text });
        }
        terrain.update();
        water.tick(&terrain, dt);
        if tick % per_frame == 0 || tick == ticks {
            recorded.samples.push(sample(scenario, &water, time));
        }
    }
    Ok(recorded)
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

fn sample(scenario: &Scenario, water: &LegacyWater, time: f32) -> Sample {
    Sample {
        time,
        probes: scenario
            .probes
            .iter()
            .map(|p| water.level_at(p.at))
            .collect(),
        volume: water.volume(),
        wet_cells: water.wet_cells(),
    }
}
