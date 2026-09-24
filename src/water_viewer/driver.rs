//! Runs a scenario: builds its world, plays its script, records the water.

use crate::level::Settle;
use crate::terrain::{BlastConfig, TerrainWorld};
use crate::water::network::Store;
use crate::water::topology::TopologyEdit;
use crate::water::WaterWorld;

use super::scenario::{Action, Scenario};

/// Water ticks per simulated second. Hydrology runs at a fixed 1/60 s (§10.1).
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

/// The water at the end of one recorded frame.
#[derive(Debug, Clone)]
pub struct Sample {
    /// Simulated seconds since the start.
    pub time: f32,
    /// Still level at each probe, in the scenario's probe order, without
    /// swell or ripples. `None` where dry.
    pub probes: Vec<Option<f32>>,
    /// Total water held, in m³.
    pub volume: f64,
    /// Basins in the network.
    pub basins: usize,
    /// Open links.
    pub links: usize,
    /// Reaches in the network.
    pub reaches: usize,
    /// Links carrying a fall.
    pub falls: usize,
    /// Volume the ledger has booked into sinks and the void so far.
    pub sunk: f64,
    /// Volume the ledger has booked as discarded so far; zero from stage 4a.
    pub discarded: f64,
    /// The ledger's imbalance, m³.
    pub ledger_error: f64,
}

/// Something that happened during a run, for the report.
#[derive(Debug, Clone)]
pub struct Event {
    pub time: f32,
    pub text: String,
}

/// Everything recorded from one run of a scenario.
pub struct Run {
    /// Each reach laid during the run, as it was when laid.
    pub reaches: Vec<String>,
    pub scenario: &'static str,
    pub probe_names: Vec<&'static str>,
    pub initial_volume: f64,
    pub samples: Vec<Sample>,
    pub events: Vec<Event>,
}

/// Run a scenario to the end of its duration.
pub fn run(scenario: &Scenario, config: RunConfig) -> Result<Run, String> {
    run_with_captures(scenario, config, &[], |_, _, _| {})
}

/// Run a scenario, handing the terrain and water to `capture` at each of
/// `captures` (simulated seconds, ascending): how a filmstrip is shot from a
/// run that edits and simulates its own world.
pub fn run_with_captures(
    scenario: &Scenario,
    config: RunConfig,
    captures: &[f32],
    mut capture: impl FnMut(f32, &mut TerrainWorld, &mut WaterWorld),
) -> Result<Run, String> {
    let (level, mut terrain) = scenario.terrain()?;
    let mut water_config = level
        .water
        .clone()
        .ok_or_else(|| format!("scenario {} has no water", scenario.name))?;
    if let Some(settle) = config.settle {
        water_config.settle = settle;
    }
    let (mut water, errors) = WaterWorld::recording(&water_config, &terrain);

    let dt = 1.0 / TICK_RATE;
    let ticks = (scenario.duration * TICK_RATE).round() as u64;
    let per_frame = config.fast_forward.max(1) as u64;
    let mut pending: Vec<_> = scenario.beats.clone();
    pending.sort_by(|a, b| a.at.total_cmp(&b.at));
    let mut pending = pending.into_iter().peekable();

    let mut recorded = Run {
        reaches: Vec::new(),
        scenario: scenario.name,
        probe_names: scenario.probes.iter().map(|p| p.name).collect(),
        initial_volume: water.volume(),
        samples: Vec::new(),
        events: Vec::new(),
    };
    for error in errors {
        recorded.events.push(Event {
            time: 0.0,
            text: error.to_string(),
        });
    }
    for (id, store) in water.network().stores() {
        if let Some(reach) = store.as_reach() {
            recorded.reaches.push(describe_reach(id.0, reach, 0.0));
        }
    }
    recorded.samples.push(sample(scenario, &water, 0.0));
    let mut edits_seen = water.topology_log().len();
    let mut captures = captures.iter().copied().peekable();
    while let Some(at) = captures.next_if(|t| *t <= 0.0) {
        capture(at, &mut terrain, &mut water);
    }

    for tick in 1..=ticks {
        let time = tick as f32 * dt;
        while let Some(beat) = pending.next_if(|b| b.at <= time) {
            let text = apply(&mut terrain, beat.action);
            recorded.events.push(Event { time, text });
        }
        terrain.update();
        water.on_terrain_update(&terrain);
        water.step(dt);
        let log = water.topology_log();
        for edit in &log[edits_seen..] {
            recorded.events.push(Event {
                time,
                text: describe(edit, &water),
            });
            if let TopologyEdit::AddStore(id) = edit {
                if let Some(reach) = water.network().store(*id).and_then(Store::as_reach) {
                    recorded.reaches.push(describe_reach(id.0, reach, time));
                }
            }
        }
        edits_seen = log.len();
        if tick % per_frame == 0 || tick == ticks {
            recorded.samples.push(sample(scenario, &water, time));
        }
        while let Some(at) = captures.next_if(|t| *t <= time) {
            capture(at, &mut terrain, &mut water);
        }
    }
    Ok(recorded)
}

/// An edit as the report prints it: links with their ends, stores with
/// their kind.
fn describe(edit: &TopologyEdit, water: &WaterWorld) -> String {
    match edit {
        TopologyEdit::AddLink(id) => match water.network().link(*id) {
            Some(l) => format!("AddLink({}: {} -> {})", id.0, l.up.0, l.down.0),
            None => format!("AddLink({})", id.0),
        },
        TopologyEdit::AddStore(id) => {
            let kind = match water.network().store(*id) {
                Some(Store::Basin(_)) => "basin",
                Some(Store::Reach(_)) => "reach",
                Some(Store::Sink) => "sink",
                Some(Store::Reservoir) => "reservoir",
                Some(Store::Ocean(_)) => "ocean",
                None => "gone",
            };
            format!("AddStore({} {kind})", id.0)
        }
        TopologyEdit::Reregion { basin, seeds } => {
            format!("Reregion({}, {} seeds)", basin.0, seeds.len())
        }
        other => format!("{other:?}"),
    }
}

fn describe_reach(id: u32, reach: &crate::water::network::Reach, time: f32) -> String {
    let (a, b) = (
        reach.centreline.points.first().copied().unwrap_or_default(),
        reach.centreline.points.last().copied().unwrap_or_default(),
    );
    let design = reach.rating.at(reach.rating.design());
    format!(
        "{time:>7.2}s reach {id}: ({:.1}, {:.1}, {:.1}) -> ({:.1}, {:.1}, {:.1}), {:.1} m; design {:.2} m³/s runs {:.2} m deep, {:.1} m wide at {:.2} m/s",
        a.x, a.y, a.z, b.x, b.y, b.z, reach.length, design.q, design.depth, design.top_width, design.velocity
    )
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

fn sample(scenario: &Scenario, water: &WaterWorld, time: f32) -> Sample {
    let query = water.query();
    Sample {
        time,
        probes: scenario
            .probes
            .iter()
            .map(|p| query.level_at(p.at))
            .collect(),
        volume: water.volume(),
        basins: water.basins().count(),
        links: water.network().links().filter(|(_, l)| l.open).count(),
        reaches: water
            .network()
            .stores()
            .filter(|(_, s)| s.as_reach().is_some())
            .count(),
        falls: water.falls().count(),
        sunk: water.ledger().sunk,
        discarded: water.ledger().discarded,
        ledger_error: water.balance().error(),
    }
}
