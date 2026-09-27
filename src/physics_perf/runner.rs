use std::collections::hash_map::DefaultHasher;
use std::hash::Hasher;
use std::time::Instant;

use crate::debug::DebugLines;
use crate::perf::stats::median;
use crate::perf::Ground;
use crate::physics::{FixedTimestep, FrameProfile, PhysicsImpulse, PhysicsStage, PhysicsWorld};

use super::record::{FrameRecord, PerfRun};
use super::scenario::{Disturbance, PerfScenario};

/// How a scenario is stepped and how often it is repeated.
///
/// The defaults are the game's: a 1/240 s fixed step, at most 12 substeps a
/// frame, and a 60 Hz frame — four substeps per frame.
#[derive(Debug, Clone, Copy)]
pub struct RunConfig {
    /// Simulated seconds per run.
    pub duration: f32,
    /// Rendered frame length the accumulator is fed, in seconds.
    pub frame_dt: f32,
    /// Physics step, in seconds.
    pub fixed_dt: f32,
    /// Cap on substeps per frame.
    pub max_substeps: u32,
    /// Runs per scenario; each frame reports the median over them, which
    /// keeps one scheduler hiccup out of the numbers.
    pub repeats: usize,
}

impl Default for RunConfig {
    fn default() -> Self {
        Self {
            duration: 4.0,
            frame_dt: 1.0 / 60.0,
            fixed_dt: 1.0 / 240.0,
            max_substeps: 12,
            repeats: 3,
        }
    }
}

/// Run `scenario` on `ground` `config.repeats` times and fold the runs into one.
pub fn run(scenario: &dyn PerfScenario, ground: &Ground, config: RunConfig) -> PerfRun {
    let repeats = config.repeats.max(1);
    let takes: Vec<Take> = (0..repeats)
        .map(|_| run_once(scenario, ground, config))
        .collect();
    let bodies = takes[0].bodies;
    let fingerprint = takes[0].fingerprint;

    PerfRun {
        scenario: scenario.name().to_string(),
        description: scenario.describe(),
        ground: ground.label(),
        bodies,
        frame_dt: config.frame_dt,
        repeats,
        fingerprint,
        frames: median_frames(takes.into_iter().map(|take| take.frames).collect()),
    }
}

/// One run of a scenario, before the repeats are folded together.
struct Take {
    bodies: usize,
    fingerprint: u64,
    frames: Vec<FrameRecord>,
}

fn run_once(scenario: &dyn PerfScenario, ground: &Ground, config: RunConfig) -> Take {
    let mut world = PhysicsWorld::new(scenario.physics_config());
    scenario.populate(&mut world, ground);
    let bodies = world
        .bodies()
        .iter()
        .filter(|(_, b)| b.is_dynamic())
        .count();

    let terrain = ground.terrain();
    let mut timestep = FixedTimestep::new(config.fixed_dt, config.max_substeps);
    let mut debug_lines = DebugLines::default();
    let mut frames = Vec::new();
    let mut sim_time = 0.0f32;
    let mut pending = scenario.disturbances(ground);
    pending.sort_by(|a, b| b.at.total_cmp(&a.at));

    while sim_time < config.duration {
        let substeps = timestep.accumulate(config.frame_dt);
        if substeps == 0 {
            continue;
        }
        let impulses = due(&mut pending, sim_time);

        let started = Instant::now();
        world.update_contacts(
            config.fixed_dt,
            substeps,
            terrain,
            &impulses,
            &mut debug_lines,
        );
        for _ in 0..substeps {
            world.substep(config.fixed_dt, terrain, &[]);
        }
        let wall = started.elapsed();
        debug_lines.clear();
        sim_time += config.fixed_dt * substeps as f32;

        let sleeping = world.sleeping_bodies().len();
        frames.push(FrameRecord {
            sim_time,
            wall,
            profile: world.frame_profile().clone(),
            awake_bodies: bodies.saturating_sub(sleeping),
            contacts: world.contact_events().len(),
            disturbances: impulses.len(),
        });
    }

    Take {
        bodies,
        fingerprint: fingerprint(&world),
        frames,
    }
}

/// Take the disturbances due by `now` off `pending`, which is sorted latest first.
fn due(pending: &mut Vec<Disturbance>, now: f32) -> Vec<PhysicsImpulse> {
    let mut impulses = Vec::new();
    while pending.last().is_some_and(|d| d.at <= now) {
        impulses.extend(pending.pop().map(|d| d.impulse));
    }
    impulses
}

/// A hash of every body's final pose and velocity, bit for bit.
///
/// A change meant only to make the simulation faster must leave this alone;
/// two runs of the same build always agree on it.
fn fingerprint(world: &PhysicsWorld) -> u64 {
    let mut hasher = DefaultHasher::new();
    for (_, body) in world.bodies().iter() {
        let position = body.position();
        let rotation = body.rotation();
        let linear = body.linear_velocity();
        let angular = body.angular_velocity();
        let values = position
            .coords
            .iter()
            .chain(rotation.coords.iter())
            .chain(linear.iter())
            .chain(angular.iter());
        for value in values {
            hasher.write_u32(value.to_bits());
        }
    }
    hasher.finish()
}

/// Frame by frame, the median of each timing over the takes.
///
/// The simulation is deterministic, so every take ran the same frames with
/// the same bodies and contacts; only the clock differs between them.
fn median_frames(takes: Vec<Vec<FrameRecord>>) -> Vec<FrameRecord> {
    let frame_count = takes.iter().map(Vec::len).min().unwrap_or(0);
    (0..frame_count)
        .map(|index| {
            let same_frame: Vec<&FrameRecord> = takes.iter().map(|take| &take[index]).collect();
            let mut profile = FrameProfile::default();
            profile.substeps = same_frame[0].profile.substeps;
            profile.ccd_corrections = same_frame[0].profile.ccd_corrections;
            for stage in PhysicsStage::ALL {
                profile.record(
                    stage,
                    median(same_frame.iter().map(|f| f.profile.stage(stage))),
                );
            }
            FrameRecord {
                wall: median(same_frame.iter().map(|f| f.wall)),
                profile,
                ..same_frame[0].clone()
            }
        })
        .collect()
}
