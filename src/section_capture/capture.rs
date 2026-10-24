use std::path::PathBuf;
use std::time::Instant;

use nalgebra::Point3;
use traversal_planner::plan::{BodyState, Limb};
use traversal_planner::scene::{Ground, Intent, Outcome};
use traversal_planner::section::{sample_section, SphereCaster, StripLayout};
use traversal_planner::{GaitParams, TraversalScene};

use crate::level::load_level;
use crate::level_check::{build_terrain, is_at_rest, RestTrial};
use crate::physics::PhysicsImpulse;
use crate::traversal::{PathFrame, PhysicsCaster};

/// How high above and below zero anything could stand, in metres: the search
/// for the body's floor spans it. The terrain refuses a query longer than
/// 256 m, so a cast across all of it would find nothing.
const SKY: f32 = 100.0;
/// How far above and below the body the strip's casts reach, in metres.
const REACH_ABOVE: f32 = 3.0;
const REACH_BELOW: f32 = 6.0;
/// How long a blasted scene may take to come to rest, in seconds: crates
/// thrown across a crater roll and topple for far longer than a level's
/// objects settle.
const SETTLE_SECONDS: f32 = 30.0;
/// How many times the strip is sampled to time it.
const TIMED_SAMPLES: usize = 10;
/// Upward share of a blast's shove, as a grenade gives.
const BLAST_UPWARD_BOOST: f32 = 0.5;

/// A shove outward from a point, as a grenade gives every body in reach.
#[derive(Clone, Copy, Debug)]
pub struct Blast {
    pub at: Point3<f32>,
    /// Impulse at the centre, N·s, falling off linearly to the radius.
    pub strength: f32,
    /// In metres.
    pub radius: f32,
}

/// What to capture.
#[derive(Clone, Debug)]
pub struct Capture {
    pub level: PathBuf,
    /// World `(x, z)` the body stands at.
    pub at: (f32, f32),
    /// The path's heading, turned about `+Y` from `+Z`, in degrees.
    pub heading_degrees: f32,
    /// Applied in order once the level has spawned, before it settles.
    pub blasts: Vec<Blast>,
    pub speed: f32,
    /// The foot that lands next.
    pub lead: Limb,
    pub expect: Option<Outcome>,
}

/// A captured scene, and how it came to rest.
#[derive(Clone, Debug)]
pub struct Captured {
    pub scene: TraversalScene,
    /// Simulated seconds until every body was at rest, if it ever was.
    pub settled_after: Option<f32>,
    /// Where the bodies still moving at the end are.
    pub restless: Vec<Point3<f32>>,
    /// Seconds one sampling of the strip took, the mean of several: what the
    /// game will pay for a section each frame.
    pub sample_seconds: f64,
}

/// Spawns, blasts and settles the level, then samples the strip.
pub fn capture(spec: &Capture) -> Result<Captured, String> {
    let level = load_level(&spec.level).map_err(|error| error.to_string())?;
    let mut trial = RestTrial::spawn(&level, build_terrain(&level));
    {
        let handles = trial.handles();
        let mut physics = trial.physics_mut();
        for blast in &spec.blasts {
            let impulse =
                PhysicsImpulse::radial(blast.at, blast.radius, blast.strength, BLAST_UPWARD_BOOST);
            for &handle in &handles {
                let Some(position) = physics.world.body(handle).map(|body| body.position()) else {
                    continue;
                };
                if let Some(shove) = impulse.impulse_at(position) {
                    physics.world.apply_impulse(handle, shove);
                }
            }
        }
    }
    let settled_after = trial.settle(SETTLE_SECONDS);
    let restless = {
        let physics = trial.physics();
        trial
            .handles()
            .into_iter()
            .filter(|&handle| !is_at_rest(&physics.world, handle))
            .filter_map(|handle| physics.world.body(handle).map(|body| body.position()))
            .collect()
    };

    let params = GaitParams::default();
    let layout = StripLayout::default();
    let terrain = trial.terrain();
    let physics = trial.physics();
    let frame = PathFrame::new(Point3::new(spec.at.0, 0.0, spec.at.1), spec.heading_degrees);
    let mut caster = PhysicsCaster {
        world: &physics.world,
        scenery: &*terrain,
        frame,
        exclude: &[],
        top: SKY,
        bottom: -SKY,
    };
    let floor = caster
        .cast_down(0.0, 0.0, params.body.radius)
        .ok_or("no floor under the start")?;
    caster.top = floor.z + REACH_ABOVE;
    caster.bottom = floor.z - REACH_BELOW;
    let started = Instant::now();
    for _ in 0..TIMED_SAMPLES - 1 {
        std::hint::black_box(sample_section(&caster, &layout, &params, 0.0));
    }
    let section = sample_section(&caster, &layout, &params, 0.0);
    let sample_seconds = started.elapsed().as_secs_f64() / TIMED_SAMPLES as f64;

    Ok(Captured {
        scene: TraversalScene {
            ground: Ground::Captured(section),
            body: BodyState {
                s: 0.0,
                z: floor.z,
                speed: spec.speed,
                climb: 0.0,
            },
            lead: spec.lead,
            intent: Intent { speed: spec.speed },
            params,
            layout,
            expect: spec.expect,
        },
        settled_after,
        restless,
        sample_seconds,
    })
}
