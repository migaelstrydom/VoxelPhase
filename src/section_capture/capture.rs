use std::path::{Path, PathBuf};
use std::time::Instant;

use nalgebra::Point3;
use traversal_planner::plan::{BodyState, Limb};
use traversal_planner::scene::{Ground, Intent, Outcome};
use traversal_planner::section::{sample_section, SphereCaster, StripLayout};
use traversal_planner::{GaitParams, TraversalScene};

use crate::level::load_level;
use crate::level_check::{build_terrain, is_at_rest, RestTrial};
use crate::physics::PhysicsImpulse;
use crate::traversal::{strip_reach, PathFrame, PhysicsCaster, WallSlide};

/// How high above and below zero anything could stand, in metres: the search
/// for the body's floor spans it. The terrain refuses a query longer than
/// 256 m, so a cast across all of it would find nothing.
const SKY: f32 = 100.0;
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

/// A level spawned headless, blasted and come to rest: strips are captured
/// from it, as many as wanted.
pub struct CaptureWorld {
    trial: RestTrial,
    /// Simulated seconds until every body was at rest, if it ever was.
    pub settled_after: Option<f32>,
    /// Where the bodies still moving at the end are.
    pub restless: Vec<Point3<f32>>,
}

impl CaptureWorld {
    /// Spawns `level`, applies `blasts` and lets it come to rest.
    pub fn open(level: &Path, blasts: &[Blast]) -> Result<Self, String> {
        let level = load_level(level).map_err(|error| error.to_string())?;
        let mut trial = RestTrial::spawn(&level, build_terrain(&level));
        {
            let handles = trial.handles();
            let mut physics = trial.physics_mut();
            for blast in blasts {
                let impulse = PhysicsImpulse::radial(
                    blast.at,
                    blast.radius,
                    blast.strength,
                    BLAST_UPWARD_BOOST,
                );
                for &handle in &handles {
                    let Some(position) = physics.world.body(handle).map(|body| body.position())
                    else {
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
        Ok(Self {
            trial,
            settled_after,
            restless,
        })
    }

    /// The strip from world `(x, z)` for a body heading along
    /// `heading_degrees` (turned about `+Y` from `+Z`), as a scene with the
    /// body standing on its floor there; and the seconds one sampling of it
    /// took. The path is laid as the game lays it: along a wall the heading
    /// meets nearly parallel ([`WallSlide`]).
    pub fn capture(
        &self,
        at: (f32, f32),
        heading_degrees: f32,
        speed: f32,
        lead: Limb,
        expect: Option<Outcome>,
    ) -> Result<(TraversalScene, f64), String> {
        let params = GaitParams::default();
        let layout = StripLayout::default();
        let terrain = self.trial.terrain();
        let physics = self.trial.physics();
        let frame = PathFrame::new(Point3::new(at.0, 0.0, at.1), heading_degrees);
        let mut caster = PhysicsCaster {
            world: &physics.world,
            scenery: &*terrain,
            frame,
            exclude: &[],
            top: SKY,
            bottom: -SKY,
        };
        // The ground right under the start, which a sphere the body's size
        // might not reach beside a wall, then the body's floor where the
        // path leaves from.
        let ground = caster
            .cast_down(0.0, 0.0, params.foot.radius, f32::INFINITY)
            .ok_or("no floor under the start")?;
        let bottom = ground.z - params.foot.radius + params.body.radius;
        caster.frame = WallSlide {
            world: &physics.world,
            scenery: &*terrain,
            exclude: &[],
            body: &params.body,
        }
        .path(
            Point3::new(at.0, bottom, at.1),
            frame.forward(),
            layout.ahead,
        )
        .frame;
        let floor = caster
            .cast_down(0.0, 0.0, params.body.radius, f32::INFINITY)
            .ok_or("no floor under the start")?;
        let (above, below) = strip_reach(&layout, &params);
        caster.top = floor.z + above;
        caster.bottom = floor.z - below;
        let started = Instant::now();
        for _ in 0..TIMED_SAMPLES - 1 {
            std::hint::black_box(sample_section(&caster, &layout, &params, 0.0, floor.z));
        }
        let section = sample_section(&caster, &layout, &params, 0.0, floor.z);
        let sample_seconds = started.elapsed().as_secs_f64() / TIMED_SAMPLES as f64;
        let scene = TraversalScene {
            ground: Ground::Captured(section),
            body: BodyState {
                s: 0.0,
                z: floor.z,
                speed,
                climb: 0.0,
            },
            lead,
            intent: Intent { speed },
            params,
            layout,
            expect,
        };
        Ok((scene, sample_seconds))
    }
}

/// Spawns, blasts and settles the level, then samples the strip.
pub fn capture(spec: &Capture) -> Result<Captured, String> {
    let world = CaptureWorld::open(&spec.level, &spec.blasts)?;
    let (scene, sample_seconds) = world.capture(
        spec.at,
        spec.heading_degrees,
        spec.speed,
        spec.lead,
        spec.expect,
    )?;
    Ok(Captured {
        scene,
        settled_after: world.settled_after,
        restless: world.restless,
        sample_seconds,
    })
}
