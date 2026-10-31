use std::collections::BTreeMap;
use std::path::PathBuf;

use nalgebra::{Point3, Vector3};
use traversal_planner::section::{sample_section, Profile, Section, SphereCaster, StripLayout};
use traversal_planner::{GaitParams, PathVec, PlannerSettings};

use crate::level::load_level;
use crate::level_check::{build_terrain, RestTrial};
use crate::physics::PhysicsImpulse;
use crate::section_capture::Blast;
use crate::traversal::{
    Legs, PathFrame, PhysicsCaster, PlanningCost, PlanningInput, TraversalPlanning,
};

/// The game's frame, in seconds.
const FRAME: f32 = 1.0 / 60.0;
/// How far above the level anything could stand, in metres: where the
/// search for the body's floor starts.
const SKY: f32 = 100.0;
/// Upward share of a blast's shove, as a grenade gives.
const BLAST_UPWARD_BOOST: f32 = 0.5;

/// What to run.
#[derive(Clone, Debug)]
pub struct PerfSpec {
    pub level: PathBuf,
    /// World `(x, z)` the body starts at.
    pub at: (f32, f32),
    /// World height the body's floor is found below; the highest floor
    /// there if none.
    pub below: Option<f32>,
    pub heading_degrees: f32,
    /// How fast the heading turns, in degrees a second, positive to the left
    /// (anticlockwise seen from above): the stick swung steadily, as the
    /// camera is turned while running.
    pub turn_degrees_per_second: f32,
    pub speed: f32,
    pub frames: usize,
    /// Shoves applied after the level settles; the world is stepped every
    /// frame from then on, so what they throw is moving while planned over.
    pub blasts: Vec<Blast>,
    /// Whether to compare each frame's kept strip with one sampled afresh.
    pub verify: bool,
}

/// One frame's planning.
#[derive(Clone, Copy, Debug)]
pub struct FrameRecord {
    pub cost: PlanningCost,
    pub crosses: bool,
    pub diverged: bool,
    /// When, in seconds from now, and where in the world the plan lands its
    /// next footfall, the committed one, if it has one.
    pub next_footfall: Option<(f32, Point3<f32>)>,
    /// How the kept strip differed from one sampled afresh, if checked.
    pub against_fresh: Option<StripDifference>,
}

/// How two strips sampled at the same stations differ.
#[derive(Clone, Copy, Debug, Default)]
pub struct StripDifference {
    /// The largest difference in height at a station both have, in metres.
    pub height: f32,
    /// Stations one has and the other lacks.
    pub missing: usize,
    /// Stations both have.
    pub shared: usize,
}

impl StripDifference {
    fn between(a: &Section, b: &Section) -> Self {
        let mut difference = Self::default();
        fn lines(section: &Section) -> Vec<&Profile> {
            let mut lines = vec![&section.body_floor, &section.ceiling, &section.upper_floor];
            lines.extend(section.foothold_lanes.iter().map(|lane| &lane.profile));
            lines
        }
        for (a, b) in lines(a).into_iter().zip(lines(b)) {
            let heights = |profile: &Profile| -> BTreeMap<u32, f32> {
                profile
                    .surfaces
                    .iter()
                    .flat_map(|surface| &surface.points)
                    .map(|point| (point.s.to_bits(), point.z))
                    .collect()
            };
            let (a, b) = (heights(a), heights(b));
            for (s, z) in &a {
                match b.get(s) {
                    Some(other) => {
                        difference.shared += 1;
                        difference.height = difference.height.max((z - other).abs());
                    }
                    None => difference.missing += 1,
                }
            }
            difference.missing += b.keys().filter(|s| !a.contains_key(s)).count();
        }
        difference
    }
}

/// Every frame's record.
pub struct PerfRun {
    pub frames: Vec<FrameRecord>,
}

pub fn run(spec: &PerfSpec) -> Result<PerfRun, String> {
    let level = load_level(&spec.level).map_err(|error| error.to_string())?;
    let mut trial = RestTrial::spawn(&level, build_terrain(&level));
    trial.settle(10.0);
    if !spec.blasts.is_empty() {
        let handles = trial.handles();
        let mut physics = trial.physics_mut();
        for blast in &spec.blasts {
            let impulse =
                PhysicsImpulse::radial(blast.at, blast.radius, blast.strength, BLAST_UPWARD_BOOST);
            for &handle in &handles {
                if let Some(shove) = physics
                    .world
                    .body(handle)
                    .and_then(|body| impulse.impulse_at(body.position()))
                {
                    physics.world.apply_impulse(handle, shove);
                }
            }
        }
    }

    let heading_at = |frame: usize| {
        let degrees = spec.heading_degrees + spec.turn_degrees_per_second * frame as f32 * FRAME;
        let (sin, cos) = degrees.to_radians().sin_cos();
        Vector3::new(sin, 0.0, cos)
    };
    let heading = heading_at(0);
    let params = GaitParams::default();
    let layout = StripLayout::default();
    let mut planning =
        TraversalPlanning::new(params.clone(), PlannerSettings::default(), layout.clone());
    let mut position = {
        let terrain = trial.terrain();
        let physics = trial.physics();
        let ground = PhysicsCaster {
            world: &physics.world,
            scenery: &*terrain,
            frame: PathFrame::along(Point3::new(spec.at.0, 0.0, spec.at.1), heading),
            exclude: &[],
            carriers: &[],
            top: SKY,
            bottom: -SKY,
        };
        let floor = ground
            .cast_down(
                0.0,
                0.0,
                params.body.radius,
                spec.below.unwrap_or(f32::INFINITY),
            )
            .ok_or("no floor under the start")?;
        Point3::new(spec.at.0, floor.z, spec.at.1)
    };
    let mut velocity = heading * spec.speed;
    let mut legs = Legs::default();

    let mut frames = Vec::with_capacity(spec.frames);
    for index in 0..spec.frames {
        let heading = heading_at(index);
        if !spec.blasts.is_empty() {
            trial.settle(FRAME);
        }
        let terrain = trial.terrain();
        let physics = trial.physics();
        let input = PlanningInput {
            world: &physics.world,
            scenery: &*terrain,
            exclude: &[],
            carriers: &[],
            terrain_changes: &[],
            position,
            velocity,
            heading,
            target_speed: spec.speed,
            dt: FRAME,
            legs,
        };
        let Some(planned) = planning.update(&input) else {
            break;
        };
        let against_fresh = spec.verify.then(|| {
            let caster = planning.caster(&physics.world, &*terrain, &[], &[]);
            StripDifference::between(
                &planned.section,
                &sample_section(
                    &caster,
                    &planned.section.layout,
                    &params,
                    planned.body.s,
                    planned.body.z,
                ),
            )
        });
        let diverged = match &planned.outcome {
            Ok(plan) => plan.diverged,
            Err(refusal) => refusal.diverged,
        };
        frames.push(FrameRecord {
            cost: planned.cost,
            crosses: planned.outcome.is_ok(),
            diverged,
            next_footfall: planned
                .plan()
                .footfalls()
                .next()
                .map(|(time, contact)| (time, planned.frame.world_point(contact.at))),
            against_fresh,
        });

        // The body follows its plan exactly.
        let query = traversal_planner::section::StripQuery::new(&planned.section, &params);
        let Some(next) = planned.plan().body_at(FRAME, &query) else {
            break;
        };
        position = planned.frame.world_point(PathVec::new(next.s, 0.0, next.z));
        velocity = planned.frame.forward() * next.speed + Vector3::y() * next.climb;
        legs = match &planned.outcome {
            Ok(_) => Legs::following(planned.plan(), &planned.frame, FRAME),
            Err(_) => Legs::default(),
        };
    }
    Ok(PerfRun { frames })
}
