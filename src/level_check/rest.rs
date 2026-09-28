//! Whether each object is at rest where it was authored.
//!
//! The game creates every body placed at rest asleep, so an object authored an
//! inch above its shelf, or half out of the water it should float in, stays
//! exactly there until something knocks it. Nothing about the placement point
//! says so — the footprint checks can see a void under a crate, not a crate
//! resting on another crate, nor a barrel's draft.
//!
//! So this asks the physics. Every object is spawned through its real
//! spawnable onto the real terrain and water, every body is woken, and the
//! world is stepped at the game's cadence until it has all come to rest.
//! Whatever moved on the way was not resting where it was authored.
//!
//! "At rest" is the sleep system's velocity test, not its verdict: a body with
//! an active constraint never sleeps (a fence post's anchor, a pendulum's
//! pivot), however still it is, and that is the engine's policy rather than
//! anything the level got wrong.
//!
//! ```text
//!   Level ──spawnables──▶ PhysicsWorld ◀── TerrainWorld (static geometry)
//!                              │       ◀── WaterWorld (buoyancy, still)
//!                   wake all, step until at rest
//!                              ▼
//!         per object: how far its bodies moved and turned ──▶ Report
//! ```
//!
//! Two kinds of body are judged differently, because the game holds them to
//! something other than their authored pose:
//!
//! - A driven body (an `Actuator`: platforms, creatures) is left asleep and
//!   unmeasured. Its drive decides where it is, and there is no drive here.
//! - A body that ends up afloat is judged on how far it rose, sank and turned,
//!   not on how far it went: a current carries a floater off by design.
//!
//! Headless, but not ECS-free: spawnables build their bodies through a specs
//! `World`, so the trial keeps one. Materials are placeholders; nothing draws.

use nalgebra::{Point3, UnitQuaternion};
use specs::shred::Fetch;
use specs::{Join, World, WorldExt};

use crate::app::world_builder::WorldBuilder;
use crate::components::RigidBodyComponent;
use crate::debug::DebugLines;
use crate::drive::Actuator;
use crate::level::{create_level_water, Level};
use crate::physics::{
    PhysicsWorld, RigidBodyHandle, SequentialStepper, Stepper, SubstepForceProvider,
};
use crate::rendering::material::MaterialId;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;
use crate::water::buoyancy::{BuoyancyForceProvider, WaterSurface};
use crate::water::WaterWorld;

use super::report::{Report, Section};

/// The game's frame and physics step: 60 Hz frames of 1/240 s substeps.
const FRAME_DT: f32 = 1.0 / 60.0;
const FIXED_DT: f32 = 1.0 / 240.0;
const MAX_SUBSTEPS: u32 = 12;

/// How long the world may take to go back to sleep before whatever is still
/// moving is reported as never coming to rest.
///
/// A crate dropped a few metres lands and sleeps within two seconds. A body
/// still awake after this is rolling, swinging or sliding, not settling.
pub const TRIAL_SECONDS: f32 = 6.0;

/// How far a body may move while settling before its object is reported.
///
/// Contact settles by the solver's allowed penetration and a floater bobs to
/// its draft; both are millimetres to a couple of centimetres. Five is clear
/// of that and still far below anything a player would notice as hovering.
pub const DRIFT_LIMIT: f32 = 0.05;

/// How far a body may turn while settling before its object is reported.
pub const TURN_LIMIT_DEGREES: f32 = 5.0;

/// How far above the water's surface a body's centre may end and still count
/// as afloat.
///
/// A floater's centre rides at or below the surface; half a metre admits a
/// large, light one riding high without taking in a body resting on a bank.
const AFLOAT_BAND: f32 = 0.5;

/// Frames every body must stay at rest before the trial calls the world
/// settled: the sleep system's own delay, so a ball at the top of a bounce is
/// not mistaken for one lying still.
const SETTLE_FRAMES: u32 = 30;

/// A level's objects spawned into a physics world of their own, ready to be
/// woken and watched.
pub struct RestTrial {
    /// Holds the physics, the terrain and the water the spawnables built into.
    world: World,
    /// Every object that spawned at least one dynamic body, in level order.
    objects: Vec<TrialObject>,
}

/// One authored object and the dynamic bodies it became.
struct TrialObject {
    /// How the object is named in findings.
    label: String,
    /// Each body and the pose it was created in.
    bodies: Vec<(RigidBodyHandle, Point3<f32>, UnitQuaternion<f32>)>,
}

/// How far one object ended up from where it was authored.
#[derive(Debug, Clone)]
pub struct Drift {
    /// How the object is named in findings.
    pub label: String,
    /// Largest distance any of its bodies moved, in metres.
    pub distance: f32,
    /// Largest height any of its bodies lost, in metres; negative if it rose.
    pub drop: f32,
    /// Largest angle any of its bodies turned through, in degrees.
    pub turn_degrees: f32,
    /// Whether any of its bodies was still awake when the trial ended.
    pub still_moving: bool,
}

impl Drift {
    fn is_significant(&self) -> bool {
        self.still_moving || self.distance > DRIFT_LIMIT || self.turn_degrees > TURN_LIMIT_DEGREES
    }

    /// Whether it went nowhere and still never slept: it jitters in place.
    fn only_restless(&self) -> bool {
        self.distance <= DRIFT_LIMIT && self.turn_degrees <= TURN_LIMIT_DEGREES
    }

    fn describe(&self) -> String {
        let mut parts = Vec::new();
        if self.drop > DRIFT_LIMIT {
            parts.push(format!("fell {:.2} m", self.drop));
        } else if self.drop < -DRIFT_LIMIT {
            parts.push(format!("rose {:.2} m", -self.drop));
        }
        if self.distance > DRIFT_LIMIT && self.drop.abs() < self.distance * 0.9 {
            parts.push(format!("moved {:.2} m", self.distance));
        }
        if self.turn_degrees > TURN_LIMIT_DEGREES {
            parts.push(format!("turned {:.0}°", self.turn_degrees));
        }
        if self.still_moving && self.only_restless() {
            parts.push(format!(
                "stayed put but never came to rest in {TRIAL_SECONDS:.0} s: it jitters in \
                 place, and once woken costs a solve every frame"
            ));
        } else if self.still_moving {
            parts.push(format!("was still moving after {TRIAL_SECONDS:.0} s"));
        }
        parts.join(", ")
    }
}

/// What a trial found.
#[derive(Debug, Clone)]
pub struct RestOutcome {
    /// Dynamic bodies the level's objects spawned.
    pub bodies: usize,
    /// Simulated seconds until every body was at rest, if it ever was.
    pub settled_after: Option<f32>,
    /// Every object that did not stay where it was authored.
    pub drifts: Vec<Drift>,
}

impl RestTrial {
    /// Spawn every object of `level` onto `terrain` and the level's water.
    ///
    /// Takes the terrain because terrain-anchored spawnables read it out of the
    /// world as they spawn; [`RestTrial::terrain`] lends it back.
    pub fn spawn(level: &Level, terrain: TerrainWorld) -> Self {
        let water = create_level_water(level, &terrain);
        let mut world = WorldBuilder::new()
            .with_default_resources()
            .build()
            .expect("a world with no device attached builds infallibly");
        world.insert(terrain);
        if let Some(water) = water {
            world.insert(water);
        }

        let mut objects = Vec::new();
        let mut known = dynamic_bodies(&world.read_resource::<PhysicsResource>().world);
        for (ordinal, (segment, object)) in level.objects().enumerate() {
            let spawnable = object.to_spawnable();
            let materials = vec![MaterialId(0); spawnable.material_count()];
            spawnable.spawn(&mut world, &materials);

            let physics = &world.read_resource::<PhysicsResource>().world;
            let now = dynamic_bodies(physics);
            let bodies: Vec<_> = now
                .iter()
                .filter(|h| !known.contains(h))
                .filter_map(|&h| physics.body(h).map(|b| (h, b.position(), b.rotation())))
                .collect();
            known = now;

            if let Some(&(_, first, _)) = bodies.first() {
                objects.push(TrialObject {
                    label: format!(
                        "{} #{} in '{}' at ({:.1}, {:.1}, {:.1})",
                        object.describe().kind,
                        ordinal + 1,
                        level.segments[segment].name,
                        first.x,
                        first.y,
                        first.z
                    ),
                    bodies,
                });
            }
        }

        let driven = driven_bodies(&world);
        for object in &mut objects {
            object.bodies.retain(|(h, _, _)| !driven.contains(h));
        }
        objects.retain(|object| !object.bodies.is_empty());

        Self { world, objects }
    }

    /// The terrain the trial was spawned on.
    pub fn terrain(&self) -> Fetch<'_, TerrainWorld> {
        self.world.read_resource::<TerrainWorld>()
    }

    /// Wake everything and step until it is all at rest again, or until
    /// [`TRIAL_SECONDS`] have passed.
    pub fn run(&mut self) -> RestOutcome {
        let terrain = self.world.read_resource::<TerrainWorld>();
        let water = self.world.try_fetch::<WaterWorld>();
        let mut physics = self.world.write_resource::<PhysicsResource>();
        let world = &mut physics.world;

        let handles: Vec<_> = self
            .objects
            .iter()
            .flat_map(|o| o.bodies.iter().map(|(h, _, _)| *h))
            .collect();
        for &handle in &handles {
            world.wake_body(handle);
        }

        let query = water.as_deref().map(WaterWorld::query);
        let buoyancy = query
            .as_ref()
            .map(|q| BuoyancyForceProvider::new(q, handles.clone()));
        let providers: Vec<&dyn SubstepForceProvider> = buoyancy
            .as_ref()
            .map(|p| vec![p as &dyn SubstepForceProvider])
            .unwrap_or_default();

        let mut stepper = SequentialStepper::new(FIXED_DT, MAX_SUBSTEPS);
        let mut debug_lines = DebugLines::default();
        let mut elapsed = 0.0;
        let mut settled_after = None;
        let mut frames_at_rest = 0;
        while elapsed < TRIAL_SECONDS {
            stepper.step(
                world,
                FRAME_DT,
                &*terrain,
                &[],
                &providers,
                &mut debug_lines,
            );
            debug_lines.clear();
            elapsed += FRAME_DT;
            if handles.iter().all(|&h| is_at_rest(world, h)) {
                frames_at_rest += 1;
            } else {
                frames_at_rest = 0;
            }
            if frames_at_rest >= SETTLE_FRAMES {
                settled_after = Some(elapsed);
                break;
            }
        }

        let drifts = self
            .objects
            .iter()
            .map(|object| object.drift(world, query.as_ref().map(|q| q as &dyn WaterSurface)))
            .filter(Drift::is_significant)
            .collect();

        RestOutcome {
            bodies: handles.len(),
            settled_after,
            drifts,
        }
    }
}

impl TrialObject {
    fn drift(&self, world: &PhysicsWorld, water: Option<&dyn WaterSurface>) -> Drift {
        let mut drift = Drift {
            label: self.label.clone(),
            distance: 0.0,
            drop: 0.0,
            turn_degrees: 0.0,
            still_moving: false,
        };
        for &(handle, start, rotation) in &self.bodies {
            // A body the trial lost — removed by a fracture, say — has moved
            // by definition, but not in a way this can measure.
            let Some(body) = world.body(handle) else {
                continue;
            };
            let end = body.position();
            let drop = start.y - end.y;
            let afloat = water
                .and_then(|w| w.sample(end))
                .is_some_and(|s| end.y < s.surface_level + AFLOAT_BAND);
            let distance = if afloat {
                drop.abs()
            } else {
                (end - start).norm()
            };
            drift.distance = drift.distance.max(distance);
            if drop.abs() > drift.drop.abs() {
                drift.drop = drop;
            }
            drift.turn_degrees = drift
                .turn_degrees
                .max(rotation.angle_to(&body.rotation()).to_degrees());
            drift.still_moving |= !afloat && !is_at_rest(world, handle);
        }
        drift
    }
}

/// Whether a body is asleep, or slow enough that the sleep system would put it
/// to sleep were it free to.
fn is_at_rest(world: &PhysicsWorld, handle: RigidBodyHandle) -> bool {
    if world.is_sleeping(handle) {
        return true;
    }
    let Some(body) = world.body(handle) else {
        return true;
    };
    let sleep = &world.config().sleep;
    body.linear_velocity().norm() < sleep.linear_threshold
        && body.angular_velocity().norm() < sleep.angular_threshold
}

/// Every body a drive holds, rather than its authored pose.
fn driven_bodies(world: &World) -> Vec<RigidBodyHandle> {
    let bodies = world.read_storage::<RigidBodyComponent>();
    let actuators = world.read_storage::<Actuator>();
    (&bodies, &actuators).join().map(|(b, _)| b.0).collect()
}

/// Every dynamic body in the world.
fn dynamic_bodies(world: &PhysicsWorld) -> Vec<RigidBodyHandle> {
    world
        .bodies()
        .iter()
        .filter(|(_, body)| body.is_dynamic())
        .map(|(index, _)| RigidBodyHandle(index))
        .collect()
}

/// Run a trial and report every object that did not stay where it was put.
///
/// Warnings, not errors: an object dropped from a height on purpose is
/// legitimate. It is still worth knowing about, because in the game it will
/// not drop — it starts asleep, and hangs there until something wakes it.
pub fn check_rest(trial: &mut RestTrial, report: &mut Report) -> Section {
    let outcome = trial.run();

    let mut section = Section::new("Rest");
    section
        .row("Dynamic bodies", outcome.bodies.to_string())
        .row(
            "At rest again after",
            match outcome.settled_after {
                Some(t) => format!("{t:.2} s"),
                None => format!("not within {TRIAL_SECONDS:.0} s"),
            },
        )
        .row("Objects not at rest", outcome.drifts.len().to_string())
        .note(
            "Every body starts asleep in the game, so an object reported here stays where \
             it was authored until something disturbs it.",
        );

    for drift in &outcome.drifts {
        report.warn(
            "rest",
            format!(
                "{} is not at rest where authored: {}",
                drift.label,
                drift.describe()
            ),
        );
    }

    section
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::*;
    use crate::level::{load_level, resolve_placements};
    use crate::level_check::build_terrain;

    /// Flat ground at y = 0, and one crate with its centre at `crate_y`.
    fn crate_level(crate_y: f32) -> Level {
        let ron = format!(
            r#"
            Level(
                name: "Rest",
                segments: [(
                    name: "main",
                    terrain: Terrain(
                        voxel_size: 1.0,
                        bounds: (min: (-32.0, -32.0, -32.0), max: (32.0, 32.0, 32.0)),
                        base_height: 0.0,
                        features: [],
                    ),
                    objects: [Crate(pos: (4.0, {crate_y}, 4.0), size: 0.5)],
                )],
                placements: [Root(segment: "main")],
                player_spawn: (0.0, 2.0, 0.0),
            )
            "#
        );
        let mut level: Level = ron::from_str(&ron).expect("rest level should parse");
        level.frames = resolve_placements(&level).expect("rest placement should resolve");
        level
    }

    fn trial(level: &Level) -> RestOutcome {
        RestTrial::spawn(level, build_terrain(level)).run()
    }

    #[test]
    fn a_crate_hanging_in_the_air_is_reported_as_falling() {
        let outcome = trial(&crate_level(3.0));
        assert_eq!(outcome.drifts.len(), 1, "{:?}", outcome.drifts);
        let drift = &outcome.drifts[0];
        assert!(drift.drop > 2.0, "a 3 m hang fell only {:.2} m", drift.drop);
        assert!(drift.describe().starts_with("fell"), "{}", drift.describe());
    }

    #[test]
    fn a_crate_resting_on_the_ground_is_silent() {
        let outcome = trial(&crate_level(0.5));
        assert!(outcome.drifts.is_empty(), "{:?}", outcome.drifts);
        assert!(outcome.settled_after.is_some());
    }

    /// Every shipped level's objects rest where they were authored.
    ///
    /// In the game each of them starts asleep, so one that is not at rest
    /// hangs, floats or sits embedded exactly as authored until something
    /// disturbs it. Slow in a debug build; run with `--release`.
    #[test]
    #[ignore = "simulates every shipped level; run with --release -- --ignored"]
    fn shipped_levels_are_at_rest_where_authored() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("levels");
        let mut paths: Vec<_> = std::fs::read_dir(&dir)
            .expect("levels directory should be readable")
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.to_string_lossy().ends_with(".level.ron"))
            .collect();
        paths.sort();

        let mut problems = Vec::new();
        for path in &paths {
            let level = load_level(path).expect("shipped level should load");
            for drift in trial(&level).drifts {
                problems.push(format!(
                    "{}: {} — {}",
                    path.file_name().unwrap().to_string_lossy(),
                    drift.label,
                    drift.describe()
                ));
            }
        }
        assert!(
            problems.is_empty(),
            "{} object(s) not at rest where authored:\n{}",
            problems.len(),
            problems.join("\n")
        );
    }
}
