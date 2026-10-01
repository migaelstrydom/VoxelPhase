//! Whether each object, knocked, behaves as physics says it must.
//!
//! The rest trial leaves every object settled. This one then gives each body
//! the same knock — a hop up and a push sideways — and watches what the
//! engine does with it. Whatever the object does next, three things must
//! hold:
//!
//! - its state stays finite;
//! - its energy never rises, less a little for rounding: nothing but the
//!   knock pushes it, so from then on it can only lose energy;
//! - it ends outside the terrain.
//!
//! Not that it comes back to rest: a pendulum swings and a ball rolls on for
//! longer than any trial worth running, and a body kept moving by energy from
//! nowhere is caught by the energy it gains.
//!
//! Each of this engine's worst bugs broke one of them without anything
//! looking wrong at a glance: a weld and its ground fighting gave a menhir a
//! speed it never used; an explicit gyroscopic term spun free bodies faster
//! and faster; two contacts sharing a load unevenly walked a stone up a ridge
//! against gravity.
//!
//! ```text
//!   RestTrial (settled) ──knock every body──▶ simulate, frame by frame
//!                                               │  energy per kilogram
//!                                               │  against its lowest yet
//!                                               ▼
//!                  per object: non-finite, gained energy,
//!                  in terrain ──▶ Report
//! ```
//!
//! Energy passes between bodies that touch, so it is judged over each group
//! of objects that touched during the trial, not object by object: one
//! object's knock may be what sets another moving. A group that touched a
//! body the trial does not judge — a driven platform, a creature — or went
//! into the water, where buoyancy lifts it, is not judged on energy: both
//! supply it from outside. A body that passes
//! through static geometry on purpose, bedded in the ground, may end inside
//! it.

use std::f32::consts::PI;
use std::ops::ControlFlow;

use nalgebra::Vector3;
use rustc_hash::FxHashMap;

use crate::physics::{PhysicsWorld, RigidBodyHandle};
use crate::terrain::TerrainWorld;
use crate::water::buoyancy::WaterSurface;

use super::report::{Report, Section};
use super::rest::{RestTrial, TrialObject, AFLOAT_BAND};

/// How long each object is watched after its knock.
pub const DISTURB_SECONDS: f32 = 8.0;

/// The knock every body gets, in m/s: straight up, and sideways in a
/// direction of its own.
const KNOCK_UP: f32 = 1.0;
const KNOCK_ACROSS: f32 = 1.0;

/// Turn between one body's sideways knock and the next's: the golden angle,
/// so neighbours are pushed in directions as different as they can be.
const KNOCK_TURN: f32 = PI * (3.0 - 2.236_068);

/// How much energy per kilogram an object may regain over the least it has
/// had since its knock before that is reported, in J/kg: what rising a centimetre takes.
///
/// Position correction moves bodies without giving them speed, and rounding
/// adds a little either way; both are far below this.
pub const ENERGY_GAIN_LIMIT: f32 = 0.1;

/// What went wrong with a knocked object.
#[derive(Debug, Clone, PartialEq)]
pub enum Misbehaviour {
    /// A position or velocity became NaN or infinite.
    NonFinite,
    /// Its energy rose, by this many J/kg at most, above the least it had
    /// had since its knock.
    GainedEnergy(f32),
    /// A body of it ended with its centre inside the terrain.
    InTerrain,
}

/// One object that misbehaved after its knock.
#[derive(Debug, Clone)]
pub struct Disturbance {
    /// How the object is named in findings.
    pub label: String,
    /// Everything it did wrong.
    pub misbehaviours: Vec<Misbehaviour>,
}

impl Disturbance {
    fn describe(&self) -> String {
        self.misbehaviours
            .iter()
            .map(|m| match m {
                Misbehaviour::NonFinite => "its state became non-finite".to_string(),
                Misbehaviour::GainedEnergy(gain) => format!(
                    "regained {gain:.2} J/kg after its knock — as if it rose {:.0} cm on its own",
                    gain / 9.81 * 100.0
                ),
                Misbehaviour::InTerrain => "ended inside the terrain".to_string(),
            })
            .collect::<Vec<_>>()
            .join(", ")
    }
}

/// How one object is doing, frame by frame.
struct Watch {
    /// Its bodies.
    handles: Vec<RigidBodyHandle>,
    /// Their total mass.
    mass: f32,
    /// Its energy, in joules, after every frame.
    energy: Vec<f32>,
    /// Whether any of its bodies has been in the water.
    wet: bool,
    /// Whether any of its bodies has become non-finite.
    non_finite: bool,
}

/// Which objects have touched one another since the knock, as groups.
///
/// Energy passes between objects that touch, so it is a group's energy that
/// must never rise, not each object's. A group that has touched a body the
/// trial does not judge — a driven platform, a creature — may have been
/// given energy by it, and is not judged on energy at all.
struct Touches {
    /// Each object's parent in its group's tree; a root is its own.
    parent: Vec<usize>,
    /// Whether each root's group has touched an unjudged body.
    tainted: Vec<bool>,
}

impl RestTrial {
    /// Knock every body of a settled trial and report every object that
    /// then gains energy, becomes non-finite or ends in the terrain. Run it
    /// after [`RestTrial::run`].
    pub fn disturb(&mut self) -> Vec<Disturbance> {
        {
            let mut physics = self.physics_mut();
            for (index, handle) in self.handles().into_iter().enumerate() {
                knock(&mut physics.world, handle, index);
            }
        }

        let mut watches: Vec<Watch> = {
            let physics = self.physics_mut();
            self.objects()
                .iter()
                .map(|object| Watch::new(&physics.world, object_handles(object).collect()))
                .collect()
        };
        let object_of: FxHashMap<RigidBodyHandle, usize> = watches
            .iter()
            .enumerate()
            .flat_map(|(object, watch)| watch.handles.iter().map(move |&h| (h, object)))
            .collect();
        let mut touches = Touches::new(watches.len());

        self.simulate(DISTURB_SECONDS, |world, water, _| {
            for watch in &mut watches {
                watch.observe(world, water);
            }
            touches.record(world, &object_of);
            ControlFlow::Continue(())
        });

        let gains = touches.energy_gains(&watches);
        let terrain = self.terrain();
        let physics = self.physics_mut();
        self.objects()
            .iter()
            .zip(&watches)
            .zip(gains)
            .filter_map(|((object, watch), gain)| {
                let misbehaviours = watch.judge(&physics.world, &terrain, gain);
                (!misbehaviours.is_empty()).then(|| Disturbance {
                    label: object.label.clone(),
                    misbehaviours,
                })
            })
            .collect()
    }
}

impl Watch {
    fn new(world: &PhysicsWorld, handles: Vec<RigidBodyHandle>) -> Self {
        let mass = handles
            .iter()
            .filter_map(|&h| world.body(h))
            .map(|body| body.mass())
            .sum();
        let knocked = energy_per_kg(world, handles.iter().copied()) * mass;
        Self {
            handles,
            mass,
            energy: vec![knocked],
            wet: false,
            non_finite: false,
        }
    }

    fn observe(&mut self, world: &PhysicsWorld, water: Option<&dyn WaterSurface>) {
        for body in self.handles.iter().filter_map(|&h| world.body(h)) {
            let position = body.position();
            self.non_finite |= !position.coords.iter().all(|c| c.is_finite())
                || !body.linear_velocity().iter().all(|c| c.is_finite())
                || !body.angular_velocity().iter().all(|c| c.is_finite());
            self.wet |= water
                .and_then(|w| w.sample(position))
                .is_some_and(|s| position.y < s.surface_level + AFLOAT_BAND);
        }
        let energy = energy_per_kg(world, self.handles.iter().copied()) * self.mass;
        self.energy.push(energy);
    }

    /// What it did wrong, given how much energy per kilogram its group
    /// regained, if its group was judged on energy.
    fn judge(
        &self,
        world: &PhysicsWorld,
        terrain: &TerrainWorld,
        gain: Option<f32>,
    ) -> Vec<Misbehaviour> {
        if self.non_finite {
            return vec![Misbehaviour::NonFinite];
        }
        let mut misbehaviours = Vec::new();
        if let Some(gain) = gain.filter(|&g| g > ENERGY_GAIN_LIMIT) {
            misbehaviours.push(Misbehaviour::GainedEnergy(gain));
        }
        let bodies = self.handles.iter().filter_map(|&h| world.body(h));
        if bodies.clone().any(|body| {
            let p = body.position();
            !body.ignores_static() && terrain.is_mesh_solid_at(p.x, p.y, p.z)
        }) {
            misbehaviours.push(Misbehaviour::InTerrain);
        }
        misbehaviours
    }
}

impl Touches {
    fn new(objects: usize) -> Self {
        Self {
            parent: (0..objects).collect(),
            tainted: vec![false; objects],
        }
    }

    fn root(&mut self, mut object: usize) -> usize {
        while self.parent[object] != object {
            self.parent[object] = self.parent[self.parent[object]];
            object = self.parent[object];
        }
        object
    }

    /// Join the groups of every pair of objects in contact this frame, and
    /// taint the group of any object touching a body not judged.
    fn record(&mut self, world: &PhysicsWorld, object_of: &FxHashMap<RigidBodyHandle, usize>) {
        for contact in world.contact_events() {
            let Some(other) = contact.body_a else {
                continue;
            };
            match (object_of.get(&other), object_of.get(&contact.body_b)) {
                (Some(&a), Some(&b)) => self.join(a, b),
                (Some(&judged), None) | (None, Some(&judged)) => self.taint(judged),
                (None, None) => {}
            }
        }
    }

    /// Put two objects in the same group.
    fn join(&mut self, a: usize, b: usize) {
        let (a, b) = (self.root(a), self.root(b));
        if a != b {
            self.parent[a] = b;
            self.tainted[b] |= self.tainted[a];
        }
    }

    /// Mark an object's group as having touched a body not judged.
    fn taint(&mut self, object: usize) {
        let root = self.root(object);
        self.tainted[root] = true;
    }

    /// For each object, the most its group's energy per kilogram rose above
    /// the least it had had; `None` where the group is not judged on energy
    /// — it touched an unjudged body, or went into the water — or an object
    /// in it became non-finite.
    fn energy_gains(&mut self, watches: &[Watch]) -> Vec<Option<f32>> {
        let roots: Vec<usize> = (0..watches.len()).map(|o| self.root(o)).collect();
        let mut judged = vec![true; watches.len()];
        for (object, &root) in roots.iter().enumerate() {
            let watch = &watches[object];
            judged[root] &= !self.tainted[root] && !watch.wet && !watch.non_finite;
        }

        let frames = watches.first().map_or(0, |w| w.energy.len());
        let mut energy = vec![vec![0.0f32; frames]; watches.len()];
        let mut mass = vec![0.0f32; watches.len()];
        for (watch, &root) in watches.iter().zip(&roots) {
            mass[root] += watch.mass;
            for (total, e) in energy[root].iter_mut().zip(&watch.energy) {
                *total += e;
            }
        }
        let gain_of_root: Vec<f32> = energy
            .iter()
            .zip(&mass)
            .map(|(series, &mass)| {
                let mut lowest = f32::INFINITY;
                series
                    .iter()
                    .map(|&e| {
                        lowest = lowest.min(e);
                        (e - lowest) / mass.max(f32::MIN_POSITIVE)
                    })
                    .fold(0.0, f32::max)
            })
            .collect();

        roots
            .iter()
            .map(|&root| judged[root].then_some(gain_of_root[root]))
            .collect()
    }
}

/// Add the knock to a body's velocity: up, and sideways in the direction
/// `index` golden angles round.
fn knock(world: &mut PhysicsWorld, handle: RigidBodyHandle, index: usize) {
    let angle = KNOCK_TURN * index as f32;
    let knock = Vector3::new(
        KNOCK_ACROSS * angle.cos(),
        KNOCK_UP,
        KNOCK_ACROSS * angle.sin(),
    );
    world.wake_body(handle);
    if let Some(body) = world.body_mut(handle) {
        let velocity = body.linear_velocity() + knock;
        body.set_linear_velocity(velocity);
    }
}

/// The bodies of a trial object.
fn object_handles(object: &TrialObject) -> impl Iterator<Item = RigidBodyHandle> + '_ {
    object.bodies.iter().map(|(h, _, _)| *h)
}

/// The bodies' kinetic energy, spin included, and height in the world's
/// gravity, per kilogram of them all.
fn energy_per_kg(world: &PhysicsWorld, handles: impl Iterator<Item = RigidBodyHandle>) -> f32 {
    let gravity = world.config().gravity;
    let (energy, mass) = handles
        .filter_map(|h| world.body(h))
        .map(|body| {
            let mass = body.mass();
            let spin = body.angular_velocity();
            let rotational = body
                .world_inv_inertia()
                .try_inverse()
                .map_or(0.0, |inertia| 0.5 * spin.dot(&(inertia * spin)));
            let kinetic = 0.5 * mass * body.linear_velocity().norm_squared();
            let height = -mass * gravity.dot(&body.position().coords);
            (kinetic + rotational + height, mass)
        })
        .fold((0.0, 0.0), |(e, m), (be, bm)| (e + be, m + bm));
    if mass > 0.0 {
        energy / mass
    } else {
        0.0
    }
}

/// Knock the settled trial's bodies and report every object that misbehaves.
///
/// Warnings: each is the engine doing something physics does not, and worth
/// a look, but a level plays on regardless.
pub fn check_disturbance(trial: &mut RestTrial, report: &mut Report) -> Section {
    let disturbances = trial.disturb();

    let mut section = Section::new("Disturbance");
    section
        .row(
            "Objects misbehaving after a knock",
            disturbances.len().to_string(),
        )
        .note(
            "Every body is knocked up and sideways at 1 m/s and watched for 8 s: it must \
             stay finite, never regain energy it has lost, and end outside the terrain.",
        );
    for disturbance in &disturbances {
        report.warn(
            "disturb",
            format!(
                "{} misbehaved after a knock: {}",
                disturbance.label,
                disturbance.describe()
            ),
        );
    }
    section
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::level::{resolve_placements, Level};
    use crate::level_check::build_terrain;

    /// Flat ground at y = 0 with `objects` on it.
    fn level(objects: &str) -> Level {
        let ron = format!(
            r#"
            Level(
                name: "Disturb",
                segments: [(
                    name: "main",
                    terrain: Terrain(
                        voxel_size: 1.0,
                        bounds: (min: (-32.0, -32.0, -32.0), max: (32.0, 32.0, 32.0)),
                        base_height: 0.0,
                        features: [],
                    ),
                    objects: [{objects}],
                )],
                placements: [Root(segment: "main")],
                player_spawn: (0.0, 2.0, 0.0),
            )
            "#
        );
        let mut level: Level = ron::from_str(&ron).expect("disturb level should parse");
        level.frames = resolve_placements(&level).expect("disturb placement should resolve");
        level
    }

    fn disturb(level: &Level) -> Vec<Disturbance> {
        let mut trial = RestTrial::spawn(level, build_terrain(level));
        trial.run();
        trial.disturb()
    }

    /// A crate knocked on flat ground hops, slides and stops, losing energy
    /// all the way.
    #[test]
    fn a_crate_knocked_on_flat_ground_behaves() {
        let disturbances = disturb(&level("Crate(pos: (4.0, 0.5, 4.0), size: 0.5)"));
        assert!(disturbances.is_empty(), "{disturbances:?}");
    }

    /// A stack knocked apart is judged as one object: the energy its blocks
    /// pass to each other is not a gain.
    #[test]
    fn a_stack_knocked_apart_behaves() {
        let disturbances = disturb(&level(
            "Stack(base: (4.0, 0.0, 4.0), items: [Crate(size: 0.5), Crate(size: 0.5), Crate(size: 0.5)])",
        ));
        assert!(disturbances.is_empty(), "{disturbances:?}");
    }

    /// An object's energy history, in joules, with a kilogram of mass.
    fn watch(energy: &[f32]) -> Watch {
        Watch {
            handles: Vec::new(),
            mass: 1.0,
            energy: energy.to_vec(),
            wet: false,
            non_finite: false,
        }
    }

    /// One object's knock sets another moving: the first loses what the
    /// second gains. Judged apart, the second gains; judged as the group
    /// their touching made them, neither does.
    #[test]
    fn energy_passed_between_objects_that_touched_is_no_gain() {
        let watches = [watch(&[2.0, 1.0, 0.5, 0.5]), watch(&[0.5, 0.5, 1.0, 0.6])];

        let apart = Touches::new(2).energy_gains(&watches);
        assert_eq!(apart[1], Some(0.5), "judged alone, the struck object gains");

        let mut touched = Touches::new(2);
        touched.join(0, 1);
        let together = touched.energy_gains(&watches);
        assert!(
            together.iter().all(|g| g.is_some_and(|g| g < 1e-6)),
            "{together:?}"
        );
    }

    /// A group that touched a body the trial does not judge may have been
    /// pushed by it, so it is not judged on energy.
    #[test]
    fn a_group_that_touched_an_unjudged_body_is_not_judged_on_energy() {
        let watches = [watch(&[0.5, 2.0]), watch(&[0.5, 0.5])];
        let mut touches = Touches::new(2);
        touches.join(0, 1);
        touches.taint(1);
        assert_eq!(touches.energy_gains(&watches), vec![None, None]);
    }

    /// Energy per kilogram counts height in the world's gravity: a body a
    /// metre higher has 9.81 J/kg more.
    #[test]
    fn energy_per_kilogram_counts_height() {
        let level = level("Crate(pos: (4.0, 0.5, 4.0), size: 0.5)");
        let trial = RestTrial::spawn(&level, build_terrain(&level));
        let handle = trial.handles()[0];
        let mut physics = trial.physics_mut();
        let low = energy_per_kg(&physics.world, [handle].into_iter());
        let body = physics.world.body_mut(handle).unwrap();
        let raised = body.position() + Vector3::y();
        body.set_position(raised);
        let high = energy_per_kg(&physics.world, [handle].into_iter());
        assert!((high - low - 9.81).abs() < 0.01, "{low} then {high}");
    }
}
