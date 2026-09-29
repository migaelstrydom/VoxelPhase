//! Whether each object, knocked, behaves as physics says it must.
//!
//! The rest trial leaves every object settled. This one then gives each body
//! the same knock — a hop up and a push sideways — and watches what the
//! engine does with it. Whatever the object does next, three things must
//! hold:
//!
//! - its state stays finite;
//! - it never has more energy than the knock gave it, less a little for
//!   rounding: nothing but the knock is pushing it;
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
//!                                               │  against the knock's
//!                                               ▼
//!                  per object: non-finite, gained energy,
//!                  in terrain ──▶ Report
//! ```
//!
//! An object is judged as a whole, so energy passing between its own bodies
//! is not a gain. A body that goes into the water is not judged on energy:
//! buoyancy lifts it, and that is energy from outside. A body that passes
//! through static geometry on purpose, bedded in the ground, may end inside
//! it.

use std::f32::consts::PI;
use std::ops::ControlFlow;

use nalgebra::Vector3;

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

/// How much energy per kilogram an object may gain over what its knock gave
/// it before that is reported, in J/kg: what rising a centimetre takes.
///
/// Position correction moves bodies without giving them speed, and rounding
/// adds a little either way; both are far below this.
pub const ENERGY_GAIN_LIMIT: f32 = 0.1;

/// What went wrong with a knocked object.
#[derive(Debug, Clone, PartialEq)]
pub enum Misbehaviour {
    /// A position or velocity became NaN or infinite.
    NonFinite,
    /// It had more energy than its knock gave it, by this many J/kg at most.
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
                    "gained {gain:.2} J/kg over its knock — as if it rose {:.0} cm on its own",
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
    /// Its energy per kilogram just after the knock.
    knocked: f32,
    /// The most it has had since, over `knocked`.
    gain: f32,
    /// Whether any of its bodies has been in the water.
    wet: bool,
    /// Whether any of its bodies has become non-finite.
    non_finite: bool,
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
                .map(|object| {
                    let knocked = energy_per_kg(&physics.world, object_handles(object));
                    Watch {
                        knocked,
                        gain: 0.0,
                        wet: false,
                        non_finite: false,
                    }
                })
                .collect()
        };
        let objects: Vec<Vec<RigidBodyHandle>> = self
            .objects()
            .iter()
            .map(|o| object_handles(o).collect())
            .collect();

        self.simulate(DISTURB_SECONDS, |world, water, _| {
            for (watch, handles) in watches.iter_mut().zip(&objects) {
                watch.observe(world, water, handles);
            }
            ControlFlow::Continue(())
        });

        let terrain = self.terrain();
        let physics = self.physics_mut();
        self.objects()
            .iter()
            .zip(&watches)
            .zip(&objects)
            .filter_map(|((object, watch), handles)| {
                let misbehaviours = watch.judge(&physics.world, &terrain, handles);
                (!misbehaviours.is_empty()).then(|| Disturbance {
                    label: object.label.clone(),
                    misbehaviours,
                })
            })
            .collect()
    }
}

impl Watch {
    fn observe(
        &mut self,
        world: &PhysicsWorld,
        water: Option<&dyn WaterSurface>,
        handles: &[RigidBodyHandle],
    ) {
        for body in handles.iter().filter_map(|&h| world.body(h)) {
            let position = body.position();
            self.non_finite |= !position.coords.iter().all(|c| c.is_finite())
                || !body.linear_velocity().iter().all(|c| c.is_finite())
                || !body.angular_velocity().iter().all(|c| c.is_finite());
            self.wet |= water
                .and_then(|w| w.sample(position))
                .is_some_and(|s| position.y < s.surface_level + AFLOAT_BAND);
        }
        if !self.wet && !self.non_finite {
            let energy = energy_per_kg(world, handles.iter().copied());
            self.gain = self.gain.max(energy - self.knocked);
        }
    }

    fn judge(
        &self,
        world: &PhysicsWorld,
        terrain: &TerrainWorld,
        handles: &[RigidBodyHandle],
    ) -> Vec<Misbehaviour> {
        if self.non_finite {
            return vec![Misbehaviour::NonFinite];
        }
        let mut misbehaviours = Vec::new();
        if !self.wet && self.gain > ENERGY_GAIN_LIMIT {
            misbehaviours.push(Misbehaviour::GainedEnergy(self.gain));
        }
        let bodies = handles.iter().filter_map(|&h| world.body(h));
        if bodies.clone().any(|body| {
            let p = body.position();
            !body.ignores_static() && terrain.is_mesh_solid_at(p.x, p.y, p.z)
        }) {
            misbehaviours.push(Misbehaviour::InTerrain);
        }
        misbehaviours
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
             stay finite, never gain energy, and end outside the terrain.",
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
