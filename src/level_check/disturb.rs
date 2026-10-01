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
//! Energy is judged by an [`EnergyAudit`]: over each group of objects that
//! touched during the trial, since one object's knock may be what sets
//! another moving, and less the work any body the trial does not judge — a
//! driven platform, a creature — did on the group through a contact. A group
//! that went into the water, where buoyancy lifts it, is not judged on energy.
//! A body that passes through static geometry on purpose, bedded in the
//! ground, may end inside it.

use std::f32::consts::PI;
use std::ops::ControlFlow;

use nalgebra::Vector3;

use crate::physics::{EnergyAudit, PhysicsWorld, RigidBodyHandle};
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

impl RestTrial {
    /// Knock every body of a settled trial and report every object that
    /// then gains energy, becomes non-finite or ends in the terrain. Run it
    /// after [`RestTrial::run`].
    pub fn disturb(&mut self) -> Vec<Disturbance> {
        let objects: Vec<Vec<RigidBodyHandle>> = self
            .objects()
            .iter()
            .map(|object| object_handles(object).collect())
            .collect();
        let mut audit = {
            let mut physics = self.physics_mut();
            for (index, handle) in self.handles().into_iter().enumerate() {
                knock(&mut physics.world, handle, index);
            }
            EnergyAudit::begin(&mut physics.world, objects.clone())
        };

        self.simulate(DISTURB_SECONDS, |world, water, _| {
            audit.observe(world);
            for (object, handles) in objects.iter().enumerate() {
                if is_wet(world, water, handles) {
                    audit.exempt(object);
                }
            }
            ControlFlow::Continue(())
        });

        let gains = audit.gains();
        let terrain = self.terrain();
        let physics = self.physics_mut();
        self.objects()
            .iter()
            .zip(&objects)
            .zip(gains)
            .enumerate()
            .filter_map(|(index, ((object, handles), gain))| {
                let misbehaviours = if audit.is_non_finite(index) {
                    vec![Misbehaviour::NonFinite]
                } else {
                    judge(&physics.world, &terrain, handles, gain)
                };
                (!misbehaviours.is_empty()).then(|| Disturbance {
                    label: object.label.clone(),
                    misbehaviours,
                })
            })
            .collect()
    }
}

/// Whether any of the bodies is in the water, where buoyancy lifts it.
fn is_wet(
    world: &PhysicsWorld,
    water: Option<&dyn WaterSurface>,
    handles: &[RigidBodyHandle],
) -> bool {
    handles.iter().filter_map(|&h| world.body(h)).any(|body| {
        let position = body.position();
        water
            .and_then(|w| w.sample(position))
            .is_some_and(|s| position.y < s.surface_level + AFLOAT_BAND)
    })
}

/// What an object did wrong, given how much energy per kilogram its group
/// regained, if its group was judged on energy.
fn judge(
    world: &PhysicsWorld,
    terrain: &TerrainWorld,
    handles: &[RigidBodyHandle],
    gain: Option<f32>,
) -> Vec<Misbehaviour> {
    let mut misbehaviours = Vec::new();
    if let Some(gain) = gain.filter(|&g| g > ENERGY_GAIN_LIMIT) {
        misbehaviours.push(Misbehaviour::GainedEnergy(gain));
    }
    if handles.iter().filter_map(|&h| world.body(h)).any(|body| {
        let p = body.position();
        !body.ignores_static() && terrain.is_mesh_solid_at(p.x, p.y, p.z)
    }) {
        misbehaviours.push(Misbehaviour::InTerrain);
    }
    misbehaviours
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
}
