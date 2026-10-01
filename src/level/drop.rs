//! Dropping a freshly spawned object straight down onto whatever is below it.
//!
//! ```text
//!   spawnable ──spawn──▶ bodies at the authored pose
//!                              │
//!         one body? resting_turn: the least turn onto a face it rests on
//!                              │
//!            BodySweep (terrain + every body already placed), straight down
//!                              │
//!                 first touch ──▶ every body (and its ECS Position) moved
//!                                 down by the same distance
//!                              │
//!         under water and buoyant? raised to where lift equals weight,
//!                                      or to the first thing above it
//! ```
//!
//! The object falls as one rigid group with its orientation held, so it stops
//! where its real colliders first touch rather than where some estimate of its
//! base says they should. That is the point of doing this with the physics
//! world and not with a footprint: a die on a temple floor, a crate on a
//! crate, a ball under a bridge all land on what is actually there.
//!
//! An object of one body is first turned onto the face it can rest on that
//! is nearest below it, so an octahedron built point-down lands on a face and
//! a square crate is not turned at all. An object of many bodies is a
//! structure with a base of its own and keeps its authored pose.
//!
//! Water is not something to land on, so a drop falls through it to the
//! bed. An object the water holds up is then raised to its draft: the height
//! at which the water's lift on it equals its weight. A crate dropped into a
//! pond floats at the depth it would float at, and a gold cube stays on the
//! bottom.
//!
//! A drop only places. It does not settle: an object dropped onto a slope, or
//! onto something narrower than itself, still tips or slides once woken, and
//! the rest lint says so.

use std::fmt;

use nalgebra::Vector3;
use rustc_hash::FxHashSet;
use specs::{Join, World, WorldExt};

use crate::components::{Orientation, Position, RigidBodyComponent};
use crate::physics::{
    resting_turn, BodySweep, PhysicsWorld, RigidBodyHandle, SweepHit, SweepObstacle,
};
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;
use crate::water::buoyancy::{lift, WaterSurface};
use crate::water::WaterWorld;

/// How far below its authored point a dropped object looks for something to
/// land on. Far enough for any drop an author means; a longer one is a typo.
const DROP_REACH: f32 = 64.0;

/// A drop longer than this lands, but is reported: an author placing a die on
/// a floor starts it a little above, not a storey above.
pub const LONG_DROP: f32 = 8.0;

/// Gravity the water's lift is weighed against, m/s², as `buoyancy::lift`
/// takes it.
const GRAVITY: f32 = 9.81;

/// Halvings of the search for a floater's draft: from a metre-scale bracket,
/// well under a millimetre.
const DRAFT_SEARCH_STEPS: usize = 24;

/// What dropping an object did.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DropOutcome {
    /// Fell `distance` metres and came to touch `on`.
    Landed { distance: f32, on: SweepObstacle },
    /// Fell `distance` metres, net, into water that holds it up, and floats
    /// at its draft.
    Afloat { distance: f32 },
    /// Already touching or inside `on` where it was authored, so it was left
    /// there. Raising the authored point is the fix.
    StartedInContact { on: SweepObstacle },
    /// Nothing within [`DROP_REACH`] below; left where it was authored.
    NothingBelow,
    /// The object built no bodies, so there is nothing to drop.
    NoBodies,
}

impl DropOutcome {
    /// What is wrong with this drop, if anything, phrased to follow the
    /// object's name in a report.
    pub fn problem(&self) -> Option<String> {
        match *self {
            DropOutcome::Landed { distance, on } if distance > LONG_DROP => Some(format!(
                "falls {distance:.1} m before it lands on {}; start it closer",
                Obstacle(on)
            )),
            DropOutcome::Landed { .. } | DropOutcome::Afloat { .. } => None,
            DropOutcome::StartedInContact { on } => Some(format!(
                "is Dropped but already touches {} where its fall starts, so it was not \
                 moved; start it higher",
                Obstacle(on)
            )),
            DropOutcome::NothingBelow => Some(format!(
                "is Dropped but has nothing within {DROP_REACH:.0} m below it"
            )),
            DropOutcome::NoBodies => {
                Some("is Dropped but has no body to drop; place it explicitly".to_string())
            }
        }
    }
}

/// Names the obstacle a drop met, for messages.
struct Obstacle(SweepObstacle);

impl fmt::Display for Obstacle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            SweepObstacle::Static => write!(f, "the terrain"),
            SweepObstacle::Body(_) => write!(f, "another object"),
        }
    }
}

/// Every body handle in the world now, so the bodies a spawn adds can be told
/// apart from those that were there before it.
pub fn body_handles(physics: &PhysicsWorld) -> FxHashSet<RigidBodyHandle> {
    physics
        .bodies()
        .iter()
        .map(|(index, _)| RigidBodyHandle(index))
        .collect()
}

/// Drop `bodies` straight down as one group onto the terrain and every other
/// body in `world`, and move them, and their entities' positions, there.
pub fn drop_onto_below(world: &mut World, bodies: &[RigidBodyHandle]) -> DropOutcome {
    if bodies.is_empty() {
        return DropOutcome::NoBodies;
    }
    if let [body] = bodies {
        turn_onto_resting_face(world, *body);
    }

    let Some(hit) = sweep(world, bodies, Vector3::new(0.0, -DROP_REACH, 0.0)) else {
        return DropOutcome::NothingBelow;
    };
    if hit.fraction <= 0.0 {
        return DropOutcome::StartedInContact { on: hit.obstacle };
    }

    let fall = Vector3::new(0.0, -hit.distance, 0.0);
    shift_bodies(
        &mut world.write_resource::<PhysicsResource>().world,
        bodies,
        fall,
    );
    let draft = match world.try_fetch::<WaterWorld>() {
        Some(water) => rise_to_draft(
            &mut world.write_resource::<PhysicsResource>().world,
            bodies,
            &water.query(),
        ),
        None => 0.0,
    };
    // A floater under a roof — a flooded tunnel, a pond under a bridge —
    // rises only until it meets it.
    let rise = match sweep(world, bodies, Vector3::new(0.0, draft, 0.0)) {
        Some(roof) if draft > 0.0 => roof.distance,
        _ => draft,
    };
    let lift = Vector3::new(0.0, rise, 0.0);
    shift_bodies(
        &mut world.write_resource::<PhysicsResource>().world,
        bodies,
        lift,
    );
    shift_positions(world, bodies, fall + lift);

    if rise > 0.0 {
        DropOutcome::Afloat {
            distance: hit.distance - rise,
        }
    } else {
        DropOutcome::Landed {
            distance: hit.distance,
            on: hit.obstacle,
        }
    }
}

/// How far `bodies` must rise for the lift of `water` on them to equal their
/// weight: zero where the water cannot hold them up. Leaves them where they
/// are.
fn rise_to_draft(
    physics: &mut PhysicsWorld,
    bodies: &[RigidBodyHandle],
    water: &dyn WaterSurface,
) -> f32 {
    let weight: f32 = bodies
        .iter()
        .filter_map(|&h| physics.body(h))
        .filter(|b| b.is_dynamic())
        .map(|b| b.mass() * GRAVITY)
        .sum();
    // Lift exceeds weight `up` metres above where the bodies are now.
    let buoyant_at = |physics: &mut PhysicsWorld, up: f32| {
        shift_bodies(physics, bodies, Vector3::new(0.0, up, 0.0));
        let lifted: f32 = bodies.iter().map(|&h| lift(physics, h, water)).sum();
        shift_bodies(physics, bodies, Vector3::new(0.0, -up, 0.0));
        lifted > weight
    };

    if !buoyant_at(physics, 0.0) {
        return 0.0;
    }
    // Lift falls as a body rises out of the water, so the draft is bracketed
    // once a height is found where the water no longer holds it up.
    let mut low = 0.0;
    let mut high = 0.25;
    while buoyant_at(physics, high) {
        low = high;
        high *= 2.0;
        if high > DROP_REACH {
            return 0.0;
        }
    }
    for _ in 0..DRAFT_SEARCH_STEPS {
        let mid = 0.5 * (low + high);
        if buoyant_at(physics, mid) {
            low = mid;
        } else {
            high = mid;
        }
    }
    0.5 * (low + high)
}

/// The first thing `bodies` touch moving together by `travel`, against the
/// terrain and every other body in `world`.
fn sweep(world: &World, bodies: &[RigidBodyHandle], travel: Vector3<f32>) -> Option<SweepHit> {
    let physics = world.read_resource::<PhysicsResource>();
    let terrain = world.try_fetch::<TerrainWorld>();
    let mut sweep = BodySweep::new(&physics.world);
    if let Some(terrain) = terrain.as_deref() {
        sweep = sweep.against_static(terrain);
    }
    sweep.cast(bodies, travel)
}

/// Turn a lone body about its centre of mass onto the face it rests on, and
/// its entity's orientation with it.
fn turn_onto_resting_face(world: &mut World, body: RigidBodyHandle) {
    let turned = {
        let mut physics = world.write_resource::<PhysicsResource>();
        let Some(turn) = resting_turn(&physics.world, body, -Vector3::y_axis()) else {
            return;
        };
        let Some(rigid) = physics.world.body_mut(body) else {
            return;
        };
        let turned = turn * rigid.rotation();
        rigid.set_rotation(turned);
        turned
    };

    let owners = world.read_storage::<RigidBodyComponent>();
    let mut orientations = world.write_storage::<Orientation>();
    for (owner, orientation) in (&owners, &mut orientations).join() {
        if owner.0 == body {
            orientation.0 = turned;
        }
    }
}

/// Move `bodies` by `delta`.
fn shift_bodies(physics: &mut PhysicsWorld, bodies: &[RigidBodyHandle], delta: Vector3<f32>) {
    for &handle in bodies {
        if let Some(body) = physics.body_mut(handle) {
            let moved = body.position() + delta;
            body.set_position(moved);
        }
    }
}

/// Move the `Position` of every entity that owns one of `bodies` by `delta`,
/// so the object is drawn where its bodies are before the first physics sync.
fn shift_positions(world: &World, bodies: &[RigidBodyHandle], delta: Vector3<f32>) {
    let owners = world.read_storage::<RigidBodyComponent>();
    let mut positions = world.write_storage::<Position>();
    for (owner, position) in (&owners, &mut positions).join() {
        if bodies.contains(&owner.0) {
            position.0 += delta;
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;
    use crate::physics::{ColliderDesc, PhysicsConfig, RigidBodyDesc};
    use crate::water::buoyancy::StillWater;

    /// A metre cube of `density` lying on a pool floor at y = 0.
    fn cube_on_the_floor(density: f32) -> (PhysicsWorld, RigidBodyHandle) {
        let mut physics = PhysicsWorld::new(PhysicsConfig::default());
        let body =
            physics.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, 0.5, 0.0)));
        physics.attach_collider(
            body,
            ColliderDesc::box_shape(Vector3::new(0.5, 0.5, 0.5)).density(density),
        );
        (physics, body)
    }

    const POOL: StillWater = StillWater {
        surface: 3.0,
        floor: 0.0,
    };

    /// Half as dense as water: it floats half under, its centre on the surface.
    #[test]
    fn a_floater_rises_to_its_draft() {
        let (mut physics, body) = cube_on_the_floor(500.0);
        let rise = rise_to_draft(&mut physics, &[body], &POOL);
        let y = physics.body(body).unwrap().position().y + rise;
        assert!((y - 3.0).abs() < 0.01, "centre would float at {y:.3}");
    }

    #[test]
    fn a_sinker_stays_on_the_bottom() {
        let (mut physics, body) = cube_on_the_floor(2000.0);
        assert_eq!(rise_to_draft(&mut physics, &[body], &POOL), 0.0);
        assert_eq!(physics.body(body).unwrap().position().y, 0.5);
    }
}
