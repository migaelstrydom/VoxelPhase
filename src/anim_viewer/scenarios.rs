//! The catalogue of things worth watching the character do.
//!
//! Organised as a cross product rather than a list: a handful of gaits, a
//! handful of grounds, and the transitions between gaits. Everything a scenario
//! varies is one of those three, so a name reads as "which gait, over what".
//!
//! Scenarios all travel along ±x with the ground varying only in x, which is
//! not laziness — it lets every shot use the same side-on camera, so two
//! filmstrips can be compared without first working out where each was standing.

use nalgebra::Vector3;

use super::ground::{Drop, Flat, Ground, Ledge, Slope, SlopeOnset, Stairs, Undulating};
use super::script::{Beat, Script};
use super::support::SupportMotion;

/// One named thing to look at.
pub struct Scenario {
    pub name: &'static str,
    /// One line saying what this scenario is for. Printed by `--list`.
    pub description: &'static str,
    pub ground: Box<dyn Ground>,
    pub script: Script,
    /// Where the character starts, in x and z.
    pub start: (f32, f32),
    /// Walk speed for this scenario, when the game's own would make it
    /// meaningless. `None` uses `LocomotionConfig::player`.
    pub speed: Option<f32>,
    /// How the ground moves under the character. `Still` for terrain.
    pub support: SupportMotion,
}

/// Every scenario, in the order they are worth reading.
pub fn catalogue() -> Vec<Scenario> {
    let mut out = steady_gaits();
    out.extend(transitions());
    out.extend(slopes());
    out.extend(broken_ground());
    out.extend(moving_ground());
    out
}

pub fn find(name: &str) -> Option<Scenario> {
    catalogue().into_iter().find(|s| s.name == name)
}

/// Scenarios matching a name, or every scenario for "all".
pub fn select(name: &str) -> Vec<Scenario> {
    if name == "all" {
        return catalogue();
    }
    // An exact name wins outright: with `walk`, `walk_up_steep` and friends in
    // the catalogue, a substring sweep would hide the one that was asked for.
    if let Some(exact) = find(name) {
        return vec![exact];
    }
    catalogue()
        .into_iter()
        .filter(|s| s.name.contains(name))
        .collect()
}

/// One gait held long enough to settle into a rhythm. The baseline: a defect
/// visible here needs no terrain to explain it.
fn steady_gaits() -> Vec<Scenario> {
    vec![
        Scenario {
            name: "stand",
            description: "Standing still. Nothing should move, and the feet should not shuffle.",
            ground: Box::new(Flat::at(0.0)),
            script: Script::new().then(Beat::stand(3.0)),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::Still,
        },
        Scenario {
            name: "walk",
            description: "A steady walk on flat ground — the control case for every other run.",
            ground: Box::new(Flat::at(0.0)),
            script: Script::new().then(Beat::stand(0.5)).then(Beat::walk(5.0)),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::Still,
        },
        Scenario {
            name: "run",
            description: "A steady sprint on flat ground; expect a flight phase and short stances.",
            ground: Box::new(Flat::at(0.0)),
            script: Script::new().then(Beat::stand(0.5)).then(Beat::run(5.0)),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::Still,
        },
        Scenario {
            name: "crouch_walk",
            description: "Crouch-walking on flat ground: low, slow, and short-strided.",
            ground: Box::new(Flat::at(0.0)),
            script: Script::new()
                .then(Beat::stand(0.5))
                .then(Beat::crouch_walk(5.0)),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::Still,
        },
        Scenario {
            name: "turn",
            description: "Walking a circle. Turning is where a planted foot has to give up its \
                          anchor for a reason other than distance.",
            ground: Box::new(Flat::at(0.0)),
            script: Script::new()
                .then(Beat::stand(0.5))
                .then(Beat::walk(6.0).turning(1.0)),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::Still,
        },
    ]
}

/// The edges between gaits. Each holds both sides long enough to settle, so the
/// report can separate "the transition is bad" from "the destination is bad".
fn transitions() -> Vec<Scenario> {
    vec![
        Scenario {
            name: "start_stop",
            description:
                "Stand, walk, stand. The first step out of rest and the last step into it.",
            ground: Box::new(Flat::at(0.0)),
            script: Script::new()
                .then(Beat::stand(1.0))
                .then(Beat::walk(3.0))
                .then(Beat::stand(2.0)),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::Still,
        },
        Scenario {
            name: "walk_to_run",
            description: "Walk, then sprint, then walk again — cadence has to change twice.",
            ground: Box::new(Flat::at(0.0)),
            script: Script::new()
                .then(Beat::walk(2.5))
                .then(Beat::run(3.0))
                .then(Beat::walk(2.5)),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::Still,
        },
        Scenario {
            name: "walk_to_crouch",
            description: "Walk, drop into a crouch-walk, stand back up.",
            ground: Box::new(Flat::at(0.0)),
            script: Script::new()
                .then(Beat::walk(2.5))
                .then(Beat::crouch_walk(3.0))
                .then(Beat::walk(2.5)),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::Still,
        },
        Scenario {
            name: "crouch_to_run",
            description: "The widest gait change there is: crouch-walk straight into a sprint.",
            ground: Box::new(Flat::at(0.0)),
            script: Script::new()
                .then(Beat::crouch_walk(2.5))
                .then(Beat::run(3.0)),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::Still,
        },
        Scenario {
            name: "jump",
            description: "Walk, jump, land, walk on. Covers takeoff, suspension and the landing \
                          splice in one take.",
            ground: Box::new(Flat::at(0.0)),
            script: Script::new()
                .then(Beat::walk(1.5))
                .then(Beat::jump(0.1))
                .then(Beat::walk(3.0)),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::Still,
        },
        Scenario {
            name: "fall",
            description: "Walking off a ledge into nothing: the placer suspends and never resumes.",
            ground: Box::new(Ledge::new("ledge", 4.0)),
            script: Script::new().then(Beat::stand(0.3)).then(Beat::walk(3.0)),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::Still,
        },
        Scenario {
            name: "step_down",
            description: "Walking off a knee-high drop and landing on the far side.",
            ground: Box::new(Drop::new("drop", 4.0, 0.6)),
            script: Script::new().then(Beat::stand(0.3)).then(Beat::walk(3.5)),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::Still,
        },
    ]
}

/// The same gaits on inclines. Slope is where the leg's reach budget shrinks,
/// so it is where a gait that is already close to its limit tips over it.
fn slopes() -> Vec<Scenario> {
    // A slope rises in +x, so which way the character walks decides which way
    // the ground goes under it.
    let uphill = || Beat::walk(5.0).towards(Vector3::x()).named("uphill");
    let downhill = || Beat::walk(5.0).towards(-Vector3::x()).named("downhill");

    vec![
        Scenario {
            name: "walk_up_gentle",
            description: "Walking up a 10° slope.",
            ground: Box::new(Slope::degrees("up10", 10.0)),
            script: Script::new().then(Beat::stand(0.5)).then(uphill()),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::Still,
        },
        Scenario {
            name: "walk_down_gentle",
            description: "Walking down a 10° slope.",
            ground: Box::new(Slope::degrees("up10", 10.0)),
            script: Script::new().then(Beat::stand(0.5)).then(downhill()),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::Still,
        },
        Scenario {
            name: "walk_up_steep",
            description: "Walking up a 25° slope — near the limit of the leg's reach budget.",
            ground: Box::new(Slope::degrees("up25", 25.0)),
            script: Script::new().then(Beat::stand(0.5)).then(uphill()),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::Still,
        },
        Scenario {
            name: "walk_down_steep",
            description: "Walking down a 25° slope, where the downhill foot has furthest to reach.",
            ground: Box::new(Slope::degrees("up25", 25.0)),
            script: Script::new().then(Beat::stand(0.5)).then(downhill()),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::Still,
        },
        Scenario {
            name: "run_up_gentle",
            description: "Sprinting up a 10° slope: the shortest reach budget meets the longest \
                          stride.",
            ground: Box::new(Slope::degrees("up10", 10.0)),
            script: Script::new().then(Beat::stand(0.5)).then(Beat::run(5.0)),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::Still,
        },
        Scenario {
            name: "onto_slope",
            description: "Flat ground turning into a 20° climb mid-stride.",
            ground: Box::new(SlopeOnset::degrees("onset20", 0.0, 20.0)),
            script: Script::new().then(Beat::stand(0.4)).then(Beat::walk(4.0)),
            start: (-4.0, 0.0),
            speed: None,
            support: SupportMotion::Still,
        },
    ]
}

/// Ground that is not a plane at all.
fn broken_ground() -> Vec<Scenario> {
    vec![
        Scenario {
            name: "stairs",
            description: "Climbing 18 cm steps. The surface under a landing target is \
                          discontinuous, so a step that lands short lands a whole rise low.",
            ground: Box::new(Stairs::new("stairs", 0.0, 0.32, 0.18)),
            script: Script::new().then(Beat::stand(0.4)).then(Beat::walk(4.0)),
            start: (-3.0, 0.0),
            // At the game's own 5 m/s this is fifteen treads a second, which is
            // not a gait anything will ever have to animate.
            speed: Some(1.6),
            support: SupportMotion::Still,
        },
        Scenario {
            name: "rolling",
            description: "Gently rolling ground: nothing steep, but the height under a planted \
                          foot is never constant.",
            ground: Box::new(Undulating::new("rolling", 0.12, 2.5)),
            script: Script::new().then(Beat::stand(0.4)).then(Beat::walk(5.0)),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::Still,
        },
        Scenario {
            name: "ledge_edge",
            description: "Walking to the lip of a ledge and stopping on it, so the outside foot \
                          keeps probing for ground that isn't there.",
            ground: Box::new(Ledge::new("ledge", 3.0)),
            script: Script::new().then(Beat::walk(0.62)).then(Beat::stand(2.0)),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::Still,
        },
    ]
}

/// Ground that is going somewhere itself.
///
/// The one class of scenario where world space is the wrong frame to think in.
/// A planted foot is a promise about a place on the *floor*, and while the
/// floor holds still the two are the same promise — these are the runs that
/// tell them apart. `plant-slip` is measured on the surface, so a foot that
/// holds a world position while the platform slides out from under it reads as
/// a skate of exactly the distance the platform travelled.
fn moving_ground() -> Vec<Scenario> {
    let platform = |speed: f32| SupportMotion::Steady(Vector3::new(speed, 0.0, 0.0));

    vec![
        Scenario {
            name: "platform_ride",
            description: "Standing still on a platform running at 3 m/s. The character is \
                          motionless relative to the floor, so nothing should step.",
            ground: Box::new(Flat::at(0.0)),
            script: Script::new().then(Beat::stand(4.0)),
            start: (0.0, 0.0),
            speed: None,
            support: platform(3.0),
        },
        Scenario {
            name: "platform_walk",
            description: "Walking along a platform that is itself moving. The gait is the same \
                          walk; only the frame it is walked in has changed.",
            ground: Box::new(Flat::at(0.0)),
            script: Script::new().then(Beat::stand(0.5)).then(Beat::walk(4.0)),
            start: (0.0, 0.0),
            speed: None,
            support: platform(2.5),
        },
        Scenario {
            name: "platform_against",
            description: "Walking against the platform's travel, so the body is nearly still in \
                          the world while the feet are doing a full walk.",
            ground: Box::new(Flat::at(0.0)),
            script: Script::new().then(Beat::stand(0.5)).then(Beat::walk(4.0)),
            start: (0.0, 0.0),
            speed: Some(2.5),
            support: platform(-2.5),
        },
        Scenario {
            name: "lift",
            description: "Standing on a rising platform. The feet have to track a floor that is \
                          climbing without ever deciding to step.",
            ground: Box::new(Flat::at(0.0)),
            script: Script::new().then(Beat::stand(3.0)),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::Steady(Vector3::new(0.0, 0.8, 0.0)),
        },
        Scenario {
            name: "collapse",
            description: "Standing on a crate that is knocked out from under the character after \
                          a second. The feet have to let go of a floor that has left.",
            ground: Box::new(Flat::at(0.0)),
            script: Script::new().then(Beat::stand(2.5)),
            start: (0.0, 0.0),
            speed: None,
            support: SupportMotion::GivesWay {
                at: 1.0,
                velocity: Vector3::new(-1.5, -6.0, 0.0),
            },
        },
        Scenario {
            name: "collapse_walking",
            description: "The same crate, kicked away mid-stride: one foot is planted on it and \
                          the other is mid-swing when it goes.",
            ground: Box::new(Flat::at(0.0)),
            script: Script::new().then(Beat::stand(0.5)).then(Beat::walk(2.0)),
            start: (0.0, 0.0),
            speed: Some(2.0),
            support: SupportMotion::GivesWay {
                at: 1.2,
                velocity: Vector3::new(-1.5, -6.0, 0.0),
            },
        },
    ]
}
