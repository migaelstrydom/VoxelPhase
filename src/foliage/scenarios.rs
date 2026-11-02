//! Loose clumps simulated for seconds on analytic ground: does a ball of
//! leaves land like one?
//!
//! What a clump should do, and what each scenario checks:
//!
//! - dropped, it lands dead: no rebound worth the name, and it is asleep
//!   within a few seconds;
//! - on a slope, it tumbles down and stops near the foot, where a ball of
//!   the same size and mass rolls on;
//! - kicked along flat ground, it skids and tumbles a short way and stops.
//!
//! The physics is the game's: its default world, stepped by its stepper at
//! 60 Hz frames.

use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

use super::leaf_core::LeafCore;
use super::shape::{FoliageShape, Lobe, ShrubSpec};
use crate::debug::DebugLines;
use crate::physics::bench_harness::geometry::{FlatQuadGeometry, RampGeometry};
use crate::physics::{
    PhysicsWorld, RigidBodyHandle, SequentialStepper, StaticGeometry, Stepper, SubstepForceProvider,
};

/// The game's frame and physics step.
const FRAME_DT: f32 = 1.0 / 60.0;
const PHYSICS_DT: f32 = 1.0 / 240.0;

/// What one run of a clump did.
#[derive(Debug)]
struct Tumble {
    /// Where the clump came to rest (or was when the run ended).
    rest: Point3<f32>,
    /// Seconds until it fell asleep, if it did.
    slept_at: Option<f32>,
    /// Highest it rose after first touching down, in metres above where it
    /// touched down.
    rebound: f32,
    /// Deepest any of its contacts went, in metres.
    deepest: f32,
    /// Fastest it spun, rad/s.
    fastest_spin: f32,
}

/// Run a clump of `shape` from `pose` with `velocity` for `seconds`.
fn tumble(
    shape: &FoliageShape,
    core: &LeafCore,
    ground: &dyn StaticGeometry,
    pose: Isometry3<f32>,
    velocity: Vector3<f32>,
    seconds: f32,
) -> Tumble {
    let mut world = PhysicsWorld::default();
    let body = core.spawn(&mut world, shape, pose);
    let mass = world.body(body).expect("just made").mass();
    world.apply_impulse(body, velocity * mass);
    run(&mut world, body, ground, seconds, &[])
}

fn run(
    world: &mut PhysicsWorld,
    body: RigidBodyHandle,
    ground: &dyn StaticGeometry,
    seconds: f32,
    providers: &[&dyn SubstepForceProvider],
) -> Tumble {
    let mut stepper = SequentialStepper::new(PHYSICS_DT, 12);
    let mut lines = DebugLines::default();
    let mut touched_at: Option<f32> = None;
    let mut out = Tumble {
        rest: Point3::origin(),
        slept_at: None,
        rebound: 0.0,
        deepest: 0.0,
        fastest_spin: 0.0,
    };
    let frames = (seconds / FRAME_DT).round() as usize;
    for frame in 0..frames {
        stepper.step(world, FRAME_DT, ground, &[], providers, &mut lines);
        lines.clear();
        let t = (frame + 1) as f32 * FRAME_DT;
        let b = world.body(body).expect("the clump is there");
        let y = b.position().y;
        out.fastest_spin = out.fastest_spin.max(b.angular_velocity().magnitude());
        let touching = world
            .contact_events()
            .iter()
            .filter(|c| c.body_b == body || c.body_a == Some(body))
            .inspect(|c| out.deepest = out.deepest.max(c.depth))
            .count()
            > 0;
        match touched_at {
            None if touching => touched_at = Some(y),
            Some(base) => out.rebound = out.rebound.max(y - base),
            None => {}
        }
        if out.slept_at.is_none() && world.is_sleeping(body) {
            out.slept_at = Some(t);
        }
        out.rest = b.position();
    }
    out
}

fn shrub() -> FoliageShape {
    ShrubSpec::with_radius(0.6).grow(11)
}

/// The shrub's leaves standing on the floor at `at`: its lowest leaf there.
fn standing(shape: &FoliageShape, at: Point3<f32>, height: f32) -> Isometry3<f32> {
    Isometry3::from_parts(
        Translation3::from(at.coords + Vector3::y() * (shape.depth_below() + height)),
        UnitQuaternion::identity(),
    )
}

/// Dropped from a metre, a clump lands dead and sleeps.
#[test]
fn a_dropped_clump_lands_dead_and_sleeps() {
    let shape = shrub();
    let ground = FlatQuadGeometry::new(20.0);
    let run = tumble(
        &shape,
        &LeafCore::default(),
        &ground,
        standing(&shape, Point3::origin(), 1.0),
        Vector3::zeros(),
        5.0,
    );
    println!("dropped: {run:?}");
    assert!(run.rebound < 0.05, "it bounced {:.3} m", run.rebound);
    let slept = run.slept_at.expect("it never slept");
    assert!(slept < 3.0, "it slept only at {slept:.2} s");
    let drift = Vector3::new(run.rest.x, 0.0, run.rest.z).magnitude();
    assert!(drift < 0.4, "it wandered {drift:.2} m from where it fell");
    assert!(
        run.deepest < 0.05,
        "a contact went {:.3} m deep",
        run.deepest
    );
}

/// The slope a clump is let go on, its foot at z = 0 and rising along +z,
/// and where on it the clump starts.
fn slope(degrees: f32) -> (RampGeometry, Point3<f32>) {
    let run = 8.0;
    let rise = run * degrees.to_radians().tan();
    let start = Point3::new(0.0, rise * 0.6, run * 0.6);
    (RampGeometry::new(6.0, run, rise), start)
}

/// How far past the foot of the slope a body came to rest, in metres along
/// the flat.
fn past_the_foot(rest: Point3<f32>) -> f32 {
    (-rest.z).max(0.0)
}

/// Let go on a 25° slope, a clump tumbles down and stops within a couple of
/// metres of the foot; a ball of the same mass and size rolls far further.
/// The ball is the check that the slope is steep enough to say anything.
#[test]
fn a_clump_tumbles_down_a_slope_and_stops_where_a_ball_rolls_on() {
    let shape = shrub();
    let core = LeafCore::default();
    let (ground, start) = slope(25.0);
    let clump = tumble(
        &shape,
        &core,
        &ground,
        standing(&shape, start, 0.3),
        Vector3::zeros(),
        8.0,
    );
    println!("clump on the slope: {clump:?}");

    let ball = FoliageShape {
        lobes: vec![Lobe {
            centre: Point3::origin(),
            radius: shape.bounding_radius() * 0.8,
        }],
    };
    let ball_core = LeafCore {
        bulk_density: core.mass(&shape) / ball.leaf_volume(),
        ..core
    };
    let rolled = tumble(
        &ball,
        &ball_core,
        &ground,
        standing(&ball, start, 0.3),
        Vector3::zeros(),
        8.0,
    );
    println!("ball on the slope: {rolled:?}");

    let stopped = past_the_foot(clump.rest);
    assert!(stopped < 2.0, "the clump ran {stopped:.2} m past the foot");
    assert!(clump.slept_at.is_some(), "the clump never slept");
    let ran = past_the_foot(rolled.rest);
    assert!(
        ran > stopped + 2.0,
        "the ball ran only {ran:.2} m past the foot"
    );
}

/// Kicked along flat ground at 4 m/s, a clump stops within a few metres.
#[test]
fn a_kicked_clump_stops_within_a_few_metres() {
    let shape = shrub();
    let ground = FlatQuadGeometry::new(30.0);
    let run = tumble(
        &shape,
        &LeafCore::default(),
        &ground,
        standing(&shape, Point3::origin(), 0.01),
        Vector3::new(4.0, 1.0, 0.0),
        6.0,
    );
    println!("kicked: {run:?}");
    assert!(run.rest.x < 3.5, "it went {:.2} m", run.rest.x);
    assert!(run.rest.x > 0.5, "it barely moved: {:.2} m", run.rest.x);
    assert!(run.slept_at.is_some(), "it never slept");
}

/// Let go on a 35° slope, steeper than most hillsides, a clump still stops
/// within a few metres of the foot.
#[test]
fn a_clump_stops_below_a_steep_slope() {
    let shape = shrub();
    let (ground, start) = slope(35.0);
    let run = tumble(
        &shape,
        &LeafCore::default(),
        &ground,
        standing(&shape, start, 0.3),
        Vector3::zeros(),
        10.0,
    );
    println!("steep: {run:?}");
    let stopped = past_the_foot(run.rest);
    assert!(stopped < 3.5, "it ran {stopped:.2} m past the foot");
    assert!(run.slept_at.is_some(), "it never slept");
}
