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

use std::sync::Arc;

use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

use super::drag::{DragVolume, FoliageDrag, LeafDrag, VolumeFrame};
use super::leaf_core::LeafCore;
use super::shape::{FoliageShape, Lobe, ShrubSpec};
use crate::collision_layers;
use crate::debug::DebugLines;
use crate::explosion::Explosion;
use crate::physics::bench_harness::geometry::{EmptyGeometry, FlatQuadGeometry, RampGeometry};
use crate::physics::{
    Atmosphere, ColliderDesc, CollisionFilter, PhysicsConfig, PhysicsWorld, RigidBodyDesc,
    RigidBodyHandle, SequentialStepper, StaticGeometry, Stepper,
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
    run(
        &mut world,
        body,
        Arc::new(shape.clone()),
        ground,
        seconds,
        LeafDrag::default(),
    )
}

/// Step a clump of `shape` on `body` for `seconds`, its leaves dragged by
/// the air as the game's are.
fn run(
    world: &mut PhysicsWorld,
    body: RigidBodyHandle,
    shape: Arc<FoliageShape>,
    ground: &dyn StaticGeometry,
    seconds: f32,
    leaf_drag: LeafDrag,
) -> Tumble {
    let volumes = vec![DragVolume {
        shape,
        frame: VolumeFrame::Body(body),
    }];
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
        let drag = FoliageDrag::new(leaf_drag, volumes.clone(), world, [], FRAME_DT);
        stepper.step(world, FRAME_DT, ground, &[], &[&drag], &mut lines);
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
        lobes: vec![Lobe::round(Point3::origin(), shape.bounding_radius() * 0.8)],
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

/// A body thrown along +x at shrub height through `volumes`, unpowered:
/// its speed when it has crossed `x_exit`.
fn speed_through(volumes: &[DragVolume], collider: ColliderDesc, speed: f32, x_exit: f32) -> f32 {
    let mut world = PhysicsWorld::default();
    let body = world.create_body(
        RigidBodyDesc::dynamic()
            .position(Point3::new(-2.0, 0.0, 0.0))
            .gravity_scale(0.0),
    );
    world.attach_collider(body, collider);
    let mass = world.body(body).expect("just made").mass();
    world.apply_impulse(body, Vector3::x() * speed * mass);
    let ground = EmptyGeometry;
    let mut stepper = SequentialStepper::new(PHYSICS_DT, 12);
    let mut lines = DebugLines::default();
    for _ in 0..600 {
        let drag = FoliageDrag::new(
            LeafDrag::default(),
            volumes.to_vec(),
            &world,
            [body],
            FRAME_DT,
        );
        stepper.step(&mut world, FRAME_DT, &ground, &[], &[&drag], &mut lines);
        lines.clear();
        let b = world.body(body).expect("there");
        if b.position().x > x_exit {
            return b.linear_velocity().magnitude();
        }
    }
    panic!("the body never got through");
}

/// A rooted shrub at the origin.
fn rooted_shrub() -> Vec<DragVolume> {
    vec![DragVolume {
        shape: Arc::new(shrub()),
        frame: VolumeFrame::Fixed(Isometry3::identity()),
    }]
}

/// A grenade thrown through a shrub comes out slowed but still going; one
/// thrown past nothing keeps its speed.
#[test]
fn a_grenade_through_a_shrub_comes_out_slowed() {
    let grenade = || ColliderDesc::sphere(0.2).density(2000.0);
    let clear = speed_through(&[], grenade(), 12.0, 2.0);
    let through = speed_through(&rooted_shrub(), grenade(), 12.0, 2.0);
    println!("grenade at 12 m/s: {clear:.2} m/s past nothing, {through:.2} m/s through a shrub");
    assert!(
        (clear - 12.0).abs() < 0.05,
        "air alone slowed it to {clear:.2}"
    );
    assert!(through < 11.0, "the shrub barely slowed it: {through:.2}");
    assert!(through > 6.0, "the shrub nearly stopped it: {through:.2}");
}

/// A grenade thrown through a loose clump passes through its core, which
/// stands for leaves, not wood: it comes out slowed by them, as through a
/// rooted shrub, and the clump is left where it was. A ball the core does
/// stop is stopped.
#[test]
fn a_grenade_passes_through_a_loose_clump() {
    let thrown = |filter: CollisionFilter| {
        let mut world = PhysicsWorld::default();
        let shape = Arc::new(shrub());
        let core = LeafCore::default().spawn(&mut world, &shape, Isometry3::identity());
        let volumes = vec![DragVolume {
            shape: shape.clone(),
            frame: VolumeFrame::Body(core),
        }];
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(-2.0, 0.0, 0.0))
                .gravity_scale(0.0)
                .linear_velocity(Vector3::x() * 12.0)
                .filter(filter),
        );
        world.attach_collider(body, ColliderDesc::sphere(0.2).density(2000.0));
        world.wake_body(body);
        let mut stepper = SequentialStepper::new(PHYSICS_DT, 12);
        let mut lines = DebugLines::default();
        for _ in 0..60 {
            let drag = FoliageDrag::new(
                LeafDrag::default(),
                volumes.clone(),
                &world,
                [body],
                FRAME_DT,
            );
            stepper.step(
                &mut world,
                FRAME_DT,
                &EmptyGeometry,
                &[],
                &[&drag],
                &mut lines,
            );
            lines.clear();
        }
        let b = world.body(body).expect("there");
        let clump = world.body(core).expect("there").position();
        (
            b.position().x,
            b.linear_velocity().x,
            clump.coords.magnitude(),
        )
    };
    let (x, speed, clump_moved) = thrown(collision_layers::GRENADE);
    println!("grenade: at x {x:.2}, {speed:.2} m/s; clump moved {clump_moved:.3} m");
    assert!(x > 2.0, "it did not get through: x {x:.2}");
    assert!(speed < 11.5, "the leaves did not slow it: {speed:.2}");
    assert!(
        clump_moved < 1e-3,
        "the clump was knocked {clump_moved:.3} m"
    );
    let (x, _, clump_moved) = thrown(CollisionFilter::DEFAULT);
    println!("ball: at x {x:.2}; clump moved {clump_moved:.3} m");
    assert!(
        x < 1.0 || clump_moved > 0.05,
        "a ball the core meets went through untouched"
    );
}

/// The player's capsule, coasting at walking pace into a shrub, is slowed
/// more than a grenade is: it is far lighter for its size.
#[test]
fn a_coasting_body_wades_into_a_shrub() {
    let capsule = || ColliderDesc::capsule(0.5, 0.25).density(56.0 / capsule_volume(0.5, 0.25));
    let clear = speed_through(&[], capsule(), 4.0, 0.0);
    let through = speed_through(&rooted_shrub(), capsule(), 4.0, 0.0);
    println!(
        "capsule at 4 m/s: {clear:.2} m/s past nothing, {through:.2} m/s at the shrub's middle"
    );
    assert!(
        through < 0.8 * clear,
        "it waded through at {through:.2} m/s"
    );
    assert!(
        through > 0.2 * clear,
        "it was all but stopped: {through:.2} m/s"
    );
}

/// A body coasting from `from` at `velocity` among `volumes`, weightless and
/// in vacuum, with every body in the world dragged by the leaves: the world
/// after `seconds`.
fn coast(
    mut world: PhysicsWorld,
    volumes: &[DragVolume],
    body: RigidBodyHandle,
    seconds: f32,
) -> PhysicsWorld {
    let mut stepper = SequentialStepper::new(PHYSICS_DT, 12);
    let mut lines = DebugLines::default();
    let bodies: Vec<RigidBodyHandle> = volumes
        .iter()
        .filter_map(|v| match v.frame {
            VolumeFrame::Body(core) => Some(core),
            VolumeFrame::Fixed(_) => None,
        })
        .chain([body])
        .collect();
    for _ in 0..(seconds / FRAME_DT) as usize {
        let drag = FoliageDrag::new(
            LeafDrag::default(),
            volumes.to_vec(),
            &world,
            bodies.iter().copied(),
            FRAME_DT,
        );
        stepper.step(
            &mut world,
            FRAME_DT,
            &EmptyGeometry,
            &[],
            &[&drag],
            &mut lines,
        );
        lines.clear();
    }
    world
}

/// A world with no gravity and no air.
fn still_space() -> PhysicsWorld {
    PhysicsWorld::new(PhysicsConfig {
        gravity: Vector3::zeros(),
        atmosphere: Atmosphere::VACUUM,
        ..PhysicsConfig::default()
    })
}

/// A plank sliding past a shrub, clear of its leaves, is not slowed by
/// them, though a ball as wide as the plank is long would reach deep into
/// them: leaves drag a body by its real shape.
#[test]
fn a_plank_sliding_past_a_shrub_is_not_dragged() {
    let reach = shrub()
        .lobes
        .iter()
        .map(|l| l.centre.coords.x.abs() + l.greatest_radius())
        .fold(0.0, f32::max);
    let mut world = still_space();
    let plank = world.create_body(
        RigidBodyDesc::dynamic()
            .position(Point3::new(-(reach + 0.15), 0.0, -3.0))
            .linear_velocity(Vector3::z() * 4.0),
    );
    world.attach_collider(
        plank,
        ColliderDesc::box_shape(Vector3::new(0.05, 0.15, 1.0)).density(600.0),
    );
    world.wake_body(plank);
    let world = coast(world, &rooted_shrub(), plank, 1.5);
    let speed = world
        .body(plank)
        .expect("there")
        .linear_velocity()
        .magnitude();
    println!("plank 0.1 m clear of the leaves: {speed:.3} m/s after passing");
    assert!(
        (speed - 4.0).abs() < 1e-3,
        "the leaves slowed it to {speed:.3}"
    );
}

/// A loose clump thrown through a heavier body standing in its leaves
/// shares its momentum with it: the body is nudged, never hauled along at
/// the clump's speed, and the two together keep the momentum the clump was
/// thrown with.
#[test]
fn a_thrown_clump_does_not_haul_a_heavier_body() {
    let mut world = still_space();
    let shape = Arc::new(shrub());
    let core = LeafCore::default().spawn(&mut world, &shape, Isometry3::identity());
    let player = world.create_body(
        RigidBodyDesc::dynamic()
            .position(Point3::origin())
            .filter(collision_layers::GRENADE),
    );
    world.attach_collider(
        player,
        ColliderDesc::capsule(0.5, 0.25).density(56.0 / capsule_volume(0.5, 0.25)),
    );
    world.wake_body(player);
    world.wake_body(core);
    let clump_mass = world.body(core).expect("there").mass();
    let player_mass = world.body(player).expect("there").mass();
    world.apply_impulse(core, Vector3::x() * 10.0 * clump_mass);
    let volumes = vec![DragVolume {
        shape,
        frame: VolumeFrame::Body(core),
    }];
    let world = coast(world, &volumes, player, 1.0);
    let clump_v = world.body(core).expect("there").linear_velocity();
    let player_v = world.body(player).expect("there").linear_velocity();
    let shared = 10.0 * clump_mass / (clump_mass + player_mass);
    let momentum = clump_v * clump_mass + player_v * player_mass;
    println!(
        "clump {clump_mass:.1} kg thrown at 10 m/s through a {player_mass:.0} kg body: \
         body {:.2} m/s, clump {:.2} m/s, momentum {:.1} of {:.1}",
        player_v.x,
        clump_v.x,
        momentum.x,
        10.0 * clump_mass
    );
    assert!(player_v.x > 0.05, "the leaves did not drag the body");
    assert!(
        player_v.x <= shared + 0.05,
        "the body was hauled to {:.2} m/s, past the shared {shared:.2}",
        player_v.x
    );
    assert!(
        (momentum.x - 10.0 * clump_mass).abs() < 0.02 * 10.0 * clump_mass,
        "momentum {:.1}, thrown with {:.1}",
        momentum.x,
        10.0 * clump_mass
    );
}

fn capsule_volume(half_height: f32, radius: f32) -> f32 {
    let pi = std::f32::consts::PI;
    pi * radius * radius * 2.0 * half_height + 4.0 / 3.0 * pi * radius.powi(3)
}

/// The game's grenade going off a metre and a half from a clump throws it a
/// few metres, not across the level. Leaves let the shock front through, so
/// the clump catches only part of the blast its lobes show it: a solid clump
/// of the same shape and weight is thrown several times as fast.
#[test]
fn a_blast_throws_a_clump_within_the_level_not_across_it() {
    let shape = shrub();
    let ground = FlatQuadGeometry::new(200.0);
    let blasted = |core: LeafCore| {
        let mut world = PhysicsWorld::default();
        let body = core.spawn(&mut world, &shape, standing(&shape, Point3::origin(), 0.01));
        let blast = Explosion::new(Point3::new(-1.5, 0.0, 0.0)).physics_impulse();
        let kick = world.impulse_on(body, &blast).unwrap();
        world.apply_impulse(body, kick);
        let launched = kick.magnitude() / world.body(body).unwrap().mass();
        let run = run(
            &mut world,
            body,
            Arc::new(shape.clone()),
            &ground,
            12.0,
            LeafDrag::default(),
        );
        (launched, run)
    };
    let porous = LeafCore::default();
    let (launched, run) = blasted(porous);
    let (solid_launch, _) = blasted(LeafCore {
        solidity: 1.0,
        ..porous
    });
    println!("blasted at {launched:.0} m/s: {run:?}");
    println!("a solid clump: {solid_launch:.0} m/s");
    assert!(run.rest.x > 2.0, "it barely moved: {:.2} m", run.rest.x);
    assert!(run.rest.x < 40.0, "it flew {:.2} m", run.rest.x);
    assert!(
        solid_launch > 2.5 * launched,
        "leaves caught as much as a solid: {launched:.1} against {solid_launch:.1} m/s"
    );
    assert!(run.slept_at.is_some(), "it never came to rest");
}

/// A column blasted loose falls over and lies on its side, within the
/// level, and comes to rest: its core is a capsule along it, not a ball of
/// its width per lobe.
#[test]
fn a_blasted_column_lies_down_and_rests() {
    let shape = ShrubSpec::with_radius(0.45)
        .rooted()
        .with_height(2.8)
        .grow(5);
    let ground = FlatQuadGeometry::new(200.0);
    let core = LeafCore::default();
    let mut world = PhysicsWorld::default();
    let body = core.spawn(&mut world, &shape, standing(&shape, Point3::origin(), 0.01));
    let blast = Explosion::new(Point3::new(-1.5, 0.3, 0.0)).physics_impulse();
    let kick = world.impulse_on(body, &blast).unwrap();
    world.apply_impulse(body, kick);
    let run = run(
        &mut world,
        body,
        Arc::new(shape),
        &ground,
        12.0,
        LeafDrag::default(),
    );
    let length = world.body(body).unwrap().rotation() * Vector3::y();
    println!(
        "column: {run:?}, its length {:.0}° from upright",
        length.y.abs().acos().to_degrees()
    );
    assert!(
        run.rest.x > 1.0 && run.rest.x < 40.0,
        "it went {:.2} m",
        run.rest.x
    );
    assert!(
        length.y.abs() < 0.3,
        "it stands {:.2} upright",
        length.y.abs()
    );
    assert!(run.slept_at.is_some(), "it never came to rest");
}
