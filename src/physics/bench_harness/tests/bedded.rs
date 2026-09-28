//! Bodies bedded in the ground on purpose: welded to the world partly inside
//! static geometry, and passing through it, as a menhir or a fence post is.
//! They rest without fighting the ground, sleep, wake when something lands on
//! them, and when released come out of the ground without flying off.

use nalgebra::{Point3, UnitQuaternion, Vector3};

use super::super::geometry::FlatQuadGeometry;
use crate::debug::DebugLines;
use crate::physics::constraint::ConstraintKind;
use crate::physics::world::PhysicsConfig;
use crate::physics::{
    ColliderDesc, ConstraintHandle, PhysicsWorld, RigidBodyDesc, RigidBodyHandle,
    SequentialStepper, Stepper,
};

const FIXED_DT: f32 = 1.0 / 240.0;
const FRAME_DT: f32 = 1.0 / 60.0;

/// Half-extents of the bedded block: a 0.6 m cube.
const HALF: f32 = 0.3;
/// How deep the block's foot is under the ground: a third of it buried.
const BURIED: f32 = 0.2;

/// A 0.6 m block welded with a third of it under a flat floor, as a level
/// beds a stone, and the world it is in, sleep enabled.
struct BeddedBlock {
    world: PhysicsWorld,
    ground: FlatQuadGeometry,
    stepper: SequentialStepper,
    block: RigidBodyHandle,
    weld: ConstraintHandle,
    start: Point3<f32>,
}

impl BeddedBlock {
    fn new() -> Self {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let start = Point3::new(0.0, HALF - BURIED, 0.0);
        let block = world.create_body(
            RigidBodyDesc::dynamic()
                .position(start)
                .ignores_static(true),
        );
        let _ = world.attach_collider(
            block,
            ColliderDesc::box_shape(Vector3::repeat(HALF)).density(2600.0),
        );
        let weld = world.create_constraint(ConstraintKind::world_fixed(
            block,
            start,
            Vector3::zeros(),
            &UnitQuaternion::identity(),
            0.0,
            f32::MAX,
        ));
        // Born asleep, as in the game; woken, as by the first thing to come by.
        world.wake_body(block);
        Self {
            world,
            ground: FlatQuadGeometry::new(10.0),
            stepper: SequentialStepper::new(FIXED_DT, 12),
            block,
            weld,
            start,
        }
    }

    fn step(&mut self) {
        let mut debug_lines = DebugLines::default();
        self.stepper.step(
            &mut self.world,
            FRAME_DT,
            &self.ground,
            &[],
            &[],
            &mut debug_lines,
        );
    }

    /// Step `frames` frames. Returns the fastest the block went, by how far
    /// it moved in a frame rather than its velocity: position correction
    /// moves a body without giving it any.
    fn run(&mut self, frames: u32) -> f32 {
        (0..frames)
            .map(|_| {
                let before = self.position();
                self.step();
                (self.position() - before).norm() / FRAME_DT
            })
            .fold(0.0, f32::max)
    }

    fn position(&self) -> Point3<f32> {
        self.world.body(self.block).unwrap().position()
    }

    fn drift(&self) -> f32 {
        (self.position() - self.start).norm()
    }
}

/// A block welded partly into the ground, passing through it, has nothing to
/// fight: it stays exactly where it was set and goes to sleep.
///
/// Without `ignores_static` the ground pushes it out and the weld holds it
/// in, forever: it rises, keeps a speed it never uses, and never sleeps.
#[test]
fn a_block_welded_into_the_ground_rests_and_sleeps() {
    let mut bedded = BeddedBlock::new();
    let fastest = bedded.run(120);
    let drift = bedded.drift();
    eprintln!("bedded block: fastest {fastest:.4} m/s, drift {drift:.5} m");

    assert!(fastest < 0.01, "the block moved at up to {fastest:.3} m/s");
    assert!(
        drift < 0.001,
        "the block drifted {drift:.4} m from where it was set"
    );
    assert!(
        bedded.world.is_sleeping(bedded.block),
        "the block was still awake after 2 s"
    );
}

/// A sleeping bedded block wakes when a ball lands on it, and holds the ball
/// up: the ground under the ball is the block, not the floor it is sunk in.
#[test]
fn a_sleeping_bedded_block_wakes_and_holds_up_what_lands_on_it() {
    let mut bedded = BeddedBlock::new();
    bedded.run(120);
    assert!(
        bedded.world.is_sleeping(bedded.block),
        "the block never slept"
    );

    let top = bedded.start.y + HALF;
    let radius = 0.2;
    let ball = bedded
        .world
        .create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, top + 1.0, 0.0)));
    let _ = bedded
        .world
        .attach_collider(ball, ColliderDesc::sphere(radius).density(500.0));
    bedded.world.wake_body(ball);

    let mut woke = false;
    for _ in 0..120 {
        bedded.step();
        woke |= !bedded.world.is_sleeping(bedded.block);
    }
    let ball_y = bedded.world.body(ball).unwrap().position().y;
    let drift = bedded.drift();
    eprintln!("ball on bedded block: woke {woke}, ball at {ball_y:.3}, block drift {drift:.5} m");

    assert!(woke, "the ball landed without waking the block");
    assert!(
        (ball_y - (top + radius)).abs() < 0.02,
        "the ball came to {ball_y:.3}, not onto the block's top at {:.3}",
        top + radius
    );
    assert!(
        drift < 0.001,
        "the ball knocked the block {drift:.4} m out of place"
    );
}

/// Released from its weld and from passing through the ground, a bedded
/// block comes up out of the ground and settles on it without flying off.
///
/// Position correction moves it out without giving it any velocity, so it
/// stops at the surface rather than carrying on past it. It does move out
/// fast: the correction cap applies per contact and per iteration, not per
/// body, so this block snaps up its 0.2 m in about a frame.
#[test]
fn a_released_bedded_block_comes_out_of_the_ground_without_flying_off() {
    let mut bedded = BeddedBlock::new();
    bedded.run(30);

    bedded.world.remove_constraint(bedded.weld);
    bedded.world.set_ignores_static(bedded.block, false);
    let mut fastest = 0.0f32;
    let mut highest = f32::MIN;
    let mut velocity = 0.0f32;
    for _ in 0..120 {
        fastest = fastest.max(bedded.run(1));
        highest = highest.max(bedded.position().y);
        let body = bedded.world.body(bedded.block).unwrap();
        velocity = velocity.max(body.linear_velocity().norm());
    }
    let resting_y = bedded.position().y;
    eprintln!(
        "released block: moved at up to {fastest:.2} m/s with velocity up to {velocity:.3} m/s, \
         highest {highest:.3}, resting at {resting_y:.3}"
    );

    assert!(
        velocity < 0.5,
        "the released block was given {velocity:.2} m/s"
    );
    assert!(
        highest < HALF + 0.02,
        "the released block flew up to {highest:.3}"
    );
    assert!(
        (resting_y - HALF).abs() < 0.01,
        "the released block settled at {resting_y:.3}, not on the ground at {HALF:.3}"
    );
}
