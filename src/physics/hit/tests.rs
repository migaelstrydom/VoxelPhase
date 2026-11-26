//! Hits recorded from whole worlds stepped as the game steps them: 1/240 s
//! substeps in 60 Hz frames, in vacuum, so arrivals can be worked out.

use nalgebra::{Point3, UnitQuaternion, Vector3};

use super::{Hit, Patch};
use crate::debug::DebugLines;
use crate::physics::bench_harness::geometry::FlatQuadGeometry;
use crate::physics::{
    Atmosphere, ColliderDesc, ConstraintKind, PhysicsConfig, PhysicsImpulse, PhysicsWorld,
    RigidBodyDesc, RigidBodyHandle, SequentialStepper, Stepper, HARD_GIVE,
};

const FRAME_DT: f32 = 1.0 / 60.0;
const GRAVITY: f32 = 9.81;

/// A world of flat ground, stepped a frame at a time.
struct Bench {
    world: PhysicsWorld,
    stepper: SequentialStepper,
    ground: FlatQuadGeometry,
    debug: DebugLines,
}

impl Bench {
    fn new() -> Self {
        Self::with(|_| {})
    }

    fn with(configure: impl FnOnce(&mut PhysicsConfig)) -> Self {
        let mut config = PhysicsConfig::default();
        config.atmosphere = Atmosphere::VACUUM;
        configure(&mut config);
        Self {
            world: PhysicsWorld::new(config),
            stepper: SequentialStepper::new(1.0 / 240.0, 12),
            ground: FlatQuadGeometry::new(50.0),
            debug: DebugLines::default(),
        }
    }

    fn body(&mut self, desc: RigidBodyDesc, collider: ColliderDesc) -> RigidBodyHandle {
        let body = self.world.create_body(desc);
        self.world.attach_collider(body, collider);
        self.world.wake_body(body);
        body
    }

    fn step(&mut self, impulses: &[PhysicsImpulse]) {
        self.stepper.step(
            &mut self.world,
            FRAME_DT,
            &self.ground,
            impulses,
            &[],
            &mut self.debug,
        );
    }

    fn hits_on(&self, body: RigidBodyHandle) -> Vec<Hit> {
        self.world.hits_on(body).copied().collect()
    }

    /// Step until `body` is hit, at most `seconds`; the frame's hits on it.
    fn first_hits(&mut self, body: RigidBodyHandle, seconds: f32) -> Vec<Hit> {
        for _ in 0..(seconds / FRAME_DT) as usize {
            self.step(&[]);
            let hits = self.hits_on(body);
            if !hits.is_empty() {
                return hits;
            }
        }
        Vec::new()
    }

    /// How many hits `body` takes over `seconds`.
    fn count_hits(&mut self, body: RigidBodyHandle, seconds: f32) -> usize {
        (0..(seconds / FRAME_DT) as usize)
            .map(|_| {
                self.step(&[]);
                self.hits_on(body).len()
            })
            .sum()
    }
}

fn crate_box() -> ColliderDesc {
    ColliderDesc::box_shape(Vector3::new(0.25, 0.25, 0.25))
        .density(1000.0)
        .restitution(0.0)
}

/// Drop a 0.5 m crate turned by `rotation` so its lowest point falls `drop`
/// m, `phase` of a frame's travel further; its first frame of hits, the
/// speed it arrived at, and its mass.
fn drop_crate(rotation: UnitQuaternion<f32>, drop: f32, phase: f32) -> (Vec<Hit>, f32, f32) {
    let mut bench = Bench::new();
    let reach = [-1.0f32, 1.0]
        .iter()
        .flat_map(|&x| [-1.0f32, 1.0].map(move |y| (x, y)))
        .flat_map(|(x, y)| [-1.0f32, 1.0].map(move |z| Vector3::new(x, y, z) * 0.25))
        .map(|corner| -(rotation * corner).y)
        .fold(0.0f32, f32::max);
    let fall = drop + phase * (2.0 * GRAVITY * drop).sqrt() * FRAME_DT;
    // Clear of the ground quad's diagonal, x = z.
    let body = bench.body(
        RigidBodyDesc::dynamic()
            .position(Point3::new(3.0, fall + reach, 1.0))
            .rotation(rotation),
        crate_box(),
    );
    let mass = bench.world.body(body).unwrap().mass();
    let hits = bench.first_hits(body, 3.0);
    (hits, (2.0 * GRAVITY * fall).sqrt(), mass)
}

fn assert_near(actual: f32, expected: f32, tolerance: f32, what: &str) {
    assert!(
        (actual - expected).abs() <= tolerance * expected.abs(),
        "{what}: {actual}, expected {expected}"
    );
}

#[test]
fn a_crate_landing_flat_is_one_face_hit_at_its_arrival_speed() {
    for phase in [0.0, 0.33, 0.67] {
        let (hits, arrival, mass) = drop_crate(UnitQuaternion::identity(), 2.0, phase);
        assert_eq!(hits.len(), 1, "phase {phase}: {hits:?}");
        let hit = hits[0];
        assert_near(hit.approach, arrival, 0.02, "approach");
        match hit.patch {
            Patch::Face { area } => assert_near(area, 0.25, 0.05, "area"),
            other => panic!("phase {phase}: a flat landing read as {other:?}"),
        }
        // Struck square in the middle of its face, all of it stops.
        assert_near(hit.effective_mass, mass, 0.02, "effective mass");
        assert_near(hit.normal.y, 1.0, 1e-3, "normal");
        assert!(hit.body_a.is_none(), "the ground is the first side");
        assert_eq!(hit.give, HARD_GIVE);
    }
}

#[test]
fn a_crate_landing_on_an_edge_is_a_line_and_on_a_corner_a_point() {
    let edge = UnitQuaternion::from_euler_angles(45f32.to_radians(), 0.0, 0.0);
    let corner = UnitQuaternion::from_euler_angles(35f32.to_radians(), 0.0, 45f32.to_radians());
    for phase in [0.0, 0.5] {
        let (hits, arrival, mass) = drop_crate(edge, 2.0, phase);
        let hit = hits[0];
        assert_near(hit.approach, arrival, 0.02, "edge approach");
        match hit.patch {
            Patch::Line { length } => assert_near(length, 0.5, 0.05, "edge length"),
            other => panic!("phase {phase}: an edge landing read as {other:?}"),
        }
        assert_near(
            hit.effective_mass,
            mass,
            0.02,
            "under its centre, all of it stops",
        );

        let (hits, arrival, mass) = drop_crate(corner, 2.0, phase);
        let hit = hits[0];
        assert_near(hit.approach, arrival, 0.02, "corner approach");
        assert_eq!(hit.patch, Patch::Point, "phase {phase}");
        assert!(
            hit.effective_mass < 0.9 * mass,
            "struck off its centre, it turns: {} of {mass}",
            hit.effective_mass
        );
        assert_near(
            hit.bluntness,
            crate::physics::CORNER_ROUNDING,
            1e-6,
            "bluntness",
        );
    }
}

/// What carries load is not a hit: a crate set down slowly, and a heavy crate
/// on a light one woken where they stand, whose contacts churn and re-solve
/// from cold, as fracture's deadbands had to allow for.
#[test]
fn a_slow_touch_and_a_woken_stack_are_not_hits() {
    let mut bench = Bench::new();
    let lowered = bench.body(
        RigidBodyDesc::dynamic()
            .position(Point3::new(3.0, 0.2501, 1.0))
            .linear_velocity(Vector3::new(0.0, -0.3, 0.0)),
        crate_box(),
    );
    assert_eq!(bench.count_hits(lowered, 2.0), 0);

    let mut bench = Bench::new();
    let under = bench.body(
        RigidBodyDesc::dynamic().position(Point3::new(-3.0, 0.25, -1.0)),
        crate_box(),
    );
    let over = bench.body(
        RigidBodyDesc::dynamic().position(Point3::new(-3.0, 0.75, -1.0)),
        crate_box().density(8000.0),
    );
    assert_eq!(
        bench.count_hits(under, 2.0) + bench.count_hits(over, 0.0),
        0
    );
}

/// Two equal balls meeting head on: the pair's mass along the line is half
/// of one, and each changes speed by what `delta_v` says.
#[test]
fn a_head_on_pair_shares_the_blow_by_its_effective_mass() {
    let mut bench = Bench::with(|config| config.gravity = Vector3::zeros());
    let ball = || ColliderDesc::sphere(0.2).density(500.0).restitution(0.3);
    let left = bench.body(
        RigidBodyDesc::dynamic()
            .position(Point3::new(-1.0, 5.0, 0.0))
            .linear_velocity(Vector3::new(2.0, 0.0, 0.0)),
        ball(),
    );
    let right = bench.body(
        RigidBodyDesc::dynamic()
            .position(Point3::new(1.0, 5.0, 0.0))
            .linear_velocity(Vector3::new(-2.0, 0.0, 0.0)),
        ball(),
    );
    let mass = bench.world.body(left).unwrap().mass();
    let hits = bench.first_hits(left, 2.0);
    assert_eq!(hits.len(), 1);
    let hit = hits[0];
    assert!(hit.involves(right));
    assert_near(hit.approach, 4.0, 0.01, "approach");
    assert_near(hit.effective_mass, mass / 2.0, 0.01, "effective mass");
    assert_eq!(hit.patch, Patch::Point);
    let speed = bench.world.body(left).unwrap().linear_velocity().x;
    assert_near(2.0 - speed, hit.delta_v(mass), 0.05, "change of speed");
    assert!(
        hit.side(left).unwrap().push().x < -0.99,
        "the left ball is pushed back"
    );
    assert!(
        hit.side(right).unwrap().push().x > 0.99,
        "the right ball is pushed on"
    );
}

/// Landing at 6 m/s 0.4 of a frame's travel late, a crate is caught at the
/// end of one frame and stopped in the next: two hits, the second the rest
/// of the first.
#[test]
fn a_stop_split_across_two_frames_is_one_blow_continued() {
    let mut bench = Bench::new();
    let speed = 6.0;
    let fall = speed * speed / (2.0 * GRAVITY) + 0.4 * speed * FRAME_DT;
    let body = bench.body(
        RigidBodyDesc::dynamic().position(Point3::new(3.0, fall + 0.25, 1.0)),
        crate_box(),
    );
    let first = bench.first_hits(body, 2.0);
    assert_eq!(first.len(), 1);
    assert!(!first[0].continues);
    bench.step(&[]);
    let rest = bench.hits_on(body);
    assert_eq!(rest.len(), 1, "the stop was not split: {rest:?}");
    assert!(rest[0].continues);
    assert!(rest[0].approach < first[0].approach);

    bench.count_hits(body, 1.0);
    bench
        .world
        .set_body_velocity(body, Vector3::new(0.0, 4.0, 0.0), Vector3::zeros());
    let again = bench.first_hits(body, 2.0);
    assert!(!again[0].continues, "a second landing is a new blow");
}

/// A blast is recorded as what it gave the body.
#[test]
fn a_blast_is_a_shove_of_the_impulse_it_gave() {
    let mut bench = Bench::with(|config| config.gravity = Vector3::zeros());
    let body = bench.body(
        RigidBodyDesc::dynamic().position(Point3::new(0.0, 5.0, 0.0)),
        crate_box(),
    );
    let mass = bench.world.body(body).unwrap().mass();
    bench.step(&[PhysicsImpulse::radial(
        Point3::new(-2.0, 5.0, 0.0),
        6.0,
        2000.0,
    )]);
    let shoves: Vec<_> = bench
        .world
        .shoves()
        .iter()
        .filter(|s| s.body == body)
        .collect();
    assert_eq!(shoves.len(), 1);
    let velocity = bench.world.body(body).unwrap().linear_velocity();
    assert!(velocity.x > 0.5, "the blast moved it: {velocity:?}");
    assert!((shoves[0].impulse / mass - velocity).norm() < 1e-3 * velocity.norm());

    bench.step(&[]);
    assert!(bench.world.shoves().is_empty(), "a shove lasts its frame");
}

/// Fast enough for CCD to catch it, a small ball still arrives at its speed.
#[test]
fn a_fast_small_ball_is_hit_at_its_speed() {
    let mut bench = Bench::new();
    let speed = 40.0;
    let ball = bench.body(
        RigidBodyDesc::dynamic()
            .position(Point3::new(3.0, 1.5, 1.0))
            .linear_velocity(Vector3::new(0.0, -speed, 0.0)),
        ColliderDesc::sphere(0.05).restitution(0.0),
    );
    let hits = bench.first_hits(ball, 1.0);
    assert_eq!(hits.len(), 1, "{hits:?}");
    assert!(
        bench.world.frame_profile().ccd_corrections > 0,
        "CCD caught the ball"
    );
    let arrival = (speed * speed + 2.0 * GRAVITY * 1.45).sqrt();
    assert_near(hits[0].approach, arrival, 0.03, "approach");
}

/// A 0.5 m crate placed at `at`, at rest.
fn resting_crate(bench: &mut Bench, at: Point3<f32>) -> RigidBodyHandle {
    let block = bench.body(RigidBodyDesc::dynamic().position(at), crate_box());
    for _ in 0..30 {
        bench.step(&[]);
    }
    block
}

/// A 110 kg ball fired at `block` from `from` off its centre at 5 m/s:
/// whether each was backed against the first hit between them, `(block,
/// ball)`.
fn strike(bench: &mut Bench, block: RigidBodyHandle, from: Vector3<f32>) -> (bool, bool) {
    let centre = bench.world.body(block).unwrap().position();
    let ball = bench.body(
        RigidBodyDesc::dynamic()
            .position(centre + from)
            .linear_velocity(-from.normalize() * 5.0),
        ColliderDesc::sphere(0.15).density(7800.0).restitution(0.0),
    );
    let hits = bench.first_hits(ball, 1.0);
    let hit = hits
        .iter()
        .find(|hit| hit.involves(block))
        .expect("the ball strikes the block");
    (
        hit.side(block).unwrap().backed(),
        hit.side(ball).unwrap().backed(),
    )
}

/// Struck from above where it lies, a block is pushed back against the
/// blow by the ground, and passes it on.
#[test]
fn a_block_struck_where_it_lies_is_backed_by_the_ground() {
    let mut bench = Bench::new();
    let block = resting_crate(&mut bench, Point3::new(3.0, 0.25, 1.0));
    assert_eq!(
        strike(&mut bench, block, Vector3::y() * 0.45),
        (true, false)
    );
}

#[test]
fn a_block_struck_in_flight_is_backed_by_nothing() {
    let mut bench = Bench::new();
    let block = bench.body(
        RigidBodyDesc::dynamic().position(Point3::new(3.0, 10.0, 1.0)),
        crate_box(),
    );
    assert_eq!(
        strike(&mut bench, block, Vector3::y() * 0.45),
        (false, false)
    );
}

/// Struck sideways where it lies, a block is pushed by the ground square to
/// the blow, not against it, and moves with it.
#[test]
fn a_block_struck_across_the_ground_it_lies_on_is_not_backed() {
    let mut bench = Bench::new();
    let block = resting_crate(&mut bench, Point3::new(3.0, 0.25, 1.0));
    assert_eq!(
        strike(&mut bench, block, Vector3::x() * 0.45),
        (false, false)
    );
}

#[test]
fn a_block_welded_to_the_world_is_backed_by_its_weld() {
    let mut bench = Bench::new();
    let at = Point3::new(3.0, 2.0, 1.0);
    let block = bench.body(RigidBodyDesc::dynamic().position(at), crate_box());
    bench.world.create_constraint(ConstraintKind::world_fixed(
        block,
        at,
        Vector3::zeros(),
        &UnitQuaternion::identity(),
        0.0,
        f32::INFINITY,
    ));
    assert_eq!(
        strike(&mut bench, block, Vector3::y() * 0.45),
        (true, false)
    );
}

/// A tall body struck low from the side tips about the blow if it is free,
/// so less of it answers it; held upright by a rigid hold, as a character
/// is, it answers with its whole mass. Either way the hit reads the change
/// of speed the solver gave its centre.
#[test]
fn a_body_held_upright_is_judged_by_what_the_blow_gave_it() {
    let mut read = Vec::new();
    for held in [false, true] {
        let mut bench = Bench::with(|config| config.gravity = Vector3::zeros());
        let post = bench.body(
            RigidBodyDesc::dynamic().position(Point3::new(0.0, 5.0, 0.0)),
            ColliderDesc::capsule(1.0, 0.25).density(500.0),
        );
        if held {
            bench.world.create_constraint(ConstraintKind::KeepAttitude {
                body: post,
                pitch: 0.0,
                compliance: 0.0,
                max_impulse: f32::INFINITY,
            });
        }
        bench.body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(-1.0, 4.4, 0.0))
                .linear_velocity(Vector3::x() * 6.0),
            ColliderDesc::sphere(0.2).density(2000.0).restitution(0.0),
        );
        let hits = bench.first_hits(post, 1.0);
        let hit = hits.first().expect("the ball strikes the post");
        let body = bench.world.body(post).unwrap();
        let (reads, took) = (hit.delta_v(body.mass()), body.linear_velocity().x);
        assert!(
            (reads - took).abs() < 0.1 * took,
            "held {held}: reads {reads:.2} m/s, took {took:.2}"
        );
        read.push(reads);
    }
    assert!(read[1] > 1.3 * read[0], "free, then held: {read:?}");
}
