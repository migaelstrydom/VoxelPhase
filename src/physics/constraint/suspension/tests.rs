use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use crate::collision::{MeshPatch, PatchTriangle, SurfaceId, Triangle, AABB};
use crate::debug::DebugLines;
use crate::physics::bench_harness::geometry::FlatQuadGeometry;
use crate::physics::constraint::spring::Spring;
use crate::physics::constraint::{ConstraintHandle, ConstraintKind};
use crate::physics::{
    ColliderDesc, ColliderMaterial, FrictionModel, PhysicsConfig, PhysicsWorld, RigidBodyDesc,
    RigidBodyHandle, StaticGeometry,
};

use super::{SuspensionAxis, SuspensionDesc};

/// The engine's fixed substep.
const DT: f32 = 1.0 / 240.0;
/// Radius of the cast sphere in every test.
const FOOT: f32 = 0.05;
/// Radius of the test body's own collider, kept well clear of the ground.
const BODY_RADIUS: f32 = 0.3;

fn gravity() -> f32 {
    PhysicsConfig::default().gravity.norm()
}

fn world() -> PhysicsWorld {
    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    PhysicsWorld::new(config)
}

/// A ball of a body whose only support is its suspension.
fn ball(world: &mut PhysicsWorld, at: Point3<f32>) -> RigidBodyHandle {
    let body = world.create_body(
        RigidBodyDesc::dynamic()
            .position(at)
            .linear_damping(0.0)
            .angular_damping(0.0),
    );
    world.attach_collider(body, ColliderDesc::sphere(BODY_RADIUS));
    body
}

/// Hold `body` upright, as a character's capsule is held: friction at the
/// foot, below the centre of mass, would otherwise tip it over.
fn keep_upright(world: &mut PhysicsWorld, body: RigidBodyHandle) {
    world.create_constraint(ConstraintKind::KeepUpright {
        body,
        target_up: Vector3::y_axis(),
        compliance: 0.0,
        max_impulse: f32::INFINITY,
    });
}

fn foot(friction: f32) -> ColliderMaterial {
    ColliderMaterial {
        restitution: 0.0,
        friction: FrictionModel::isotropic(friction),
    }
}

/// A suspension straight down from the body's centre of mass.
fn legs(body: RigidBodyHandle, spring: Spring, rest_length: f32, friction: f32) -> SuspensionDesc {
    SuspensionDesc {
        body,
        anchor: Vector3::zeros(),
        axis: SuspensionAxis::World(Vector3::y_axis()),
        cast_radius: FOOT,
        rest_length,
        slack: 0.1,
        spring,
        max_force: 1.0e5,
        exclude: SmallVec::new(),
        friction: Some(foot(friction)),
        torsion_radius: 0.0,
    }
}

/// Run one frame of `substeps`, calling `each` after every substep.
fn frame(
    world: &mut PhysicsWorld,
    ground: &dyn StaticGeometry,
    substeps: u32,
    mut each: impl FnMut(&PhysicsWorld),
) {
    world.update_contacts(DT, substeps, ground, &[], &mut DebugLines::default());
    for _ in 0..substeps {
        world.substep(DT, ground, &[]);
        each(world);
    }
}

fn height(world: &PhysicsWorld, body: RigidBodyHandle) -> f32 {
    world.body(body).unwrap().position().y
}

/// Anchor height at which a suspension of `rest_length` standing on flat
/// ground at y = 0 is at rest under its body's weight: the sphere sits on the
/// ground and the spring is pressed in by `g/ω²`.
fn equilibrium(spring: &Spring, rest_length: f32) -> f32 {
    FOOT + rest_length - gravity() / spring.angular_frequency.powi(2)
}

/// A released spring rings at its own frequency and decays at its own
/// damping ratio, however many substeps a frame has: the rows are refreshed
/// every substep, and their terms depend only on the substep's length.
///
/// The soft step is implicit, which damps a little of its own: about `ωh/2`
/// on top of `ζ`, 0.02 here. The tolerance allows that; the agreement
/// between substep counts is held far tighter.
#[test]
fn a_released_spring_rings_at_its_frequency_and_damping() {
    let spring = Spring::from_hz(1.5, 0.1);
    let rest_length = 1.0;
    let rest_height = equilibrium(&spring, rest_length);

    let mut measured = Vec::new();
    for substeps in [1, 4, 12] {
        let mut world = world();
        let ground = FlatQuadGeometry::new(20.0);
        let body = ball(&mut world, Point3::new(0.0, rest_height + 0.05, 0.0));
        world.create_suspension(legs(body, spring, rest_length, 0.5));

        let mut trace = Vec::new();
        while trace.len() < (3.5 / DT) as usize {
            frame(&mut world, &ground, substeps, |world| {
                trace.push(height(world, body) - rest_height)
            });
        }
        let (omega, zeta) = ring(&trace);
        measured.push((substeps, omega, zeta));
    }

    for &(substeps, omega, zeta) in &measured {
        assert!(
            (omega / spring.angular_frequency - 1.0).abs() < 0.03,
            "{substeps} substeps: ω {omega} against {}",
            spring.angular_frequency
        );
        assert!(
            (zeta - spring.damping_ratio).abs() < 0.03,
            "{substeps} substeps: ζ {zeta} against {}",
            spring.damping_ratio
        );
    }
    let (_, omega_1, zeta_1) = measured[0];
    for &(substeps, omega, zeta) in &measured[1..] {
        assert!(
            (omega - omega_1).abs() < 0.005 * omega_1 && (zeta - zeta_1).abs() < 0.005,
            "{substeps} substeps rang at (ω {omega}, ζ {zeta}), one at (ω {omega_1}, ζ {zeta_1})"
        );
    }
}

/// Natural frequency and damping ratio of a decaying oscillation about zero,
/// from its successive peaks.
fn ring(trace: &[f32]) -> (f32, f32) {
    let peaks: Vec<(usize, f32)> = (1..trace.len() - 1)
        .filter(|&i| trace[i] > trace[i - 1] && trace[i] >= trace[i + 1] && trace[i] > 0.0)
        .map(|i| (i, trace[i]))
        .collect();
    assert!(peaks.len() >= 3, "too few peaks: {peaks:?}");
    let cycles = (peaks.len() - 1) as f32;
    let period = (peaks.last().unwrap().0 - peaks[0].0) as f32 * DT / cycles;
    let decrement = (peaks[0].1 / peaks.last().unwrap().1).ln() / cycles;
    let zeta = decrement / (4.0 * std::f32::consts::PI.powi(2) + decrement * decrement).sqrt();
    let omega = std::f32::consts::TAU / period / (1.0 - zeta * zeta).sqrt();
    (omega, zeta)
}

/// Each substep's spring impulse is the implicit spring force at the end of
/// that substep, `k·x + c·ẋ`, with `x` the compression the body has reached
/// by then. A frame of twelve substeps crosses a good part of the spring's
/// travel, so a bias left at the frame's opening compression would miss by
/// most of a newton; refreshed, it is exact.
#[test]
fn the_spring_impulse_tracks_compression_every_substep() {
    let spring = Spring::from_hz(2.0, 0.3);
    let rest_length = 1.0;
    let mut world = world();
    let ground = FlatQuadGeometry::new(20.0);
    let body = ball(&mut world, Point3::new(0.0, FOOT + rest_length - 0.02, 0.0));
    world
        .body_mut(body)
        .unwrap()
        .set_linear_velocity(Vector3::new(0.0, -1.0, 0.0));
    let legs = world.create_suspension(legs(body, spring, rest_length, 0.5));

    let mass = world.body(body).unwrap().mass();
    let stiffness = spring.stiffness(mass);
    let damping = 2.0 * mass * spring.damping_ratio * spring.angular_frequency;
    let mut worst: f32 = 0.0;
    let mut checked = 0;
    for _ in 0..6 {
        frame(&mut world, &ground, 12, |world| {
            let state = world.body(body).unwrap();
            let compression = FOOT + rest_length - state.position().y;
            let rate = -state.linear_velocity().y;
            let force = stiffness * compression + damping * rate;
            let impulse = world.constraint(legs).unwrap().warm_impulses[0];
            if force > 0.0 {
                worst = worst.max((impulse / DT - force).abs() / force);
                checked += 1;
            }
        });
    }
    assert!(
        checked > 30,
        "the spring was hardly loaded: {checked} substeps"
    );
    assert!(
        worst < 0.01,
        "a substep's spring force was {:.1}% off k·x + c·ẋ",
        worst * 100.0
    );
}

/// Dropped from above its reach, a body lands on its suspension and settles
/// pressed in by `g/ω²` over static ground.
#[test]
fn a_dropped_body_settles_at_its_sag() {
    let spring = Spring::from_hz(4.0, 1.0);
    let rest_length = 0.8;
    let mut world = world();
    let ground = FlatQuadGeometry::new(20.0);
    let body = ball(&mut world, Point3::new(0.0, 1.5, 0.0));
    let legs = world.create_suspension(legs(body, spring, rest_length, 0.5));

    for _ in 0..180 {
        frame(&mut world, &ground, 4, |_| {});
    }
    let readout = world.suspension(legs).unwrap();
    let sag = gravity() / spring.angular_frequency.powi(2);
    assert!(readout.grounded);
    assert!(
        (readout.compression - sag).abs() < 0.05 * sag,
        "pressed in {} m; g/ω² is {sag} m",
        readout.compression
    );
    let weight = world.body(body).unwrap().mass() * gravity();
    assert!(
        (readout.load - weight).abs() < 0.01 * weight,
        "carries {} N of a {weight} N body",
        readout.load
    );
    assert!(world.body(body).unwrap().linear_velocity().norm() < 1e-3);
}

/// A plane through the origin, tipped up by `angle` toward +Z.
struct Slope {
    patch: MeshPatch,
}

impl Slope {
    fn new(angle: f32) -> Self {
        let half = 20.0;
        let (rise, run) = (angle.sin() * half, angle.cos() * half);
        let corner = |x: f32, s: f32| Point3::new(x, rise * s, run * s);
        let (a, b, c, d) = (
            corner(-half, -1.0),
            corner(half, -1.0),
            corner(half, 1.0),
            corner(-half, 1.0),
        );
        let triangle = |t: Triangle| PatchTriangle {
            triangle: t,
            neighbors: [None; 3],
            surface: SurfaceId::UNSPECIFIED,
        };
        Self {
            patch: MeshPatch {
                triangles: vec![
                    triangle(Triangle::new(a, c, b)),
                    triangle(Triangle::new(a, d, c)),
                ],
            },
        }
    }
}

impl StaticGeometry for Slope {
    fn query_region(&self, aabb: &AABB) -> MeshPatch {
        let _ = aabb;
        self.patch.clone()
    }
}

/// How far an idle body standing on a vertical suspension drifts across a
/// slope of `degrees` in two seconds, after a second to settle.
fn drift_on_slope(degrees: f32, mu: f32) -> f32 {
    let spring = Spring::from_hz(4.0, 1.0);
    let rest_length = 0.8;
    let mut world = world();
    let slope = Slope::new(degrees.to_radians());
    let start = Point3::new(0.0, equilibrium(&spring, rest_length), 0.0);
    let body = ball(&mut world, start);
    keep_upright(&mut world, body);
    world.create_suspension(legs(body, spring, rest_length, mu));

    for _ in 0..60 {
        frame(&mut world, &slope, 4, |_| {});
    }
    let settled = world.body(body).unwrap().position();
    for _ in 0..120 {
        frame(&mut world, &slope, 4, |_| {});
    }
    let moved = world.body(body).unwrap().position() - settled;
    Vector3::new(moved.x, 0.0, moved.z).norm()
}

/// Coulomb friction for a vertical spring: the leg's push has a share along
/// the slope, and the cone bounds it with the friction rows. Under `atan μ`
/// an idle body holds; over it, it slides, as a block on that slope must.
#[test]
fn an_idle_body_holds_below_atan_mu_and_slides_above_it() {
    let mu: f32 = 0.5;
    let limit = mu.atan().to_degrees();

    let held = drift_on_slope(limit - 6.0, mu);
    assert!(held < 0.005, "slid {held} m on a {}° slope", limit - 6.0);

    let slid = drift_on_slope(limit + 6.0, mu);
    assert!(slid > 0.5, "slid only {slid} m on a {}° slope", limit + 6.0);
}

/// A box on the ground for a suspension to stand on.
fn crate_at(world: &mut PhysicsWorld, at: Point3<f32>) -> RigidBodyHandle {
    let body = world.create_body(RigidBodyDesc::dynamic().position(at));
    world.attach_collider(body, ColliderDesc::box_shape(Vector3::new(0.5, 0.25, 0.5)));
    body
}

fn warm(world: &PhysicsWorld, legs: ConstraintHandle) -> Vec<f32> {
    world.constraint(legs).unwrap().warm_impulses.to_vec()
}

/// Warm impulses carry over while the suspension stands on the same thing,
/// and are dropped the frame it finds itself on something else: a body's
/// weight is not warm-started into whatever it has just stepped onto.
#[test]
fn warm_impulses_reset_when_the_partner_changes() {
    let spring = Spring::from_hz(4.0, 1.0);
    let rest_length = 0.8;
    let mut world = world();
    let ground = FlatQuadGeometry::new(20.0);
    let first = crate_at(&mut world, Point3::new(0.0, 0.25, 0.0));
    let second = crate_at(&mut world, Point3::new(3.0, 0.25, 0.0));
    let body = ball(
        &mut world,
        Point3::new(0.0, 0.5 + equilibrium(&spring, rest_length), 0.0),
    );
    let legs = world.create_suspension(legs(body, spring, rest_length, 0.5));

    for _ in 0..60 {
        frame(&mut world, &ground, 4, |_| {});
    }
    assert_eq!(
        world.suspension(legs).unwrap().hit.unwrap().partner,
        Some(first)
    );

    world.update_contacts(DT, 4, &ground, &[], &mut DebugLines::default());
    assert!(
        warm(&world, legs)[0] > 0.0,
        "the same partner keeps its warm start"
    );
    for _ in 0..4 {
        world.substep(DT, &ground, &[]);
    }

    let position = world.body(body).unwrap().position();
    world
        .body_mut(body)
        .unwrap()
        .set_position(Point3::new(3.0, position.y, position.z));
    world.update_contacts(DT, 4, &ground, &[], &mut DebugLines::default());
    assert_eq!(
        world.suspension(legs).unwrap().hit.unwrap().partner,
        Some(second)
    );
    assert!(
        warm(&world, legs).iter().all(|&impulse| impulse == 0.0),
        "warm impulses carried onto a new partner: {:?}",
        warm(&world, legs)
    );
}

#[test]
fn a_suspension_over_nothing_holds_nothing_up() {
    let spring = Spring::from_hz(4.0, 1.0);
    let mut world = world();
    let ground = FlatQuadGeometry::new(20.0);
    let body = ball(&mut world, Point3::new(0.0, 5.0, 0.0));
    let legs = world.create_suspension(legs(body, spring, 0.8, 0.5));

    frame(&mut world, &ground, 4, |_| {});
    let readout = world.suspension(legs).unwrap();
    assert!(readout.hit.is_none());
    assert!(!readout.grounded);
}
