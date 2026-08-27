//! End-to-end checks on the powered moving platform.
//!
//! `MovingPlatformSystem` states an intent as a `Velocity` and `PhysicsSyncSystem`
//! turns it into a drive target. These tests skip the ECS and do both steps by
//! hand against the real `PhysicsWorld`, so what they exercise is the patrol
//! logic and the solver's response to it — including whether the motor's
//! acceleration budget actually clears gravity, which is the failure that would
//! leave a lift sitting on the floor buzzing.

use nalgebra::{Point3, UnitVector3, Vector3};

use super::super::geometry::FlatQuadGeometry;
use crate::debug::DebugLines;
use crate::physics::constraint::ConstraintKind;
use crate::physics::world::PhysicsConfig;
use crate::physics::{ColliderDesc, PhysicsWorld, RigidBodyDesc, RigidBodyHandle};
use crate::platform::MovingPlatform;

const HALF_EXTENTS: Vector3<f32> = Vector3::new(2.0, 0.3, 2.0);
const MOTOR_MAX_ACCEL: f32 = 40.0;

/// Build a platform at `from`, shuttling to `to`.
fn spawn_platform(
    world: &mut PhysicsWorld,
    from: Vector3<f32>,
    to: Vector3<f32>,
    speed: f32,
) -> (RigidBodyHandle, MovingPlatform) {
    let body = world.create_body(
        RigidBodyDesc::dynamic()
            .position(Point3::from(from))
            .gravity_scale(1.0)
            .linear_damping(0.0)
            .angular_damping(0.05),
    );
    let _ = world.attach_collider(
        body,
        ColliderDesc::box_shape(HALF_EXTENTS)
            .density(300.0)
            .friction(0.9),
    );
    let _ = world.create_constraint(ConstraintKind::KeepUpright {
        body,
        target_up: UnitVector3::new_normalize(Vector3::y()),
        compliance: 0.0,
        max_impulse: f32::INFINITY,
    });

    let platform = MovingPlatform::new(from, to, speed);
    (body, platform)
}

/// One frame: patrol logic, then drive sync, then substeps. Mirrors the
/// dispatcher order (`moving_platform` → `physics_sync`).
fn run_frame(
    world: &mut PhysicsWorld,
    body: RigidBodyHandle,
    platform: &mut MovingPlatform,
    geometry: &FlatQuadGeometry,
    debug_lines: &mut DebugLines,
) {
    let dt = 1.0 / 240.0;
    let position = {
        let p = world.body(body).unwrap().position();
        Vector3::new(p.x, p.y, p.z)
    };
    platform.update_heading(&position);
    let _ = world.set_body_velocity_drive(
        body,
        platform.target_velocity(&position),
        Vector3::zeros(),
        MOTOR_MAX_ACCEL,
        0.0,
    );

    world.update_contacts(dt, 4, geometry, &[], debug_lines);
    for _ in 0..4 {
        world.substep(dt, geometry, &[]);
    }
}

/// A lift must climb against gravity, turn around at the top, and come back.
#[test]
fn lift_shuttles_between_endpoints() {
    let geometry = FlatQuadGeometry::new(64.0);
    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    let mut world = PhysicsWorld::new(config);

    let from = Vector3::new(0.0, 6.0, 0.0);
    let to = Vector3::new(0.0, 14.0, 0.0);
    let (body, mut platform) = spawn_platform(&mut world, from, to, 2.0);
    let mut debug_lines = DebugLines::default();

    let mut highest: f32 = from.y;
    let mut lowest: f32 = from.y;
    let mut reversals = 0;
    let mut last_heading = platform.heading;

    // 8 s: at 2 m/s over 8 m of travel, that is several full sweeps.
    for _ in 0..1920 {
        run_frame(&mut world, body, &mut platform, &geometry, &mut debug_lines);
        let y = world.body(body).unwrap().position().y;
        highest = highest.max(y);
        lowest = lowest.min(y);
        if platform.heading != last_heading {
            reversals += 1;
            last_heading = platform.heading;
        }
    }

    eprintln!(
        "lift shuttle: lowest={lowest:.3} highest={highest:.3} reversals={reversals} \
         (endpoints {:.1}..{:.1})",
        from.y, to.y
    );

    // Arriving is turning around within `arrival_radius`, so the endpoint
    // itself need not be touched exactly.
    let slack = platform.arrival_radius;
    assert!(
        highest >= to.y - slack,
        "lift should reach the top of its travel: {highest:.3} < {:.3}",
        to.y - slack
    );
    assert!(
        lowest <= from.y + slack,
        "lift should reach the bottom of its travel: {lowest:.3} > {:.3}",
        from.y + slack
    );
    assert!(
        reversals >= 2,
        "lift should turn around at both ends: {reversals} reversals"
    );
    // Overshoot past the endpoint is expected — the motor decelerates at a
    // finite rate — but a lift that sails metres past its stop is broken.
    assert!(
        highest < to.y + 1.0 && lowest > from.y - 1.0,
        "endpoint overshoot too large: {lowest:.3}..{highest:.3}"
    );
}

/// Knocked off its line, the platform must fly back to its endpoint.
///
/// This is what separates the shuttle from a platform that merely holds a
/// heading: the drive is aimed at the target *point*, recomputed from wherever
/// the platform actually is, so displacement is transient. A grenade moves the
/// path it takes, never the route it runs.
#[test]
fn platform_returns_to_its_route_after_a_shove() {
    let geometry = FlatQuadGeometry::new(64.0);
    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    let mut world = PhysicsWorld::new(config);

    // A horizontal run, so gravity is a disturbance rather than the point.
    let from = Vector3::new(-10.0, 10.0, 0.0);
    let to = Vector3::new(10.0, 10.0, 0.0);
    let (body, mut platform) = spawn_platform(&mut world, from, to, 2.0);
    let mut debug_lines = DebugLines::default();

    for _ in 0..120 {
        run_frame(&mut world, body, &mut platform, &geometry, &mut debug_lines);
    }

    // Shove it hard off the line, the way a blast would.
    let shove = Vector3::new(0.0, 0.0, 6.0);
    {
        let b = world.body_mut(body).unwrap();
        let v = b.linear_velocity();
        b.set_linear_velocity(v + shove);
    }

    let mut peak_offset: f32 = 0.0;
    for _ in 0..960 {
        run_frame(&mut world, body, &mut platform, &geometry, &mut debug_lines);
        let p = world.body(body).unwrap().position();
        peak_offset = peak_offset.max(p.z.abs());
    }

    let end = world.body(body).unwrap().position();
    eprintln!(
        "after shove: peak z offset {peak_offset:.3}, settled at \
         ({:.3}, {:.3}, {:.3})",
        end.x, end.y, end.z
    );

    assert!(
        peak_offset > 0.2,
        "the shove should actually have displaced it: {peak_offset:.3}"
    );
    assert!(
        end.z.abs() < 0.3,
        "platform should converge back on to its route: z = {:.3}",
        end.z
    );
    assert!(
        end.x.abs() <= 10.5,
        "platform should still be running between its endpoints: x = {:.3}",
        end.x
    );
}

/// The motor climbs at cruise speed less one frame of gravity.
///
/// The drive target is refreshed once per frame while gravity is integrated
/// once per substep, so a climbing lift settles a predictable `g · frame_dt`
/// below its authored speed — about 8% at 2 m/s. That sag is the servo working,
/// not failing; what this test guards is that it stays *bounded*. Drop
/// `MOTOR_MAX_ACCEL` below gravity and the lift sinks instead, which shows up
/// here immediately.
#[test]
fn lift_holds_cruise_speed_against_gravity() {
    let geometry = FlatQuadGeometry::new(64.0);
    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    let gravity = config.gravity.norm();
    let mut world = PhysicsWorld::new(config);

    let speed = 2.0;
    let (body, mut platform) = spawn_platform(
        &mut world,
        Vector3::new(0.0, 10.0, 0.0),
        Vector3::new(0.0, 50.0, 0.0),
        speed,
    );
    let mut debug_lines = DebugLines::default();

    // Half a second to spin up, then sample.
    for _ in 0..120 {
        run_frame(&mut world, body, &mut platform, &geometry, &mut debug_lines);
    }

    let gravity_sag = gravity * 4.0 / 240.0; // one frame of substeps
    let expected = speed - gravity_sag;
    let climb = world.body(body).unwrap().linear_velocity().y;
    eprintln!("lift climb speed: {climb:.4} (target {speed:.1}, expected {expected:.4})");
    assert!(
        (climb - expected).abs() < 0.02,
        "climb should be cruise speed less one frame of gravity: \
         {climb:.4} vs {expected:.4}"
    );
}
