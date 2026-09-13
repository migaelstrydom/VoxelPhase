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
use crate::physics::{ColliderDesc, DriveCommand, PhysicsWorld, RigidBodyDesc, RigidBodyHandle};
use crate::platform::{DeckSuspension, MovingPlatform, RouteLoop, SeekMotion, REFERENCE_LOAD_KG};

const HALF_EXTENTS: Vector3<f32> = Vector3::new(2.0, 0.3, 2.0);
const MOTOR_MAX_ACCEL: f32 = 40.0;
const SUBSTEPS: u32 = 4;
const SUBSTEP_DT: f32 = 1.0 / 240.0;
/// One frame of wall-clock, which is what the dispatcher hands
/// `MovingPlatformSystem`. The motion model carries state between frames and is
/// advanced once per frame, so giving it a substep's length instead would run
/// it at a quarter speed — a four-fold error in every time constant, visible
/// only as a platform that corners four times more widely than authored.
const FRAME_DT: f32 = SUBSTEP_DT * SUBSTEPS as f32;

/// Build a rigid-decked platform at `from`, shuttling to `to`.
fn spawn_platform(
    world: &mut PhysicsWorld,
    from: Vector3<f32>,
    to: Vector3<f32>,
    speed: f32,
) -> (RigidBodyHandle, MovingPlatform) {
    spawn_platform_with_suspension(world, from, to, speed, DeckSuspension::RIGID)
}

/// The same platform with a stated deck suspension. Mirrors what
/// `MovingPlatformDef::spawn` builds, including routing both of the
/// suspension's outputs to the places that consume them.
fn spawn_platform_with_suspension(
    world: &mut PhysicsWorld,
    from: Vector3<f32>,
    to: Vector3<f32>,
    speed: f32,
    suspension: DeckSuspension,
) -> (RigidBodyHandle, MovingPlatform) {
    let tuning = suspension.tune(&HALF_EXTENTS);
    let body = world.create_body(
        RigidBodyDesc::dynamic()
            .position(Point3::from(from))
            .gravity_scale(1.0)
            .linear_damping(0.0)
            .angular_damping(tuning.angular_damping),
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
        compliance: tuning.compliance,
        max_impulse: f32::INFINITY,
    });

    let platform = MovingPlatform::shuttle(from, to, speed);
    (body, platform)
}

/// How far the deck leans out of level, in degrees, signed so that a load on
/// the +x edge reads positive.
///
/// A deck whose +x edge has dropped has its normal tilted towards +x — away
/// from the raised side, not towards it — so the sign comes straight off the
/// cross product with no negation.
fn deck_tilt_degrees(world: &PhysicsWorld, body: RigidBodyHandle) -> f32 {
    let up = world.body(body).unwrap().rotation() * Vector3::y();
    up.cross(&Vector3::y()).z.asin().to_degrees()
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
    run_frame_with_motor(
        world,
        body,
        platform,
        geometry,
        debug_lines,
        MOTOR_MAX_ACCEL,
    );
}

/// The same frame with a motor of a stated size, for the scenarios that ask
/// what the actuator's declared authority actually buys.
fn run_frame_with_motor(
    world: &mut PhysicsWorld,
    body: RigidBodyHandle,
    platform: &mut MovingPlatform,
    geometry: &FlatQuadGeometry,
    debug_lines: &mut DebugLines,
    max_accel: f32,
) {
    let position = {
        let p = world.body(body).unwrap().position();
        Vector3::new(p.x, p.y, p.z)
    };
    platform.update_heading(&position);
    let _ = world.set_body_drive(
        body,
        &DriveCommand::medium(
            platform.target_velocity(&position, FRAME_DT),
            Vector3::zeros(),
            max_accel,
            0.0,
        ),
    );

    world.update_contacts(SUBSTEP_DT, SUBSTEPS, geometry, &[], debug_lines);
    for _ in 0..SUBSTEPS {
        world.substep(SUBSTEP_DT, geometry, &[]);
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
    let mut last_leg = platform.route.target_index();

    // 8 s: at 2 m/s over 8 m of travel, that is several full sweeps.
    for _ in 0..1920 {
        run_frame(&mut world, body, &mut platform, &geometry, &mut debug_lines);
        let y = world.body(body).unwrap().position().y;
        highest = highest.max(y);
        lowest = lowest.min(y);
        if platform.route.target_index() != last_leg {
            reversals += 1;
            last_leg = platform.route.target_index();
        }
    }

    eprintln!(
        "lift shuttle: lowest={lowest:.3} highest={highest:.3} reversals={reversals} \
         (endpoints {:.1}..{:.1})",
        from.y, to.y
    );

    // Arriving is crossing the endpoint's plane, and the motion model then
    // swings about `0.31 · speed · tau` past it before coming back. The
    // endpoint is therefore reached and a little exceeded, never fallen short
    // of; the slack is for the sampling, not for the model.
    let slack = 0.05;
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

/// The motor climbs at its authored cruise speed against gravity.
///
/// Under the medium anchor the motor is a solver row rather than a pre-solve
/// chase, so it is solved alongside the weight it is lifting instead of ahead
/// of it. There is nothing left over to absorb, and the lift simply holds its
/// speed. Before Stage 4 it settled a frame of gravity short — 1.8365 against
/// an authored 2.0 — because the drive ran first and then had gravity folded
/// into its target so it would not fight it.
#[test]
fn lift_holds_cruise_speed_against_gravity() {
    let geometry = FlatQuadGeometry::new(64.0);
    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    let mut world = PhysicsWorld::new(config);

    let speed = 2.0;
    let (body, mut platform) = spawn_platform(
        &mut world,
        Vector3::new(0.0, 10.0, 0.0),
        Vector3::new(0.0, 50.0, 0.0),
        speed,
    );
    let mut debug_lines = DebugLines::default();

    // Spin up, then sample. The motion model reaches cruise on a time
    // constant, so this has to clear several of them — at the default 0.25 s
    // that is about 0.75 s, and sampling at half a second would catch the lift
    // still on its way up. Two seconds is eight time constants.
    for _ in 0..120 {
        run_frame(&mut world, body, &mut platform, &geometry, &mut debug_lines);
    }

    let climb = world.body(body).unwrap().linear_velocity().y;
    eprintln!("lift climb speed: {climb:.4} (authored {speed:.1})");
    assert!(
        (climb - speed).abs() < 0.02,
        "climb should be the authored cruise speed: {climb:.4} vs {speed:.4}"
    );
}

/// A motor may spend no more than the acceleration its actuator declares.
///
/// The medium rows' bounds are the whole of the actuator's authority, so a lift
/// whose motor is weaker than gravity cannot hold itself up however hard the
/// route servo asks it to climb: it sinks, and the acceleration it does produce
/// stays under the ceiling it declared. A row whose bound had stopped meaning
/// anything — a body carrying both a support drive and a medium row, say —
/// would climb regardless, so this is the check that the declaration is the
/// only authority in play.
///
/// The bound is per substep, so a row pinned at it delivers the declared
/// acceleration over the frame. An ordinary platform never finds out: at
/// 40 m/s² against a 9.81 m/s² world the rows are nowhere near their bounds
/// outside the first few frames of spin-up.
#[test]
fn a_motor_weaker_than_gravity_cannot_hold_the_lift_up() {
    let geometry = FlatQuadGeometry::new(64.0);
    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    let gravity = config.gravity.norm();
    let mut world = PhysicsWorld::new(config);

    let motor = gravity * 0.25;
    let (body, mut platform) = spawn_platform(
        &mut world,
        Vector3::new(0.0, 40.0, 0.0),
        Vector3::new(0.0, 90.0, 0.0),
        2.0,
    );
    let mut debug_lines = DebugLines::default();

    let start = world.body(body).unwrap().linear_velocity().y;
    let frames = 60;
    for _ in 0..frames {
        run_frame_with_motor(
            &mut world,
            body,
            &mut platform,
            &geometry,
            &mut debug_lines,
            motor,
        );
    }

    let fall = world.body(body).unwrap().linear_velocity().y;
    let elapsed = frames as f32 * 4.0 / 240.0;
    let realised = gravity - (start - fall) / elapsed;
    eprintln!(
        "underpowered lift: motor={motor:.2} m/s² realised={realised:.2} m/s² \
         fall={fall:.4} m/s after {elapsed:.2}s"
    );

    assert!(
        fall < start - 1.0,
        "an underpowered lift should be sinking, not climbing: {fall:.4} m/s"
    );
    assert!(
        realised <= motor + 1e-2,
        "a motor may not spend more than the {motor:.2} m/s² it declares: \
         realised {realised:.2} m/s²"
    );
    assert!(
        realised > 0.0,
        "the motor should still be pushing what it has: realised {realised:.2} m/s²"
    );
}

// ---------------------------------------------------------------------------
// Deck suspension
// ---------------------------------------------------------------------------

/// The authored tilt is a promise about degrees, and this is where it is kept.
///
/// `DeckSuspension` converts degrees to a compliance through the solver's
/// position-correction beta, so the number only survives the round trip if that
/// conversion matches what the rows actually do. Torque is applied directly
/// rather than by standing a body on the deck: what is under test is the
/// spring, not the contact.
#[test]
fn the_deck_tips_by_its_authored_angle_under_the_reference_load() {
    let geometry = FlatQuadGeometry::new(40.0);

    for tilt_degrees in [1.0_f32, 2.0, 4.0] {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        let mut world = PhysicsWorld::new(config);
        let at = Vector3::new(0.0, 20.0, 0.0);
        let (body, mut platform) = spawn_platform_with_suspension(
            &mut world,
            at,
            at,
            0.0,
            DeckSuspension {
                tilt_degrees,
                damping: 0.5,
                yaw_resistance: DeckSuspension::default_yaw_resistance(),
            },
        );

        // The reference load standing on the +x edge, as a torque about z.
        let lever_arm = HALF_EXTENTS.x;
        let torque = Vector3::new(0.0, 0.0, -REFERENCE_LOAD_KG * 9.81 * lever_arm);
        let dt = 1.0 / 240.0;
        let mut debug_lines = DebugLines::default();
        for _ in 0..1200 {
            let position = {
                let p = world.body(body).unwrap().position();
                Vector3::new(p.x, p.y, p.z)
            };
            platform.update_heading(&position);
            let _ = world.set_body_drive(
                body,
                &DriveCommand::medium(
                    platform.target_velocity(&position, FRAME_DT),
                    Vector3::zeros(),
                    MOTOR_MAX_ACCEL,
                    0.0,
                ),
            );
            world.update_contacts(dt, 4, &geometry, &[], &mut debug_lines);
            for _ in 0..4 {
                world.substep(dt, &geometry, &[]);
                world
                    .body_mut(body)
                    .unwrap()
                    .apply_angular_impulse(torque * dt);
            }
        }

        let settled = deck_tilt_degrees(&world, body);
        assert!(
            (settled - tilt_degrees).abs() < 0.15,
            "authored {tilt_degrees} deg, deck settled at {settled:.3}"
        );
    }
}

/// A rigid deck is still rigid. The suspension is opt-in, and a platform that
/// did not ask for give must not acquire any.
#[test]
fn a_rigid_deck_does_not_tip() {
    let geometry = FlatQuadGeometry::new(40.0);
    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    let mut world = PhysicsWorld::new(config);
    let at = Vector3::new(0.0, 20.0, 0.0);
    let (body, mut platform) = spawn_platform(&mut world, at, at, 0.0);

    let torque = Vector3::new(0.0, 0.0, -REFERENCE_LOAD_KG * 9.81 * HALF_EXTENTS.x);
    let dt = 1.0 / 240.0;
    let mut debug_lines = DebugLines::default();
    for _ in 0..600 {
        let position = {
            let p = world.body(body).unwrap().position();
            Vector3::new(p.x, p.y, p.z)
        };
        platform.update_heading(&position);
        let _ = world.set_body_drive(
            body,
            &DriveCommand::medium(
                platform.target_velocity(&position, FRAME_DT),
                Vector3::zeros(),
                MOTOR_MAX_ACCEL,
                0.0,
            ),
        );
        world.update_contacts(dt, 4, &geometry, &[], &mut debug_lines);
        for _ in 0..4 {
            world.substep(dt, &geometry, &[]);
            world
                .body_mut(body)
                .unwrap()
                .apply_angular_impulse(torque * dt);
        }
    }

    let settled = deck_tilt_degrees(&world, body).abs();
    assert!(settled < 0.05, "rigid deck leaned {settled:.4} deg");
}

/// What the player is meant to see: land on the edge and the deck swings well
/// past where standing there would hold it, then rings back to level.
///
/// The peak matters as much as the settle. A deck that only sagged to its
/// static tilt would be a suspension the player never notices, and a deck that
/// never came back would be a platform that had stopped working.
#[test]
fn a_landing_rings_the_deck_and_then_dies_away() {
    let geometry = FlatQuadGeometry::new(40.0);
    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    let mut world = PhysicsWorld::new(config);
    let at = Vector3::new(0.0, 20.0, 0.0);
    let suspension = DeckSuspension {
        tilt_degrees: 2.0,
        damping: 0.5,
        yaw_resistance: DeckSuspension::default_yaw_resistance(),
    };
    let (body, mut platform) = spawn_platform_with_suspension(&mut world, at, at, 0.0, suspension);

    // The reference load arriving on the +x edge at jump speed.
    world
        .body_mut(body)
        .unwrap()
        .apply_angular_impulse(Vector3::new(
            0.0,
            0.0,
            -REFERENCE_LOAD_KG * 7.0 * HALF_EXTENTS.x,
        ));

    let dt = 1.0 / 240.0;
    let mut debug_lines = DebugLines::default();
    let mut peak = 0.0f32;
    let mut peak_frame = 0;
    let mut settled_by = None;
    for frame in 0..600 {
        let position = {
            let p = world.body(body).unwrap().position();
            Vector3::new(p.x, p.y, p.z)
        };
        platform.update_heading(&position);
        let _ = world.set_body_drive(
            body,
            &DriveCommand::medium(
                platform.target_velocity(&position, FRAME_DT),
                Vector3::zeros(),
                MOTOR_MAX_ACCEL,
                0.0,
            ),
        );
        world.update_contacts(dt, 4, &geometry, &[], &mut debug_lines);
        for _ in 0..4 {
            world.substep(dt, &geometry, &[]);
        }

        let tilt = deck_tilt_degrees(&world, body).abs();
        if tilt > peak {
            peak = tilt;
            peak_frame = frame;
        }
        if tilt > 0.1 {
            settled_by = None;
        } else if settled_by.is_none() {
            settled_by = Some(frame);
        }
    }

    let settled_at = settled_by.map(|f| f as f32 * dt);
    eprintln!(
        "landing: peak {peak:.2} deg at {:.2}s, level again by {settled_at:?}",
        peak_frame as f32 * dt
    );
    assert!(
        peak > 3.0 * suspension.tilt_degrees,
        "a landing should throw the deck well past its standing tilt, peaked at {peak:.2} deg"
    );
    assert!(
        peak < 15.0,
        "a landing should wobble the deck, not capsize it: {peak:.2} deg"
    );
    let settled_at = settled_at.expect("deck should come back to level");
    assert!(
        settled_at < 2.0,
        "the ring should be over quickly, still swinging at {settled_at:.2}s"
    );
}

/// The suspension must not cost the platform its day job. A deck that rings
/// still has to shuttle, and the angular damping it brings must not be felt by
/// a motor that only ever pushes through the centre of mass.
#[test]
fn a_wobbling_deck_still_runs_its_route() {
    let geometry = FlatQuadGeometry::new(80.0);
    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    let mut world = PhysicsWorld::new(config);
    let from = Vector3::new(0.0, 20.0, 0.0);
    let to = Vector3::new(10.0, 20.0, 0.0);
    let (body, mut platform) = spawn_platform_with_suspension(
        &mut world,
        from,
        to,
        2.0,
        DeckSuspension {
            tilt_degrees: 2.0,
            damping: 0.5,
            yaw_resistance: DeckSuspension::default_yaw_resistance(),
        },
    );

    let dt = 1.0 / 240.0;
    let mut debug_lines = DebugLines::default();
    let mut reached_far_end = false;
    for _ in 0..2400 {
        run_frame(&mut world, body, &mut platform, &geometry, &mut debug_lines);
        let p = world.body(body).unwrap().position();
        if (p.x - to.x).abs() < 0.3 {
            reached_far_end = true;
        }
    }

    assert!(
        reached_far_end,
        "platform never reached the far end of its route"
    );
    let tilt = deck_tilt_degrees(&world, body).abs();
    assert!(
        tilt < 0.5,
        "a platform running its route should stay level, leaning {tilt:.3} deg"
    );
}

// ---------------------------------------------------------------------------
// Routes and cornering
// ---------------------------------------------------------------------------

/// Build a platform on an arbitrary route.
fn spawn_route(
    world: &mut PhysicsWorld,
    waypoints: Vec<Vector3<f32>>,
    looping: RouteLoop,
    motion: SeekMotion,
) -> (RigidBodyHandle, MovingPlatform) {
    let start = waypoints[0];
    let (body, _) = spawn_platform(world, start, start, motion.speed);
    (body, MovingPlatform::new(waypoints, looping, motion))
}

/// A platform with more than two waypoints visits all of them, in order, and
/// goes round again.
#[test]
fn a_circuit_visits_every_waypoint_in_order() {
    let geometry = FlatQuadGeometry::new(128.0);
    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    let mut world = PhysicsWorld::new(config);

    let corners = vec![
        Vector3::new(0.0, 20.0, 0.0),
        Vector3::new(12.0, 20.0, 0.0),
        Vector3::new(12.0, 20.0, 12.0),
        Vector3::new(0.0, 20.0, 12.0),
    ];
    let (body, mut platform) = spawn_route(
        &mut world,
        corners.clone(),
        RouteLoop::Circuit,
        SeekMotion {
            speed: 3.0,
            tau: 0.25,
        },
    );

    let mut debug_lines = DebugLines::default();
    let mut visited = vec![platform.target_index()];
    for _ in 0..7200 {
        run_frame(&mut world, body, &mut platform, &geometry, &mut debug_lines);
        if *visited.last().unwrap() != platform.target_index() {
            visited.push(platform.target_index());
        }
    }

    eprintln!("circuit visited {visited:?}");
    assert!(
        visited.len() > 6,
        "a circuit should keep going round: only {} legs in 30 s",
        visited.len()
    );
    assert!(
        visited.windows(2).all(|w| w[1] == (w[0] + 1) % 4),
        "waypoints should be visited in order, got {visited:?}"
    );
}

/// The point of the whole model: the commanded velocity is continuous, so a
/// turnaround is a deceleration and an acceleration rather than an inversion.
///
/// Measured as the largest single-frame change in the platform's velocity
/// across a full reversal. The old model stepped its target from `+speed` to
/// `−speed` in one frame and the motor chased at its full 40 m/s² budget; this
/// one asks for a change no larger than thrust × frame.
#[test]
fn a_turnaround_eases_through_zero_rather_than_inverting() {
    let geometry = FlatQuadGeometry::new(64.0);
    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    let mut world = PhysicsWorld::new(config);

    let motion = SeekMotion {
        speed: 2.0,
        tau: 0.25,
    };
    let from = Vector3::new(0.0, 20.0, 0.0);
    let to = Vector3::new(8.0, 20.0, 0.0);
    let (body, mut platform) = spawn_route(&mut world, vec![from, to], RouteLoop::Shuttle, motion);

    let mut debug_lines = DebugLines::default();
    let mut previous = world.body(body).unwrap().linear_velocity();
    let mut biggest_step = 0.0f32;
    let mut reversed = false;
    let mut furthest = from.x;

    // Long enough to spin up, run the leg, and turn around.
    for _ in 0..2400 {
        run_frame(&mut world, body, &mut platform, &geometry, &mut debug_lines);
        let velocity = world.body(body).unwrap().linear_velocity();
        furthest = furthest.max(world.body(body).unwrap().position().x);
        if platform.target_index() == 0 {
            reversed = true;
            biggest_step = biggest_step.max((velocity - previous).magnitude());
        }
        previous = velocity;
    }

    assert!(reversed, "platform never turned around");

    // What the old model demanded: an inverted target, chased at the motor's
    // whole budget. The body can still outpace the command's own rate of
    // change, because it is closing tracking lag at the same time, so the
    // meaningful claim is not `thrust · dt` — it is that the turn no longer
    // saturates the motor.
    let motor_budget = MOTOR_MAX_ACCEL * FRAME_DT;
    let overshoot = furthest - to.x;
    let predicted_overshoot = 0.31 * motion.speed * motion.tau;
    eprintln!(
        "turnaround: biggest velocity step {biggest_step:.4} m/s (the motor \
         could do {motor_budget:.4}), overshot the endpoint by {overshoot:.3} m \
         against a predicted {predicted_overshoot:.3}"
    );
    assert!(
        biggest_step < motor_budget * 0.5,
        "the turnaround should ease, not invert: stepped {biggest_step:.4} m/s \
         of an available {motor_budget:.4}"
    );

    // `0.31 · speed · tau` is what `SeekMotion::tau` tells authors to leave
    // clearance for, so it is worth holding the model to it.
    assert!(
        (overshoot - predicted_overshoot).abs() < 0.2 * predicted_overshoot,
        "the overshoot should match the documented {predicted_overshoot:.3} m, \
         measured {overshoot:.3} m"
    );
}

/// A gentle bend should barely be felt, and a hard corner should be visible.
/// One mechanism, and the difference between them is the turn angle alone.
#[test]
fn a_slight_bend_costs_less_speed_than_a_hard_corner() {
    let geometry = FlatQuadGeometry::new(256.0);

    let corner_speed = |turn: Vector3<f32>| {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        let mut world = PhysicsWorld::new(config);
        let motion = SeekMotion {
            speed: 3.0,
            tau: 0.25,
        };
        let waypoints = vec![
            Vector3::new(0.0, 20.0, 0.0),
            Vector3::new(20.0, 20.0, 0.0),
            Vector3::new(20.0, 20.0, 0.0) + turn,
        ];
        let (body, mut platform) = spawn_route(&mut world, waypoints, RouteLoop::Shuttle, motion);

        let mut debug_lines = DebugLines::default();
        let mut slowest = f32::MAX;
        for _ in 0..3600 {
            run_frame(&mut world, body, &mut platform, &geometry, &mut debug_lines);
            // Only once it is on the second leg, i.e. taking the corner.
            if platform.target_index() == 2 {
                slowest = slowest.min(world.body(body).unwrap().linear_velocity().magnitude());
            }
        }
        slowest
    };

    // A 10 degree bend, and a square corner, both 20 m along.
    let bend = corner_speed(Vector3::new(19.7, 0.0, 3.47));
    let square = corner_speed(Vector3::new(0.0, 0.0, 20.0));

    eprintln!(
        "slowest through a 10 deg bend: {bend:.3} m/s; through a square corner: {square:.3} m/s"
    );
    assert!(
        bend > 2.7,
        "a slight bend should barely cost any speed: dropped to {bend:.3} of 3.0"
    );
    assert!(
        square < bend - 0.3,
        "a square corner should cost visibly more than a bend: {square:.3} vs {bend:.3}"
    );
}

/// Knocked off course, the platform returns to the waypoint it was already
/// heading for — it does not skip ahead, and it does not go backwards to find
/// its place again.
///
/// There is no schedule to fall out of step with, so this needs no recovery
/// rule: the thrust points at the target waypoint from wherever the platform
/// actually is. The hard case is being knocked *forwards*, which an earlier
/// design would have answered by reversing until it was behind an imaginary
/// pacer.
#[test]
fn a_platform_knocked_off_course_resumes_the_waypoint_it_was_heading_for() {
    let geometry = FlatQuadGeometry::new(256.0);

    for shove in [
        Vector3::new(0.0, 0.0, 14.0),  // sideways
        Vector3::new(-14.0, 0.0, 0.0), // backwards
        Vector3::new(14.0, 0.0, 0.0),  // forwards, but not past the waypoint
    ] {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        let mut world = PhysicsWorld::new(config);

        let from = Vector3::new(0.0, 20.0, 0.0);
        let to = Vector3::new(40.0, 20.0, 0.0);
        let (body, mut platform) = spawn_route(
            &mut world,
            vec![from, to],
            RouteLoop::Shuttle,
            SeekMotion {
                speed: 2.0,
                tau: 0.25,
            },
        );

        let mut debug_lines = DebugLines::default();
        for _ in 0..240 {
            run_frame(&mut world, body, &mut platform, &geometry, &mut debug_lines);
        }
        let heading_for = platform.target_index();

        // The blast.
        {
            let body_mut = world.body_mut(body).unwrap();
            let velocity = body_mut.linear_velocity();
            body_mut.set_linear_velocity(velocity + shove);
        }

        let mut went_backwards_past_the_start = false;
        for _ in 0..2400 {
            run_frame(&mut world, body, &mut platform, &geometry, &mut debug_lines);
            let p = world.body(body).unwrap().position();
            if p.x < from.x - 1.0 {
                went_backwards_past_the_start = true;
            }
            if platform.target_index() != heading_for {
                break;
            }
        }

        let p = world.body(body).unwrap().position();
        let off_route = (p.z - from.z).abs();
        eprintln!(
            "shove {shove:?}: ended at ({:.2}, {:.2}, {:.2}), off-route {off_route:.3} m",
            p.x, p.y, p.z
        );

        assert!(
            !went_backwards_past_the_start,
            "shove {shove:?}: the platform should fly at its waypoint, never \
             reverse down its own route to find its place"
        );
        assert!(
            platform.target_index() != heading_for,
            "shove {shove:?}: the platform should still reach the waypoint it \
             was heading for"
        );
        assert!(
            off_route < 0.5,
            "shove {shove:?}: the platform should be back on its line by the \
             time it arrives, {off_route:.3} m off"
        );
    }
}
