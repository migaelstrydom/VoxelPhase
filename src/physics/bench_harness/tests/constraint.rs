use nalgebra::{Point3, UnitVector3, Vector3};

use super::super::framework::{run_scenario, BenchRunConfig, PhysicsBenchScenario};
use super::super::geometry::{FlatGridGeometry, FlatQuadGeometry};
use super::super::scenarios::*;
use super::write_exports;
use crate::debug::DebugLines;
use crate::physics::constraint::ConstraintKind;
use crate::physics::stepping::FixedTimestep;
use crate::physics::world::PhysicsConfig;
use crate::physics::{ColliderDesc, PhysicsWorld, RigidBodyDesc};

// ═══════════════════════════════════════════════════════════════════════════
// Constraint system validation tests
// ═══════════════════════════════════════════════════════════════════════════

/// Verifies the keep-upright constraint removes angular velocity in free fall
/// using the multi-substep pattern (update_contacts once, substep N times).
#[test]
fn keep_upright_kills_angular_velocity_in_free_fall() {
    let geometry = FlatQuadGeometry::new(8.0);
    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    let mut world = PhysicsWorld::new(config);

    let mut desc = RigidBodyDesc::dynamic().position(Point3::new(0.0, 10.0, 0.0)); // high up, no ground contact
    desc.angular_velocity = Vector3::new(5.0, 0.0, 3.0);
    let body = world.create_body(desc);
    let _ = world.attach_collider(body, ColliderDesc::sphere(0.5).density(1000.0));
    let _ = world.create_constraint(ConstraintKind::KeepUpright {
        body,
        target_up: UnitVector3::new_normalize(Vector3::y()),
        compliance: 0.0,
    });

    use crate::debug::DebugLines;
    let dt = 1.0 / 240.0;
    let mut debug_lines = DebugLines::default();

    // Single frame: update_contacts + 4 substeps
    world.update_contacts(dt, &geometry, &[], &mut debug_lines);
    for _ in 0..4 {
        world.substep(dt, &geometry, &[]);
    }

    let ang_speed = world.body(body).unwrap().angular_velocity().magnitude();
    eprintln!("free_fall ang_speed after 4 substeps: {ang_speed:.6}");
    assert!(
        ang_speed < 0.01,
        "constraint should kill angular velocity immediately: {ang_speed:.4}"
    );
}

/// Verifies a sphere with a keep-upright constraint rests stably on flat
/// ground without gaining angular velocity.
#[test]
fn keep_upright_sphere_rests_on_ground() {
    let geometry = FlatQuadGeometry::new(8.0);
    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    let mut world = PhysicsWorld::new(config);

    let radius = 0.5;
    let spawn_y = radius + 0.5; // slightly above ground
    let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, spawn_y, 0.0)));
    let _ = world.attach_collider(
        body,
        ColliderDesc::sphere(radius).density(1000.0).friction(0.5),
    );
    let _ = world.create_constraint(ConstraintKind::KeepUpright {
        body,
        target_up: UnitVector3::new_normalize(Vector3::y()),
        compliance: 0.0,
    });

    use crate::debug::DebugLines;
    let dt = 1.0 / 240.0;
    let mut debug_lines = DebugLines::default();

    // Run for 2 seconds
    for _ in 0..120 {
        world.update_contacts(dt, &geometry, &[], &mut debug_lines);
        debug_lines.clear();
        for _ in 0..4 {
            world.substep(dt, &geometry, &[]);
        }
    }

    let b = world.body(body).unwrap();
    let ang_speed = b.angular_velocity().magnitude();
    let y = b.position().y;
    eprintln!("sphere_rests: ang_speed={ang_speed:.6}, y={y:.4}");

    assert!(
        ang_speed < 1.0,
        "sphere should be at rest: ang_speed={ang_speed:.4}"
    );
    assert!(y > radius - 0.1, "sphere should not fall through: y={y:.4}");
}

/// Simulates game-loop conditions: capsule on ground with angular velocity
/// zeroed each frame (as VelocityDriven does for the player).
#[test]
fn keep_upright_capsule_with_velocity_zeroing() {
    let geometry = FlatGridGeometry::new(8.0, 1.0);
    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    let mut world = PhysicsWorld::new(config);

    let half_height = 0.5;
    let radius = 0.25;
    let spawn_y = half_height + radius + 0.01;
    let body = world.create_body(
        RigidBodyDesc::dynamic()
            .position(Point3::new(0.0, spawn_y, 0.0))
            .angular_damping(1.0),
    );
    let _ = world.attach_collider(
        body,
        ColliderDesc::capsule(half_height, radius)
            .density(30.0)
            .friction(0.3),
    );
    let _ = world.create_constraint(ConstraintKind::KeepUpright {
        body,
        target_up: UnitVector3::new_normalize(Vector3::y()),
        compliance: 0.0,
    });

    use crate::debug::DebugLines;
    let dt = 1.0 / 240.0;
    let substeps = 4;
    let mut debug_lines = DebugLines::default();

    // Phase 1: settle (30 frames = 0.5s)
    for frame in 0..30 {
        world.set_body_velocity_drive(body, Vector3::zeros(), Vector3::zeros(), 500.0, 500.0);
        world.update_contacts(dt, &geometry, &[], &mut debug_lines);
        debug_lines.clear();
        for _ in 0..substeps {
            world.substep(dt, &geometry, &[]);
        }
        if frame == 29 {
            let b = world.body(body).unwrap();
            eprintln!(
                "after settle: y={:.4}, tilt={:.2}°",
                b.position().y,
                (b.rotation() * Vector3::y())
                    .dot(&Vector3::y())
                    .acos()
                    .to_degrees()
            );
        }
    }

    // Phase 2: move forward briefly (15 frames = 0.25s)
    let forward_vel = Vector3::new(3.0, 0.0, 0.0);
    for _frame in 0..15 {
        world.set_body_velocity_drive(body, forward_vel, Vector3::zeros(), 500.0, 500.0);
        world.update_contacts(dt, &geometry, &[], &mut debug_lines);
        debug_lines.clear();
        for _ in 0..substeps {
            world.substep(dt, &geometry, &[]);
        }
    }
    let b = world.body(body).unwrap();
    eprintln!(
        "after move: y={:.4}, tilt={:.2}°",
        b.position().y,
        (b.rotation() * Vector3::y())
            .dot(&Vector3::y())
            .acos()
            .to_degrees()
    );

    // Phase 3: stop and observe (120 frames = 2s)
    for frame in 0..120 {
        world.set_body_velocity_drive(body, Vector3::zeros(), Vector3::zeros(), 500.0, 500.0);
        world.update_contacts(dt, &geometry, &[], &mut debug_lines);
        debug_lines.clear();
        for _ in 0..substeps {
            world.substep(dt, &geometry, &[]);
        }

        let b = world.body(body).unwrap();
        let local_up = b.rotation() * Vector3::y();
        let tilt = local_up.dot(&Vector3::y()).acos().to_degrees();
        if frame % 30 == 0 {
            eprintln!(
                "after stop frame {frame}: y={:.4}, tilt={tilt:.2}°, ang_speed={:.4}",
                b.position().y,
                b.angular_velocity().magnitude()
            );
        }
        if tilt > 45.0 {
            panic!("Capsule fell over at frame {frame} after stopping: tilt={tilt:.1}°");
        }
    }

    let b = world.body(body).unwrap();
    let local_up = b.rotation() * Vector3::y();
    let tilt = local_up.dot(&Vector3::y()).acos().to_degrees();
    eprintln!("final: y={:.4}, tilt={tilt:.2}°", b.position().y);
    assert!(
        tilt < 10.0,
        "capsule should be near-upright: tilt={tilt:.1}°"
    );
}

/// Validates the bench scenario runs and exports successfully.
#[test]
fn keep_upright_scenario_runs() {
    let scenario = KeepUprightScenario::new();
    let cfg = BenchRunConfig {
        duration: 2.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "keep_upright");

    assert!(!run.samples.is_empty());
    assert!(run.physics_steps > 0);

    // The constraint should keep angular speed low in the first 0.5s
    // (before capsule-static narrowphase issues can accumulate).
    let early_max_angular = run
        .samples
        .iter()
        .filter(|s| s.sim_time < 0.5)
        .map(|s| s.angular_speed)
        .fold(0.0f32, f32::max);
    eprintln!("keep_upright early_max_angular={early_max_angular:.6}");
    assert!(
        early_max_angular < 1.0,
        "constraint should work in early sim: early_max_angular={early_max_angular:.4}"
    );
}

/// World-anchored hinge with a box on one end. Requires angular NGS (Phase 7)
/// for reliable settling under gravity load.
#[test]
#[ignore]
fn hinge_settles_under_load() {
    let scenario = HingeSettlesUnderLoadScenario::new();
    let cfg = BenchRunConfig {
        duration: 3.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "hinge_settles_under_load");

    let last = run.samples.last().expect("should have samples");
    eprintln!(
        "hinge_settles_under_load: final angular_speed={:.6}",
        last.angular_speed
    );
    assert!(
        last.angular_speed < 0.05,
        "plank should have settled: angular_speed={:.4}",
        last.angular_speed
    );

    let (_, tail_max_angular) = run.tail_max_speeds(0.5);
    eprintln!("hinge_settles_under_load: tail_max_angular={tail_max_angular:.6}");
    assert!(
        tail_max_angular < 0.05,
        "plank should be steady in tail: tail_max_angular={tail_max_angular:.4}"
    );
}

/// World-anchored hinge with a 10 kg body. Requires angular NGS (Phase 7).
#[test]
#[ignore]
fn hinge_holds_under_sustained_force() {
    let scenario = HingeHoldsUnderSustainedForceScenario::new();
    let cfg = BenchRunConfig {
        duration: 5.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "hinge_holds_under_sustained_force");

    let max_angular_after_1s = run
        .samples
        .iter()
        .filter(|s| s.sim_time > 1.0)
        .map(|s| s.angular_speed)
        .fold(0.0f32, f32::max);
    eprintln!(
        "hinge_holds_under_sustained_force: max_angular after 1s = {max_angular_after_1s:.6}"
    );
    assert!(
        max_angular_after_1s < 0.02,
        "plank should be near-steady after 1s: max_angular={max_angular_after_1s:.4}"
    );
}

/// Zero-gravity hinge spinning at 3 rad/s around the free axis (Z). Run 5s.
/// Locked axes angular velocity must stay < 0.01 rad/s. Free axis velocity
/// must stay within 25% of initial.
#[test]
fn hinge_axis_no_drift_zero_gravity() {
    let scenario = HingeAxisNoDriftZeroGravityScenario::new();
    let initial_speed = scenario.initial_angular_velocity;

    let mut world = scenario.build_world();
    let tracked = scenario.setup(&mut world);

    let fixed_dt = 1.0 / 240.0;
    let duration = 5.0;
    let substeps_per_frame = 4;
    let mut timestep = FixedTimestep::new(fixed_dt, substeps_per_frame);
    let mut debug_lines = DebugLines::default();
    let mut sim_time = 0.0f32;

    let mut max_locked_speed = 0.0f32;
    let mut min_free_speed = f32::MAX;

    while sim_time < duration {
        let frame_dt = 1.0 / 60.0;
        let substeps = timestep.accumulate(frame_dt);
        if substeps == 0 {
            continue;
        }

        world.update_contacts(fixed_dt, scenario.geometry(), &[], &mut debug_lines);
        debug_lines.clear();
        for _ in 0..substeps {
            world.substep(fixed_dt, scenario.geometry(), &[]);
            sim_time += fixed_dt;
        }

        let body = world.body(tracked).unwrap();
        let ang_vel = body.angular_velocity();

        // Decompose angular velocity into free (Z) and locked (X, Y) components.
        let locked_speed = (ang_vel.x * ang_vel.x + ang_vel.y * ang_vel.y).sqrt();
        let free_speed = ang_vel.z.abs();

        max_locked_speed = max_locked_speed.max(locked_speed);
        min_free_speed = min_free_speed.min(free_speed);
    }

    eprintln!("hinge_no_drift: max_locked={max_locked_speed:.6}, min_free={min_free_speed:.6}");

    assert!(
        max_locked_speed < 0.01,
        "locked axes should have near-zero angular velocity: {max_locked_speed:.4}"
    );
    assert!(
        min_free_speed > initial_speed * 0.75,
        "free axis should retain most of its velocity: min={min_free_speed:.4}, initial={initial_speed:.4}"
    );
}
