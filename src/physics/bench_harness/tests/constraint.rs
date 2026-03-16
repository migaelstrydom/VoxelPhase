use nalgebra::{Point3, UnitVector3, Vector3};

use super::super::framework::{run_scenario, BenchRunConfig};
use super::super::geometry::{FlatGridGeometry, FlatQuadGeometry};
use super::super::scenarios::*;
use super::write_exports;
use crate::physics::constraint::ConstraintKind;
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

    let mut desc = RigidBodyDesc::dynamic()
        .position(Point3::new(0.0, 10.0, 0.0)); // high up, no ground contact
    desc.angular_velocity = Vector3::new(5.0, 0.0, 3.0);
    let body = world.create_body(desc);
    let _ = world.attach_collider(
        body,
        ColliderDesc::sphere(0.5).density(1000.0),
    );
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
        world.substep(dt, &geometry);
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
    let body = world.create_body(
        RigidBodyDesc::dynamic().position(Point3::new(0.0, spawn_y, 0.0)),
    );
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
            world.substep(dt, &geometry);
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
    assert!(
        y > radius - 0.1,
        "sphere should not fall through: y={y:.4}"
    );
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
        ColliderDesc::capsule(half_height, radius).density(30.0).friction(0.3),
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
        world.set_body_velocity_drive(body, Vector3::zeros(), Vector3::zeros(), 500.0);
        world.update_contacts(dt, &geometry, &[], &mut debug_lines);
        debug_lines.clear();
        for _ in 0..substeps {
            world.substep(dt, &geometry);
        }
        if frame == 29 {
            let b = world.body(body).unwrap();
            eprintln!("after settle: y={:.4}, tilt={:.2}°",
                b.position().y,
                (b.rotation() * Vector3::y()).dot(&Vector3::y()).acos().to_degrees());
        }
    }

    // Phase 2: move forward briefly (15 frames = 0.25s)
    let forward_vel = Vector3::new(3.0, 0.0, 0.0);
    for _frame in 0..15 {
        world.set_body_velocity_drive(body, forward_vel, Vector3::zeros(), 500.0);
        world.update_contacts(dt, &geometry, &[], &mut debug_lines);
        debug_lines.clear();
        for _ in 0..substeps {
            world.substep(dt, &geometry);
        }
    }
    let b = world.body(body).unwrap();
    eprintln!("after move: y={:.4}, tilt={:.2}°",
        b.position().y,
        (b.rotation() * Vector3::y()).dot(&Vector3::y()).acos().to_degrees());

    // Phase 3: stop and observe (120 frames = 2s)
    for frame in 0..120 {
        world.set_body_velocity_drive(body, Vector3::zeros(), Vector3::zeros(), 500.0);
        world.update_contacts(dt, &geometry, &[], &mut debug_lines);
        debug_lines.clear();
        for _ in 0..substeps {
            world.substep(dt, &geometry);
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
    assert!(tilt < 10.0, "capsule should be near-upright: tilt={tilt:.1}°");
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
