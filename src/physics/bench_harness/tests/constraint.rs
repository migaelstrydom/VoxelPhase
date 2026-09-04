use nalgebra::{Point3, UnitVector3, Vector3};

use super::super::framework::{run_scenario, BenchRunConfig, PhysicsBenchScenario};
use super::super::geometry::{FlatGridGeometry, FlatQuadGeometry};
use super::super::scenarios::*;
use super::write_exports;
use crate::debug::DebugLines;
use crate::physics::constraint::ConstraintKind;
use crate::physics::stepping::FixedTimestep;
use crate::physics::world::PhysicsConfig;
use crate::physics::{ColliderDesc, ConstraintHandle, DriveCommand, PhysicsWorld, RigidBodyDesc};

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
        max_impulse: f32::INFINITY,
    });

    use crate::debug::DebugLines;
    let dt = 1.0 / 240.0;
    let mut debug_lines = DebugLines::default();

    // Single frame: update_contacts + 4 substeps
    world.update_contacts(dt, 4, &geometry, &[], &mut debug_lines);
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

/// A bounded KeepUpright must keep righting after its rows saturate.
///
/// `check_constraint_breakage` deactivates any constraint whose finite-bound
/// rows saturate, which is right for joints and wrong for an attitude
/// controller. Without the KeepUpright exemption this constraint dies on its
/// first frame and the spin survives, decaying only through angular damping.
#[test]
fn keep_upright_bounded_survives_saturation() {
    let geometry = FlatQuadGeometry::new(8.0);
    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    let mut world = PhysicsWorld::new(config);

    let desc = RigidBodyDesc::dynamic()
        .position(Point3::new(0.0, 50.0, 0.0)) // free fall, no ground contact
        .angular_velocity(Vector3::new(5.0, 0.0, 0.0));
    let body = world.create_body(desc);
    let _ = world.attach_collider(body, ColliderDesc::sphere(0.5).density(1000.0));
    let _ = world.create_constraint(ConstraintKind::KeepUpright {
        body,
        target_up: UnitVector3::new_normalize(Vector3::y()),
        compliance: 0.0,
        max_impulse: 400.0,
    });

    let dt = 1.0 / 240.0;
    let mut debug_lines = DebugLines::default();
    for _ in 0..30 {
        world.update_contacts(dt, 4, &geometry, &[], &mut debug_lines);
        for _ in 0..4 {
            world.substep(dt, &geometry, &[]);
        }
    }

    let ang_speed = world.body(body).unwrap().angular_velocity().magnitude();
    eprintln!("bounded keep-upright ang_speed after 30 frames: {ang_speed:.6}");
    assert!(
        ang_speed < 0.5,
        "bounded constraint should stay active and right the body: {ang_speed:.4}"
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
        max_impulse: f32::INFINITY,
    });

    use crate::debug::DebugLines;
    let dt = 1.0 / 240.0;
    let mut debug_lines = DebugLines::default();

    // Run for 2 seconds
    for _ in 0..120 {
        world.update_contacts(dt, 4, &geometry, &[], &mut debug_lines);
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
/// zeroed each frame (as the player's velocity drive does).
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
        max_impulse: f32::INFINITY,
    });

    use crate::debug::DebugLines;
    let dt = 1.0 / 240.0;
    let substeps = 4;
    let mut debug_lines = DebugLines::default();

    // Phase 1: settle (30 frames = 0.5s)
    for frame in 0..30 {
        world.set_body_drive(
            body,
            &DriveCommand::support(Vector3::zeros(), Vector3::zeros(), 500.0, 500.0),
        );
        world.update_contacts(dt, substeps as u32, &geometry, &[], &mut debug_lines);
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
        world.set_body_drive(
            body,
            &DriveCommand::support(forward_vel, Vector3::zeros(), 500.0, 500.0),
        );
        world.update_contacts(dt, substeps as u32, &geometry, &[], &mut debug_lines);
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
        world.set_body_drive(
            body,
            &DriveCommand::support(Vector3::zeros(), Vector3::zeros(), 500.0, 500.0),
        );
        world.update_contacts(dt, substeps as u32, &geometry, &[], &mut debug_lines);
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

/// World-anchored hinge with a box on one end. Angular NGS position correction
/// enables reliable settling under gravity load.
#[test]
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

/// World-anchored hinge with a 10 kg body. Angular NGS position correction
/// prevents angular drift under sustained gravitational torque.
#[test]
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

/// Verify angular NGS doesn't cause oscillation growth. Reuses the
/// hinge_settles_under_load setup. Samples angular velocity peaks each
/// half-oscillation cycle. After the first full cycle, each successive
/// peak must decay monotonically (within 5% tolerance for solver noise).
/// Total kinetic energy at t=3s must be < 1% of energy at t=0.2s.
#[test]
#[cfg(feature = "bench_harness")]
fn hinge_angular_ngs_no_oscillation() {
    let scenario = HingeSettlesUnderLoadScenario::new();
    let cfg = BenchRunConfig {
        duration: 3.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "hinge_angular_ngs_no_oscillation");

    // Find angular velocity peaks: local maxima where sign of angular_speed
    // change flips from increasing to decreasing (half-cycle boundaries).
    let speeds: Vec<f32> = run.samples.iter().map(|s| s.angular_speed).collect();
    let mut peaks: Vec<(f32, f32)> = Vec::new(); // (sim_time, peak_speed)
    for i in 1..speeds.len().saturating_sub(1) {
        if speeds[i] > speeds[i - 1] && speeds[i] >= speeds[i + 1] && speeds[i] > 0.01 {
            peaks.push((run.samples[i].sim_time, speeds[i]));
        }
    }

    eprintln!("hinge_ngs_oscillation: found {} peaks", peaks.len());
    for (i, (t, s)) in peaks.iter().enumerate() {
        eprintln!("  peak {i}: t={t:.4}, angular_speed={s:.6}");
    }

    // After the first full cycle (skip the first peak, which is the initial
    // deflection), each successive peak should be <= 1.05x the previous.
    if peaks.len() >= 3 {
        for i in 2..peaks.len() {
            let ratio = peaks[i].1 / peaks[i - 1].1;
            assert!(
                ratio <= 1.05,
                "peak {} ({:.6}) > 1.05 * peak {} ({:.6}): ratio={ratio:.4} — oscillation growing",
                i,
                peaks[i].1,
                i - 1,
                peaks[i - 1].1,
            );
        }
    }

    // Energy check: kinetic energy at t=3s should be < 1% of energy at t=0.2s.
    // Angular speed is proportional to sqrt(kinetic energy), so we compare
    // speed² values.
    let energy_early = run
        .samples
        .iter()
        .filter(|s| s.sim_time >= 0.15 && s.sim_time <= 0.25)
        .map(|s| s.angular_speed * s.angular_speed)
        .fold(0.0f32, f32::max);
    let energy_late = run
        .samples
        .last()
        .map(|s| s.angular_speed * s.angular_speed)
        .unwrap_or(0.0);

    eprintln!(
        "hinge_ngs_oscillation: energy_early={energy_early:.6}, energy_late={energy_late:.8}"
    );
    if energy_early > 1e-6 {
        let ratio = energy_late / energy_early;
        assert!(
            ratio < 0.01,
            "kinetic energy should decay to < 1%: ratio={ratio:.6}"
        );
    }
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

        world.update_contacts(
            fixed_dt,
            substeps as u32,
            scenario.geometry(),
            &[],
            &mut debug_lines,
        );
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

// ═══════════════════════════════════════════════════════════════════════════
// Fixed joint tests
// ═══════════════════════════════════════════════════════════════════════════

/// Two boxes connected by a breakable Fixed joint, elevated so they're in free
/// fall. A heavy sphere dropped from above impacts one box, stressing the joint
/// beyond its impulse limit. The joint must break cleanly.
#[test]
#[cfg(feature = "bench_harness")]
fn breakable_fixed_joint_breaks_cleanly() {
    let geometry = FlatQuadGeometry::new(10.0);
    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    let mut world = PhysicsWorld::new(config);

    let box_he = Vector3::new(0.25, 0.25, 0.25);
    let box_mass = 2.0f32;
    let box_volume = box_he.x * box_he.y * box_he.z * 8.0;
    let box_density = box_mass / box_volume;

    // Two boxes side by side, elevated so ground contacts don't absorb the
    // impact — the joint is the only connection between them.
    let box_a_pos = Point3::new(-0.3, 3.0, 0.0);
    let box_b_pos = Point3::new(0.3, 3.0, 0.0);

    let box_a = world.create_body(RigidBodyDesc::dynamic().position(box_a_pos));
    let _ = world.attach_collider(
        box_a,
        ColliderDesc::box_shape(box_he)
            .density(box_density)
            .restitution(0.0)
            .friction(0.5),
    );

    let box_b = world.create_body(RigidBodyDesc::dynamic().position(box_b_pos));
    let _ = world.attach_collider(
        box_b,
        ColliderDesc::box_shape(box_he)
            .density(box_density)
            .restitution(0.0)
            .friction(0.5),
    );

    // Fixed joint with a low impulse limit so the sphere impact breaks it.
    // At 240 Hz substep rate, the per-substep impulse from a 20 kg sphere
    // at ~6 m/s hitting a 2 kg box exceeds this threshold on the Y-axis row.
    let fixed_handle: ConstraintHandle = world.create_constraint(ConstraintKind::Fixed {
        body_a: Some(box_a),
        body_b: box_b,
        local_anchor_a: Vector3::new(0.3, 0.0, 0.0),
        local_anchor_b: Vector3::new(-0.3, 0.0, 0.0),
        compliance: 0.0,
        max_impulse: 5.0,
    });

    // Heavy sphere just above box_b with a large initial downward velocity.
    // Since both boxes and sphere are in free fall, relative velocity from
    // a height difference alone is zero. An initial velocity ensures impact.
    let sphere_radius: f32 = 0.3;
    let sphere_mass = 20.0f32;
    let sphere_volume = (4.0 / 3.0) * std::f32::consts::PI * sphere_radius.powi(3);
    let sphere_density = sphere_mass / sphere_volume;
    let sphere_pos = Point3::new(0.3, box_b_pos.y + box_he.y + sphere_radius + 0.5, 0.0);

    let sphere = world.create_body(
        RigidBodyDesc::dynamic()
            .position(sphere_pos)
            .linear_velocity(Vector3::new(0.0, -10.0, 0.0)),
    );
    let _ = world.attach_collider(
        sphere,
        ColliderDesc::sphere(sphere_radius)
            .density(sphere_density)
            .restitution(0.0)
            .friction(0.3),
    );

    let dt = 1.0 / 240.0;
    let substeps = 4;
    let mut debug_lines = DebugLines::default();
    let mut broken = false;
    let mut break_frame = 0u32;

    // Simulate 3 seconds — enough for sphere to fall, impact, and settle.
    for frame in 0..180 {
        world.update_contacts(dt, substeps as u32, &geometry, &[], &mut debug_lines);
        debug_lines.clear();
        for _ in 0..substeps {
            world.substep(dt, &geometry, &[]);
        }

        if !broken {
            if let Some(c) = world.constraint(fixed_handle) {
                if !c.active {
                    broken = true;
                    break_frame = frame;
                    eprintln!("joint broke at frame {frame}");
                }
            }
        }
    }

    assert!(
        broken,
        "Fixed joint should have broken under the sphere impact"
    );

    eprintln!("breakable_fixed: broke at frame {break_frame}");

    // Both boxes should have bounded speeds (no explosion).
    let speed_a = world.body(box_a).unwrap().linear_velocity().magnitude();
    let speed_b = world.body(box_b).unwrap().linear_velocity().magnitude();
    eprintln!("breakable_fixed: speed_a={speed_a:.4}, speed_b={speed_b:.4}");

    assert!(
        speed_a < 10.0,
        "box_a should not explode: speed={speed_a:.4}"
    );
    assert!(
        speed_b < 10.0,
        "box_b should not explode: speed={speed_b:.4}"
    );

    // Constraint should be deactivated.
    let constraint = world.constraint(fixed_handle).unwrap();
    assert!(
        !constraint.active,
        "constraint should be deactivated after break"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// BallJoint tests
// ═══════════════════════════════════════════════════════════════════════════

/// World-anchored BallJoint with a 5 kg weight hanging 1 m below the anchor.
/// A 5 m/s horizontal impulse starts the pendulum swinging. After 5 seconds
/// it must have settled nearly directly below the anchor.
#[test]
#[cfg(feature = "bench_harness")]
fn pendulum_ball_joint_settles() {
    let scenario = PendulumBallJointSettlesScenario::new();
    let cfg = BenchRunConfig {
        fixed_dt: 1.0 / 240.0,
        duration: 12.0,
        ..Default::default()
    };
    let result = run_scenario(&scenario, cfg);
    write_exports(&result, "pendulum_ball_joint_settles");

    let last = result.samples.last().unwrap();

    eprintln!(
        "pendulum_ball_joint: final speed={:.4}, x={:.4}, y={:.4}",
        last.linear_speed, last.x, last.y,
    );

    assert!(
        last.linear_speed < 0.05,
        "pendulum should have settled: speed={:.4}",
        last.linear_speed,
    );

    // Weight should be nearly directly below the anchor (x=0, z=0).
    // The scenario tracks x; z isn't captured by BenchSample, but the
    // pendulum starts with x-only velocity so z stays near zero.
    // Tolerance is 0.05 m — a freely-swinging pendulum with only linear
    // damping takes longer to center than a contact-damped body.
    assert!(
        last.x.abs() < 0.05,
        "pendulum should be directly below anchor: x={:.4}",
        last.x,
    );

    // Y should be near anchor_y - pendulum_length = 3.0 - 1.0 = 2.0
    let expected_y = 3.0 - scenario.pendulum_length;
    assert!(
        (last.y - expected_y).abs() < 0.05,
        "pendulum y should be near {expected_y:.2}: y={:.4}",
        last.y,
    );
}
