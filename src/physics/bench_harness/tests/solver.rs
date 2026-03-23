use std::fs;

use nalgebra::{Point3, Vector3};

use super::super::framework::{run_scenario, BenchRunConfig};
use super::super::geometry::FlatQuadGeometry;
use super::super::scenarios::*;
use super::assertions::*;
use super::write_exports;
use crate::debug::DebugLines;
use crate::physics::world::PhysicsConfig;
use crate::physics::{ColliderDesc, PhysicsWorld, RigidBodyDesc};

// ═══════════════════════════════════════════════════════════════════════════
// Solver stability validation tests
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn sliding_sphere_decelerates_without_angular_spikes() {
    let scenario = SlidingSphereScenario::new();
    let cfg = BenchRunConfig {
        duration: 6.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "sliding_sphere");

    assert!(!run.samples.is_empty());

    // Sphere should decelerate and nearly stop
    let final_speed = run.samples.last().unwrap().linear_speed;
    assert!(
        final_speed < 0.5,
        "sliding sphere should have mostly stopped: final_speed={final_speed:.4}"
    );

    // Check for angular speed spikes. During smooth friction deceleration,
    // angular speed should remain modest (rolling friction, not torque spikes).
    let max_angular = run
        .samples
        .iter()
        .map(|s| s.angular_speed)
        .fold(0.0f32, f32::max);
    eprintln!("sliding_sphere max_angular_speed={max_angular:.6}");
    assert!(
        max_angular < 30.0,
        "sliding sphere angular speed spike: max_angular={max_angular:.4}"
    );

    // Angular speed should settle in the tail window
    let tail_start = (cfg.duration - 2.0).max(0.0);
    let tail_max_angular = run
        .samples
        .iter()
        .filter(|s| s.sim_time >= tail_start)
        .map(|s| s.angular_speed)
        .fold(0.0f32, f32::max);
    assert!(
        tail_max_angular < 1.0,
        "sliding sphere tail angular jitter: tail_max_angular={tail_max_angular:.4}"
    );
}

#[test]
fn low_friction_ramp_sphere_slides_down() {
    let scenario = LowFrictionRampScenario::new();
    let cfg = BenchRunConfig {
        duration: 4.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "low_friction_ramp");

    assert!(!run.samples.is_empty());

    // Sphere should be moving faster at the end than at the start.
    let early_window = 0.5;
    let early_max_speed = run
        .samples
        .iter()
        .filter(|s| s.sim_time <= early_window)
        .map(|s| s.linear_speed)
        .fold(0.0f32, f32::max);

    let tail_start = cfg.duration - 1.0;
    let tail_min_speed = run
        .samples
        .iter()
        .filter(|s| s.sim_time >= tail_start)
        .map(|s| s.linear_speed)
        .fold(f32::MAX, f32::min);

    eprintln!(
        "low_friction_ramp early_max_speed={early_max_speed:.4} tail_min_speed={tail_min_speed:.4}"
    );
    assert!(
        tail_min_speed > early_max_speed + 1.0,
        "sphere should accelerate on low-friction ramp: \
         early_max={early_max_speed:.4}, tail_min={tail_min_speed:.4}"
    );

    assert_above_floor(&run, -1.0);
}

// ── Box-on-plank stacking jitter ────────────────────────────────────

/// Heavy box placed in one corner of a plank on flat terrain.
///
/// The asymmetric load creates a torque that the solver must counterbalance
/// through the plank's four ground contacts. With the current solver this
/// causes the plank to rock — angular velocity never fully settles.
#[test]
fn box_on_plank_no_rotational_jitter() {
    let scenario = BoxOnPlankScenario::new();
    let cfg = BenchRunConfig {
        duration: 6.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "box_on_plank");

    assert!(!run.samples.is_empty());

    // Plank should settle onto the ground.
    let expected_y = 0.5;
    assert_final_y_near(&run, expected_y, 0.05);

    // After 3 seconds of settling, both linear and angular velocity should
    // be negligible. The angular threshold is the key one — rotational
    // jitter from asymmetric loading is the failure mode.
    assert_settled(&run, 3.0, 0.005, 0.005);
}

// ── Sphere pushing a box across flat ground ────────────────────────

#[test]
fn sphere_pushing_box_no_jitter() {
    let geometry = FlatQuadGeometry::new(50.0);
    let box_half = Vector3::new(0.5, 0.5, 0.5);
    let sphere_radius = 0.5;

    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    config.deterministic_contact_ordering = true;
    let mut world = PhysicsWorld::new(config);
    let mut debug_lines = DebugLines::default();

    // Single box just ahead of the sphere.
    let box_handle = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(
        1.5,
        box_half.y + 0.01,
        0.0,
    )));
    let _ = world.attach_collider(
        box_handle,
        ColliderDesc::box_shape(box_half)
            .density(500.0)
            .restitution(0.0)
            .friction(0.6),
    );

    // Player sphere — velocity-driven, matching in-game config.
    let sphere_handle = world.create_body(
        RigidBodyDesc::dynamic()
            .position(Point3::new(0.5 - 0.01, sphere_radius + 0.01, 0.0))
            .angular_damping(1.0),
    );
    let _ = world.attach_collider(
        sphere_handle,
        ColliderDesc::sphere(sphere_radius)
            .density(30.0)
            .restitution(0.0)
            .friction(0.3),
    );

    let fixed_dt = 1.0 / 240.0;
    let frame_dt = 1.0f32 / 60.0;
    let substeps = (frame_dt / fixed_dt).round() as usize;
    let duration = 6.0f32;
    let push_speed_x = 3.9f32;
    let push_speed_z = 0.1f32;
    let push_speed = (push_speed_x * push_speed_x + push_speed_z * push_speed_z).sqrt();
    let num_frames = (duration / frame_dt).round() as usize;

    let mut csv = String::from("sim_time,sphere_x,sphere_vx,box_x,box_vx\n");
    let mut sim_time = 0.0f32;
    let mut sphere_vx_samples: Vec<f32> = Vec::new();
    let mut gap_samples: Vec<f32> = Vec::new();

    for _ in 0..num_frames {
        // Drive sphere velocity per frame (matches game loop).
        let sb = world.body(sphere_handle).unwrap();
        let vel = *sb.linear_velocity();
        world.set_body_velocity_drive(
            sphere_handle,
            Vector3::new(push_speed_x, vel.y, push_speed_z),
            Vector3::zeros(),
            500.0,
            500.0,
        );

        // Narrowphase once per frame, then multiple substeps (matches game).
        world.update_contacts(fixed_dt, &geometry, &[], &mut debug_lines);
        debug_lines.clear();

        for _ in 0..substeps {
            world.substep(fixed_dt, &geometry, &[]);
            sim_time += fixed_dt;

            let sb = world.body(sphere_handle).unwrap();
            let bb = world.body(box_handle).unwrap();
            let svx = sb.linear_velocity().x;
            let gap = bb.position().x - sb.position().x;
            sphere_vx_samples.push(svx);
            gap_samples.push(gap);

            csv.push_str(&format!(
                "{:.6},{:.6},{:.6},{:.6},{:.6}\n",
                sim_time,
                sb.position().x,
                svx,
                bb.position().x,
                bb.linear_velocity().x,
            ));
        }
    }

    // Export CSV.
    let dir = "target/physics_bench";
    fs::create_dir_all(dir).expect("create target/physics_bench");
    fs::write(format!("{dir}/sphere_pushing_box.csv"), &csv).expect("write bench csv");

    // Jitter check: when the sphere pushes the box, both should move
    // forward together with the gap staying roughly constant.
    let contact_dist = sphere_radius + box_half.x;
    let contact_threshold = contact_dist + 0.05;

    // Group samples by frame and compute per-frame average sphere vx.
    let mut frame_avg_vx: Vec<f32> = Vec::new();
    let mut frame_in_contact: Vec<bool> = Vec::new();
    for chunk in sphere_vx_samples
        .chunks(substeps)
        .zip(gap_samples.chunks(substeps))
    {
        let (vx_chunk, gap_chunk) = chunk;
        let avg_vx: f32 = vx_chunk.iter().sum::<f32>() / vx_chunk.len() as f32;
        let min_gap = gap_chunk.iter().fold(f32::MAX, |a, &b| a.min(b));
        frame_avg_vx.push(avg_vx);
        frame_in_contact.push(min_gap > 0.0 && min_gap < contact_threshold);
    }

    // Skip warmup: first 60 frames (1 second).
    let warmup_frames = 60;
    let contact_frames: Vec<f32> = frame_avg_vx
        .iter()
        .zip(frame_in_contact.iter())
        .skip(warmup_frames)
        .filter(|(_, &in_contact)| in_contact)
        .map(|(&vx, _)| vx)
        .collect();

    let overall_avg = if contact_frames.is_empty() {
        0.0
    } else {
        contact_frames.iter().sum::<f32>() / contact_frames.len() as f32
    };
    let efficiency = overall_avg / push_speed;

    eprintln!(
        "sphere_pushing_box: contact_frames={}, overall_avg_vx={overall_avg:.3}, \
         push_speed={push_speed}, efficiency={efficiency:.3}",
        contact_frames.len(),
    );

    // The sphere should transfer a meaningful fraction of its
    // commanded velocity to forward motion.
    assert!(
        efficiency > 0.30,
        "sphere push efficiency too low: {efficiency:.3} (avg_vx={overall_avg:.3}, \
         push_speed={push_speed}). The solver is absorbing the velocity override, \
         causing back-and-forth jitter at the contact boundary.",
    );
}

/// A velocity-driven sphere pushed against a static wall should reach a steady
/// resting contact — not oscillate due to restitution firing on persisted contacts.
///
/// Regression test: without persisted-contact restitution suppression, the solver
/// treats every frame as a new high-speed impact (because the drive resets approach
/// velocity above the threshold), causing repeated bouncing.
#[test]
fn velocity_driven_sphere_against_wall_no_bounce() {
    use super::super::geometry::WallAndFloorGeometry;

    // Wall at x=0 facing +X, sphere starts at x=2 driven toward -X.
    let geometry = WallAndFloorGeometry::new(50.0, 5.0);

    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    config.deterministic_contact_ordering = true;
    let mut world = PhysicsWorld::new(config);
    let mut debug_lines = DebugLines::default();

    let radius = 0.5;
    let sphere = world.create_body(
        RigidBodyDesc::dynamic()
            .position(Point3::new(2.0, radius + 0.01, 0.0))
            .angular_damping(1.0),
    );
    let _ = world.attach_collider(
        sphere,
        ColliderDesc::sphere(radius)
            .density(50.0)
            .restitution(0.5)
            .friction(0.3),
    );

    let fixed_dt: f32 = 1.0 / 240.0;
    let frame_dt: f32 = 1.0 / 60.0;
    let substeps = (frame_dt / fixed_dt).round() as usize;
    let drive_speed = 5.0;

    // Run for 3 seconds
    let num_frames = (3.0f32 / frame_dt).round() as usize;
    let mut x_samples: Vec<f32> = Vec::new();

    for _ in 0..num_frames {
        let sb = world.body(sphere).unwrap();
        let vel_y = sb.linear_velocity().y;
        // Drive toward the wall (negative X)
        world.set_body_velocity_drive(
            sphere,
            Vector3::new(-drive_speed, vel_y, 0.0),
            Vector3::zeros(),
            500.0,
            500.0,
        );

        world.update_contacts(fixed_dt, &geometry, &[], &mut debug_lines);
        debug_lines.clear();
        for _ in 0..substeps {
            world.substep(fixed_dt, &geometry, &[]);
        }

        x_samples.push(world.body(sphere).unwrap().position().x);
    }

    // After reaching the wall, X position should be stable (not oscillating).
    // Skip first second for approach/settling.
    let tail_start = 60;
    let tail = &x_samples[tail_start..];

    let max_x = tail.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));
    let min_x = tail.iter().fold(f32::INFINITY, |a, &b| a.min(b));
    let x_range = max_x - min_x;

    eprintln!(
        "wall_bounce: tail x_range={x_range:.4}, min={min_x:.4}, max={max_x:.4}"
    );

    // Position should be within a small band — no large oscillations
    assert!(
        x_range < 0.1,
        "velocity-driven sphere oscillates against wall: x_range={x_range:.4}. \
         Restitution may be firing on persisted contacts.",
    );
}
