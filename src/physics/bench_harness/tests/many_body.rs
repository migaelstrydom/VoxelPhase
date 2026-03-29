use nalgebra::{Point3, Vector3};

use super::super::framework::{run_scenario, BenchRunConfig};
use super::super::geometry::FlatQuadGeometry;
use super::super::scenarios::BoxGridScenario;
use super::write_exports;
use super::CubeShellGeometry;
use crate::debug::DebugLines;
use crate::physics::world::PhysicsConfig;
use crate::physics::{ColliderDesc, PhysicsImpulse, PhysicsWorld, RigidBodyDesc};

// ── Many-body narrowphase throughput ───────────────────────────────

#[test]
fn box_grid_settles_without_explosions() {
    // 5x5 = 25 boxes. Exercises the narrowphase work buffer with many
    // dynamic-dynamic pairs. Verifies no boxes explode or fall through.
    let scenario = BoxGridScenario::new(5);
    let cfg = BenchRunConfig {
        duration: 0.5,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "box_grid_5x5");

    assert!(!run.samples.is_empty());

    // The tracked body (first box) should settle somewhere above the ground.
    let last = run.samples.last().unwrap();
    assert!(
        last.y > -0.5,
        "box should not fall through ground: y={}",
        last.y
    );

    // Tail speed should be low (settled or nearly settled).
    let (tail_linear, _) = run.tail_max_speeds(1.0);
    assert!(
        tail_linear < 1.0,
        "box grid should be settling: tail linear speed {tail_linear:.3}"
    );
}

#[test]
fn grenade_like_impulses_keep_states_finite() {
    let geometry = CubeShellGeometry::new(32.0);

    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    let mut world = PhysicsWorld::new(config);
    let mut debug_lines = DebugLines::default();

    // Dense jumble of boxes near the center.
    let box_half = Vector3::new(0.3, 0.3, 0.3);
    for x in 0..5 {
        for z in 0..5 {
            let px = -1.2 + x as f32 * 0.6;
            let pz = -1.2 + z as f32 * 0.6;
            let py = 1.0 + ((x + z) % 3) as f32 * 0.6;
            let body =
                world.create_body(RigidBodyDesc::dynamic().position(Point3::new(px, py, pz)));
            let _ = world.attach_collider(
                body,
                ColliderDesc::box_shape(box_half)
                    .density(0.5)
                    .restitution(0.2)
                    .friction(0.6),
            );
        }
    }

    // Small spheres mixed into the pile (grenade-sized).
    let sphere_radius = 0.2622022;
    for i in 0..10 {
        let t = i as f32;
        let px = (t * 0.37).sin() * 1.0;
        let pz = (t * 0.61).cos() * 1.0;
        let py = 1.3 + (i % 4) as f32 * 0.45;
        let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(px, py, pz)));
        let _ = world.attach_collider(
            body,
            ColliderDesc::sphere(sphere_radius)
                .density(1000.0)
                .restitution(0.2)
                .friction(0.5),
        );
    }

    let fixed_dt = 1.0 / 60.0;
    let num_steps = (20.0 / fixed_dt) as usize;
    let mut failure: Option<String> = None;

    for step in 0..num_steps {
        let impulses = if step > 30 && step % 15 == 0 {
            vec![
                PhysicsImpulse::radial(Point3::new(0.0, 0.8, 0.0), 10.0, 26000.0, 0.6),
                PhysicsImpulse::radial(Point3::new(0.0, 0.2, 0.0), 8.0, 22000.0, 0.4),
            ]
        } else {
            Vec::new()
        };

        world.update_contacts(fixed_dt, &geometry, &impulses, &mut debug_lines);
        debug_lines.clear();
        world.substep(fixed_dt, &geometry, &[]);

        for (idx, body) in world.bodies().iter() {
            let pos = body.position();
            let lin = body.linear_velocity();
            let ang = body.angular_velocity();
            let lin_speed = lin.magnitude();
            let ang_speed = ang.magnitude();

            let finite = pos.x.is_finite()
                && pos.y.is_finite()
                && pos.z.is_finite()
                && lin.x.is_finite()
                && lin.y.is_finite()
                && lin.z.is_finite()
                && ang.x.is_finite()
                && ang.y.is_finite()
                && ang.z.is_finite()
                && lin_speed.is_finite()
                && ang_speed.is_finite();

            if !finite || lin_speed > 1.0e8 || ang_speed > 1.0e8 {
                failure = Some(format!(
                    "step={step} body={idx:?} pos=({:.3},{:.3},{:.3}) lin=({:.3},{:.3},{:.3}) ang=({:.3},{:.3},{:.3}) lin_speed={:.3} ang_speed={:.3}",
                    pos.x, pos.y, pos.z, lin.x, lin.y, lin.z, ang.x, ang.y, ang.z, lin_speed, ang_speed
                ));
                break;
            }
        }

        if failure.is_some() {
            break;
        }
    }

    assert!(
        failure.is_none(),
        "physics state became unstable under grenade-like stress: {}",
        failure.unwrap_or_default()
    );
}

#[test]
fn box_grid_narrowphase_throughput() {
    // Throughput benchmark: 8x8 = 64 boxes producing many broadphase pairs.
    // Measures wall-clock time for 2 seconds of simulated time.
    let scenario = BoxGridScenario::new(8);
    let cfg = BenchRunConfig {
        duration: 0.5,
        ..BenchRunConfig::default()
    };

    let t0 = std::time::Instant::now();
    let run = run_scenario(&scenario, cfg);
    let elapsed = t0.elapsed();

    write_exports(&run, "box_grid_8x8_throughput");

    eprintln!(
        "Box grid 8x8 (64 bodies): {} physics steps in {:.1}ms ({:.0} steps/sec)",
        run.physics_steps,
        elapsed.as_secs_f64() * 1000.0,
        run.physics_steps as f64 / elapsed.as_secs_f64(),
    );

    assert!(!run.samples.is_empty());
    assert_eq!(run.dropped_steps, 0);
}

#[test]
fn large_sphere_sliding_into_low_box_does_not_end_intersecting() {
    let geometry = FlatQuadGeometry::new(30.0);
    let box_half_extents = Vector3::new(3.0, 0.1, 3.0);
    let sphere_radius = 0.9;

    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    config.deterministic_contact_ordering = true;
    let mut world = PhysicsWorld::new(config);
    let mut debug_lines = DebugLines::default();

    let box_handle = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(
        0.0,
        box_half_extents.y + 0.01,
        0.0,
    )));
    let _ = world.attach_collider(
        box_handle,
        ColliderDesc::box_shape(box_half_extents)
            .density(10000.0)
            .restitution(0.0)
            .friction(0.7),
    );

    let sphere_handle = world.create_body(
        RigidBodyDesc::dynamic()
            .position(Point3::new(-8.0, sphere_radius + 0.02, 0.0))
            .linear_velocity(Vector3::new(9.0, 0.0, 0.0)),
    );
    let _ = world.attach_collider(
        sphere_handle,
        ColliderDesc::sphere(sphere_radius)
            .density(1000.0)
            .restitution(0.0)
            .friction(0.2),
    );

    let fixed_dt: f32 = 1.0 / 240.0;
    let frame_dt: f32 = 1.0 / 60.0;
    let substeps_per_frame = (frame_dt / fixed_dt).round() as usize;
    let num_frames = (4.0 / frame_dt) as usize;
    for _ in 0..num_frames {
        world.update_contacts(fixed_dt, &geometry, &[], &mut debug_lines);
        debug_lines.clear();
        for _ in 0..substeps_per_frame {
            world.substep(fixed_dt, &geometry, &[]);
        }
    }

    let sphere = world.body(sphere_handle).expect("sphere body should exist");
    let obstacle = world.body(box_handle).expect("box body should exist");

    let sphere_center_world = sphere.position();
    let box_center_world = obstacle.position();
    let sphere_center_in_box = obstacle
        .rotation()
        .inverse_transform_vector(&(sphere_center_world - box_center_world));
    let closest_on_box_local = Vector3::new(
        sphere_center_in_box
            .x
            .clamp(-box_half_extents.x, box_half_extents.x),
        sphere_center_in_box
            .y
            .clamp(-box_half_extents.y, box_half_extents.y),
        sphere_center_in_box
            .z
            .clamp(-box_half_extents.z, box_half_extents.z),
    );
    let separation_vec = sphere_center_in_box - closest_on_box_local;
    let separation_sq = separation_vec.magnitude_squared();
    let penetration = sphere_radius - separation_sq.sqrt();

    assert!(
        penetration <= 1.0e-3,
        "sphere should not end intersecting low box: penetration={penetration:.6}"
    );
}

// ── Box stacking: dynamic-dynamic stability ─────────────────────────

#[test]
fn box_stack_settles_without_overlap() {
    let box_half_extents = Vector3::new(2.5, 0.5, 2.5);
    let geometry = FlatQuadGeometry::new(20.0);

    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    config.deterministic_contact_ordering = true;
    let mut world = PhysicsWorld::new(config);
    let mut debug_lines = DebugLines::default();

    let mut box_handles = Vec::new();
    for i in 0..4 {
        let y = 5.0 + (i as f32) * 5.0;
        let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, y, 0.0)));
        let _ = world.attach_collider(
            body,
            ColliderDesc::box_shape(box_half_extents)
                .density(1000.0)
                .restitution(0.0)
                .friction(0.6),
        );
        box_handles.push(body);
    }

    let fixed_dt: f32 = 1.0 / 240.0;
    let frame_dt: f32 = 1.0 / 60.0;
    let substeps_per_frame = (frame_dt / fixed_dt).round() as usize;
    let num_frames = (10.0 / frame_dt) as usize;
    let tail_frames = (2.0 / frame_dt) as usize;
    let tail_start_frame = num_frames.saturating_sub(tail_frames);
    let mut tail_min_y = vec![f32::INFINITY; box_handles.len()];
    let mut tail_max_y = vec![f32::NEG_INFINITY; box_handles.len()];
    let mut tail_max_speed = 0.0f32;
    let mut tail_max_manifold_churn = 0usize;
    let mut tail_max_contact_depth = 0.0f32;
    let mut tail_points_sum = 0usize;
    let mut tail_samples = 0usize;

    for step_idx in 0..num_frames {
        world.update_contacts(fixed_dt, &geometry, &[], &mut debug_lines);
        debug_lines.clear();
        for _ in 0..substeps_per_frame {
            world.substep(fixed_dt, &geometry, &[]);
        }

        if step_idx >= tail_start_frame {
            let manifold = world.manifold_frame_stats();
            let manifold_churn =
                manifold.point_adds + manifold.point_replacements + manifold.point_pruned;
            tail_max_manifold_churn = tail_max_manifold_churn.max(manifold_churn);
            tail_max_contact_depth = tail_max_contact_depth.max(
                world
                    .contact_events()
                    .iter()
                    .map(|c| c.depth)
                    .fold(0.0f32, f32::max),
            );
            tail_points_sum += manifold.points;
            tail_samples += 1;
            for (i, &handle) in box_handles.iter().enumerate() {
                let body = world.body(handle).expect("box body should exist");
                let y = body.position().y;
                tail_min_y[i] = tail_min_y[i].min(y);
                tail_max_y[i] = tail_max_y[i].max(y);
                let spd = body.linear_velocity().magnitude();
                if spd > tail_max_speed {
                    tail_max_speed = spd;
                }
            }
        }

        // Diagnostic: print every 60 frames (1s intervals) and last 5 frames
        if step_idx % 60 == 0 || step_idx >= num_frames - 5 {
            let manifold = world.manifold_frame_stats();
            let manifold_churn =
                manifold.point_adds + manifold.point_replacements + manifold.point_pruned;
            let mut line = format!(
                "step {step_idx:4} m_points={} m_churn={} :",
                manifold.points, manifold_churn,
            );
            for (i, &handle) in box_handles.iter().enumerate() {
                let body = world.body(handle).expect("box body should exist");
                let p = body.position();
                let v = body.linear_velocity();
                let av = body.angular_velocity();
                line.push_str(&format!(
                    " B{i}(y={:.3} vy={:.3} spd={:.3} aspd={:.3})",
                    p.y,
                    v.y,
                    v.magnitude(),
                    av.magnitude()
                ));
            }
            eprintln!("{line}");
        }
    }

    let mut y_positions = Vec::new();
    let mut max_speed = 0.0f32;
    for &handle in &box_handles {
        let body = world.body(handle).expect("box body should exist");
        y_positions.push(body.position().y);
        max_speed = max_speed.max(body.linear_velocity().magnitude());
    }

    y_positions.sort_by(|a, b| a.total_cmp(b));

    eprintln!("tail_max_speed={tail_max_speed:.6} final_max_speed={max_speed:.6}");
    if tail_samples > 0 {
        let tail_avg_manifold_points = tail_points_sum as f32 / tail_samples as f32;
        eprintln!(
            "tail_max_manifold_churn={} tail_avg_manifold_points={:.2} tail_max_contact_depth={:.4}",
            tail_max_manifold_churn,
            tail_avg_manifold_points,
            tail_max_contact_depth
        );
    }
    assert!(
        max_speed < 0.2,
        "boxes should have mostly settled: max speed {max_speed:.6}"
    );
    assert!(
        tail_max_speed < 0.08,
        "boxes should be near-rest in tail window: tail max speed {tail_max_speed:.6}"
    );

    for i in 0..tail_min_y.len() {
        assert!(
            tail_min_y[i].is_finite() && tail_max_y[i].is_finite(),
            "tail window sampling failed for box {i}"
        );
        let tail_peak_to_peak = tail_max_y[i] - tail_min_y[i];
        assert!(
            tail_peak_to_peak < 0.03,
            "box {i} jitters in tail window: peak-to-peak y={tail_peak_to_peak:.6}"
        );
    }

    for i in 0..y_positions.len() {
        assert!(
            y_positions[i] > 0.0,
            "box {i} fell through ground: y={}",
            y_positions[i]
        );
    }

    for i in 1..y_positions.len() {
        let separation = y_positions[i] - y_positions[i - 1];
        let min_separation = 2.0 * box_half_extents.y;
        assert!(
            separation >= min_separation * 0.95,
            "boxes {} and {} overlap: separation={:.3}, min_separation={:.3}",
            i - 1,
            i,
            separation,
            min_separation
        );
    }

    let bottom_box_y = y_positions[0];
    assert!(
        bottom_box_y > 0.4 && bottom_box_y < 0.6,
        "bottom box should rest on ground at y ≈ 0.5, got y={}",
        bottom_box_y
    );
}
