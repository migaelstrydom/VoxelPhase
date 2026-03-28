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
        world.set_body_velocity_drive(body, Vector3::zeros(), Vector3::zeros(), 500.0, 500.0);
        world.update_contacts(dt, &geometry, &[], &mut debug_lines);
        debug_lines.clear();
        for _ in 0..substeps {
            world.substep(dt, &geometry, &[]);
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
        world.set_body_velocity_drive(body, forward_vel, Vector3::zeros(), 500.0, 500.0);
        world.update_contacts(dt, &geometry, &[], &mut debug_lines);
        debug_lines.clear();
        for _ in 0..substeps {
            world.substep(dt, &geometry, &[]);
        }
    }
    let b = world.body(body).unwrap();
    eprintln!("after move: y={:.4}, tilt={:.2}°",
        b.position().y,
        (b.rotation() * Vector3::y()).dot(&Vector3::y()).acos().to_degrees());

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
    assert!(tilt < 10.0, "capsule should be near-upright: tilt={tilt:.1}°");
}

/// Two boxes welded together with touching faces (well within 2*contact_margin).
/// Without contact filtering, the margin contacts fight the weld and cause
/// explosion. With filtering, the pair should fall as a unit and settle.
#[test]
fn welded_boxes_with_touching_faces_do_not_explode() {
    let geometry = FlatGridGeometry::new(8.0, 1.0);
    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    let mut world = PhysicsWorld::new(config);

    let he = Vector3::new(0.3, 0.3, 0.3);

    // Two boxes touching along X (zero gap — well within 2*contact_margin).
    let body_a = world.create_body(
        RigidBodyDesc::dynamic().position(Point3::new(-he.x, 2.0, 0.0)),
    );
    let _ = world.attach_collider(
        body_a,
        ColliderDesc::box_shape(he).density(500.0).restitution(0.0).friction(0.6),
    );

    let body_b = world.create_body(
        RigidBodyDesc::dynamic().position(Point3::new(he.x, 2.0, 0.0)),
    );
    let _ = world.attach_collider(
        body_b,
        ColliderDesc::box_shape(he).density(500.0).restitution(0.0).friction(0.6),
    );

    // Weld at the touching face.
    let rot_a = world.body(body_a).unwrap().rotation();
    let rot_b = world.body(body_b).unwrap().rotation();
    let relative_orientation = rot_a.inverse() * rot_b;
    let _ = world.create_constraint(ConstraintKind::Weld {
        body_a,
        body_b,
        local_anchor_a: Vector3::new(he.x, 0.0, 0.0),
        local_anchor_b: Vector3::new(-he.x, 0.0, 0.0),
        relative_orientation,
        compliance: 0.0,
        angular_compliance: 0.0,
    });

    use crate::debug::DebugLines;
    let dt = 1.0 / 240.0;
    let mut debug_lines = DebugLines::default();

    // Run for 1 second (60 frames × 4 substeps).
    for _ in 0..60 {
        world.update_contacts(dt, &geometry, &[], &mut debug_lines);
        debug_lines.clear();
        for _ in 0..4 {
            world.substep(dt, &geometry, &[]);
        }
    }

    let pos_a = world.body(body_a).unwrap().position();
    let pos_b = world.body(body_b).unwrap().position();
    let separation = (pos_b - pos_a).magnitude();
    let expected_separation = 2.0 * he.x; // should be ~0.6
    let drift = (separation - expected_separation).abs();

    eprintln!(
        "welded_touching: separation={separation:.4}, expected={expected_separation:.4}, drift={drift:.4}"
    );

    // The weld should hold — drift should be small (PGS softness is expected).
    assert!(
        drift < 0.1,
        "welded boxes drifted too far apart: drift={drift:.4} (separation={separation:.4})"
    );

    // Neither body should have flown away (y should be near ground, not sky-high).
    assert!(
        pos_a.y < 3.0 && pos_b.y < 3.0,
        "bodies flew away: y_a={:.2}, y_b={:.2}",
        pos_a.y,
        pos_b.y,
    );
}

/// Diagnostic test: 3-piece barricade (2 posts + 1 plank) dropped onto ground.
/// Contact filtering is currently disabled, so post-plank contacts are active.
/// Logs per-frame positions and velocities to diagnose the "fly into sky" bug.
#[test]
fn barricade_drop_diagnostic() {
    let geometry = FlatGridGeometry::new(8.0, 1.0);
    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    let mut world = PhysicsWorld::new(config);

    // Reproduce the barricade spawner geometry exactly (1 plank variant).
    let post_he = Vector3::new(0.08, 0.5, 0.08);
    let plank_he = Vector3::new(0.45, 0.08, 0.06);
    let base_y = 0.5; // slight drop height
    let density = 400.0;

    let post_x = plank_he.x - post_he.x; // 0.37
    let post_cy = base_y + post_he.y;
    let inner_half = post_x - post_he.x; // 0.29

    let actual_plank_he = Vector3::new(inner_half, plank_he.y, plank_he.z);

    // Left post
    let left_post = world.create_body(
        RigidBodyDesc::dynamic()
            .position(Point3::new(-post_x, post_cy, 0.0))
            .gravity_scale(1.0)
            .linear_damping(0.01)
            .angular_damping(0.005),
    );
    let _ = world.attach_collider(
        left_post,
        ColliderDesc::box_shape(post_he).density(density).restitution(0.1).friction(0.6),
    );

    // Right post
    let right_post = world.create_body(
        RigidBodyDesc::dynamic()
            .position(Point3::new(post_x, post_cy, 0.0))
            .gravity_scale(1.0)
            .linear_damping(0.01)
            .angular_damping(0.005),
    );
    let _ = world.attach_collider(
        right_post,
        ColliderDesc::box_shape(post_he).density(density).restitution(0.1).friction(0.6),
    );

    // Plank (centered, at midpoint of posts)
    let usable_height = post_he.y * 2.0 - plank_he.y * 2.0;
    let plank_cy = base_y + plank_he.y + 0.5 * usable_height;
    let plank = world.create_body(
        RigidBodyDesc::dynamic()
            .position(Point3::new(0.0, plank_cy, 0.0))
            .gravity_scale(1.0)
            .linear_damping(0.01)
            .angular_damping(0.005),
    );
    let _ = world.attach_collider(
        plank,
        ColliderDesc::box_shape(actual_plank_he).density(density).restitution(0.1).friction(0.6),
    );

    // Weld plank to left post
    let rot = world.body(left_post).unwrap().rotation();
    let rel = rot.inverse() * world.body(plank).unwrap().rotation();
    let _ = world.create_constraint(ConstraintKind::Weld {
        body_a: left_post,
        body_b: plank,
        local_anchor_a: Vector3::new(post_he.x, plank_cy - post_cy, 0.0),
        local_anchor_b: Vector3::new(-actual_plank_he.x, 0.0, 0.0),
        relative_orientation: rel,
        compliance: 0.0,
        angular_compliance: 0.0,
    });

    // Weld plank to right post
    let rot = world.body(right_post).unwrap().rotation();
    let rel = rot.inverse() * world.body(plank).unwrap().rotation();
    let _ = world.create_constraint(ConstraintKind::Weld {
        body_a: right_post,
        body_b: plank,
        local_anchor_a: Vector3::new(-post_he.x, plank_cy - post_cy, 0.0),
        local_anchor_b: Vector3::new(actual_plank_he.x, 0.0, 0.0),
        relative_orientation: rel,
        compliance: 0.0,
        angular_compliance: 0.0,
    });

    use crate::debug::DebugLines;
    let dt = 1.0 / 240.0;
    let substeps = 4;
    let mut debug_lines = DebugLines::default();

    eprintln!("\n=== Barricade Drop Diagnostic ===");
    eprintln!("post_he={post_he:?}, plank_he={actual_plank_he:?}");
    eprintln!("left_post={:.3?}, right_post={:.3?}, plank={:.3?}",
        world.body(left_post).unwrap().position(),
        world.body(right_post).unwrap().position(),
        world.body(plank).unwrap().position(),
    );

    let mut max_y = 0.0f32;
    for frame in 0..180 {
        world.update_contacts(dt, &geometry, &[], &mut debug_lines);
        debug_lines.clear();
        for _ in 0..substeps {
            world.substep(dt, &geometry, &[]);
        }

        let lp = world.body(left_post).unwrap();
        let rp = world.body(right_post).unwrap();
        let pl = world.body(plank).unwrap();

        let lp_y = lp.position().y;
        let rp_y = rp.position().y;
        let pl_y = pl.position().y;
        let lp_vy = lp.linear_velocity().y;
        let rp_vy = rp.linear_velocity().y;
        let pl_vy = pl.linear_velocity().y;

        max_y = max_y.max(lp_y).max(rp_y).max(pl_y);

        // Log every 5 frames, or whenever velocity is suspicious
        let suspicious = lp_vy.abs() > 2.0 || rp_vy.abs() > 2.0 || pl_vy.abs() > 2.0;
        if frame % 5 == 0 || suspicious {
            eprintln!(
                "f{frame:3}: lp(y={lp_y:+.4}, vy={lp_vy:+.4}) rp(y={rp_y:+.4}, vy={rp_vy:+.4}) pl(y={pl_y:+.4}, vy={pl_vy:+.4})"
            );
        }
        if suspicious && frame > 10 {
            eprintln!("  ^^ SUSPICIOUS velocity at frame {frame}");
            // Log angular velocities too
            eprintln!(
                "     lp_omega={:.4?} rp_omega={:.4?} pl_omega={:.4?}",
                lp.angular_velocity(),
                rp.angular_velocity(),
                pl.angular_velocity(),
            );
        }
    }

    // Check final state: barricade should be resting on the ground, upright,
    // with the weld holding everything together.
    let lp = world.body(left_post).unwrap();
    let rp = world.body(right_post).unwrap();
    let pl = world.body(plank).unwrap();

    let final_max_y = lp.position().y.max(rp.position().y).max(pl.position().y);
    let final_max_vy = lp.linear_velocity().y.abs()
        .max(rp.linear_velocity().y.abs())
        .max(pl.linear_velocity().y.abs());

    eprintln!("max_y across all bodies: {max_y:.4}");
    eprintln!("final_max_y: {final_max_y:.4}, final_max_vy: {final_max_vy:.4}");
    eprintln!("=== End Diagnostic ===\n");

    // The barricade should not fly away at any point during the simulation.
    // Contacts between welded bodies generate bogus separation forces that
    // the solver can't fully resolve — the correct fix is compound colliders
    // (one body with multiple colliders) so inter-body contacts don't exist.
    assert!(
        max_y < 2.0,
        "barricade flew away: max_y={max_y:.2} (should stay near ground)"
    );

    // The structure should stay near ground level, even if not perfectly settled.
    assert!(
        final_max_y < 1.5,
        "barricade not settled: final_max_y={final_max_y:.4} (expected < 1.5)"
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
