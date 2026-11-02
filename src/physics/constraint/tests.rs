//! The joints' position correction, through the whole step: a body knocked
//! off its joint is carried back toward it, never away. Each joint row's bias
//! must drive its error toward zero under the solver's convention (a row
//! converges where `J·v + bias` is zero), or its velocity pushes the body
//! further off while the position pass drags it back.

use nalgebra::{Point3, UnitQuaternion, UnitVector3, Vector3};

use crate::debug::DebugLines;
use crate::physics::bench_harness::geometry::EmptyGeometry;
use crate::physics::{
    ColliderDesc, ConstraintKind, PhysicsConfig, PhysicsWorld, RigidBodyDesc, RigidBodyHandle,
    SequentialStepper, Stepper,
};

/// A world without gravity, so only the joint moves the body.
fn still_world() -> PhysicsWorld {
    PhysicsWorld::new(PhysicsConfig {
        gravity: Vector3::zeros(),
        ..PhysicsConfig::default()
    })
}

/// A unit box at `at`, awake.
fn block(world: &mut PhysicsWorld, at: Point3<f32>) -> RigidBodyHandle {
    let body = world.create_body(RigidBodyDesc::dynamic().position(at));
    world.attach_collider(body, ColliderDesc::box_shape(Vector3::new(0.5, 0.5, 0.5)));
    world.wake_body(body);
    body
}

/// One 60 Hz frame of the game's stepping.
fn step(world: &mut PhysicsWorld) {
    let mut stepper = SequentialStepper::new(1.0 / 240.0, 12);
    stepper.step(
        world,
        1.0 / 60.0,
        &EmptyGeometry,
        &[],
        &[],
        &mut DebugLines::default(),
    );
}

/// The body's velocity after a frame, its joint's anchor 5 cm above where
/// its own anchor sits.
fn pulled_up(kind: impl FnOnce(RigidBodyHandle) -> ConstraintKind) -> Vector3<f32> {
    let mut world = still_world();
    let body = block(&mut world, Point3::new(0.0, 0.95, 0.0));
    world.create_constraint(kind(body));
    step(&mut world);
    world.body(body).unwrap().linear_velocity()
}

#[test]
fn a_ball_joint_carries_its_body_back_up_to_its_anchor() {
    let velocity = pulled_up(|body| ConstraintKind::BallJoint {
        body_a: None,
        body_b: body,
        local_anchor_a: Vector3::new(0.0, 1.5, 0.0),
        anchor_velocity: Vector3::zeros(),
        local_anchor_b: Vector3::new(0.0, 0.5, 0.0),
        compliance: 0.0,
        max_impulse: f32::INFINITY,
    });
    assert!(velocity.y >= 0.0, "moving off the anchor at {velocity:?}");
}

#[test]
fn a_fixed_joint_carries_its_body_back_up_to_its_anchor() {
    let velocity = pulled_up(|body| {
        ConstraintKind::world_fixed(
            body,
            Point3::new(0.0, 1.5, 0.0),
            Vector3::new(0.0, 0.5, 0.0),
            &UnitQuaternion::identity(),
            0.0,
            f32::INFINITY,
        )
    });
    assert!(velocity.y >= 0.0, "moving off the anchor at {velocity:?}");
}

#[test]
fn a_hinge_carries_its_body_back_up_to_its_anchor() {
    let velocity = pulled_up(|body| {
        ConstraintKind::world_hinge(
            body,
            Point3::new(0.0, 1.5, 0.0),
            Vector3::new(0.0, 0.5, 0.0),
            UnitVector3::new_normalize(Vector3::x()),
            &UnitQuaternion::identity(),
            0.0,
            f32::INFINITY,
        )
    });
    assert!(velocity.y >= 0.0, "moving off the anchor at {velocity:?}");
}

#[test]
fn a_fixed_joint_turns_its_body_back_to_its_reference() {
    let mut world = still_world();
    let body = block(&mut world, Point3::origin());
    let turned = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.05);
    world.body_mut(body).unwrap().set_rotation(turned);
    world.create_constraint(ConstraintKind::world_fixed(
        body,
        Point3::origin(),
        Vector3::zeros(),
        &UnitQuaternion::identity(),
        0.0,
        f32::INFINITY,
    ));
    step(&mut world);
    let spin = world.body(body).unwrap().angular_velocity();
    assert!(spin.z <= 0.0, "turning further off at {spin:?}");
}

/// A ball joint whose anchor its owner moves carries the body at the
/// anchor's stated speed.
#[test]
fn a_moving_ball_joint_anchor_carries_its_body_at_its_speed() {
    let mut world = still_world();
    let body = block(&mut world, Point3::new(0.0, 1.0, 0.0));
    let joint = world.create_constraint(ConstraintKind::BallJoint {
        body_a: None,
        body_b: body,
        local_anchor_a: Vector3::new(0.0, 1.5, 0.0),
        anchor_velocity: Vector3::zeros(),
        local_anchor_b: Vector3::new(0.0, 0.5, 0.0),
        compliance: 0.0,
        max_impulse: f32::INFINITY,
    });
    let dt = 1.0 / 60.0;
    for frame in 0..30 {
        let anchor = Vector3::new(0.0, 1.5 + frame as f32 * dt, 0.0);
        if let ConstraintKind::BallJoint {
            local_anchor_a,
            anchor_velocity,
            ..
        } = &mut world.constraint_mut(joint).unwrap().kind
        {
            *local_anchor_a = anchor;
            *anchor_velocity = Vector3::y();
        }
        world.wake_body(body);
        step(&mut world);
    }
    let body = world.body(body).unwrap();
    assert!(
        (body.linear_velocity().y - 1.0).abs() < 0.05,
        "carried at {:?}, at {:?}",
        body.linear_velocity(),
        body.position()
    );
    assert!(
        (body.position().y - (1.0 + 30.0 * dt)).abs() < 0.02,
        "at {:?}",
        body.position()
    );
}

/// A compliant joint keeps a velocity bias, its spring, and it pulls the body
/// toward the anchor.
#[test]
fn a_compliant_ball_joint_springs_its_body_toward_its_anchor() {
    let velocity = pulled_up(|body| ConstraintKind::BallJoint {
        body_a: None,
        body_b: body,
        local_anchor_a: Vector3::new(0.0, 1.5, 0.0),
        anchor_velocity: Vector3::zeros(),
        local_anchor_b: Vector3::new(0.0, 0.5, 0.0),
        compliance: 1.0e-4,
        max_impulse: f32::INFINITY,
    });
    assert!(velocity.y > 0.0, "sprung at {velocity:?}");
}
