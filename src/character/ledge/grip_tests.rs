//! The grip carrying a body: a swimmer caught lying along its stroke stands
//! up about its own centre into the hang, with no fight between the joint and
//! the attitude that holds it.

use std::f32::consts::FRAC_PI_2;

use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::character::locomotion::{Ledge, LedgePhase, LocomotionState};
use crate::character::LocomotionConfig;
use crate::debug::DebugLines;
use crate::physics::bench_harness::geometry::EmptyGeometry;
use crate::physics::{
    ColliderDesc, CollisionFilter, ConstraintHandle, ConstraintKind, PhysicsConfig, PhysicsWorld,
    RigidBodyDesc, RigidBodyHandle,
};
use crate::traversal::WorldCaster;

use super::{edge_at_hands, Grip, GripFrame, Reach, SensedEdge};

/// The frame the game steps, and the substeps it takes.
const DT: f32 = 1.0 / 60.0;
const SUBSTEPS: u32 = 4;

/// How far the swimmer is tipped over when caught: about its stroke.
const STROKE_PITCH: f32 = 80.0;

/// Fastest a body standing up from the stroke may spin, rad/s: standing
/// over `settle` asks for a few; asked to stand at once while the joint pins
/// its head, it spins without bound.
const STEADY_SPIN: f32 = 10.0;

/// The wall's face, along +x, and its top, above the hanger's centre as
/// first found: in the catch band.
const FACE: f32 = 0.3;
const LIP: f32 = 1.45;

/// A wall to catch, static, its face at `FACE` and its top at `LIP`.
fn wall(world: &mut PhysicsWorld) {
    let (min, max) = (
        Point3::new(FACE, 0.0, -1.0),
        Point3::new(FACE + 1.0, LIP, 1.0),
    );
    let body =
        world.create_body(RigidBodyDesc::static_body().position(nalgebra::center(&min, &max)));
    world.attach_collider(body, ColliderDesc::box_shape((max - min) / 2.0));
}

/// The wall's edge, as the hands find it from a body upright at `centre`.
fn sensed(world: &PhysicsWorld, centre: Point3<f32>, config: &LocomotionConfig) -> SensedEdge {
    let caster = WorldCaster {
        world,
        scenery: &EmptyGeometry,
        exclude: &[],
        filter: CollisionFilter::DEFAULT,
        carriers: &[],
    };
    let reach = Reach {
        centre,
        heading: Vector3::x(),
        radius: config.collider_radius,
        half_height: config.collider_half_height,
        mass: 70.0,
    };
    let edge = edge_at_hands(&caster, &reach, &config.ledge).expect("the hands find the lip");
    SensedEdge {
        edge,
        at: edge.in_world(world).expect("a static edge"),
        room: [false; 2],
    }
}

/// A body built as the player's is, tipped `pitch` degrees towards the wall,
/// and the attitude holding it there.
fn swimmer(
    world: &mut PhysicsWorld,
    centre: Point3<f32>,
    pitch: f32,
    config: &LocomotionConfig,
) -> (RigidBodyHandle, ConstraintHandle) {
    // Facing the wall (its front, local +Z, along +x), then tipped over
    // towards it about its side.
    let facing = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), FRAC_PI_2);
    let tipped = UnitQuaternion::from_axis_angle(&-Vector3::z_axis(), pitch.to_radians()) * facing;
    let body = world.create_body(RigidBodyDesc::dynamic().position(centre).rotation(tipped));
    world.attach_collider(
        body,
        ColliderDesc::capsule(config.collider_half_height, config.collider_radius),
    );
    world
        .body_mut(body)
        .expect("the swimmer")
        .scale_local_inertia(Vector3::new(1.0, 50.0, 1.0));
    world.wake_body(body);
    let attitude = world.create_constraint(ConstraintKind::KeepAttitude {
        body,
        pitch: pitch.to_radians(),
        compliance: 0.0,
        max_impulse: f32::INFINITY,
    });
    (body, attitude)
}

/// What a catch from the stroke came to.
struct Caught {
    /// Fastest the body spun, rad/s.
    fastest_spin: f32,
    /// Its axis, and its centre, at the end.
    axis: Vector3<f32>,
    centre: Point3<f32>,
    /// Where the centre hangs.
    hang: Point3<f32>,
}

/// Catch the wall's lip lying `pitch` degrees over, then hang for `seconds`,
/// the attitude held at the pitch the grip asks, as `CharacterControlSystem`
/// holds it.
fn catch_from_the_stroke(pitch: f32, seconds: f32) -> Caught {
    let config = LocomotionConfig::default();
    let mut world = PhysicsWorld::new(PhysicsConfig::default());
    wall(&mut world);
    let found_from = Point3::new(0.0, 1.0, 0.0);
    let sensed = sensed(&world, found_from, &config);
    // Back from the face by the head end of the tipped capsule, which rests
    // against it, as a swimmer stroking into a wall does.
    let reach = config.collider_half_height + config.collider_radius;
    let centre = Point3::new(FACE - reach * pitch.to_radians().sin() - 0.01, 1.0, 0.0);
    let (body, upright) = swimmer(&mut world, centre, pitch, &config);

    let mut grip = None;
    let mut debug = DebugLines::default();
    let mut fastest_spin = 0.0f32;
    let frames = (seconds / DT).round() as usize;
    for frame in 0..frames {
        let elapsed = frame as f32 * DT;
        let phase = if elapsed < config.ledge.settle {
            LedgePhase::Grab { elapsed }
        } else {
            LedgePhase::Hang { lean: None }
        };
        let state = LocomotionState::Ledge(Ledge {
            edge: sensed.edge,
            phase,
        });
        Grip::reconcile(
            &mut grip,
            &mut world,
            &GripFrame {
                body,
                state: &state,
                sensed: Some(&sensed),
                config: &config,
                dt: DT,
            },
        );
        let held = grip.as_ref().expect("the hands hold the lip");
        let pitch = held.pitch(&config);
        if let Some(constraint) = world.constraint_mut(upright) {
            if let ConstraintKind::KeepAttitude { pitch: asked, .. } = &mut constraint.kind {
                *asked = pitch;
            }
        }
        world.update_contacts(DT, SUBSTEPS, &EmptyGeometry, &[], &mut debug);
        for _ in 0..SUBSTEPS {
            world.substep(DT / SUBSTEPS as f32, &EmptyGeometry, &[]);
            let spin = world
                .body(body)
                .expect("the swimmer")
                .angular_velocity()
                .norm();
            fastest_spin = fastest_spin.max(spin);
        }
    }

    let end = world.body(body).expect("the swimmer");
    let ledge = &config.ledge;
    let hang = sensed.at.face()
        + sensed.at.outward.into_inner() * (config.collider_radius + ledge.face_gap)
        - Vector3::y() * ledge.hang_reach;
    Caught {
        fastest_spin,
        axis: end.rotation() * Vector3::y(),
        centre: end.position(),
        hang,
    }
}

#[test]
fn a_swimmer_caught_lying_over_stands_up_into_the_hang() {
    let caught = catch_from_the_stroke(STROKE_PITCH, 1.0);
    assert!(
        caught.fastest_spin < STEADY_SPIN,
        "it spun at up to {} rad/s",
        caught.fastest_spin
    );
    assert!(caught.axis.y > 0.99, "it hangs along {:?}", caught.axis);
    assert!(
        (caught.centre - caught.hang).norm() < 0.05,
        "it hangs at {:?}, not {:?}",
        caught.centre,
        caught.hang
    );
}

#[test]
fn a_body_caught_upright_is_held_upright_from_the_first_frame() {
    let config = LocomotionConfig::default();
    let mut world = PhysicsWorld::new(PhysicsConfig::default());
    wall(&mut world);
    let centre = Point3::new(0.0, 1.0, 0.0);
    let sensed = sensed(&world, centre, &config);
    let (body, _) = swimmer(&mut world, centre, 0.0, &config);
    let state = LocomotionState::Ledge(Ledge {
        edge: sensed.edge,
        phase: LedgePhase::Grab { elapsed: 0.0 },
    });
    let mut grip = None;
    Grip::reconcile(
        &mut grip,
        &mut world,
        &GripFrame {
            body,
            state: &state,
            sensed: Some(&sensed),
            config: &config,
            dt: DT,
        },
    );
    let pitch = grip.expect("the hands hold the lip").pitch(&config);
    assert!(pitch.abs() < 1.0e-3, "held at {pitch} rad");
}
